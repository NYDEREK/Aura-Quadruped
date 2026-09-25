#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "board.h"
#include "servo_bus.h"

static const uart_port_t port = UART_NUM_2;
static SemaphoreHandle_t mutex;
static bool ready;
static bool external_power_enabled;

esp_err_t servo_bus_init(void)
{
    // Disable the external transmitter before assigning UART pins.
    ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_SERVO_DIR, 0), "servo", "direction latch");
    const gpio_config_t direction = {
        .pin_bit_mask = 1ULL << BOARD_SERVO_DIR, .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&direction), "servo", "direction output");
    const uart_config_t config = {
        .baud_rate = 1000000, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(port, 512, 0, 0, NULL, 0), "servo", "UART driver");
    ESP_RETURN_ON_ERROR(uart_param_config(port, &config), "servo", "UART config");
    ESP_RETURN_ON_ERROR(uart_set_pin(port, BOARD_SERVO_TX, BOARD_SERVO_RX,
                                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), "servo", "UART pins");
    mutex = xSemaphoreCreateMutex();
    if (!mutex) return ESP_ERR_NO_MEM;
    // This stays false until VIN has been measured above the safe threshold.
    // On USB-only power, U8 remains high impedance and cannot back-power an
    // unpowered chain of ST3215 TTL inputs through DATA.
    external_power_enabled = false;
    ready = true;
    return ESP_OK;
}

void servo_bus_set_external_power_enabled(bool enabled)
{
    if (!ready || !mutex) return;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    external_power_enabled = enabled;
    if (!enabled) (void)gpio_set_level(BOARD_SERVO_DIR, 0);
    xSemaphoreGive(mutex);
}

static esp_err_t exchange_timed(const uint8_t *packet, size_t size, servo_status_t *status, bool realtime)
{
    if (!ready) return ESP_ERR_INVALID_STATE;
    if (size == 0 || packet[2] > 253 ||
        (packet[4] != 1 && packet[4] != 2 && packet[4] != 3)) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(mutex, realtime ? 0 : pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (!external_power_enabled) {
        (void)gpio_set_level(BOARD_SERVO_DIR, 0);
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = uart_flush_input(port);
    if (result == ESP_OK) result = gpio_set_level(BOARD_SERVO_DIR, 1);
    if (result == ESP_OK && uart_write_bytes(port, packet, size) != (int)size) result = ESP_FAIL;
    if (result == ESP_OK) result = uart_wait_tx_done(port, pdMS_TO_TICKS(realtime ? 2 : 20));
    // Release the bus after the last stop bit, including all error paths.
    esp_err_t release = gpio_set_level(BOARD_SERVO_DIR, 0);
    if (result == ESP_OK) result = release;
    if (result == ESP_OK) {
        uint8_t received[128];
        size_t used = 0;
        const int64_t deadline = esp_timer_get_time() + (realtime ? 5000 : 30000);
        result = ESP_ERR_TIMEOUT;
        while (esp_timer_get_time() < deadline) {
            if (used == sizeof(received)) {
                memmove(received, received + 64, 64);
                used = 64;
            }
            int count = uart_read_bytes(port, received + used, sizeof(received) - used, pdMS_TO_TICKS(2));
            if (count < 0) { result = ESP_FAIL; break; }
            used += count;
            if (servo_find_status(received, used, packet[2], packet, size, status)) {
                result = ESP_OK;
                break;
            }
        }
    }
    xSemaphoreGive(mutex);
    return result;
}

static esp_err_t exchange(const uint8_t *packet, size_t size, servo_status_t *status)
{
    return exchange_timed(packet,size,status,false);
}

static esp_err_t transmit_broadcast(const uint8_t *packet, size_t size)
{
    if (!ready || !packet || size == 0 || size > SERVO_MAX_PACKET) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (!external_power_enabled) {
        (void)gpio_set_level(BOARD_SERVO_DIR, 0);
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = uart_flush_input(port);
    if (result == ESP_OK) result = gpio_set_level(BOARD_SERVO_DIR, 1);
    if (result == ESP_OK && uart_write_bytes(port, packet, size) != (int)size) result = ESP_FAIL;
    if (result == ESP_OK) result = uart_wait_tx_done(port, pdMS_TO_TICKS(20));
    const esp_err_t release = gpio_set_level(BOARD_SERVO_DIR, 0);
    if (result == ESP_OK) result = release;
    xSemaphoreGive(mutex);
    return result;
}

esp_err_t servo_ping(uint8_t id, servo_status_t *status)
{
    uint8_t request[6];
    const size_t size = servo_make_ping(id, request);
    memset(status, 0, sizeof(*status));
    esp_err_t result = exchange(request, size, status);
    if (result == ESP_OK && status->data_size != 0) return ESP_ERR_INVALID_SIZE;
    return result;
}

esp_err_t servo_feedback(uint8_t id, servo_status_t *status)
{
    uint8_t request[8];
    const size_t size = servo_make_read(id, 56, 15, request);
    memset(status, 0, sizeof(*status));
    esp_err_t result = exchange(request, size, status);
    if (result == ESP_OK && status->data_size != 15) return ESP_ERR_INVALID_SIZE;
    return result;
}

esp_err_t servo_feedback_realtime(uint8_t id, servo_status_t *status)
{
    if (!status) return ESP_ERR_INVALID_ARG;
    uint8_t request[8];
    const size_t size=servo_make_read(id,56,15,request);
    memset(status,0,sizeof(*status));
    const esp_err_t result=exchange_timed(request,size,status,true);
    if (result==ESP_OK && status->data_size!=15) return ESP_ERR_INVALID_SIZE;
    return result;
}

static esp_err_t write_registers(uint8_t id, uint8_t reg, const uint8_t *data, size_t count)
{
    uint8_t request[SERVO_MAX_PACKET];
    const size_t size = servo_make_write(id, reg, data, count, request);
    servo_status_t status = {0};
    esp_err_t result = exchange(request, size, &status);
    if (result == ESP_OK && status.data_size != 0) return ESP_ERR_INVALID_SIZE;
    if (result == ESP_OK && status.error != 0) return ESP_ERR_INVALID_RESPONSE;
    return result;
}

static esp_err_t read_registers(uint8_t id, uint8_t reg, uint8_t *data, size_t count)
{
    if (!data || count == 0 || count > SERVO_MAX_DATA) return ESP_ERR_INVALID_ARG;
    uint8_t request[8];
    const size_t size = servo_make_read(id, reg, (uint8_t)count, request);
    if (size == 0) return ESP_ERR_INVALID_ARG;
    servo_status_t status = {0};
    esp_err_t result = exchange(request, size, &status);
    if (result == ESP_OK && status.error != 0) return ESP_ERR_INVALID_RESPONSE;
    if (result == ESP_OK && status.data_size != count) return ESP_ERR_INVALID_SIZE;
    if (result == ESP_OK) memcpy(data, status.data, count);
    return result;
}

esp_err_t servo_read_mode(uint8_t id, servo_mode_t *mode)
{
    if (!mode) return ESP_ERR_INVALID_ARG;
    uint8_t value = 0xff;
    ESP_RETURN_ON_ERROR(read_registers(id, 33, &value, 1), "servo", "read mode");
    if (value > SERVO_MODE_MOTOR) return ESP_ERR_INVALID_RESPONSE;
    *mode = (servo_mode_t)value;
    return ESP_OK;
}

esp_err_t servo_read_position_limits(uint8_t id, servo_position_limits_t *limits)
{
    if (!limits) return ESP_ERR_INVALID_ARG;
    uint8_t data[4] = {0};
    ESP_RETURN_ON_ERROR(read_registers(id, 9, data, sizeof(data)), "servo", "read position limits");
    const uint16_t encoded_minimum = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    const uint16_t encoded_maximum = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    limits->minimum = (int16_t)servo_signed_magnitude(encoded_minimum, 15);
    limits->maximum = (int16_t)servo_signed_magnitude(encoded_maximum, 15);
    return ESP_OK;
}

esp_err_t servo_set_torque(uint8_t id, bool enabled)
{
    const uint8_t value = enabled ? 1 : 0;
    return write_registers(id, 40, &value, 1);
}

esp_err_t servo_set_torque_all(bool enabled)
{
    uint8_t values[SERVO_MAX_SYNC_SERVOS];
    servo_sync_write_item_t items[SERVO_MAX_SYNC_SERVOS];
    const uint8_t value = enabled ? 1 : 0;
    for (size_t index = 0; index < SERVO_MAX_SYNC_SERVOS; ++index) {
        values[index] = value;
        items[index] = (servo_sync_write_item_t){
            .id = (uint8_t)(index + 1), .data = &values[index],
        };
    }
    uint8_t packet[SERVO_MAX_PACKET];
    const size_t size = servo_make_sync_write(40, 1, items, SERVO_MAX_SYNC_SERVOS, packet);
    if (size == 0) return ESP_ERR_INVALID_ARG;
    return transmit_broadcast(packet, size);
}

esp_err_t servo_move(uint8_t id, int16_t position, uint16_t speed, uint8_t acceleration)
{
    if (position < -4095 || position > 4095 || speed > 3400) return ESP_ERR_INVALID_ARG;
    const uint16_t encoded_position = servo_signed_magnitude_encode(position, 15);
    const uint8_t values[] = {
        acceleration,
        (uint8_t)encoded_position, (uint8_t)(encoded_position >> 8),
        0, 0,
        (uint8_t)speed, (uint8_t)(speed >> 8),
    };
    return write_registers(id, 41, values, sizeof(values));
}

esp_err_t servo_move_sync(const servo_position_command_t *commands, size_t count)
{
    if (!commands || count == 0 || count > SERVO_MAX_SYNC_SERVOS) return ESP_ERR_INVALID_ARG;
    uint8_t values[SERVO_MAX_SYNC_SERVOS][7];
    servo_sync_write_item_t items[SERVO_MAX_SYNC_SERVOS];
    for (size_t i = 0; i < count; ++i) {
        if (commands[i].id > 253 || commands[i].position < -4095 ||
            commands[i].position > 4095 || commands[i].speed > 3400)
            return ESP_ERR_INVALID_ARG;
        const uint16_t encoded_position =
            servo_signed_magnitude_encode(commands[i].position, 15);
        values[i][0] = commands[i].acceleration;
        values[i][1] = (uint8_t)encoded_position;
        values[i][2] = (uint8_t)(encoded_position >> 8);
        values[i][3] = 0; // Goal time: controller operates from the 50 Hz tick.
        values[i][4] = 0;
        values[i][5] = (uint8_t)commands[i].speed;
        values[i][6] = (uint8_t)(commands[i].speed >> 8);
        items[i] = (servo_sync_write_item_t){.id = commands[i].id, .data = values[i]};
    }
    uint8_t packet[SERVO_MAX_PACKET];
    const size_t size = servo_make_sync_write(41, sizeof(values[0]), items, count, packet);
    if (size == 0) return ESP_ERR_INVALID_ARG;
    return transmit_broadcast(packet, size);
}

esp_err_t servo_motor_speed(uint8_t id, int16_t speed, uint8_t acceleration)
{
    if (speed < -3400 || speed > 3400) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(write_registers(id, 41, &acceleration, 1), "servo", "write acceleration");
    const uint16_t encoded = servo_signed_magnitude_encode(speed, 15);
    const uint8_t values[] = {(uint8_t)encoded, (uint8_t)(encoded >> 8)};
    return write_registers(id, 46, values, sizeof(values));
}

esp_err_t servo_set_mode(uint8_t id, servo_mode_t mode)
{
    if (mode != SERVO_MODE_POSITION && mode != SERVO_MODE_MOTOR) return ESP_ERR_INVALID_ARG;

    // Changing mode writes EEPROM. Stop first, keep torque disabled afterwards,
    // and always make a best effort to lock EEPROM again after it was unlocked.
    ESP_RETURN_ON_ERROR(servo_set_torque(id, false), "servo", "disable torque");
    const uint8_t unlock = 0;
    ESP_RETURN_ON_ERROR(write_registers(id, 55, &unlock, 1), "servo", "unlock EEPROM");

    const uint8_t requested = (uint8_t)mode;
    esp_err_t result = write_registers(id, 33, &requested, 1);
    if (result == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(30));
        servo_mode_t actual;
        result = servo_read_mode(id, &actual);
        if (result == ESP_OK && actual != mode) result = ESP_ERR_INVALID_RESPONSE;
    }

    const uint8_t lock = 1;
    const esp_err_t lock_result = write_registers(id, 55, &lock, 1);
    if (result == ESP_OK) result = lock_result;
    if (result == ESP_OK && mode == SERVO_MODE_MOTOR)
        result = servo_motor_speed(id, 0, 0);
    return result;
}

esp_err_t servo_calibrate_center(uint8_t id)
{
    // Feetech's STS library defines TORQUE_ENABLE=128 as CalibrationOfs:
    // store the current shaft position as the servo's mechanical midpoint.
    const uint8_t calibrate = 128;
    return write_registers(id, 40, &calibrate, 1);
}

esp_err_t servo_assign_id(uint8_t current_id, uint8_t new_id)
{
    if (current_id > 253 || new_id > 253 || current_id == new_id) return ESP_ERR_INVALID_ARG;
    servo_status_t status;
    ESP_RETURN_ON_ERROR(servo_ping(current_id, &status), "servo", "current ID did not answer");
    if (servo_ping(new_id, &status) == ESP_OK) return ESP_ERR_INVALID_STATE;

    const uint8_t unlock = 0;
    ESP_RETURN_ON_ERROR(write_registers(current_id, 55, &unlock, 1), "servo", "unlock EEPROM");
    ESP_RETURN_ON_ERROR(write_registers(current_id, 5, &new_id, 1), "servo", "write ID");
    vTaskDelay(pdMS_TO_TICKS(30));
    ESP_RETURN_ON_ERROR(servo_ping(new_id, &status), "servo", "verify new ID");
    const uint8_t lock = 1;
    ESP_RETURN_ON_ERROR(write_registers(new_id, 55, &lock, 1), "servo", "lock EEPROM");
    return servo_ping(new_id, &status);
}
