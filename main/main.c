#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_state.h"
#include "aura_radio.h"
#include "dualsense.h"
#include "aura_network.h"
#include "dualsense.h"
#include "board.h"
#include "mpu6050.h"
#include "power.h"
#include "robot_control.h"
#include "robot_gait.h"
#include "robot_locomotion.h"
#include "servo_bus.h"
#include "tof.h"
#include "ws2812.h"

#define CONSOLE_UART UART_NUM_0
#define LINE_SIZE 96
#define RADIO_VIN_ENABLE_V 7.0f
#define VIN_MONITOR_PERIOD_MS 50
#define VIN_ENABLE_CONFIRM_SAMPLES 3
// A battery hot-plug has a slower rail rise than a manual EN reset.  Keep
// every bus driver passive while the regulator and ADC reference settle.
#define STARTUP_POWER_SETTLE_MS 750
// Temporary USB/UART-only MPU6050 isolation. No INA226 or VL53 I2C request
// is issued; robot torque is still disabled and Wi-Fi/Bluetooth stay off.
#define AURA_MPU6050_EXCLUSIVE_TEST 0

static esp_err_t radio_nvs_status = ESP_ERR_INVALID_STATE;
#define STATE_INDICATOR_PIXEL_COUNT 64

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} state_colour_t;

// The strip and DualSense lightbar are an authoritative robot-state
// indicator. Reassert them so a manual LED test or a pad reconnect cannot
// leave a stale colour (for example the DualSense default blue) on either.
static void state_indicator_task(void *argument)
{
    (void)argument;
    uint8_t strip[STATE_INDICATOR_PIXEL_COUNT * 3] = {0};
    for (;;) {
        robot_gait_snapshot_t gait = {0};
        robot_gait_snapshot(&gait);
        const bool armed = robot_control_is_armed();
        const bool safety_fault = robot_control_has_safety_fault();
        const bool red_phase = ((unsigned)(xTaskGetTickCount() / pdMS_TO_TICKS(250)) & 1U) == 0;
        // A latched physical-stall fault has priority over all normal state
        // colours. The board strip and the DualSense lightbar share this exact
        // blinking signal until the operator clears the latch while disarmed.
        const state_colour_t current = safety_fault
            ? (red_phase ? (state_colour_t){.red = 255, .green = 0, .blue = 0}
                         : (state_colour_t){.red = 0, .green = 0, .blue = 0})
            : gait.calibration_test
                ? (state_colour_t){.red = 255, .green = 0, .blue = 0}       // calibration
                : armed ? (state_colour_t){.red = 0, .green = 255, .blue = 0} // armed
                        : (state_colour_t){.red = 255, .green = 96, .blue = 0}; // disarmed
        for (size_t pixel = 0; pixel < STATE_INDICATOR_PIXEL_COUNT; ++pixel) {
            strip[pixel * 3] = current.red;
            strip[pixel * 3 + 1] = current.green;
            strip[pixel * 3 + 2] = current.blue;
        }
        (void)ws2812_write_rgb(strip, STATE_INDICATOR_PIXEL_COUNT);
        // Sony's centred patterns show 0...5 white dots independently of RGB.
        // Count = selected mode: stand, trot, crawl, run, climb, three-leg walk.
        const uint8_t players = !safety_fault && !gait.calibration_test
            ? robot_locomotion_player_leds(gait.selected_gait) : 0;
        dualsense_set_indicators(current.red, current.green, current.blue, players);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static esp_err_t init_calibration_nvs(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // This partition stores ESP-IDF radio calibration and VL53L4CD settings.
        // Recovering it cannot affect the factory application partition.
        esp_err_t erase_result = nvs_flash_erase();
        if (erase_result != ESP_OK) return erase_result;
        result = nvs_flash_init();
    }
    return result;
}

static uint16_t little_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void print_help(void)
{
    printf("Pad UART: pad connect | pad disconnect | pad scan | pad list\n");
    printf("Commands:\n"
           "  help              - this list\n"
           "  status            - latest IMU and power readings\n"
           "  imu retry         - reconfigure MPU6050 on S1 I2C and restart its observer\n"
           "  servo ping <id>   - ping one ST3215 ID (0..253)\n"
           "  servo read <id>   - read feedback registers 56..70\n"
           "  robot arm|disarm  - torque all configured robot axes on or off\n"
           "  gait virtual off  - return gait input to the paired pad\n"
           "  gait virtual F [L T H] - virtual sticks, each -1000..1000\n"
           "  tof retry         - detect both VL53L4CD sensors again\n"
           "  ble retry         - restart radios; both require external VIN > 7.0 V\n"
           "Servo configuration is available through Wi-Fi.\n");
}

static void print_status(void)
{
    dualsense_snapshot_t controller;
    dualsense_get_snapshot(&controller);
    printf("DualSense: state=%d saved=%d reports=%lu\n", controller.state, controller.has_saved_controller, (unsigned long)controller.sample_count);
    dualsense_print_diagnostics();
    printf("Pad controls: fresh=%d LX=%u LY=%u RX=%u RY=%u L2=%u R2=%u buttons=%02x:%02x:%02x\n", controller.has_input, controller.left_x, controller.left_y, controller.right_x, controller.right_y, controller.left_trigger, controller.right_trigger, controller.buttons[0], controller.buttons[1], controller.buttons[2]);
    robot_gait_snapshot_t gait = {0};
    robot_model_t robot = {0};
    robot_gait_snapshot(&gait);
    robot_control_snapshot(&robot);
    unsigned assigned_axes = 0, feedback_axes = 0, moving_axes = 0;
    float maximum_tracking_error = 0.0f;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg)
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
            assigned_axes += robot.axis[leg][axis].config.servo_id != ROBOT_SERVO_ID_UNASSIGNED;
            feedback_axes += robot.axis[leg][axis].present;
            moving_axes += robot.axis[leg][axis].moving;
            if (robot.axis[leg][axis].present)
                maximum_tracking_error = fmaxf(maximum_tracking_error,
                                               fabsf(robot.axis[leg][axis].target_radians -
                                                     robot.axis[leg][axis].measured_radians));
        }
    printf("Robot: armed=%d axes=%u/12 gait=%d active=%d phase=%u input=%d:%d:%d spin=%d jump=%d virtual=%d rate=%u%% ticks=%lu\n",
           robot_control_is_armed(), assigned_axes, gait.selected_gait, gait.active_gait,
           gait.phase_milli, gait.input_forward_milli, gait.input_lateral_milli,
           gait.input_turn_milli, gait.spin_mode, gait.jumping, gait.virtual_input,
           gait.phase_rate_percent, (unsigned long)gait.tick_count);
    printf("Planner: last=%lu us max=%lu us overruns=%lu frame_drops=%lu (20 ms budget) frequency=%u cHz\n",
           (unsigned long)gait.planner_last_us,(unsigned long)gait.planner_max_us,
           (unsigned long)gait.planner_overruns,(unsigned long)gait.target_frame_drops,gait.effective_frequency_centi_hz);
    printf("Servo feedback: %u/%u live, %u moving, largest target error %.1f deg\n",
           feedback_axes, assigned_axes, moving_axes, maximum_tracking_error * 180.0f / 3.14159265f);
    aura_network_print_status();
    app_state_snapshot_t state;
    app_state_get(&state);

    if (state.imu_error == ESP_OK && state.imu.sample_count) {
        mpu6050_attitude_t attitude = {0};
        mpu6050_get_attitude(&attitude);
        printf("IMU MPU6050 [I2C 0x%02x] sample=%lu accel[g]=%.3f %.3f %.3f "
               "gyro[dps]=%.2f %.2f %.2f temp=%.1f C roll/pitch=%.1f/%.1f deg%s\n",
               state.imu_identity, (unsigned long)state.imu.sample_count,
               state.imu.accel_g[0], state.imu.accel_g[1], state.imu.accel_g[2],
               state.imu.gyro_dps[0], state.imu.gyro_dps[1], state.imu.gyro_dps[2],
               state.imu.temperature_c,
               attitude.roll_radians * 180.0f / 3.14159265f,
               attitude.pitch_radians * 180.0f / 3.14159265f,
               attitude.valid ? "" : " (filter settling)");
    } else {
        printf("IMU unavailable: WHO_AM_I=0x%02x, %s\n",
               state.imu_identity, esp_err_to_name(state.imu_error));
    }
    if (state.power.ina_error == ESP_OK) {
        printf("INA226 [manufacturer=0x%04x die=0x%04x] servo bus=%.3f V "
               "shunt=%.3f mV current=%.3f A estimated power=%.3f W\n",
               state.power.manufacturer_id, state.power.die_id,
               state.power.bus_v, state.power.shunt_mv,
               state.power.servo_current_a, state.power.servo_input_power_w);
    } else {
        printf("INA226 unavailable: manufacturer=0x%04x die=0x%04x, %s\n",
               state.power.manufacturer_id, state.power.die_id,
               esp_err_to_name(state.power.ina_error));
    }
    if (state.power.battery_error == ESP_OK) {
        printf("VIN divider: ADC=%d voltage=%.3f V (%s calibration)\n",
               state.power.battery_adc_raw, state.power.battery_v,
               state.power.battery_calibrated ? "eFuse" : "default 1100 mV reference");
    } else {
        printf("VIN ADC unavailable: %s\n", esp_err_to_name(state.power.battery_error));
    }
    printf("Bluetooth: %s (requires external VIN > %.1f V)\n",
           aura_radio_is_running() ? "ON" : "OFF", RADIO_VIN_ENABLE_V);
    tof_snapshot_t tof;
    memset(&tof, 0, sizeof(tof));
    tof_get_snapshot(&tof);
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        const tof_sensor_state_t *sensor = &tof.sensor[i];
        if (sensor->present) {
            printf("VL53L4CD port %d [0x%02x]: %u mm, status=%u, samples=%lu%s\n",
                   i + 1, sensor->address_7bit, sensor->distance_mm,
                   sensor->range_status, (unsigned long)sensor->sample_count,
                   sensor->ranging ? "" : " (idle)");
        } else {
            printf("VL53L4CD port %d [0x%02x]: unavailable (%s)\n", i + 1,
                   sensor->address_7bit, esp_err_to_name(sensor->error));
        }
    }
}

static bool parse_servo_id(const char *text, uint8_t *id)
{
    if (!text || !isdigit((unsigned char)*text)) return false;
    char *end;
    long value = strtol(text, &end, 10);
    while (*end == ' ') ++end;
    if (*end != '\0' || value < 0 || value > 253) return false;
    *id = (uint8_t)value;
    return true;
}

static void servo_command(bool feedback, const char *argument)
{
    uint8_t id;
    if (!parse_servo_id(argument, &id)) {
        printf("Invalid servo ID; use 0..253.\n");
        return;
    }
    servo_status_t status;
    esp_err_t result = feedback ? servo_feedback(id, &status) : servo_ping(id, &status);
    if (result != ESP_OK) {
        printf("Servo %u did not answer: %s. Check VIN power and wiring.\n",
               id, esp_err_to_name(result));
        return;
    }
    if (!feedback) {
        printf("Servo %u answered ping; status error=0x%02x.\n", id, status.error);
        return;
    }
    const uint16_t position = little_u16(&status.data[0]);
    const int speed = servo_signed_magnitude(little_u16(&status.data[2]), 15);
    const int load = servo_signed_magnitude(little_u16(&status.data[4]), 10);
    const uint16_t current = little_u16(&status.data[13]);
    printf("Servo %u: error=0x%02x position=%u/4095 speed_raw=%d load_raw=%d "
           "voltage=%.1f V temp=%u C moving=%u current_raw=%u\n",
           id, status.error, position, speed, load, status.data[6] / 10.0f,
           status.data[7], status.data[10], current);
}

static void gait_virtual_command(const char *argument)
{
    if (!strcmp(argument, "off")) {
        printf("Virtual gait input: %s\n", esp_err_to_name(robot_gait_clear_virtual_input()));
        return;
    }
    int forward = 0, lateral = 0, turn = 0, height = 0;
    char extra = '\0';
    const int fields = sscanf(argument, "%d %d %d %d %c",
                              &forward, &lateral, &turn, &height, &extra);
    if (fields < 1 || fields == 5 || forward < -1000 || forward > 1000 ||
        lateral < -1000 || lateral > 1000 || turn < -1000 || turn > 1000 ||
        height < -1000 || height > 1000) {
        printf("Use: gait virtual F [L T H], each -1000..1000; or gait virtual off\n");
        return;
    }
    const robot_gait_virtual_input_t input = {
        .enabled = true,
        .forward_milli = (int16_t)forward,
        .lateral_milli = (int16_t)lateral,
        .turn_milli = (int16_t)turn,
        .height_milli = (int16_t)height,
    };
    printf("Virtual gait input: %s\n", esp_err_to_name(robot_gait_set_virtual_input(&input)));
}

static void execute_command(char *line)
{
    while (*line == ' ') ++line;
    size_t length = strlen(line);
    while (length && line[length - 1] == ' ') line[--length] = '\0';
    if (!strcmp(line, "help")) print_help();
    else if (!strcmp(line, "status")) print_status();
    else if (!strcmp(line, "pad connect"))
        printf("Pad connect: %s\n", esp_err_to_name(dualsense_connect_saved()));
    else if (!strcmp(line, "pad disconnect"))
        printf("Pad disconnect: %s\n", esp_err_to_name(dualsense_disconnect()));
    else if (!strcmp(line, "pad scan"))
        printf("Pad scan: %s\n", esp_err_to_name(dualsense_start_pairing()));
    else if (!strcmp(line, "pad list")) {
        dualsense_scan_device_t devices[DUALSENSE_MAX_SCAN_DEVICES];
        size_t count = dualsense_get_scanned_devices(devices, DUALSENSE_MAX_SCAN_DEVICES);
        for (size_t i = 0; i < count; ++i) {
            uint8_t *a = devices[i].address;
            printf("%02x:%02x:%02x:%02x:%02x:%02x %s\n", a[0],a[1],a[2],a[3],a[4],a[5],devices[i].name);
        }
    }
    else if (!strcmp(line, "robot arm"))
        printf("Robot arm: %s\n", esp_err_to_name(robot_gait_arm()));
    else if (!strcmp(line, "robot disarm"))
        printf("Robot disarm: %s\n", esp_err_to_name(robot_control_disarm()));
    else if (!strncmp(line, "gait virtual ", 13)) gait_virtual_command(line + 13);
    else if (!strncmp(line, "servo ping ", 11)) servo_command(false, line + 11);
    else if (!strncmp(line, "servo read ", 11)) servo_command(true, line + 11);
    else if (!strcmp(line, "imu retry")) {
        uint8_t identity = 0;
        const esp_err_t result = mpu6050_reinitialize(&identity);
        app_state_set_imu_status(result, identity);
        printf("MPU6050 reconfiguration: %s (I2C address 0x%02x)\n",
               esp_err_to_name(result), identity);
    }
    else if (!strcmp(line, "tof retry"))
        printf("VL53L4CD detection: %s\n", esp_err_to_name(tof_reinitialize()));
    else if (!strcmp(line, "ble retry")) {
        printf("Restarting Bluetooth...\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    }
    else if (*line) printf("Unknown command. Type: help\n");
}

static void console_task(void *argument)
{
    (void)argument;
    uint8_t buffer[32];
    char line[LINE_SIZE];
    size_t used = 0;
    bool overflow = false;
    while (true) {
        int count = uart_read_bytes(CONSOLE_UART, buffer, sizeof(buffer), pdMS_TO_TICKS(100));
        for (int i = 0; i < count; ++i) {
            uint8_t byte = buffer[i];
            if (byte == '\r' || byte == '\n') {
                if (overflow) printf("Command too long; discarded.\n");
                else if (used) { line[used] = '\0'; execute_command(line); }
                used = 0;
                overflow = false;
            } else if (byte >= 0x20 && byte != 0x7f && !overflow) {
                if (used < sizeof(line) - 1) line[used++] = (char)byte;
                else overflow = true;
            }
        }
    }
}

static void blink_task(void *argument)
{
    (void)argument;
    bool on = true;
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        on = !on;
        gpio_set_level(BOARD_LED, on);
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(500));
    }
}

static void telemetry_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    unsigned report_divider = 0;
    while (true) {
        // Keep polling after a transient I2C error. The next coherent sample
        // restores the public IMU status instead of permanently stopping
        // the owner task after one arbitration timeout.
        imu_sample_t sample;
        const esp_err_t result = mpu6050_read(&sample);
        app_state_set_imu_sample(result, &sample);
        if (++report_divider >= 100) {
            print_status();
            report_divider = 0;
        }
        // MPU6050 DATA_RDY is configured at 100 Hz. A 10 ms owner task
        // keeps the observer fresh without letting the 50 Hz gait task touch
        // the shared I2C bus.
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}

static void vin_radio_guard_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    TickType_t next_start_attempt = 0;
    unsigned present_samples = 0;

    while (true) {
        float vin = 0.0f;
        const esp_err_t vin_error = power_read_vin(&vin, NULL);
        const bool external_power = vin_error == ESP_OK && vin > RADIO_VIN_ENABLE_V;
        // Keep the half-duplex transmitter high impedance whenever the servo
        // supply is absent. USB may power only logic; it must not back-power
        // connected ST3215 input stages through the DATA wire.
        servo_bus_set_external_power_enabled(external_power);

        if (aura_radio_is_running() && !external_power) {
            printf("External VIN fell below %.1f V (measured %.3f V); stopping Wi-Fi and Bluetooth.\n",
                   RADIO_VIN_ENABLE_V, vin);
            const esp_err_t result = aura_radio_deinit();
            printf("Radio power gate: %s\n", esp_err_to_name(result));
            present_samples = 0;
        } else if (!aura_radio_is_running()) {
            if (external_power) {
                if (present_samples < VIN_ENABLE_CONFIRM_SAMPLES) ++present_samples;
            } else {
                present_samples = 0;
            }

            const TickType_t now = xTaskGetTickCount();
            if (present_samples >= VIN_ENABLE_CONFIRM_SAMPLES &&
                radio_nvs_status == ESP_OK &&
                (int32_t)(now - next_start_attempt) >= 0) {
                printf("External VIN %.3f V is stable; starting Aura Wi-Fi and Bluetooth.\n", vin);
                const esp_err_t result = aura_radio_init();
                printf("Radio power gate: %s\n", esp_err_to_name(result));
                present_samples = 0;
                next_start_attempt = now + pdMS_TO_TICKS(2000);
            }
        }

        // INA226 runs continuous conversions. Sampling its shunt and bus
        // registers at 50 Hz gives the desktop current trace real data at
        // the same cadence as controls, rather than repeating a 1 Hz value.
        power_sample_t power;
        power_read(&power);
        app_state_set_power(&power);
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(VIN_MONITOR_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(app_state_init());

    // Configure the fitted LED first. HIGH means on per the schematic.
    ESP_ERROR_CHECK(gpio_set_level(BOARD_LED, 0));
    const gpio_config_t led = {
        .pin_bit_mask = 1ULL << BOARD_LED, .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led));
    gpio_set_level(BOARD_LED, 1);

    // S1 is now an MPU6050 connector. Its former XSHUT net (GPIO14) is the
    // MPU DATA_RDY output and must never be driven by the ToF service.
    ESP_ERROR_CHECK(tof_set_port_reserved(0, true));
    // No VL53 is connected during MPU bring-up. Reserving S2 prevents an
    // absent sensor timeout from resetting the shared I2C controller.
    ESP_ERROR_CHECK(tof_set_port_reserved(1, true));
    const esp_err_t xshut_prepare = tof_prepare_pins();

    esp_err_t console_init = uart_driver_install(CONSOLE_UART, 1024, 1024, 0, NULL, 0);
    if (console_init == ESP_OK) {
        uart_vfs_dev_use_driver(CONSOLE_UART);
        setvbuf(stdout, NULL, _IONBF, 0);
    }

    // Robot configuration is persisted in NVS, so NVS must be online before
    // robot_control_init() tries to load the saved axis map.
    radio_nvs_status = init_calibration_nvs();

    printf("\nAura Main Board firmware: ST3215, INA226, MPU6050 on S1, VL53L4CD and WS2812.\n");
    printf("Console UART: %s. Wi-Fi telemetry and Classic Bluetooth HID.\n",
           esp_err_to_name(console_init));

    // A cold battery start can leave the regulator and ADC reference below
    // their final values for longer than an EN-button reset.  Do not probe
    // VIN, INA or attached buses until they have settled; all actuator lines
    // are still passive here and the robot remains disarmed.
    vTaskDelay(pdMS_TO_TICKS(STARTUP_POWER_SETTLE_MS));

    // Initialise MPU first and, while diagnosing it, keep it as the only
    // bus client. This preserves the known-good GRUZIK transaction sequence
    // without an INA226 or VL53 timeout altering the shared controller state.
    uint8_t imu_identity = 0;
    esp_err_t imu_error = mpu6050_init(&imu_identity);
    app_state_set_imu_status(imu_error, imu_identity);

#if AURA_MPU6050_EXCLUSIVE_TEST
    const power_sample_t power = {
        .ina_error = ESP_ERR_NOT_SUPPORTED,
        .battery_error = ESP_ERR_NOT_SUPPORTED,
    };
    app_state_set_power(&power);
    printf("MPU6050 exclusive I2C test: INA226 and VL53L4CD requests disabled.\n");
#else
    // Establish the actual ServoPower/VIN state before the controller can send
    // even a torque-off frame. With USB alone the DATA driver must stay off.
    power_init();
    power_sample_t power;
    power_read(&power);
    app_state_set_power(&power);
#endif

    // Put the half-duplex transmitter in receive/high-impedance mode before
    // initializing the robot core. The bus transaction layer remains disabled
    // until external VIN has been measured above the threshold.
    esp_err_t servo_init = servo_bus_init();
    if (servo_init == ESP_OK)
        servo_bus_set_external_power_enabled(power.battery_error == ESP_OK &&
                                             power.battery_v > RADIO_VIN_ENABLE_V);
    // The robot controller starts empty and disarmed. On USB-only power its
    // requested torque-off frames are blocked at the bus layer, so unpowered
    // ST3215 inputs see no DATA drive.
    esp_err_t robot_init = servo_init == ESP_OK ? robot_control_init() : servo_init;
    if (robot_init == ESP_OK) robot_init = robot_control_start();
    if (robot_init == ESP_OK) robot_init = robot_gait_init();
    if (robot_init == ESP_OK) robot_init = robot_gait_start();

    // S1 is reserved for MPU6050, so no ToF raw-XSHUT probe may drive GPIO14.
#if AURA_MPU6050_EXCLUSIVE_TEST
    const esp_err_t tof_init_result = ESP_ERR_NOT_SUPPORTED;
#else
    // S2 remains available to an optional VL53L4CD; S1 is reported reserved.
    const esp_err_t tof_init_result = radio_nvs_status == ESP_OK
                                          ? tof_init() : radio_nvs_status;
#endif
    esp_err_t pixel_init = ws2812_init(64);
    printf("MPU6050 init: %s (WHO_AM_I/I2C=0x%02x), servo interface: %s, robot core: %s (disarmed)\n",
           esp_err_to_name(imu_error), imu_identity, esp_err_to_name(servo_init),
           esp_err_to_name(robot_init));
    printf("Radio power gate armed: VIN > %.1f V (now %.3f V, RF NVS: %s).\n",
           RADIO_VIN_ENABLE_V, power.battery_v, esp_err_to_name(radio_nvs_status));
    printf("RF NVS: %s, XSHUT startup: %s, S1 ToF probe: reserved for MPU DATA_RDY, WS2812: %s\n",
           esp_err_to_name(radio_nvs_status), esp_err_to_name(xshut_prepare),
           esp_err_to_name(pixel_init));
    printf("ToF service: %s (S1/GPIO14 reserved for MPU6050; S2 XSHUT GPIO15 remains available)\n",
           esp_err_to_name(tof_init_result));
    print_help();
    print_status();

    BaseType_t blink_started = xTaskCreate(blink_task, "board_led", 2048, NULL, 3, NULL);
    BaseType_t telemetry_started = xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 4, NULL);
    BaseType_t indicator_started = xTaskCreate(state_indicator_task, "state_indicator", 3072, NULL, 3, NULL);
    BaseType_t vin_guard_started = xTaskCreate(vin_radio_guard_task, "vin_radio_guard", 8192,
                                               NULL, 6, NULL);
    BaseType_t console_started = console_init == ESP_OK
        ? xTaskCreate(console_task, "console", 4096, NULL, 5, NULL) : pdFAIL;
    printf("Tasks: LED=%s telemetry=%s state indicator=%s VIN/radio guard=%s console=%s\n",
           blink_started == pdPASS ? "OK" : "FAILED",
           telemetry_started == pdPASS ? "OK" : "FAILED",
           indicator_started == pdPASS ? "OK" : "FAILED",
           vin_guard_started == pdPASS ? "OK" : "FAILED",
           console_started == pdPASS ? "OK" : "FAILED");

}
