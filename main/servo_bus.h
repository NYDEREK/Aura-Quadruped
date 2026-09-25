#pragma once
#include "esp_err.h"
#include "servo_protocol.h"

typedef enum {
    SERVO_MODE_POSITION = 0,
    SERVO_MODE_MOTOR = 1,
} servo_mode_t;

typedef struct {
    uint8_t id;
    // STS position is a signed-magnitude 15-bit value on the wire.  Keeping
    // it signed here preserves a joint that crosses electronic zero instead
    // of turning a small negative target into a near-full positive rotation.
    int16_t position;
    uint16_t speed;
    uint8_t acceleration;
} servo_position_command_t;

// The ST3215 keeps its physical position limits in EEPROM. A motor-mode
// servo has no usable position interval, so callers issuing a position command
// can verify this independently of the mode byte.
typedef struct {
    int16_t minimum;
    int16_t maximum;
} servo_position_limits_t;

esp_err_t servo_bus_init(void);
// The DATA pin must never be driven while ServoPower/VIN is absent. This gate
// protects unpowered servos from back-powering through their TTL input.
void servo_bus_set_external_power_enabled(bool enabled);
esp_err_t servo_ping(uint8_t id, servo_status_t *status);
esp_err_t servo_feedback(uint8_t id, servo_status_t *status);
esp_err_t servo_read_mode(uint8_t id, servo_mode_t *mode);
esp_err_t servo_read_position_limits(uint8_t id, servo_position_limits_t *limits);
esp_err_t servo_set_torque(uint8_t id, bool enabled);
// Safety broadcast for the known 1...12 robot axes. It is used only to
// release torque during startup/fault recovery, never to arm motors.
esp_err_t servo_set_torque_all(bool enabled);
esp_err_t servo_move(uint8_t id, int16_t position, uint16_t speed, uint8_t acceleration);
// Broadcast one identical-time position update to 1…12 STS/ST3215 servos.
// No response is expected from a broadcast packet.
esp_err_t servo_move_sync(const servo_position_command_t *commands, size_t count);
esp_err_t servo_set_mode(uint8_t id, servo_mode_t mode);
esp_err_t servo_motor_speed(uint8_t id, int16_t speed, uint8_t acceleration);
esp_err_t servo_calibrate_center(uint8_t id);
esp_err_t servo_assign_id(uint8_t current_id, uint8_t new_id);

// Armed periodic polling: no mutex wait, 5 ms response budget. A missing
// servo must not hold the shared TTL bus for a full 20 ms motion frame.
esp_err_t servo_feedback_realtime(uint8_t id, servo_status_t *status);
