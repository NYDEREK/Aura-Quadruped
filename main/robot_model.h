#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ROBOT_LEG_COUNT 4
#define ROBOT_AXIS_COUNT 3
#define ROBOT_SERVO_ID_UNASSIGNED 0xff

typedef enum {
    ROBOT_LEG_LEFT_FRONT = 0,
    ROBOT_LEG_RIGHT_FRONT = 1,
    ROBOT_LEG_LEFT_REAR = 2,
    ROBOT_LEG_RIGHT_REAR = 3,
} robot_leg_t;

typedef enum {
    ROBOT_AXIS_ABDUCTION = 0,
    ROBOT_AXIS_HIP = 1,
    ROBOT_AXIS_KNEE = 2,
} robot_axis_type_t;

typedef enum {
    ROBOT_AXIS_POSITION = 0,
    ROBOT_AXIS_MOTOR = 1,
} robot_axis_mode_t;

// Servo configuration is attached to exactly one mechanical axis. Angles are
// in radians in the robot model; raw ticks stay confined to this boundary.
typedef struct {
    uint8_t servo_id;
    int8_t direction;              // +1 or -1 from robot angle to servo angle
    // Kinematic zero may be virtual: a folded +/-90° knee can place it away
    // from the electronic index while every permitted target remains within
    // the ST3215 signed physical interval.  It is stored signed, but
    // serialised as the same two-byte field for NVS and the desktop protocol.
    int16_t center_tick;
    // Per-axis software barriers, expressed in the robot model's coordinate
    // system.  They are persisted by calibration and enforced for every gait
    // target before it reaches the servo bus.
    int16_t minimum_cdeg;
    int16_t maximum_cdeg;
    uint16_t maximum_speed_raw;    // ST3215 native 0…3400 speed limit
    uint8_t acceleration;
} robot_axis_config_t;

typedef struct {
    robot_axis_config_t config;
    robot_axis_mode_t mode;
    float target_radians;
    float measured_radians;
    int16_t measured_speed_raw;
    int16_t measured_load_raw;
    int16_t measured_current_raw;
    uint8_t voltage_raw;
    uint8_t temperature_c;
    bool moving;
    bool present;
    int64_t feedback_time_us; // monotonic acquisition time; never persisted
} robot_axis_state_t;

typedef struct {
    robot_axis_state_t axis[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT];
} robot_model_t;

void robot_model_init(robot_model_t *model);
bool robot_model_assign(robot_model_t *model, robot_leg_t leg, robot_axis_type_t axis,
                        const robot_axis_config_t *config);
bool robot_model_clear_axis(robot_model_t *model, robot_leg_t leg, robot_axis_type_t axis);
bool robot_model_set_target(robot_model_t *model, robot_leg_t leg, robot_axis_type_t axis,
                            float target_radians, uint16_t speed_raw, uint8_t acceleration);
bool robot_model_encode_target(const robot_axis_state_t *axis, int16_t *position,
                               uint16_t *speed, uint8_t *acceleration);
// ST3215 encodes position as 15-bit signed magnitude.  This preserves the
// continuous coordinate on either side of the electronic 0° index.
bool robot_model_encode_encoder_tick(float continuous_tick, int16_t *position);
int16_t robot_model_decode_encoder_tick(uint16_t encoded_tick);
void robot_model_update_feedback(robot_axis_state_t *axis, const uint8_t data[15]);
bool robot_model_is_complete(const robot_model_t *model);
const char *robot_leg_name(robot_leg_t leg);
const char *robot_axis_name(robot_axis_type_t axis);

// Validate the complete frame before changing any target; ignores unmapped
// axes while commissioning. Caller holds the model lock for this operation.
bool robot_model_set_frame(robot_model_t *model, const float radians[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT],
                           uint16_t speed_raw, uint8_t acceleration);
