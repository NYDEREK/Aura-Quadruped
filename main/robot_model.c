#include <math.h>
#include <string.h>

#include "robot_model.h"

#define ROBOT_PI 3.14159265358979323846f
#define SERVO_TICKS_PER_RADIAN (4096.0f / (2.0f * ROBOT_PI))
#define SERVO_ENCODER_MAX_TICK 4095.0f

static bool valid_axis(robot_leg_t leg, robot_axis_type_t axis)
{
    return leg >= 0 && leg < ROBOT_LEG_COUNT && axis >= 0 && axis < ROBOT_AXIS_COUNT;
}

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static bool target_range_fits_servo(const robot_axis_config_t *config)
{
    if (!config) return false;
    // ST3215 uses sign-magnitude positions.  The two endpoints must remain
    // in that signed physical interval; wrapping a command across its index
    // would make the actuator travel almost a full rotation.
    const float low = (float)config->minimum_cdeg * ROBOT_PI / 18000.0f;
    const float high = (float)config->maximum_cdeg * ROBOT_PI / 18000.0f;
    const float low_tick = (float)config->center_tick +
                           (float)config->direction * low * SERVO_TICKS_PER_RADIAN;
    const float high_tick = (float)config->center_tick +
                            (float)config->direction * high * SERVO_TICKS_PER_RADIAN;
    return isfinite(low_tick) && isfinite(high_tick) &&
           fminf(low_tick, high_tick) >= -SERVO_ENCODER_MAX_TICK &&
           fmaxf(low_tick, high_tick) <= SERVO_ENCODER_MAX_TICK;
}

bool robot_model_encode_encoder_tick(float continuous_tick, int16_t *position)
{
    if (!position || !isfinite(continuous_tick)) return false;
    const long rounded = lroundf(continuous_tick);
    if (rounded < -(long)SERVO_ENCODER_MAX_TICK || rounded > (long)SERVO_ENCODER_MAX_TICK)
        return false;
    *position = (int16_t)rounded;
    return true;
}

int16_t robot_model_decode_encoder_tick(uint16_t encoded_tick)
{
    const int16_t magnitude = (int16_t)(encoded_tick & 0x7fff);
    return (encoded_tick & 0x8000) ? (int16_t)-magnitude : magnitude;
}

void robot_model_init(robot_model_t *model)
{
    if (!model) return;
    memset(model, 0, sizeof(*model));
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
            robot_axis_state_t *state = &model->axis[leg][axis];
            state->config.servo_id = ROBOT_SERVO_ID_UNASSIGNED;
            state->config.direction = 1;
            state->config.center_tick = 2048;
            // These are conservative software defaults. The final mechanical
            // range is configured separately for every installed joint.
            state->config.minimum_cdeg = -9000;
            state->config.maximum_cdeg = 9000;
            state->config.maximum_speed_raw = 3400;
            state->config.acceleration = 254;
            state->mode = ROBOT_AXIS_POSITION;
        }
    }
}

bool robot_model_assign(robot_model_t *model, robot_leg_t leg, robot_axis_type_t axis,
                        const robot_axis_config_t *config)
{
    if (!model || !config || !valid_axis(leg, axis) || config->servo_id > 253 ||
        (config->direction != -1 && config->direction != 1) ||
        config->minimum_cdeg >= config->maximum_cdeg || config->maximum_speed_raw > 3400 ||
        !target_range_fits_servo(config))
        return false;
    for (int other_leg = 0; other_leg < ROBOT_LEG_COUNT; ++other_leg) {
        for (int other_axis = 0; other_axis < ROBOT_AXIS_COUNT; ++other_axis) {
            if (other_leg == (int)leg && other_axis == (int)axis) continue;
            if (model->axis[other_leg][other_axis].config.servo_id == config->servo_id)
                return false; // a physical servo may not drive two axes
        }
    }
    robot_axis_state_t *state = &model->axis[leg][axis];
    state->config = *config;
    state->target_radians = 0;
    state->present = false;
    return true;
}

bool robot_model_clear_axis(robot_model_t *model, robot_leg_t leg, robot_axis_type_t axis)
{
    if (!model || !valid_axis(leg, axis)) return false;
    robot_axis_state_t *state = &model->axis[leg][axis];
    const robot_axis_config_t defaults = {
        .servo_id = ROBOT_SERVO_ID_UNASSIGNED, .direction = 1, .center_tick = 2048,
        .minimum_cdeg = -9000, .maximum_cdeg = 9000,
        .maximum_speed_raw = 3400, .acceleration = 254,
    };
    state->config = defaults;
    state->target_radians = 0;
    state->measured_radians = 0;
    state->present = false;
    return true;
}

bool robot_model_set_target(robot_model_t *model, robot_leg_t leg, robot_axis_type_t axis,
                            float target_radians, uint16_t speed_raw, uint8_t acceleration)
{
    if (!model || !valid_axis(leg, axis) || !isfinite(target_radians) || speed_raw > 3400)
        return false;
    robot_axis_state_t *state = &model->axis[leg][axis];
    if (state->config.servo_id == ROBOT_SERVO_ID_UNASSIGNED || state->mode != ROBOT_AXIS_POSITION)
        return false;
    const float minimum = (float)state->config.minimum_cdeg * ROBOT_PI / 18000.0f;
    const float maximum = (float)state->config.maximum_cdeg * ROBOT_PI / 18000.0f;
    state->target_radians = clampf(target_radians, minimum, maximum);
    state->config.maximum_speed_raw = speed_raw;
    state->config.acceleration = acceleration;
    return true;
}

bool robot_model_encode_target(const robot_axis_state_t *axis, int16_t *position,
                               uint16_t *speed, uint8_t *acceleration)
{
    if (!axis || !position || !speed || !acceleration ||
        axis->config.servo_id == ROBOT_SERVO_ID_UNASSIGNED ||
        axis->mode != ROBOT_AXIS_POSITION) return false;
    const float minimum = (float)axis->config.minimum_cdeg * ROBOT_PI / 18000.0f;
    const float maximum = (float)axis->config.maximum_cdeg * ROBOT_PI / 18000.0f;
    // This is the final safety barrier before the hardware packet.  The gait
    // planner already goes through robot_model_set_target(), but arming seeds
    // a fresh target from feedback, so clamp here as well.
    const float safe_target = clampf(axis->target_radians, minimum, maximum);
    const float tick = (float)axis->config.center_tick +
        (float)axis->config.direction * safe_target * SERVO_TICKS_PER_RADIAN;
    if (!robot_model_encode_encoder_tick(tick, position)) return false;
    *speed = axis->config.maximum_speed_raw;
    *acceleration = axis->config.acceleration;
    return true;
}

void robot_model_update_feedback(robot_axis_state_t *axis, const uint8_t data[15])
{
    if (!axis || !data) return;
    const uint16_t encoded_position = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    const int16_t raw_position = robot_model_decode_encoder_tick(encoded_position);
    const uint16_t raw_speed = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    const uint16_t raw_load = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    const uint16_t raw_current = (uint16_t)data[13] | ((uint16_t)data[14] << 8);
    axis->measured_radians = (float)axis->config.direction *
        ((float)raw_position - (float)axis->config.center_tick) / SERVO_TICKS_PER_RADIAN;
    axis->measured_speed_raw = (int16_t)((raw_speed & 0x7fff) * (raw_speed & 0x8000 ? -1 : 1));
    axis->measured_load_raw = (int16_t)((raw_load & 0x03ff) * (raw_load & 0x0400 ? -1 : 1));
    axis->voltage_raw = data[6];
    axis->temperature_c = data[7];
    axis->moving = data[10] != 0;
    axis->measured_current_raw = (int16_t)((raw_current & 0x7fff) * (raw_current & 0x8000 ? -1 : 1));
    axis->present = true;
}

bool robot_model_is_complete(const robot_model_t *model)
{
    if (!model) return false;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg)
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis)
            if (model->axis[leg][axis].config.servo_id == ROBOT_SERVO_ID_UNASSIGNED)
                return false;
    return true;
}

const char *robot_leg_name(robot_leg_t leg)
{
    static const char *const names[] = {"LF", "RF", "LR", "RR"};
    return leg >= 0 && leg < ROBOT_LEG_COUNT ? names[leg] : "?";
}

const char *robot_axis_name(robot_axis_type_t axis)
{
    static const char *const names[] = {"abduction", "hip", "knee"};
    return axis >= 0 && axis < ROBOT_AXIS_COUNT ? names[axis] : "?";
}

bool robot_model_set_frame(robot_model_t *model, const float radians[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT],
                           uint16_t speed_raw, uint8_t acceleration)
{
    if (!model || !radians || speed_raw>3400) return false;
    for (int leg=0;leg<ROBOT_LEG_COUNT;++leg) for (int axis=0;axis<ROBOT_AXIS_COUNT;++axis) {
        const robot_axis_state_t *a=&model->axis[leg][axis];
        if (!isfinite(radians[leg][axis]) || (a->config.servo_id!=ROBOT_SERVO_ID_UNASSIGNED &&
            a->mode!=ROBOT_AXIS_POSITION)) return false;
    }
    for (int leg=0;leg<ROBOT_LEG_COUNT;++leg) for (int axis=0;axis<ROBOT_AXIS_COUNT;++axis)
        if (model->axis[leg][axis].config.servo_id!=ROBOT_SERVO_ID_UNASSIGNED)
            (void)robot_model_set_target(model,leg,axis,radians[leg][axis],speed_raw,acceleration);
    return true;
}
