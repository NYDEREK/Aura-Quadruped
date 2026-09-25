#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "robot_control.h"
#include "esp_timer.h"
#include "servo_bus.h"

#define ROBOT_CONTROL_PERIOD_MS 20
#define ROBOT_FEEDBACK_PERIOD_MS 5
#define ROBOT_PI 3.14159265358979323846f
#define ROBOT_TICKS_PER_RADIAN (4096.0f / (2.0f * ROBOT_PI))
#define ROBOT_VIRTUAL_CENTER_MIN_TICK -8191.0f
#define ROBOT_VIRTUAL_CENTER_MAX_TICK 8191.0f
#define CALIBRATION_SPEED_RAW 650
#define CALIBRATION_ACCELERATION 50
// A direction test moves by an explicit, bounded raw encoder delta from the
// saved reference. The current encoder sample is only used to prove that the
// user returned the physical joint to that reference.
#define CALIBRATION_PROBE_DELTA_CDEG 1200
#define CALIBRATION_PROBE_RAW_TICKS 137
#define CALIBRATION_REFERENCE_TOLERANCE_RAW_TICKS 32
#define CALIBRATION_PROBE_SETTLE_TOLERANCE_RAW_TICKS 24
#define CALIBRATION_PROBE_ABORT_TRAVEL_RAW_TICKS 190
#define CALIBRATION_PROBE_TIMEOUT_MS 800
#define CALIBRATION_PROBE_POLL_MS 15

// ST3215 current feedback uses 6.5 mA per raw unit. 308 is just over 2.0 A.
// A fault is intentionally not based on current alone: a leg may draw a brief
// current peak while accelerating normally. The axis must be highly loaded,
// materially behind its target, and making no measurable encoder progress for
// three consecutive per-axis samples (about 180 ms with the 5 ms round robin).
#define ROBOT_STALL_CURRENT_RAW 308
#define ROBOT_STALL_LOAD_RAW 800
#define ROBOT_STALL_ERROR_CDEG 800
#define ROBOT_STALL_NO_PROGRESS_TICKS 6
#define ROBOT_STALL_CONFIRMATIONS 3

static const char *TAG = "robot_control";

static robot_model_t robot;
static SemaphoreHandle_t robot_mutex;
static TaskHandle_t control_task;
static TaskHandle_t feedback_task;
static bool armed;
static bool calibration_torque_enabled[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT];
// Unlike calibration_torque_enabled, this flag only permits periodic reads
// from the selected three encoders. It has no actuator side effects.
static bool calibration_preview_enabled;

typedef struct {
    bool have_previous_position;
    int16_t previous_position_tick;
    uint8_t consecutive_stall_samples;
} robot_axis_safety_state_t;

static robot_axis_safety_state_t axis_safety[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT];
static robot_safety_fault_t safety_fault;

#define ROBOT_CONFIG_NAMESPACE "aura_robot"
#define ROBOT_CONFIG_KEY "axes_v1"

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    robot_axis_config_t axis[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT];
} robot_config_blob_t;

#define ROBOT_CONFIG_MAGIC 0x41555241u
#define ROBOT_CONFIG_VERSION 1u

static bool valid_axis(robot_leg_t leg, robot_axis_type_t axis)
{
    return leg >= 0 && leg < ROBOT_LEG_COUNT && axis >= 0 && axis < ROBOT_AXIS_COUNT;
}

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static int16_t calibration_reference_cdeg(robot_leg_t leg, robot_axis_type_t axis)
{
    // The mechanical calibration pose uses the folded knee reference.  The
    // direction probe is defined from this pose, never from a stale planner
    // target left over by a prior test or a board reboot.
    if (axis != ROBOT_AXIS_KNEE) return 0;
    return (leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_LEFT_REAR) ? 9000 : -9000;
}

static int32_t absolute_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static uint16_t radians_to_cdeg(float radians)
{
    if (!isfinite(radians)) return UINT16_MAX;
    const float cdeg = fabsf(radians) * 18000.0f / ROBOT_PI;
    return cdeg >= (float)UINT16_MAX ? UINT16_MAX : (uint16_t)lroundf(cdeg);
}

static void reset_safety_monitor_locked(void)
{
    memset(axis_safety, 0, sizeof(axis_safety));
}

// Called with robot_mutex held immediately after fresh addressed feedback was
// received. Returns true only once for a new fault; the caller then releases
// all twelve torque outputs outside the mutex.
static bool update_safety_monitor_locked(robot_leg_t leg, robot_axis_type_t axis,
                                         int16_t present_tick)
{
    if (!armed || safety_fault.latched) return false;
    robot_axis_state_t *state = &robot.axis[leg][axis];
    robot_axis_safety_state_t *monitor = &axis_safety[leg][axis];
    const uint16_t tracking_error_cdeg =
        radians_to_cdeg(state->target_radians - state->measured_radians);
    const bool no_progress = monitor->have_previous_position &&
        absolute_i32((int32_t)present_tick - monitor->previous_position_tick) <=
            ROBOT_STALL_NO_PROGRESS_TICKS;
    const bool overloaded = absolute_i32(state->measured_current_raw) >= ROBOT_STALL_CURRENT_RAW &&
                            absolute_i32(state->measured_load_raw) >= ROBOT_STALL_LOAD_RAW;
    const bool candidate = overloaded &&
                           tracking_error_cdeg >= ROBOT_STALL_ERROR_CDEG &&
                           no_progress;
    monitor->previous_position_tick = present_tick;
    monitor->have_previous_position = true;
    if (!candidate) {
        monitor->consecutive_stall_samples = 0;
        return false;
    }
    if (monitor->consecutive_stall_samples < UINT8_MAX)
        monitor->consecutive_stall_samples++;
    if (monitor->consecutive_stall_samples < ROBOT_STALL_CONFIRMATIONS) return false;

    const uint8_t confirmations = monitor->consecutive_stall_samples;
    // Kill output scheduling before the broadcast torque-off reaches the bus.
    // This also means a pending 50 Hz planner tick cannot emit another target.
    armed = false;
    reset_safety_monitor_locked();
    memset(calibration_torque_enabled, 0, sizeof(calibration_torque_enabled));
    calibration_preview_enabled = false;
    safety_fault = (robot_safety_fault_t){
        .latched = true,
        .leg = leg,
        .axis = axis,
        .servo_id = state->config.servo_id,
        .current_raw = state->measured_current_raw,
        .load_raw = state->measured_load_raw,
        .tracking_error_cdeg = tracking_error_cdeg,
        .confirmations = confirmations,
    };
    return true;
}

static float axis_tick_for_radians(const robot_axis_state_t *state, float radians)
{
    return (float)state->config.center_tick +
           (float)state->config.direction * radians * ROBOT_TICKS_PER_RADIAN;
}

static float axis_tick_for_cdeg(const robot_axis_state_t *state, int16_t cdeg)
{
    return axis_tick_for_radians(state, (float)cdeg * ROBOT_PI / 18000.0f);
}

static bool servo_position_is_permitted(const servo_position_limits_t *limits, int16_t position)
{
    if (!limits || limits->minimum >= limits->maximum ||
        limits->minimum < -4095 || limits->maximum > 4095) return false;
    return position >= limits->minimum && position <= limits->maximum;
}

static bool axis_range_is_permitted(const robot_axis_state_t *state,
                                    const servo_position_limits_t *limits)
{
    if (!state || !limits) return false;
    int16_t minimum_tick = 0;
    int16_t maximum_tick = 0;
    return robot_model_encode_encoder_tick(
               axis_tick_for_cdeg(state, state->config.minimum_cdeg), &minimum_tick) &&
           robot_model_encode_encoder_tick(
               axis_tick_for_cdeg(state, state->config.maximum_cdeg), &maximum_tick) &&
           servo_position_is_permitted(limits, minimum_tick) &&
           servo_position_is_permitted(limits, maximum_tick);
}

// A direction probe is a physical safety transaction, not a normal planner
// target. It begins only at the reference pose just commissioned for this
// axis. Deriving a probe from an arbitrary present position made a stale or
// manually moved position look like a new zero after a direction change.
static bool calibration_reference_tick(const robot_axis_state_t *state,
                                       int16_t reference_cdeg, float *unwrapped_tick,
                                       int16_t *raw_tick)
{
    if (!state || !unwrapped_tick || !raw_tick) return false;
    const float raw = axis_tick_for_cdeg(state, reference_cdeg);
    if (!isfinite(raw)) return false;
    *unwrapped_tick = raw;
    return robot_model_encode_encoder_tick(raw, raw_tick);
}

static bool calibration_target_command(const robot_axis_state_t *state, int16_t requested_cdeg,
                                       servo_position_command_t *command, float *target_radians)
{
    if (!state || !command || !target_radians ||
        state->config.servo_id == ROBOT_SERVO_ID_UNASSIGNED ||
        state->mode != ROBOT_AXIS_POSITION) return false;
    const float low = (float)state->config.minimum_cdeg * ROBOT_PI / 18000.0f;
    const float high = (float)state->config.maximum_cdeg * ROBOT_PI / 18000.0f;
    const float target = clampf((float)requested_cdeg * ROBOT_PI / 18000.0f, low, high);
    const float tick = axis_tick_for_radians(state, target);
    if (!isfinite(tick)) return false;
    *target_radians = target;
    *command = (servo_position_command_t){
        .id = state->config.servo_id,
        .position = 0,
        .speed = CALIBRATION_SPEED_RAW,
        .acceleration = CALIBRATION_ACCELERATION,
    };
    return robot_model_encode_encoder_tick(tick, &command->position);
}

static bool canonicalize_numbered_layout(robot_config_blob_t *blob)
{
    // Aura's mechanical convention is fixed and explicit:
    // LF 1-3, LR 4-6, RF 7-9, RR 10-12; within a leg: ab/ad, hip, knee.
    // Reorder full records, not values, so zero, direction and limits remain
    // attached to the physical servo that was calibrated.
    if (!blob) return false;
    robot_axis_config_t by_id[13] = {0};
    bool found[13] = {false};
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
            const robot_axis_config_t config = blob->axis[leg][axis];
            if (config.servo_id < 1 || config.servo_id > 12 || found[config.servo_id]) return false;
            found[config.servo_id] = true;
            by_id[config.servo_id] = config;
        }
    }
    for (int id = 1; id <= 12; ++id) if (!found[id]) return false;

    static const robot_leg_t physical_order[] = {
        ROBOT_LEG_LEFT_FRONT, ROBOT_LEG_LEFT_REAR,
        ROBOT_LEG_RIGHT_FRONT, ROBOT_LEG_RIGHT_REAR,
    };
    robot_axis_config_t canonical[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT] = {0};
    for (int group = 0; group < 4; ++group) {
        const robot_leg_t leg = physical_order[group];
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis)
            canonical[leg][axis] = by_id[group * ROBOT_AXIS_COUNT + axis + 1];
    }
    if (memcmp(blob->axis, canonical, sizeof(canonical)) == 0) return false;
    memcpy(blob->axis, canonical, sizeof(canonical));
    return true;
}

static esp_err_t calibration_axis_id(robot_leg_t leg, robot_axis_type_t axis, uint8_t *id)
{
    if (!robot_mutex || !id || !valid_axis(leg, axis)) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    const robot_axis_state_t *state = &robot.axis[leg][axis];
    const bool accepted = !armed && state->config.servo_id != ROBOT_SERVO_ID_UNASSIGNED;
    if (accepted) *id = state->config.servo_id;
    xSemaphoreGive(robot_mutex);
    return accepted ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static void control_tick(void)
{
    servo_position_command_t commands[SERVO_MAX_SYNC_SERVOS];
    size_t count = 0;
    if (xSemaphoreTake(robot_mutex, 0) != pdTRUE) return;
    if (armed) {
        for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
            for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
                const robot_axis_state_t *state = &robot.axis[leg][axis];
                int16_t position;
                uint16_t speed;
                uint8_t acceleration;
                if (!robot_model_encode_target(state, &position, &speed, &acceleration)) continue;
                commands[count++] = (servo_position_command_t){
                    .id = state->config.servo_id, .position = position,
                    .speed = speed, .acceleration = acceleration,
                };
            }
        }
    }
    xSemaphoreGive(robot_mutex);
    // A single broadcast packet aligns all configured axes to the same 50 Hz
    // control instant. A bus failure never causes the loop to block on reads.
    if (count) (void)servo_move_sync(commands, count);
}

static void robot_control_task(void *unused)
{
    (void)unused;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        control_tick();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ROBOT_CONTROL_PERIOD_MS));
    }
}

static bool calibration_axis_should_be_read(robot_leg_t leg, robot_axis_type_t axis)
{
    if (!robot_mutex || !valid_axis(leg, axis)) return false;
    if (xSemaphoreTake(robot_mutex, 0) != pdTRUE) return false;
    // The live calibration viewer polls the entire robot. It is still only
    // an addressed READ transaction: no torque state, goal or configuration
    // is changed here.
    const bool enabled = !armed && (calibration_torque_enabled[leg][axis] ||
                                    calibration_preview_enabled);
    xSemaphoreGive(robot_mutex);
    return enabled;
}

static void update_timed_feedback(robot_axis_state_t *axis, const uint8_t data[15])
{
    robot_model_update_feedback(axis, data);
    axis->feedback_time_us = esp_timer_get_time();
}

static void robot_feedback_task(void *unused)
{
    (void)unused;
    TickType_t wake = xTaskGetTickCount();
    unsigned cursor = 0;
    for (;;) {
        // One short addressed read every 5 ms refreshes all twelve axes at
        // about 16 Hz when the complete robot is armed. During commissioning
        // it skips released axes unless the read-only live preview is enabled;
        // then it refreshes all twelve encoder values without touching torque
        // or position registers.
        if (robot_control_is_armed()) {
            const robot_leg_t leg = (robot_leg_t)(cursor / ROBOT_AXIS_COUNT);
            const robot_axis_type_t axis = (robot_axis_type_t)(cursor % ROBOT_AXIS_COUNT);
            (void)robot_control_read_axis(leg, axis);
            cursor = (cursor + 1) % (ROBOT_LEG_COUNT * ROBOT_AXIS_COUNT);
        } else {
            for (unsigned attempt = 0; attempt < ROBOT_LEG_COUNT * ROBOT_AXIS_COUNT; ++attempt) {
                const robot_leg_t leg = (robot_leg_t)(cursor / ROBOT_AXIS_COUNT);
                const robot_axis_type_t axis = (robot_axis_type_t)(cursor % ROBOT_AXIS_COUNT);
                cursor = (cursor + 1) % (ROBOT_LEG_COUNT * ROBOT_AXIS_COUNT);
                if (!calibration_axis_should_be_read(leg, axis)) continue;
                (void)robot_control_read_axis(leg, axis);
                break;
            }
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ROBOT_FEEDBACK_PERIOD_MS));
    }
}

esp_err_t robot_control_init(void)
{
    if (robot_mutex) return ESP_OK;
    robot_model_init(&robot);
    robot_mutex = xSemaphoreCreateMutex();
    if (!robot_mutex) return ESP_ERR_NO_MEM;
    // Do this before loading NVS. A corrupt or incompatible calibration map
    // must never leave a servo holding its last target after a reset.
    (void)servo_set_torque_all(false);
    // A corrupt or absent record leaves the deliberately safe empty model.
    const esp_err_t loaded = robot_control_load_configuration();
    if (loaded != ESP_OK && loaded != ESP_ERR_NVS_NOT_FOUND) return loaded;
    // A servo retains its own torque state across an ESP reset. Clear it after
    // loading the saved map so a firmware flash, brownout or watchdog reset
    // can never leave a calibration joint pushing against a mechanical stop.
    (void)robot_control_disarm();
    return ESP_OK;
}

esp_err_t robot_control_start(void)
{
    if (!robot_mutex) return ESP_ERR_INVALID_STATE;
    if (!control_task && xTaskCreate(robot_control_task, "robot_control", 4096, NULL, 9, &control_task) != pdPASS)
        return ESP_ERR_NO_MEM;
    if (!feedback_task && xTaskCreate(robot_feedback_task, "robot_feedback", 4096, NULL, 3, &feedback_task) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}

esp_err_t robot_control_assign_axis(robot_leg_t leg, robot_axis_type_t axis,
                                    const robot_axis_config_t *config)
{
    if (!robot_mutex || !valid_axis(leg, axis) || !config) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    const bool accepted = !armed && robot_model_assign(&robot, leg, axis, config);
    xSemaphoreGive(robot_mutex);
    return accepted ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t robot_control_clear_axis(robot_leg_t leg, robot_axis_type_t axis)
{
    if (!robot_mutex || !valid_axis(leg, axis)) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    const bool accepted = !armed && robot_model_clear_axis(&robot, leg, axis);
    xSemaphoreGive(robot_mutex);
    return accepted ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t robot_control_save_configuration(void)
{
    if (!robot_mutex) return ESP_ERR_INVALID_STATE;
    robot_config_blob_t blob = {.magic = ROBOT_CONFIG_MAGIC, .version = ROBOT_CONFIG_VERSION};
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg)
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis)
            blob.axis[leg][axis] = robot.axis[leg][axis].config;
    xSemaphoreGive(robot_mutex);
    nvs_handle_t handle;
    esp_err_t result = nvs_open(ROBOT_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (result != ESP_OK) return result;
    result = nvs_set_blob(handle, ROBOT_CONFIG_KEY, &blob, sizeof(blob));
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    return result;
}

esp_err_t robot_control_load_configuration(void)
{
    if (!robot_mutex) return ESP_ERR_INVALID_STATE;
    nvs_handle_t handle;
    esp_err_t result = nvs_open(ROBOT_CONFIG_NAMESPACE, NVS_READONLY, &handle);
    if (result != ESP_OK) return result;
    robot_config_blob_t blob = {0};
    size_t size = sizeof(blob);
    result = nvs_get_blob(handle, ROBOT_CONFIG_KEY, &blob, &size);
    nvs_close(handle);
    if (result != ESP_OK) return result;
    if (size != sizeof(blob) || blob.magic != ROBOT_CONFIG_MAGIC || blob.version != ROBOT_CONFIG_VERSION)
        return ESP_ERR_INVALID_VERSION;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    robot_model_t candidate;
    bool migrated_knee_limits = canonicalize_numbered_layout(&blob);
    robot_model_init(&candidate);
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
            robot_axis_config_t config = blob.axis[leg][axis];
            if (config.servo_id == ROBOT_SERVO_ID_UNASSIGNED) continue;
            // Earlier desktop builds applied ±90° to every joint.  That is
            // valid for hip and ab/ad, but a 3-link leg's knee coordinate is
            // naturally near ±153° in the neutral stance. Migrate only this
            // untouched default; explicitly commissioned limits are retained.
            if (axis == ROBOT_AXIS_KNEE && config.minimum_cdeg == -9000 &&
                config.maximum_cdeg == 9000) {
                config.minimum_cdeg = -17000;
                config.maximum_cdeg = 17000;
                blob.axis[leg][axis] = config;
                migrated_knee_limits = true;
            }
            if (!robot_model_assign(&candidate, (robot_leg_t)leg, (robot_axis_type_t)axis,
                                    &config)) {
                xSemaphoreGive(robot_mutex);
                return ESP_ERR_INVALID_RESPONSE;
            }
        }
    }
    robot = candidate;
    xSemaphoreGive(robot_mutex);
    return migrated_knee_limits ? robot_control_save_configuration() : ESP_OK;
}

esp_err_t robot_control_set_axis_target(robot_leg_t leg, robot_axis_type_t axis,
                                        float radians, uint16_t speed_raw,
                                        uint8_t acceleration)
{
    if (!robot_mutex || !valid_axis(leg, axis)) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    const bool accepted = robot_model_set_target(&robot, leg, axis, radians, speed_raw, acceleration);
    xSemaphoreGive(robot_mutex);
    return accepted ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t robot_control_set_frame(const float radians[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT],
                                  uint16_t speed_raw, uint8_t acceleration)
{
    if (!robot_mutex || !radians) return ESP_ERR_INVALID_ARG;
    // Never stall the gait clock behind calibration or a bus operation.
    if (xSemaphoreTake(robot_mutex,0)!=pdTRUE) return ESP_ERR_TIMEOUT;
    const bool ok=robot_model_set_frame(&robot,radians,speed_raw,acceleration);
    xSemaphoreGive(robot_mutex);
    return ok ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t robot_control_arm(void)
{
    if (!robot_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (armed) { xSemaphoreGive(robot_mutex); return ESP_OK; }
    if (safety_fault.latched || !robot_model_is_complete(&robot)) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    reset_safety_monitor_locked();
    // Take fresh position samples before torque is enabled. The gait task
    // therefore seeds its bounded setpoint trajectory from the position that
    // is actually on the table, not a sample retained from a previous run.
    // Holding the model lock makes arming transactional: no 50 Hz gait tick
    // can overwrite this safe initial target mid-sequence.
    esp_err_t result = ESP_OK;
    for (int leg = 0; leg < ROBOT_LEG_COUNT && result == ESP_OK; ++leg) {
        for (int axis = 0; axis < ROBOT_AXIS_COUNT && result == ESP_OK; ++axis) {
            robot_axis_state_t *state = &robot.axis[leg][axis];
            servo_mode_t mode = SERVO_MODE_MOTOR;
            result = servo_read_mode(state->config.servo_id, &mode);
            if (result == ESP_OK && mode != SERVO_MODE_POSITION) result = ESP_ERR_INVALID_STATE;
            servo_position_limits_t limits = {0};
            if (result == ESP_OK)
                result = servo_read_position_limits(state->config.servo_id, &limits);
            if (result == ESP_OK && !axis_range_is_permitted(state, &limits))
                result = ESP_ERR_NOT_SUPPORTED;
            if (result != ESP_OK) break;
            servo_status_t feedback;
            result = servo_feedback(state->config.servo_id, &feedback);
            if (result == ESP_OK) {
                // Before torque comes on, replace any stale goal retained by
                // the servo with the pose that is physically present now.
                // Otherwise torque-on can briefly chase an old goal before
                // the first 50 Hz planner packet arrives.
                const uint16_t encoded_present = (uint16_t)feedback.data[0] |
                                                 ((uint16_t)feedback.data[1] << 8);
                const int16_t present = robot_model_decode_encoder_tick(encoded_present);
                result = servo_move(state->config.servo_id, present,
                                    CALIBRATION_SPEED_RAW, CALIBRATION_ACCELERATION);
                if (result == ESP_OK) {
                    update_timed_feedback(state, feedback.data);
                    state->target_radians = state->measured_radians;
                }
            }
        }
    }
    for (int leg = 0; leg < ROBOT_LEG_COUNT && result == ESP_OK; ++leg)
        for (int axis = 0; axis < ROBOT_AXIS_COUNT && result == ESP_OK; ++axis)
            result = servo_set_torque(robot.axis[leg][axis].config.servo_id, true);
    if (result != ESP_OK) {
        for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg)
            for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis)
                (void)servo_set_torque(robot.axis[leg][axis].config.servo_id, false);
    }
    armed = result == ESP_OK;
    xSemaphoreGive(robot_mutex);
    return result;
}

esp_err_t robot_control_disarm(void)
{
    if (!robot_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    // Clear the state before I/O so a failed transmission cannot leave the
    // 50 Hz task producing another motion packet.
    armed = false;
    reset_safety_monitor_locked();
    memset(calibration_torque_enabled, 0, sizeof(calibration_torque_enabled));
    calibration_preview_enabled = false;
    esp_err_t result = ESP_OK;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg)
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
            const uint8_t id = robot.axis[leg][axis].config.servo_id;
            if (id != ROBOT_SERVO_ID_UNASSIGNED) {
                const esp_err_t stop = servo_set_torque(id, false);
                if (result == ESP_OK) result = stop;
            }
        }
    xSemaphoreGive(robot_mutex);
    return result;
}

bool robot_control_is_armed(void)
{
    if (!robot_mutex || xSemaphoreTake(robot_mutex, 0) != pdTRUE) return false;
    const bool result = armed;
    xSemaphoreGive(robot_mutex);
    return result;
}

bool robot_control_has_safety_fault(void)
{
    if (!robot_mutex || xSemaphoreTake(robot_mutex, 0) != pdTRUE) return true;
    const bool latched = safety_fault.latched;
    xSemaphoreGive(robot_mutex);
    return latched;
}

void robot_control_get_safety_fault(robot_safety_fault_t *fault)
{
    if (!fault) return;
    memset(fault, 0, sizeof(*fault));
    if (!robot_mutex || xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
    *fault = safety_fault;
    xSemaphoreGive(robot_mutex);
}

esp_err_t robot_control_clear_safety_fault(void)
{
    if (!robot_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (armed) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    memset(&safety_fault, 0, sizeof(safety_fault));
    reset_safety_monitor_locked();
    xSemaphoreGive(robot_mutex);
    return ESP_OK;
}

bool robot_control_calibration_is_active(void)
{
    if (!robot_mutex || xSemaphoreTake(robot_mutex, 0) != pdTRUE) return true;
    bool active = calibration_preview_enabled;
    for (int leg = 0; leg < ROBOT_LEG_COUNT && !active; ++leg)
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis)
            active = calibration_torque_enabled[leg][axis];
    xSemaphoreGive(robot_mutex);
    return active;
}

esp_err_t robot_control_read_axis(robot_leg_t leg, robot_axis_type_t axis)
{
    if (!robot_mutex || !valid_axis(leg, axis)) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    robot_axis_state_t *state = &robot.axis[leg][axis];
    const uint8_t id = state->config.servo_id;
    const bool realtime = armed;
    xSemaphoreGive(robot_mutex);
    if (id == ROBOT_SERVO_ID_UNASSIGNED) return ESP_ERR_NOT_FOUND;
    servo_status_t feedback;
    const esp_err_t result = realtime ? servo_feedback_realtime(id, &feedback) : servo_feedback(id, &feedback);
    if (result != ESP_OK) return result;
    const uint16_t encoded_position = (uint16_t)feedback.data[0] |
                                      ((uint16_t)feedback.data[1] << 8);
    const int16_t present_tick = robot_model_decode_encoder_tick(encoded_position);
    bool release_all_torque = false;
    robot_safety_fault_t fault = {0};
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    update_timed_feedback(&robot.axis[leg][axis], feedback.data);
    release_all_torque = update_safety_monitor_locked(leg, axis, present_tick);
    if (release_all_torque) fault = safety_fault;
    xSemaphoreGive(robot_mutex);
    if (release_all_torque) {
        // Broadcast release is independent of the calibration map and is the
        // fastest safe action after detecting a physical obstruction.
        const esp_err_t stopped = servo_set_torque_all(false);
        ESP_LOGE(TAG, "STALL L%d A%d ID%u: current=%d (%.3f A), load=%d, error=%u cdeg; torque off: %s",
                 fault.leg, fault.axis, fault.servo_id, fault.current_raw,
                 (float)absolute_i32(fault.current_raw) * 0.0065f,
                 fault.load_raw, fault.tracking_error_cdeg, esp_err_to_name(stopped));
        return stopped == ESP_OK ? ESP_ERR_INVALID_STATE : stopped;
    }
    return ESP_OK;
}

esp_err_t robot_control_calibration_capture_zero(robot_leg_t leg, robot_axis_type_t axis,
                                                 int16_t reference_cdeg)
{
    if (reference_cdeg < -18000 || reference_cdeg > 18000) return ESP_ERR_INVALID_ARG;
    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(calibration_axis_id(leg, axis, &id), "robot", "calibration axis");
    // The user positions the leg by hand. Never apply holding torque while
    // sampling that pose, even if a previous direction test enabled it.
    ESP_RETURN_ON_ERROR(servo_set_torque(id, false), "robot", "release calibration axis");
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        calibration_torque_enabled[leg][axis] = false;
        xSemaphoreGive(robot_mutex);
    }
    servo_status_t feedback = {0};
    ESP_RETURN_ON_ERROR(servo_feedback(id, &feedback), "robot", "read calibration zero");
    const uint16_t encoded_position = (uint16_t)feedback.data[0] | ((uint16_t)feedback.data[1] << 8);
    const int16_t raw_position = robot_model_decode_encoder_tick(encoded_position);

    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (armed || robot.axis[leg][axis].config.servo_id != id) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    robot_axis_state_t *state = &robot.axis[leg][axis];
    // A calibration pose is not necessarily the kinematic 0°. In particular,
    // the mechanical knee is deliberately folded to +/-90° because its real
    // linkage cannot reach the straight 0° pose. Store the inverse offset so
    // the feedback and all later IK targets are expressed in model degrees.
    const float reference_radians = (float)reference_cdeg * ROBOT_PI / 18000.0f;
    int8_t direction = state->config.direction >= 0 ? 1 : -1;
    float center = (float)raw_position - (float)direction *
                   reference_radians * ROBOT_TICKS_PER_RADIAN;

    // A kinematic zero is allowed to be virtual: a folded +/-90° knee may sit
    // away from the signed electronic index.  Only a centre that cannot be
    // represented at all justifies an automatic sign change; the following
    // direction test still verifies it.
    if ((center < ROBOT_VIRTUAL_CENTER_MIN_TICK || center > ROBOT_VIRTUAL_CENTER_MAX_TICK) &&
        axis == ROBOT_AXIS_KNEE && reference_cdeg != 0) {
        const int8_t alternate_direction = -direction;
        const float alternate_center = (float)raw_position -
                                       (float)alternate_direction * reference_radians *
                                       ROBOT_TICKS_PER_RADIAN;
        if (alternate_center >= ROBOT_VIRTUAL_CENTER_MIN_TICK &&
            alternate_center <= ROBOT_VIRTUAL_CENTER_MAX_TICK) {
            direction = alternate_direction;
            center = alternate_center;
        }
    }
    if (center < ROBOT_VIRTUAL_CENTER_MIN_TICK || center > ROBOT_VIRTUAL_CENTER_MAX_TICK) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_ARG;
    }

    // The first zero capture must work where the joint's correct reference
    // straddles electronic zero. Its limits are model coordinates and must
    // fit within the STS signed position range. A negative target is encoded
    // with bit 15, never transformed into 4095-x.
    float requested_minimum = (float)state->config.minimum_cdeg;
    float requested_maximum = (float)state->config.maximum_cdeg;
    if (reference_cdeg < state->config.minimum_cdeg ||
        reference_cdeg > state->config.maximum_cdeg) {
        requested_minimum = axis == ROBOT_AXIS_KNEE ? -17000.0f : -9000.0f;
        requested_maximum = axis == ROBOT_AXIS_KNEE ? 17000.0f : 9000.0f;
    }
    const float low_radians = requested_minimum * ROBOT_PI / 18000.0f;
    const float high_radians = requested_maximum * ROBOT_PI / 18000.0f;
    const float low_tick = center + (float)direction * low_radians * ROBOT_TICKS_PER_RADIAN;
    const float high_tick = center + (float)direction * high_radians * ROBOT_TICKS_PER_RADIAN;
    if (requested_minimum >= requested_maximum ||
        fminf(low_tick, high_tick) < -4095.0f || fmaxf(low_tick, high_tick) > 4095.0f ||
        reference_cdeg < requested_minimum || reference_cdeg > requested_maximum) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    state->config.direction = direction;
    state->config.center_tick = (int16_t)lroundf(center);
    state->config.minimum_cdeg = (int16_t)requested_minimum;
    state->config.maximum_cdeg = (int16_t)requested_maximum;
    state->target_radians = reference_radians;
    update_timed_feedback(state, feedback.data);
    xSemaphoreGive(robot_mutex);
    return robot_control_save_configuration();
}

esp_err_t robot_control_calibration_release_axis(robot_leg_t leg, robot_axis_type_t axis)
{
    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(calibration_axis_id(leg, axis, &id), "robot", "calibration axis");
    const esp_err_t result = servo_set_torque(id, false);
    if (result == ESP_OK && xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        calibration_torque_enabled[leg][axis] = false;
        xSemaphoreGive(robot_mutex);
    }
    return result;
}

static esp_err_t calibration_rebase_direction(robot_leg_t leg, robot_axis_type_t axis,
                                              int16_t reference_cdeg, bool reverse_sign)
{
    if (reference_cdeg < -18000 || reference_cdeg > 18000) return ESP_ERR_INVALID_ARG;
    uint8_t id = 0;
    // Releasing torque is intentional: changing a sign must never cause a
    // position command or keep pushing an already misplaced joint.
    ESP_RETURN_ON_ERROR(robot_control_calibration_release_axis(leg, axis), "robot", "release direction axis");
    ESP_RETURN_ON_ERROR(calibration_axis_id(leg, axis, &id), "robot", "direction axis");

    servo_status_t feedback = {0};
    ESP_RETURN_ON_ERROR(servo_feedback(id, &feedback), "robot", "read direction reference");
    const uint16_t encoded_position = (uint16_t)feedback.data[0] | ((uint16_t)feedback.data[1] << 8);
    const int16_t raw_position = robot_model_decode_encoder_tick(encoded_position);
    const float reference_radians = (float)reference_cdeg * ROBOT_PI / 18000.0f;

    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (armed || robot.axis[leg][axis].config.servo_id != id) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    robot_axis_state_t *state = &robot.axis[leg][axis];
    const int8_t new_direction = reverse_sign
        ? (state->config.direction >= 0 ? -1 : 1)
        : (state->config.direction >= 0 ? 1 : -1);
    // The raw encoder value is measured with the joint manually returned to
    // `reference_cdeg`. Recalculate centre for the new sign from that value;
    // reusing the old centre is the half-turn error that can drive a ±90° knee
    // into a mechanical stop.
    const float new_center = (float)raw_position - (float)new_direction *
                             reference_radians * ROBOT_TICKS_PER_RADIAN;
    if (new_center < ROBOT_VIRTUAL_CENTER_MIN_TICK ||
        new_center > ROBOT_VIRTUAL_CENTER_MAX_TICK) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_ARG;
    }

    // The physical end stops stay fixed. Express their model coordinates in
    // the reflected system about the reference pose before persisting them.
    // Reflect the physical interval around the known reference.  The model
    // protocol is intentionally bounded to one turn (-180...+180°), so clip
    // only the unrepresentable tail rather than rejecting a safe +12° test.
    const float reflected_min = fmaxf(-18000.0f,
                                      2.0f * (float)reference_cdeg - (float)state->config.maximum_cdeg);
    const float reflected_max = fminf(18000.0f,
                                      2.0f * (float)reference_cdeg - (float)state->config.minimum_cdeg);
    const float reflected_low_tick = new_center + (float)new_direction *
        reflected_min * ROBOT_PI / 18000.0f * ROBOT_TICKS_PER_RADIAN;
    const float reflected_high_tick = new_center + (float)new_direction *
        reflected_max * ROBOT_PI / 18000.0f * ROBOT_TICKS_PER_RADIAN;
    if (reflected_min >= reflected_max ||
        fminf(reflected_low_tick, reflected_high_tick) < -4095.0f ||
        fmaxf(reflected_low_tick, reflected_high_tick) > 4095.0f ||
        reference_cdeg < reflected_min || reference_cdeg > reflected_max) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    state->config.direction = new_direction;
    state->config.center_tick = (int16_t)lroundf(new_center);
    state->config.minimum_cdeg = (int16_t)lroundf(reflected_min);
    state->config.maximum_cdeg = (int16_t)lroundf(reflected_max);
    state->target_radians = reference_radians;
    calibration_torque_enabled[leg][axis] = false;
    update_timed_feedback(state, feedback.data);
    xSemaphoreGive(robot_mutex);
    return robot_control_save_configuration();
}

esp_err_t robot_control_calibration_flip_direction(robot_leg_t leg, robot_axis_type_t axis,
                                                   int16_t reference_cdeg)
{
    return calibration_rebase_direction(leg, axis, reference_cdeg, true);
}

esp_err_t robot_control_calibration_rebase_direction(robot_leg_t leg, robot_axis_type_t axis,
                                                     int16_t reference_cdeg)
{
    return calibration_rebase_direction(leg, axis, reference_cdeg, false);
}

// Kept as a protocol compatibility guard for older desktop builds.  A previous
// implementation tried to return a mounted joint to its reference, invert the
// map, and send a second probe as one transaction.  That makes the board move
// during the direction-change action, so it is intentionally disabled.  The
// only permitted direction change is calibration_flip_direction(): it releases
// torque and records a new map at the pose manually set by the operator.
esp_err_t robot_control_calibration_reverse_probe(robot_leg_t leg, robot_axis_type_t axis,
                                                  int16_t reference_cdeg, int16_t probe_delta_cdeg)
{
    (void)leg;
    (void)axis;
    (void)reference_cdeg;
    (void)probe_delta_cdeg;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t robot_control_calibration_capture_limit(robot_leg_t leg, robot_axis_type_t axis,
                                                  bool maximum)
{
    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(robot_control_calibration_release_axis(leg, axis), "robot", "release limit axis");
    ESP_RETURN_ON_ERROR(calibration_axis_id(leg, axis, &id), "robot", "calibration axis");
    servo_status_t feedback = {0};
    ESP_RETURN_ON_ERROR(servo_feedback(id, &feedback), "robot", "read calibration limit");
    const uint16_t encoded_position = (uint16_t)feedback.data[0] | ((uint16_t)feedback.data[1] << 8);
    const int16_t raw_position = robot_model_decode_encoder_tick(encoded_position);

    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (armed || robot.axis[leg][axis].config.servo_id != id) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    robot_axis_state_t *state = &robot.axis[leg][axis];
    const float degrees = (float)state->config.direction *
                          ((float)raw_position - (float)state->config.center_tick) * 360.0f / 4096.0f;
    const int16_t captured_cdeg = (int16_t)lroundf(clampf(degrees * 100.0f, -18000.0f, 18000.0f));
    const bool valid = maximum
        ? captured_cdeg > state->config.minimum_cdeg + 100
        : captured_cdeg < state->config.maximum_cdeg - 100;
    if (!valid) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    const int16_t next_minimum = maximum ? state->config.minimum_cdeg : captured_cdeg;
    const int16_t next_maximum = maximum ? captured_cdeg : state->config.maximum_cdeg;
    const float low_tick = axis_tick_for_cdeg(state, next_minimum);
    const float high_tick = axis_tick_for_cdeg(state, next_maximum);
    if (fminf(low_tick, high_tick) < -4095.0f || fmaxf(low_tick, high_tick) > 4095.0f) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    state->config.minimum_cdeg = next_minimum;
    state->config.maximum_cdeg = next_maximum;
    state->target_radians = clampf(state->target_radians,
                                   (float)state->config.minimum_cdeg * ROBOT_PI / 18000.0f,
                                   (float)state->config.maximum_cdeg * ROBOT_PI / 18000.0f);
    update_timed_feedback(state, feedback.data);
    xSemaphoreGive(robot_mutex);
    return robot_control_save_configuration();
}

esp_err_t robot_control_calibration_nudge_axis(robot_leg_t leg, robot_axis_type_t axis,
                                               int16_t target_cdeg)
{
    if (!robot_mutex || !valid_axis(leg, axis) || target_cdeg < -18000 || target_cdeg > 18000)
        return ESP_ERR_INVALID_ARG;

    const int16_t reference_cdeg = calibration_reference_cdeg(leg, axis);
    const int32_t requested_delta_cdeg = (int32_t)target_cdeg - (int32_t)reference_cdeg;
    // This endpoint has one purpose only: a single, observable +12° direction
    // probe from the manually set reference pose.
    if (requested_delta_cdeg != CALIBRATION_PROBE_DELTA_CDEG)
        return ESP_ERR_NOT_SUPPORTED;

    uint8_t id = ROBOT_SERVO_ID_UNASSIGNED;
    int8_t direction = 1;
    float reference_unwrapped_tick = 0.0f;
    int16_t reference_tick = 0;
    int16_t next_tick = 0;
    float next_unwrapped_tick = 0.0f;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    const robot_axis_state_t *state = &robot.axis[leg][axis];
    bool accepted = !armed &&
        state->config.servo_id != ROBOT_SERVO_ID_UNASSIGNED &&
        state->mode == ROBOT_AXIS_POSITION &&
        !calibration_torque_enabled[leg][axis];
    if (accepted) {
        id = state->config.servo_id;
        direction = state->config.direction >= 0 ? 1 : -1;
        accepted = target_cdeg >= state->config.minimum_cdeg &&
                   target_cdeg <= state->config.maximum_cdeg &&
                   calibration_reference_tick(state, reference_cdeg,
                                              &reference_unwrapped_tick, &reference_tick);
        next_unwrapped_tick = reference_unwrapped_tick +
                               (float)direction * CALIBRATION_PROBE_RAW_TICKS;
        accepted = accepted && robot_model_encode_encoder_tick(next_unwrapped_tick, &next_tick);
    }
    xSemaphoreGive(robot_mutex);
    if (!accepted) return ESP_ERR_INVALID_STATE;

    // Calibration never changes servo modes. A position command in motor mode
    // could produce continuous rotation, so fail before touching torque or a
    // goal register unless both independent hardware checks describe a usable
    // position servo.
    servo_mode_t mode = SERVO_MODE_MOTOR;
    ESP_RETURN_ON_ERROR(servo_read_mode(id, &mode), "robot", "read calibration mode");
    if (mode != SERVO_MODE_POSITION) return ESP_ERR_INVALID_STATE;
    servo_position_limits_t limits = {0};
    ESP_RETURN_ON_ERROR(servo_read_position_limits(id, &limits), "robot", "read calibration limits");
    if (!servo_position_is_permitted(&limits, reference_tick) ||
        !servo_position_is_permitted(&limits, next_tick)) return ESP_ERR_NOT_SUPPORTED;

    servo_status_t feedback = {0};
    ESP_RETURN_ON_ERROR(servo_feedback(id, &feedback), "robot", "read calibration position");
    const uint16_t encoded_present = (uint16_t)feedback.data[0] |
                                     ((uint16_t)feedback.data[1] << 8);
    const int16_t present = robot_model_decode_encoder_tick(encoded_present);
    // Do not reinterpret the present encoder value as a new zero. The test is
    // accepted only when the user has really returned the joint to its stored
    // reference pose. Its target comes from that reference, never from a
    // potentially stale present sample.
    if (absolute_i32((int32_t)present - (int32_t)reference_tick) >
        CALIBRATION_REFERENCE_TOLERANCE_RAW_TICKS) {
        ESP_LOGW(TAG, "probe rejected L%d A%d ID%u: present=%d reference=%d delta=%ld",
                 leg, axis, id, present, reference_tick,
                 (long)((int32_t)present - (int32_t)reference_tick));
        return ESP_ERR_INVALID_STATE;
    }
    // A signed STS position keeps a small movement across electronic zero
    // small on the actual motor as well.  Never convert a negative target to
    // 4095-x, which would request almost a complete physical turn.
    ESP_LOGW(TAG, "probe L%d A%d ID%u: ref=%d/%0.1f present=%d target=%d/%0.1f dir=%d limits=%d..%d",
             leg, axis, id, reference_tick, reference_unwrapped_tick, present,
             next_tick, next_unwrapped_tick, direction,
             limits.minimum, limits.maximum);

    // A servo remembers Goal Position while torque is off. Write its *current*
    // position as a hold goal first, then enable torque. This prevents a
    // torque-on transition from chasing a stale goal from an earlier test.
    ESP_RETURN_ON_ERROR(servo_move(id, present, CALIBRATION_SPEED_RAW,
                                   CALIBRATION_ACCELERATION), "robot", "seed calibration goal");
    esp_err_t result = servo_set_torque(id, true);
    if (result != ESP_OK) return result;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        (void)servo_set_torque(id, false);
        return ESP_ERR_TIMEOUT;
    }
    calibration_torque_enabled[leg][axis] = true;
    xSemaphoreGive(robot_mutex);

    // Only after the safe hold is committed and torque is on do we command the
    // 137-tick / 12° probe. Poll the physical encoder and always release the
    // joint afterwards. A wrong-way or oversized motion is cut off long before
    // a full turn can develop.
    result = servo_move(id, next_tick, CALIBRATION_SPEED_RAW,
                        CALIBRATION_ACCELERATION);
    if (result != ESP_OK) {
        (void)robot_control_calibration_release_axis(leg, axis);
        return result;
    }
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(CALIBRATION_PROBE_TIMEOUT_MS);
    bool reached = false;
    while ((int32_t)(xTaskGetTickCount() - deadline) < 0) {
        vTaskDelay(pdMS_TO_TICKS(CALIBRATION_PROBE_POLL_MS));
        servo_status_t observed = {0};
        result = servo_feedback(id, &observed);
        if (result != ESP_OK) break;
        const uint16_t encoded_current = (uint16_t)observed.data[0] |
                                         ((uint16_t)observed.data[1] << 8);
        const int16_t current = robot_model_decode_encoder_tick(encoded_current);
        const float current_unwrapped_tick = (float)current;
        const float travelled = (float)direction *
                                (current_unwrapped_tick - reference_unwrapped_tick);
        if (travelled < -CALIBRATION_PROBE_SETTLE_TOLERANCE_RAW_TICKS ||
            fabsf(current_unwrapped_tick - reference_unwrapped_tick) >
                CALIBRATION_PROBE_ABORT_TRAVEL_RAW_TICKS) {
            ESP_LOGE(TAG, "probe abort L%d A%d ID%u: current=%d/%0.1f ref=%d/%0.1f target=%d/%0.1f",
                     leg, axis, id, current, current_unwrapped_tick,
                     reference_tick, reference_unwrapped_tick, next_tick, next_unwrapped_tick);
            result = ESP_ERR_INVALID_RESPONSE;
            break;
        }
        if (fabsf(current_unwrapped_tick - next_unwrapped_tick) <=
            CALIBRATION_PROBE_SETTLE_TOLERANCE_RAW_TICKS) {
            reached = true;
            break;
        }
    }
    (void)robot_control_calibration_release_axis(leg, axis);
    if (result != ESP_OK) return result;
    if (!reached) return ESP_ERR_TIMEOUT;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        robot.axis[leg][axis].target_radians =
            (float)target_cdeg * ROBOT_PI / 18000.0f;
        xSemaphoreGive(robot_mutex);
    }
    return ESP_OK;
}

esp_err_t robot_control_calibration_drive_leg_profile(robot_leg_t leg,
                                                      const int16_t target_cdeg[ROBOT_AXIS_COUNT],
                                                      uint16_t speed_raw, uint8_t acceleration)
{
    if (!robot_mutex || !target_cdeg || leg < 0 || leg >= ROBOT_LEG_COUNT) return ESP_ERR_INVALID_ARG;
    servo_position_command_t commands[ROBOT_AXIS_COUNT] = {0};
    float next_target_radians[ROBOT_AXIS_COUNT] = {0};
    bool enable_torque[ROBOT_AXIS_COUNT] = {false};
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    bool accepted = !armed;
    for (int axis = 0; axis < ROBOT_AXIS_COUNT && accepted; ++axis) {
        accepted = calibration_target_command(&robot.axis[leg][axis], target_cdeg[axis],
                                              &commands[axis], &next_target_radians[axis]);
        // Calibration retains its axis limits and mapping; only the output
        // profile changes so a one-leg Trot is sent like normal gait motion.
        commands[axis].speed = speed_raw;
        commands[axis].acceleration = acceleration;
        enable_torque[axis] = accepted && !calibration_torque_enabled[leg][axis];
    }
    xSemaphoreGive(robot_mutex);
    if (!accepted) return ESP_ERR_INVALID_STATE;
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        servo_mode_t mode = SERVO_MODE_MOTOR;
        ESP_RETURN_ON_ERROR(servo_read_mode(commands[axis].id, &mode), "robot", "read leg calibration mode");
        if (mode != SERVO_MODE_POSITION) return ESP_ERR_INVALID_STATE;
    }
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        if (!enable_torque[axis]) continue;
        // Seed a hold target while torque is still off. This is the same
        // torque-on safety transaction used by the single-axis direction test.
        servo_status_t feedback = {0};
        ESP_RETURN_ON_ERROR(servo_feedback(commands[axis].id, &feedback), "robot", "read leg hold pose");
        const uint16_t encoded_present = (uint16_t)feedback.data[0] |
                                         ((uint16_t)feedback.data[1] << 8);
        const int16_t present = robot_model_decode_encoder_tick(encoded_present);
        ESP_RETURN_ON_ERROR(servo_move(commands[axis].id, present, speed_raw, acceleration),
                            "robot", "seed leg profile goal");
    }
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        if (!enable_torque[axis]) continue;
        const esp_err_t enable_result = servo_set_torque(commands[axis].id, true);
        if (enable_result != ESP_OK) {
            for (int rollback = 0; rollback < ROBOT_AXIS_COUNT; ++rollback)
                if (enable_torque[rollback]) (void)servo_set_torque(commands[rollback].id, false);
            return enable_result;
        }
        if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            calibration_torque_enabled[leg][axis] = true;
            xSemaphoreGive(robot_mutex);
        }
    }
    const esp_err_t result = servo_move_sync(commands, ROBOT_AXIS_COUNT);
    if (result != ESP_OK) return result;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis)
            robot.axis[leg][axis].target_radians = next_target_radians[axis];
        xSemaphoreGive(robot_mutex);
    }
    return ESP_OK;
}

esp_err_t robot_control_calibration_drive_leg(robot_leg_t leg,
                                              const int16_t target_cdeg[ROBOT_AXIS_COUNT])
{
    return robot_control_calibration_drive_leg_profile(leg, target_cdeg,
                                                       CALIBRATION_SPEED_RAW,
                                                       CALIBRATION_ACCELERATION);
}

esp_err_t robot_control_calibration_set_leg_torque(robot_leg_t leg, bool enabled)
{
    if (leg < 0 || leg >= ROBOT_LEG_COUNT) return ESP_ERR_INVALID_ARG;
    if (!enabled) return robot_control_calibration_release_leg(leg);
    if (!robot_mutex) return ESP_ERR_INVALID_STATE;

    uint8_t ids[ROBOT_AXIS_COUNT] = {0};
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    const bool accepted = !armed;
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        const robot_axis_state_t *state = &robot.axis[leg][axis];
        if (state->config.servo_id == ROBOT_SERVO_ID_UNASSIGNED) {
            xSemaphoreGive(robot_mutex);
            return ESP_ERR_NOT_FOUND;
        }
        ids[axis] = state->config.servo_id;
    }
    xSemaphoreGive(robot_mutex);
    if (!accepted) return ESP_ERR_INVALID_STATE;

    // Capture the present pose and write it as the goal *before* torque is
    // applied. The selected leg holds exactly where the operator left it.
    servo_status_t feedback[ROBOT_AXIS_COUNT] = {0};
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        servo_mode_t mode = SERVO_MODE_MOTOR;
        ESP_RETURN_ON_ERROR(servo_read_mode(ids[axis], &mode), "robot", "read torque calibration mode");
        if (mode != SERVO_MODE_POSITION) return ESP_ERR_INVALID_STATE;
        ESP_RETURN_ON_ERROR(servo_feedback(ids[axis], &feedback[axis]), "robot", "read leg pose");
        const uint16_t encoded_position = (uint16_t)feedback[axis].data[0] |
                                          ((uint16_t)feedback[axis].data[1] << 8);
        const int16_t position = robot_model_decode_encoder_tick(encoded_position);
        ESP_RETURN_ON_ERROR(servo_move(ids[axis], position, CALIBRATION_SPEED_RAW,
                                       CALIBRATION_ACCELERATION), "robot", "seed leg goal");
    }
    esp_err_t result = ESP_OK;
    int enabled_count = 0;
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        result = servo_set_torque(ids[axis], true);
        if (result != ESP_OK) break;
        enabled_count++;
    }
    if (result != ESP_OK) {
        for (int axis = 0; axis < enabled_count; ++axis) (void)servo_set_torque(ids[axis], false);
        return result;
    }
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        (void)robot_control_calibration_release_leg(leg);
        return ESP_ERR_TIMEOUT;
    }
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        robot_axis_state_t *state = &robot.axis[leg][axis];
        update_timed_feedback(state, feedback[axis].data);
        state->target_radians = state->measured_radians;
        calibration_torque_enabled[leg][axis] = true;
    }
    xSemaphoreGive(robot_mutex);
    return ESP_OK;
}

esp_err_t robot_control_calibration_set_leg_preview(robot_leg_t leg, bool enabled)
{
    if (leg < 0 || leg >= ROBOT_LEG_COUNT || !robot_mutex) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (armed) {
        xSemaphoreGive(robot_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    // `leg` remains in the wire command for protocol compatibility and input
    // validation. Preview deliberately reads every configured leg so the 3D
    // calibration view can expose a bad zero or direction immediately.
    calibration_preview_enabled = enabled;
    xSemaphoreGive(robot_mutex);
    return ESP_OK;
}

esp_err_t robot_control_calibration_release_leg(robot_leg_t leg)
{
    if (leg < 0 || leg >= ROBOT_LEG_COUNT) return ESP_ERR_INVALID_ARG;
    esp_err_t result = ESP_OK;
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        const esp_err_t release = robot_control_calibration_release_axis(leg, (robot_axis_type_t)axis);
        if (result == ESP_OK) result = release;
    }
    return result;
}

void robot_control_snapshot(robot_model_t *model)
{
    if (!model || !robot_mutex || xSemaphoreTake(robot_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    memcpy(model, &robot, sizeof(*model));
    xSemaphoreGive(robot_mutex);
}
