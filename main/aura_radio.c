#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "app_state.h"
#include "aura_radio.h"
#include "aura_features.h"
#include "aura_network.h"
#include "dualsense.h"
#include "mpu6050.h"
#include "servo_bus.h"
#include "robot_control.h"
#include "robot_gait.h"
#include "tof.h"
#include "ws2812.h"

#define AURA_NAME "Aura Main Board"
#define AURA_FRAME_SIZE 20
#define AURA_PIXEL_COUNT 64

enum {
    AURA_CMD_SCAN = 0x01,
    AURA_CMD_FEEDBACK = 0x02,
    AURA_CMD_TORQUE = 0x03,
    AURA_CMD_MOVE = 0x04,
    AURA_CMD_ZERO = 0x05,
    AURA_CMD_ASSIGN_ID = 0x06,
    AURA_CMD_LED = 0x07,
    AURA_CMD_TELEMETRY = 0x08,
    AURA_CMD_SERVO_MODE = 0x09,
    AURA_CMD_MOTOR_SPEED = 0x0a,
    AURA_CMD_TOF_CONFIGURE = 0x0b,
    AURA_CMD_TOF_CONFIGS = 0x0c,
    AURA_CMD_TOF_REINITIALIZE = 0x0d,
    AURA_CMD_TOF_SET_OFFSET = 0x0e,
    AURA_CMD_IMU_REINITIALIZE = 0x0f,
    AURA_CMD_TOF_SET_TUNING = 0x10,
    AURA_CMD_TOF_TEMPERATURE_UPDATE = 0x11,
    AURA_CMD_DUALSENSE_PAIR = 0x12,
    AURA_CMD_DUALSENSE_DISCONNECT = 0x13,
    AURA_CMD_DUALSENSE_CONNECT_SAVED = 0x14,
    AURA_CMD_DUALSENSE_FORGET = 0x15,
    AURA_CMD_DUALSENSE_CONNECT_DISCOVERED = 0x16,
    AURA_CMD_ROBOT_AXIS_CONFIG = 0x17,
    AURA_CMD_ROBOT_ARM = 0x18,
    AURA_CMD_CALIBRATION_CAPTURE_ZERO = 0x19,
    AURA_CMD_CALIBRATION_NUDGE = 0x1a,
    AURA_CMD_CALIBRATION_RELEASE_AXIS = 0x1b,
    AURA_CMD_CALIBRATION_CAPTURE_LIMIT = 0x1c,
    AURA_CMD_CALIBRATION_LEG_TEST = 0x1d,
    AURA_CMD_CALIBRATION_LEG_TORQUE = 0x1e,
    // Read-only export of all saved axis records. It never changes NVS,
    // torque, targets, or an active calibration test.
    AURA_CMD_ROBOT_CONFIGS = 0x1f,
    AURA_CMD_CALIBRATION_FLIP_DIRECTION = 0x20,
    AURA_CMD_CALIBRATION_REBASE_DIRECTION = 0x21,
    AURA_CMD_CALIBRATION_REVERSE_PROBE = 0x22,
    AURA_CMD_CALIBRATION_LEG_PREVIEW = 0x23,
    // Clears only an already-disarmed emergency-stop latch. It never arms a
    // servo or sends a goal position.
    AURA_CMD_ROBOT_CLEAR_SAFETY_FAULT = 0x24,
    // Persisted neutral standing height. The gait module accepts this only
    // while the robot is disarmed and does not issue a motor command.
    AURA_CMD_ROBOT_BODY_HEIGHT = 0x25,
    AURA_CMD_ROBOT_SINGLE_FOOT_TUNING = 0x26,
    // Persisted centre-of-mass offset and interior support margin for the
    // three-foot crawl. It is accepted only while the robot is disarmed.
    AURA_CMD_ROBOT_STATIC_BALANCE = 0x27,
    // Saved final-foot lateral offset per side. Like height and CoM, this
    // setup command is rejected while torque or calibration is active.
    AURA_CMD_ROBOT_LATERAL_STANCE = 0x28,
    // Enables only the bounded MPU6050 attitude controller for a later arm.
    // It is rejected while torque is on and cannot cause a move by itself.
    AURA_CMD_ROBOT_ATTITUDE_BALANCE = 0x29,
    // Persists the IMU correction ceiling in centidegrees while disarmed.
    AURA_CMD_ROBOT_ATTITUDE_LIMIT = 0x2a,
    // Full profile: gait, stride, lift, frequency and duty factor. It is
    // planning data only and is rejected while torque is active.
    AURA_CMD_ROBOT_GAIT_PROFILE = 0x2b,
    // Moving-balance controller gains and CoM trajectory limits.  This is
    // persisted commissioning data and is rejected while torque is active.
    AURA_CMD_ROBOT_MOTION_TUNING = 0x2c,
    AURA_CMD_ROBOT_TRIPOD_WALK = 0x2d,
    AURA_EVT_POWER = 0x10,
    AURA_EVT_IMU = 0x11,
    AURA_EVT_TOF = 0x12,
    AURA_EVT_TOF_DIAGNOSTICS = 0x13,
    AURA_EVT_DUALSENSE = 0x14,
    AURA_EVT_RESULT = 0x20,
    AURA_EVT_SERVO_FOUND = 0x21,
    AURA_EVT_SERVO_FEEDBACK = 0x22,
    AURA_EVT_TOF_CONFIG = 0x23,
    AURA_EVT_TOF_TUNING = 0x24,
    AURA_EVT_DUALSENSE_STATUS = 0x25,
    AURA_EVT_DUALSENSE_DEVICE = 0x26,
    AURA_EVT_ROBOT_STATUS = 0x27,
    AURA_EVT_ROBOT_LEG = 0x28,
    AURA_EVT_SERVO_CAPACITY = 0x29,
    AURA_EVT_ROBOT_POSE = 0x2a,
    AURA_EVT_ROBOT_AXIS_CONFIG = 0x2b,
    AURA_EVT_ROBOT_SAFETY_FAULT = 0x2c,
    AURA_EVT_ROBOT_ATTITUDE = 0x2d,
    // Read-only contact-observer result.  It is intentionally a separate
    // telemetry product so commissioning it cannot alter the gait protocol.
    AURA_EVT_ROBOT_CONTACT = 0x2e,
    // One persisted gait profile. The board also sends all four in response
    // to the existing read-only configuration request.
    AURA_EVT_ROBOT_GAIT_PROFILE = 0x2f,
    // One persisted set of moving-controller gains and trajectory limits.
    AURA_EVT_ROBOT_MOTION_TUNING = 0x30,
    AURA_EVT_ROBOT_BODY_REFERENCE = 0x31,
};

typedef struct {
    uint8_t size;
    uint8_t data[AURA_FRAME_SIZE];
} aura_command_t;

static const char *TAG = "aura_radio";
static QueueHandle_t command_queue;
static TaskHandle_t command_task_handle;
static TaskHandle_t controller_task_handle;
static TaskHandle_t telemetry_task_handle;
static volatile bool shutdown_requested;
static volatile bool radio_running;
static bool controller_running;
static bool bluedroid_running;
static bool create_was_pressed;
static bool controller_was_available;
static const uint16_t telemetry_handle = 1;
static const uint16_t event_handle = 2;
static uint8_t last_power[AURA_FRAME_SIZE];
static void queue_command(const uint8_t *data, uint16_t size);
static void send_result(uint8_t operation, esp_err_t result, uint8_t detail);

// Create is the deliberate physical arming switch. A press edge, rather than
// a held level, prevents reconnects or a queued HID report from toggling the
// robot twice. Losing the controller disarms the robot on the next 10 ms
// command-loop turn.
static void process_create_arm_switch(void)
{
    dualsense_snapshot_t controller = {0};
    dualsense_get_snapshot(&controller);
    const bool available = controller.state == DUALSENSE_STATE_CONNECTED && controller.has_input;
    const bool create_pressed = available && (controller.buttons[1] & 0x10u) != 0;

    if (!available) {
        if (controller_was_available && robot_control_is_armed()) {
            const esp_err_t result = robot_control_disarm();
            ESP_LOGW(TAG, "DualSense lost; robot disarmed: %s", esp_err_to_name(result));
        }
        create_was_pressed = false;
        controller_was_available = false;
        return;
    }

    if (create_pressed && !create_was_pressed) {
        robot_gait_snapshot_t gait = {0};
        robot_gait_snapshot(&gait);
        if (gait.calibration_test || robot_control_calibration_is_active()) {
            ESP_LOGW(TAG, "Create ignored while calibration owns an axis or TTL preview is active");
            // The physical Create button has no desktop request to wait on, so
            // explicitly publish the refusal.  This keeps the operator from
            // mistaking an active calibration session for a failed controller.
            send_result(AURA_CMD_ROBOT_ARM, ESP_ERR_INVALID_STATE, 0xff);
        } else {
            const bool was_armed = robot_control_is_armed();
            const esp_err_t result = was_armed ? robot_control_disarm() : robot_gait_arm();
            ESP_LOGW(TAG, "Create: robot %s: %s", was_armed ? "disarm" : "arm",
                     esp_err_to_name(result));
            send_result(AURA_CMD_ROBOT_ARM, result,
                        was_armed ? 0 : robot_gait_last_arm_preflight_slot());
        }
    }
    create_was_pressed = create_pressed;
    controller_was_available = true;
}

static void put_u16(uint8_t *target, uint16_t value)
{
    target[0] = (uint8_t)value;
    target[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *target, uint32_t value)
{
    target[0] = (uint8_t)value;
    target[1] = (uint8_t)(value >> 8);
    target[2] = (uint8_t)(value >> 16);
    target[3] = (uint8_t)(value >> 24);
}

static uint16_t get_u16(const uint8_t *source)
{
    return (uint16_t)source[0] | ((uint16_t)source[1] << 8);
}

static int16_t scaled_i16(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled > INT16_MAX) return INT16_MAX;
    if (scaled < INT16_MIN) return INT16_MIN;
    return (int16_t)scaled;
}

static void put_i16(uint8_t *target, int16_t value)
{
    put_u16(target, (uint16_t)value);
}

static uint16_t scaled_u16(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled <= 0) return 0;
    if (scaled > UINT16_MAX) return UINT16_MAX;
    return (uint16_t)scaled;
}

static int32_t scaled_i32(float value, float scale)
{
    const double scaled = (double)value * scale;
    if (scaled > INT32_MAX) return INT32_MAX;
    if (scaled < INT32_MIN) return INT32_MIN;
    return (int32_t)scaled;
}

static void notify(uint16_t channel, const uint8_t *data, size_t size, bool subscribed)
{
    if (subscribed) aura_network_send((uint8_t)channel, data, size);
}

static void make_power_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    app_state_snapshot_t state;
    app_state_get(&state);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_POWER;
    frame[1] = 1;
    put_u32(frame + 2, (uint32_t)(esp_timer_get_time() / 1000));
    put_u16(frame + 6, scaled_u16(state.power.bus_v, 1000));
    put_u32(frame + 8, (uint32_t)scaled_i32(state.power.servo_current_a, 1000));
    put_u32(frame + 12, (uint32_t)scaled_i32(state.power.servo_input_power_w, 1000));
    put_u16(frame + 16, scaled_u16(state.power.battery_v, 1000));
    if (state.power.ina_error == ESP_OK) frame[18] |= 1;
    if (state.power.battery_error == ESP_OK) frame[18] |= 2;
    if (state.imu_error == ESP_OK && state.imu.sample_count) frame[18] |= 4;
    frame[19] = state.imu_identity;
}

static void make_imu_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    app_state_snapshot_t state;
    app_state_get(&state);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_IMU;
    frame[1] = 1;
    put_u32(frame + 2, state.imu.sample_count);
    for (int axis = 0; axis < 3; ++axis) {
        put_u16(frame + 6 + axis * 2, (uint16_t)scaled_i16(state.imu.accel_g[axis], 1000));
        put_u16(frame + 12 + axis * 2, (uint16_t)scaled_i16(state.imu.gyro_dps[axis], 10));
    }
    put_u16(frame + 18, (uint16_t)scaled_i16(state.imu.temperature_c, 100));
}

static void make_robot_attitude_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    robot_gait_snapshot_t gait = {0};
    app_state_snapshot_t app = {0};
    robot_gait_snapshot(&gait);
    app_state_get(&app);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_ROBOT_ATTITUDE;
    // valid / enabled / active / arm-reference-captured / waiting-for-neutral
    if (gait.attitude_valid) frame[1] |= 1;
    if (gait.attitude_control_enabled) frame[1] |= 2;
    if (gait.attitude_control_active) frame[1] |= 4;
    if (gait.attitude_reference_valid) frame[1] |= 8;
    if (gait.attitude_reference_pending) frame[1] |= 16;
    put_u32(frame + 2, app.imu.sample_count);
    put_i16(frame + 6, gait.attitude_roll_cdeg);
    put_i16(frame + 8, gait.attitude_pitch_cdeg);
    put_i16(frame + 10, gait.attitude_reference_roll_cdeg);
    put_i16(frame + 12, gait.attitude_reference_pitch_cdeg);
    put_i16(frame + 14, gait.attitude_correction_roll_cdeg);
    put_i16(frame + 16, gait.attitude_correction_pitch_cdeg);
    frame[18] = app.imu_identity;
    // 0.1 degree units; the commissioning range is 3°…20°.
    frame[19] = (uint8_t)(gait.attitude_max_correction_cdeg / 10U);
}

static void make_robot_gait_profile_frame(uint8_t frame[AURA_FRAME_SIZE],
                                          robot_gait_mode_t gait)
{
    robot_locomotion_profile_t profile = {0};
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_ROBOT_GAIT_PROFILE;
    frame[1] = 1; // schema
    frame[2] = (uint8_t)gait;
    if (!robot_gait_get_locomotion_profile(gait, &profile)) return;
    put_u16(frame + 3, profile.stride_mm);
    put_u16(frame + 5, profile.step_height_mm);
    put_u16(frame + 7, profile.frequency_centi_hz);
    frame[9] = profile.duty_percent;
}

static void make_robot_motion_tuning_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    robot_gait_motion_tuning_t tuning = {0};
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_ROBOT_MOTION_TUNING;
    frame[1] = 1; // schema
    if (!robot_gait_get_motion_tuning(&tuning)) return;
    put_u16(frame + 2, tuning.moving_attitude_gain_per_mille);
    put_u16(frame + 4, tuning.com_max_velocity_mm_s);
    put_u16(frame + 6, tuning.com_max_acceleration_mm_s2);
}

static void make_robot_contact_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    robot_gait_snapshot_t gait = {0};
    robot_gait_snapshot(&gait);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_ROBOT_CONTACT;
    // schema 2: the observer is available; Aura can gate touchdown itself.
    if (gait.contact_feedback_available) frame[1] |= 1;
    if (gait.contact_control_active) frame[1] |= 2;
    if (gait.contact_touchdown_waiting) frame[1] |= 4;
    // Logical leg bit order is LF, RF, LR, RR and matches robot telemetry.
    frame[2] = gait.expected_contact_mask;
    frame[3] = gait.detected_contact_mask;
    frame[4] = gait.early_touchdown_mask;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg)
        frame[5 + leg] = gait.contact_confidence_percent[leg];
    frame[9] = 2;
}

static void make_tof_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    tof_snapshot_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    tof_get_snapshot(&snapshot);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_TOF;
    frame[1] = 1;
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        const tof_sensor_state_t *sensor = &snapshot.sensor[i];
        if (sensor->present) frame[2] |= 1U << i;
        if (sensor->ranging) frame[2] |= 1U << (i + 2);
        if (sensor->valid) frame[2] |= 1U << (i + 4);
        const int offset = 4 + i * 8;
        frame[offset] = sensor->address_7bit;
        frame[offset + 1] = sensor->range_status;
        put_u16(frame + offset + 2, sensor->distance_mm);
        put_u16(frame + offset + 4, sensor->signal_per_spad_kcps);
        put_u16(frame + offset + 6, sensor->sigma_mm);
    }
}

static void make_tof_diagnostics_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    tof_snapshot_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    tof_get_snapshot(&snapshot);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_TOF_DIAGNOSTICS;
    frame[1] = 1;
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        const tof_sensor_state_t *sensor = &snapshot.sensor[i];
        if (sensor->present) frame[2] |= 1U << i;
        if (sensor->valid) frame[2] |= 1U << (i + 2);
        const int offset = 4 + i * 8;
        put_u16(frame + offset, sensor->ambient_per_spad_kcps);
        put_u16(frame + offset + 2, sensor->number_of_spad);
        put_u32(frame + offset + 4, sensor->sample_count);
    }
}

static void make_dualsense_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    dualsense_snapshot_t controller;
    memset(&controller, 0, sizeof(controller));
    dualsense_get_snapshot(&controller);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_DUALSENSE;
    frame[1] = 1;
    if (controller.state == DUALSENSE_STATE_CONNECTED) frame[2] |= 1;
    if (controller.state == DUALSENSE_STATE_SCANNING) frame[2] |= 2;
    if (controller.has_input) frame[2] |= 4;
    if (controller.has_battery) frame[2] |= 8;
    frame[3] = controller.report_id;
    frame[4] = controller.left_x;
    frame[5] = controller.left_y;
    frame[6] = controller.right_x;
    frame[7] = controller.right_y;
    frame[8] = controller.left_trigger;
    frame[9] = controller.right_trigger;
    frame[10] = controller.buttons[0];
    frame[11] = controller.buttons[1];
    frame[12] = controller.buttons[2];
    frame[13] = controller.battery_percent;
    frame[14] = controller.report_length;
    put_u32(frame + 16, controller.sample_count);
}

static void make_robot_status_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    robot_gait_snapshot_t gait = {0};
    robot_gait_snapshot(&gait);
    robot_model_t model = {0};
    robot_control_snapshot(&model);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_ROBOT_STATUS;
    // 0 = regular 12-axis mode; 1...4 = a calibration test is driving
    // exactly that logical leg. Kept outside the bit flags for compatibility.
    frame[1] = gait.calibration_test ? (uint8_t)gait.calibration_leg + 1 : 0;
    if (robot_control_is_armed()) frame[2] |= 1;
    if (robot_model_is_complete(&model)) frame[2] |= 2;
    if (gait.controller_connected) frame[2] |= 4;
    if (gait.controller_has_input) frame[2] |= 8;
    if (gait.spin_mode) frame[2] |= 16;
    if (gait.jumping) frame[2] |= 32;
    if (gait.tracking_limited) frame[2] |= 64;
    if (gait.virtual_input) frame[2] |= 128;
    frame[3] = (uint8_t)gait.selected_gait;
    frame[4] = (uint8_t)gait.active_gait;
    put_u16(frame + 5, gait.phase_milli);
    put_i16(frame + 7, gait.input_forward_milli);
    put_i16(frame + 9, gait.input_lateral_milli);
    put_i16(frame + 11, gait.input_turn_milli);
    put_i16(frame + 13, gait.body_height_mm);
    put_u32(frame + 15, gait.tick_count);
    frame[19] = gait.phase_rate_percent;
}

static void make_robot_safety_fault_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    robot_safety_fault_t fault = {0};
    robot_control_get_safety_fault(&fault);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_ROBOT_SAFETY_FAULT;
    frame[1] = fault.latched ? 1 : 0;
    // 0xff keeps an inactive report unambiguous even for logical leg zero.
    frame[2] = fault.latched ? (uint8_t)fault.leg : 0xff;
    frame[3] = fault.latched ? (uint8_t)fault.axis : 0xff;
    frame[4] = fault.servo_id;
    put_i16(frame + 5, fault.current_raw);
    put_i16(frame + 7, fault.load_raw);
    put_u16(frame + 9, fault.tracking_error_cdeg);
    frame[11] = fault.confirmations;
}

static void make_robot_pose_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    robot_gait_snapshot_t gait = {0};
    robot_gait_snapshot(&gait);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_ROBOT_POSE;
    frame[1] = gait.odometry_valid ? 1 : 0;
    // Millimetres in tenth-millimetre units leave ample range for a long
    // session while preserving visually smooth camera tracking.
    put_u32(frame + 2, (uint32_t)scaled_i32(gait.odometry_x_mm, 10.0f));
    put_u32(frame + 6, (uint32_t)scaled_i32(gait.odometry_z_mm, 10.0f));
    put_i16(frame + 10, scaled_i16(gait.odometry_yaw_radians, 18000.0f / 3.14159265f));
    put_u32(frame + 12, gait.tick_count);
}

static void make_robot_body_reference_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    robot_gait_snapshot_t gait = {0};
    robot_gait_snapshot(&gait);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_ROBOT_BODY_REFERENCE;
    frame[1] = 1 | (gait.body_preview_active ? 2 : 0);
    put_i16(frame + 2, scaled_i16(gait.body_shift_x_mm, 10));
    put_i16(frame + 4, scaled_i16(gait.body_shift_z_mm, 10));
    put_i16(frame + 6, gait.attitude_correction_roll_cdeg);
    put_i16(frame + 8, gait.attitude_correction_pitch_cdeg);
    put_u32(frame + 10, gait.tick_count);
    frame[14] = gait.tripod_walk_enabled ? 1 : 0;
    frame[15] = (uint8_t)gait.tripod_walk_excluded;
    frame[16] = gait.swing_balance_active ? 1 : 0;
    frame[17] = gait.constrained_leg_mask;
    frame[18] = 2; // v2: runtime phase frequency in 0.02 Hz units
    frame[19] = (uint8_t)((gait.effective_frequency_centi_hz + 1) / 2);
}

static void make_servo_capacity_frame(uint8_t frame[AURA_FRAME_SIZE])
{
    app_state_snapshot_t state;
    app_state_get(&state);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_SERVO_CAPACITY;
    frame[1] = 1;
    // microampere-hours: preserves three decimal places in the desktop mAh UI.
    put_u32(frame + 2, (uint32_t)fminf(state.power.servo_consumed_mAh * 1000.0f, UINT32_MAX));
}

static void make_robot_leg_frame(uint8_t frame[AURA_FRAME_SIZE], robot_leg_t leg)
{
    robot_model_t model = {0};
    robot_control_snapshot(&model);
    memset(frame, 0, AURA_FRAME_SIZE);
    frame[0] = AURA_EVT_ROBOT_LEG;
    frame[1] = (uint8_t)leg;
    frame[2] = robot_control_is_armed() ? 1 : 0;
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        const robot_axis_state_t *state = &model.axis[leg][axis];
        if (state->config.servo_id != ROBOT_SERVO_ID_UNASSIGNED) frame[2] |= 1U << (axis + 1);
        if (state->present) frame[3] |= 1U << axis;
        if (state->moving) frame[3] |= 1U << (axis + 3);
        put_i16(frame + 4 + axis * 2, scaled_i16(state->target_radians, 18000.0f / 3.14159265f));
        put_i16(frame + 10 + axis * 2, scaled_i16(state->measured_radians, 18000.0f / 3.14159265f));
    }
    frame[16] = model.axis[leg][0].temperature_c;
    frame[17] = model.axis[leg][1].temperature_c;
    frame[18] = model.axis[leg][2].temperature_c;
    frame[19] = (uint8_t)leg;
}

static void send_dualsense_status(void)
{
    dualsense_snapshot_t controller;
    dualsense_get_snapshot(&controller);
    uint8_t frame[AURA_FRAME_SIZE] = {0};
    frame[0] = AURA_EVT_DUALSENSE_STATUS;
    frame[1] = controller.state;
    frame[2] = controller.has_saved_controller;
    frame[3] = controller.has_battery ? controller.battery_percent : 0xff;
    memcpy(frame + 4, controller.address, sizeof(controller.address));
    put_u16(frame + 10, controller.vendor_id);
    put_u16(frame + 12, controller.product_id);
    frame[14] = controller.report_id;
    frame[15] = controller.has_input;
    put_u32(frame + 16, controller.sample_count);
    notify(event_handle, frame, sizeof(frame), aura_network_connected());
}

static void send_dualsense_devices(void)
{
    dualsense_scan_device_t devices[DUALSENSE_MAX_SCAN_DEVICES];
    const size_t count = dualsense_get_scanned_devices(devices, DUALSENSE_MAX_SCAN_DEVICES);
    for (size_t i = 0; i < count; ++i) {
        uint8_t frame[AURA_FRAME_SIZE] = {0};
        frame[0] = AURA_EVT_DUALSENSE_DEVICE;
        frame[1] = (uint8_t)i;
        frame[2] = (uint8_t)devices[i].rssi;
        const size_t name_length = strnlen(devices[i].name, DUALSENSE_DEVICE_NAME_SIZE);
        frame[3] = (uint8_t)(name_length > 10 ? 10 : name_length);
        memcpy(frame + 4, devices[i].address, sizeof(devices[i].address));
        memcpy(frame + 10, devices[i].name, frame[3]);
        notify(event_handle, frame, sizeof(frame), aura_network_connected());
    }
}

static void send_result(uint8_t operation, esp_err_t result, uint8_t detail)
{
    // Command processing and 50 Hz telemetry run in separate FreeRTOS tasks.
    // A per-call frame prevents another event from corrupting this result.
    uint8_t frame[AURA_FRAME_SIZE] = {0};
    frame[0] = AURA_EVT_RESULT;
    frame[1] = operation;
    put_u32(frame + 2, (uint32_t)result);
    frame[6] = detail;
    notify(event_handle, frame, sizeof(frame), aura_network_connected());
}

static void send_servo_found(uint8_t id)
{
    uint8_t frame[AURA_FRAME_SIZE] = {0};
    frame[0] = AURA_EVT_SERVO_FOUND;
    frame[1] = id;
    notify(event_handle, frame, sizeof(frame), aura_network_connected());
}

static void send_servo_feedback(uint8_t id, const servo_status_t *status, uint8_t mode)
{
    uint8_t frame[AURA_FRAME_SIZE] = {0};
    frame[0] = AURA_EVT_SERVO_FEEDBACK;
    frame[1] = id;
    frame[2] = status->error;
    frame[3] = status->data[10] != 0;
    put_i16(frame + 4, (int16_t)servo_signed_magnitude(get_u16(status->data), 15));
    put_u16(frame + 6, (uint16_t)servo_signed_magnitude(get_u16(status->data + 2), 15));
    put_u16(frame + 8, (uint16_t)servo_signed_magnitude(get_u16(status->data + 4), 10));
    frame[10] = status->data[6];
    frame[11] = status->data[7];
    put_u16(frame + 12, (uint16_t)servo_signed_magnitude(get_u16(status->data + 13), 15));
    frame[14] = mode;
    notify(event_handle, frame, sizeof(frame), aura_network_connected());
}

static void send_tof_config(uint8_t index)
{
    tof_config_t config;
    tof_snapshot_t snapshot;
    memset(&config, 0, sizeof(config));
    memset(&snapshot, 0, sizeof(snapshot));
    tof_get_config(index, &config);
    tof_get_snapshot(&snapshot);
    const tof_sensor_state_t *state = &snapshot.sensor[index];

    uint8_t frame[AURA_FRAME_SIZE] = {0};
    frame[0] = AURA_EVT_TOF_CONFIG;
    frame[1] = index;
    frame[2] = config.address_7bit;
    frame[3] = config.enabled;
    put_u16(frame + 4, config.timing_budget_ms);
    put_u16(frame + 6, config.intermeasurement_ms);
    frame[8] = state->present;
    frame[9] = state->ranging;
    put_u32(frame + 10, (uint32_t)state->error);
    put_u16(frame + 14, (uint16_t)config.offset_mm);
    notify(event_handle, frame, sizeof(frame), aura_network_connected());
}

static void send_tof_tuning(uint8_t index)
{
    tof_config_t config;
    memset(&config, 0, sizeof(config));
    tof_get_config(index, &config);

    uint8_t frame[AURA_FRAME_SIZE] = {0};
    frame[0] = AURA_EVT_TOF_TUNING;
    frame[1] = index;
    put_u16(frame + 2, config.xtalk_kcps);
    put_u16(frame + 4, config.signal_threshold_kcps);
    put_u16(frame + 6, config.sigma_threshold_mm);
    put_u16(frame + 8, config.detection_low_mm);
    put_u16(frame + 10, config.detection_high_mm);
    frame[12] = config.detection_window;
    frame[13] = config.detection_enabled;
    notify(event_handle, frame, sizeof(frame), aura_network_connected());
}

static void send_robot_axis_config(robot_leg_t leg, robot_axis_type_t axis)
{
    if (leg < 0 || leg >= ROBOT_LEG_COUNT || axis < 0 || axis >= ROBOT_AXIS_COUNT) return;
    robot_model_t model = {0};
    robot_control_snapshot(&model);
    const robot_axis_config_t *config = &model.axis[leg][axis].config;
    uint8_t frame[AURA_FRAME_SIZE] = {0};
    frame[0] = AURA_EVT_ROBOT_AXIS_CONFIG;
    frame[1] = (uint8_t)leg;
    frame[2] = (uint8_t)axis;
    frame[3] = config->servo_id;
    frame[4] = (uint8_t)config->direction;
    put_i16(frame + 5, config->center_tick);
    put_i16(frame + 7, config->minimum_cdeg);
    put_i16(frame + 9, config->maximum_cdeg);
    put_u16(frame + 11, config->maximum_speed_raw);
    frame[13] = config->acceleration;
    notify(event_handle, frame, sizeof(frame), aura_network_connected());
}

static void send_all_tof_configs(void)
{
    for (uint8_t index = 0; index < TOF_SENSOR_COUNT; ++index) {
        send_tof_config(index);
        send_tof_tuning(index);
    }
}

static esp_err_t set_led(const aura_command_t *command)
{
    if (command->size != 6 || command->data[2] == 0 || command->data[2] > AURA_PIXEL_COUNT ||
        command->data[1] > 2) return ESP_ERR_INVALID_ARG;
    uint8_t pixels[AURA_PIXEL_COUNT * 3] = {0};
    const uint8_t mode = command->data[1];
    const uint8_t count = command->data[2];
    if (mode == 1) {
        for (uint8_t pixel = 0; pixel < count; ++pixel) {
            pixels[pixel * 3] = command->data[3];
            pixels[pixel * 3 + 1] = command->data[4];
            pixels[pixel * 3 + 2] = command->data[5];
        }
    } else if (mode == 2) {
        for (uint8_t pixel = 0; pixel < count; ++pixel) {
            uint8_t phase = 255 - (uint8_t)(pixel * 256 / count);
            if (phase < 85) {
                pixels[pixel * 3] = 255 - phase * 3;
                pixels[pixel * 3 + 2] = phase * 3;
            } else if (phase < 170) {
                phase -= 85;
                pixels[pixel * 3 + 1] = phase * 3;
                pixels[pixel * 3 + 2] = 255 - phase * 3;
            } else {
                phase -= 170;
                pixels[pixel * 3] = phase * 3;
                pixels[pixel * 3 + 1] = 255 - phase * 3;
            }
        }
    }
    return ws2812_write_rgb(pixels, AURA_PIXEL_COUNT);
}

static bool is_motor_stop(const aura_command_t *command)
{
    return command->size == 5 && command->data[0] == AURA_CMD_MOTOR_SPEED &&
           get_u16(command->data + 2) == 0;
}

static void process_command(const aura_command_t *command)
{
    const uint8_t operation = command->data[0];
    esp_err_t result = ESP_ERR_INVALID_ARG;
    uint8_t detail = 0;
    servo_status_t status;
    switch (operation) {
    case AURA_CMD_SCAN:
        if (command->size != 1) break;
        result = ESP_OK;
        for (int id = 0; id <= 253 && !shutdown_requested; ++id) {
            if (servo_ping((uint8_t)id, &status) == ESP_OK) {
                send_servo_found((uint8_t)id);
                ++detail;
            }
            // A full scan can take several seconds. Let a queued STOP interrupt it.
            aura_command_t queued;
            if (xQueuePeek(command_queue, &queued, 0) == pdTRUE && is_motor_stop(&queued)) break;
        }
        break;
    case AURA_CMD_FEEDBACK:
        if (command->size != 2) break;
        result = servo_feedback(command->data[1], &status);
        if (result == ESP_OK) {
            servo_mode_t mode;
            const esp_err_t mode_result = servo_read_mode(command->data[1], &mode);
            send_servo_feedback(command->data[1], &status,
                                mode_result == ESP_OK ? (uint8_t)mode : 0xff);
        }
        break;
    case AURA_CMD_TORQUE:
        if (command->size != 3 || command->data[2] > 1) break;
        result = servo_set_torque(command->data[1], command->data[2] != 0);
        break;
    case AURA_CMD_MOVE:
        if (command->size != 7) break;
        result = servo_move(command->data[1], (int16_t)get_u16(command->data + 2),
                            get_u16(command->data + 4), command->data[6]);
        break;
    case AURA_CMD_ZERO:
        if (command->size != 2) break;
        result = servo_calibrate_center(command->data[1]);
        break;
    case AURA_CMD_ASSIGN_ID:
        if (command->size != 3) break;
        result = servo_assign_id(command->data[1], command->data[2]);
        detail = command->data[2];
        break;
    case AURA_CMD_LED:
        result = set_led(command);
        break;
    case AURA_CMD_TELEMETRY:
        if (command->size != 1) break;
        make_power_frame(last_power);
        notify(telemetry_handle, last_power, sizeof(last_power), aura_network_connected());
        result = ESP_OK;
        break;
    case AURA_CMD_SERVO_MODE:
        if (command->size != 3 || command->data[2] > SERVO_MODE_MOTOR) break;
        result = servo_set_mode(command->data[1], (servo_mode_t)command->data[2]);
        detail = command->data[2];
        break;
    case AURA_CMD_MOTOR_SPEED:
        if (command->size != 5) break;
        {
            const int16_t speed = (int16_t)get_u16(command->data + 2);
            if (speed == 0) {
                // Disable torque first so STOP still works if the speed write fails.
                result = servo_set_torque(command->data[1], false);
                const esp_err_t speed_result = servo_motor_speed(command->data[1], 0,
                                                                  command->data[4]);
                if (result == ESP_OK) result = speed_result;
                detail = 0;
            } else {
                result = servo_motor_speed(command->data[1], speed, command->data[4]);
                if (result == ESP_OK) result = servo_set_torque(command->data[1], true);
                detail = result == ESP_OK ? 1 : 0;
            }
        }
        break;
    case AURA_CMD_TOF_CONFIGURE:
        if (command->size != 8 || command->data[1] >= TOF_SENSOR_COUNT ||
            command->data[3] > 1) break;
        {
            tof_config_t config;
            // Configuration and calibration are independent actions in the
            // app; preserve a saved optical offset while timing/address are
            // changed.
            tof_get_config(command->data[1], &config);
            config.address_7bit = command->data[2];
            config.enabled = command->data[3] != 0;
            config.timing_budget_ms = get_u16(command->data + 4);
            config.intermeasurement_ms = get_u16(command->data + 6);
            result = tof_configure(command->data[1], &config);
            detail = command->data[1];
            send_all_tof_configs();
        }
        break;
    case AURA_CMD_TOF_CONFIGS:
        if (command->size != 1) break;
        send_all_tof_configs();
        result = ESP_OK;
        break;
    case AURA_CMD_TOF_REINITIALIZE:
        if (command->size != 1) break;
        result = tof_reinitialize();
        detail = tof_present_count();
        send_all_tof_configs();
        break;
    case AURA_CMD_TOF_SET_OFFSET:
        if (command->size != 4 || command->data[1] >= TOF_SENSOR_COUNT) break;
        result = tof_set_offset(command->data[1], (int16_t)get_u16(command->data + 2));
        detail = command->data[1];
        send_tof_config(command->data[1]);
        break;
    case AURA_CMD_TOF_SET_TUNING:
        if (command->size != 14 || command->data[1] >= TOF_SENSOR_COUNT ||
            command->data[12] > 3 || command->data[13] > 1) break;
        {
            tof_config_t config;
            tof_get_config(command->data[1], &config);
            config.xtalk_kcps = get_u16(command->data + 2);
            config.signal_threshold_kcps = get_u16(command->data + 4);
            config.sigma_threshold_mm = get_u16(command->data + 6);
            config.detection_low_mm = get_u16(command->data + 8);
            config.detection_high_mm = get_u16(command->data + 10);
            config.detection_window = command->data[12];
            config.detection_enabled = command->data[13] != 0;
            result = tof_set_tuning(command->data[1], &config);
            detail = command->data[1];
            send_tof_config(command->data[1]);
            send_tof_tuning(command->data[1]);
        }
        break;
    case AURA_CMD_TOF_TEMPERATURE_UPDATE:
        if (command->size != 2 || command->data[1] >= TOF_SENSOR_COUNT) break;
        result = tof_temperature_update(command->data[1]);
        detail = command->data[1];
        send_tof_config(command->data[1]);
        break;
    case AURA_CMD_IMU_REINITIALIZE:
        if (command->size != 1) break;
        {
            uint8_t identity = 0;
            result = mpu6050_reinitialize(&identity);
            app_state_set_imu_status(result, identity);
            detail = identity;
        }
        break;
    case AURA_CMD_DUALSENSE_PAIR:
        if (command->size != 1) break;
        result = dualsense_start_pairing();
        send_dualsense_status();
        send_dualsense_devices();
        break;
    case AURA_CMD_DUALSENSE_DISCONNECT:
        if (command->size != 1) break;
        result = dualsense_disconnect();
        send_dualsense_status();
        break;
    case AURA_CMD_DUALSENSE_CONNECT_SAVED:
        if (command->size != 1) break;
        result = dualsense_connect_saved();
        send_dualsense_status();
        break;
    case AURA_CMD_DUALSENSE_FORGET:
        if (command->size != 1) break;
        result = dualsense_forget();
        send_dualsense_status();
        break;
    case AURA_CMD_DUALSENSE_CONNECT_DISCOVERED:
        if (command->size != 7) break;
        result = dualsense_connect_discovered(command->data + 1);
        send_dualsense_status();
        break;
    case AURA_CMD_ROBOT_AXIS_CONFIG:
        if (command->size != 14 || command->data[1] >= ROBOT_LEG_COUNT ||
            command->data[2] >= ROBOT_AXIS_COUNT) break;
        (void)robot_gait_set_calibration_test(false, (robot_leg_t)command->data[1]);
        if (command->data[3] == ROBOT_SERVO_ID_UNASSIGNED) {
            result = robot_control_clear_axis((robot_leg_t)command->data[1],
                                               (robot_axis_type_t)command->data[2]);
        } else {
            const robot_axis_config_t config = {
                .servo_id = command->data[3],
                .direction = (int8_t)command->data[4],
                .center_tick = (int16_t)get_u16(command->data + 5),
                .minimum_cdeg = (int16_t)get_u16(command->data + 7),
                .maximum_cdeg = (int16_t)get_u16(command->data + 9),
                .maximum_speed_raw = get_u16(command->data + 11),
                .acceleration = command->data[13],
            };
            result = robot_control_assign_axis((robot_leg_t)command->data[1],
                                                (robot_axis_type_t)command->data[2], &config);
        }
        if (result == ESP_OK) result = robot_control_save_configuration();
        if (result == ESP_OK)
            send_robot_axis_config((robot_leg_t)command->data[1],
                                   (robot_axis_type_t)command->data[2]);
        detail = command->data[1] * ROBOT_AXIS_COUNT + command->data[2];
        break;
    case AURA_CMD_ROBOT_ARM:
        if (command->size != 2 || command->data[1] > 1) break;
        result = command->data[1] ? robot_gait_arm() : robot_control_disarm();
        detail = command->data[1] ? robot_gait_last_arm_preflight_slot() : 0;
        break;
    case AURA_CMD_ROBOT_CLEAR_SAFETY_FAULT:
        if (command->size != 1) break;
        // Deliberately does not change gait, torque, targets or calibration.
        // The operator must explicitly arm again after inspecting the robot.
        result = robot_control_clear_safety_fault();
        detail = 0;
        break;
    case AURA_CMD_ROBOT_BODY_HEIGHT:
        if (command->size != 3) break;
        result = robot_gait_set_body_height_mm((int16_t)get_u16(command->data + 1));
        detail = result == ESP_OK ? (uint8_t)(robot_gait_get_body_height_mm() - 120) : 0;
        break;
    case AURA_CMD_ROBOT_SINGLE_FOOT_TUNING:
        if (command->size != 7) break;
        result = robot_gait_set_single_foot_tuning((int16_t)get_u16(command->data + 1),
                                                    (int16_t)get_u16(command->data + 3),
                                                    get_u16(command->data + 5));
        break;
    case AURA_CMD_ROBOT_GAIT_PROFILE: {
        if (command->size != 9 || command->data[1] < ROBOT_GAIT_TROT ||
            command->data[1] > ROBOT_GAIT_TRIPOD)
            break;
        const robot_gait_mode_t gait = (robot_gait_mode_t)command->data[1];
        const robot_locomotion_profile_t profile = {
            .stride_mm = get_u16(command->data + 2),
            .step_height_mm = get_u16(command->data + 4),
            .frequency_centi_hz = get_u16(command->data + 6),
            .duty_percent = command->data[8],
        };
        result = robot_gait_set_locomotion_profile(gait, &profile);
        detail = (uint8_t)gait;
        break;
    }
    case AURA_CMD_ROBOT_TRIPOD_WALK:
        if (command->size != 3 || command->data[1] > 1 || command->data[2] >= ROBOT_LEG_COUNT) break;
        result = robot_gait_set_tripod_walk(command->data[1] != 0, (robot_leg_t)command->data[2]);
        break;
    case AURA_CMD_ROBOT_MOTION_TUNING: {
        if (command->size != 7) break;
        const robot_gait_motion_tuning_t tuning = {
            .moving_attitude_gain_per_mille = get_u16(command->data + 1),
            .com_max_velocity_mm_s = get_u16(command->data + 3),
            .com_max_acceleration_mm_s2 = get_u16(command->data + 5),
        };
        result = robot_gait_set_motion_tuning(&tuning);
        detail = result == ESP_OK ? 1 : 0;
        break;
    }
    case AURA_CMD_ROBOT_STATIC_BALANCE:
        if (command->size != 7) break;
        result = robot_gait_set_static_balance((int16_t)get_u16(command->data + 1),
                                               (int16_t)get_u16(command->data + 3),
                                               get_u16(command->data + 5));
        break;
    case AURA_CMD_ROBOT_LATERAL_STANCE:
        if (command->size != 3) break;
        result = robot_gait_set_lateral_stance_mm((int16_t)get_u16(command->data + 1));
        detail = result == ESP_OK ? (uint8_t)robot_gait_get_lateral_stance_mm() : 0;
        break;
    case AURA_CMD_CALIBRATION_CAPTURE_ZERO:
        if (command->size != 5 || command->data[1] >= ROBOT_LEG_COUNT ||
            command->data[2] >= ROBOT_AXIS_COUNT) break;
        (void)robot_gait_set_calibration_test(false, (robot_leg_t)command->data[1]);
        result = robot_control_calibration_capture_zero((robot_leg_t)command->data[1],
                                                        (robot_axis_type_t)command->data[2],
                                                        (int16_t)get_u16(command->data + 3));
        if (result == ESP_OK)
            send_robot_axis_config((robot_leg_t)command->data[1],
                                   (robot_axis_type_t)command->data[2]);
        detail = command->data[1] * ROBOT_AXIS_COUNT + command->data[2];
        ESP_LOGW(TAG, "calibration zero L%d A%d: %s", command->data[1], command->data[2],
                 esp_err_to_name(result));
        break;
    case AURA_CMD_CALIBRATION_NUDGE:
        if (command->size != 5 || command->data[1] >= ROBOT_LEG_COUNT ||
            command->data[2] >= ROBOT_AXIS_COUNT) break;
        (void)robot_gait_set_calibration_test(false, (robot_leg_t)command->data[1]);
        result = robot_control_calibration_nudge_axis((robot_leg_t)command->data[1],
                                                       (robot_axis_type_t)command->data[2],
                                                       (int16_t)get_u16(command->data + 3));
        detail = command->data[1] * ROBOT_AXIS_COUNT + command->data[2];
        break;
    case AURA_CMD_CALIBRATION_FLIP_DIRECTION:
        if (command->size != 5 || command->data[1] >= ROBOT_LEG_COUNT ||
            command->data[2] >= ROBOT_AXIS_COUNT) break;
        (void)robot_gait_set_calibration_test(false, (robot_leg_t)command->data[1]);
        result = robot_control_calibration_flip_direction((robot_leg_t)command->data[1],
                                                           (robot_axis_type_t)command->data[2],
                                                           (int16_t)get_u16(command->data + 3));
        if (result == ESP_OK)
            send_robot_axis_config((robot_leg_t)command->data[1],
                                   (robot_axis_type_t)command->data[2]);
        detail = command->data[1] * ROBOT_AXIS_COUNT + command->data[2];
        break;
    case AURA_CMD_CALIBRATION_REBASE_DIRECTION:
        if (command->size != 5 || command->data[1] >= ROBOT_LEG_COUNT ||
            command->data[2] >= ROBOT_AXIS_COUNT) break;
        (void)robot_gait_set_calibration_test(false, (robot_leg_t)command->data[1]);
        result = robot_control_calibration_rebase_direction((robot_leg_t)command->data[1],
                                                             (robot_axis_type_t)command->data[2],
                                                             (int16_t)get_u16(command->data + 3));
        if (result == ESP_OK)
            send_robot_axis_config((robot_leg_t)command->data[1],
                                   (robot_axis_type_t)command->data[2]);
        detail = command->data[1] * ROBOT_AXIS_COUNT + command->data[2];
        break;
    case AURA_CMD_CALIBRATION_REVERSE_PROBE:
        // Compatibility guard for an old desktop command.  It formerly
        // performed two automatic position moves while changing direction.
        // Reject it before torque, goal position, or calibration data change.
        if (command->size != 7 || command->data[1] >= ROBOT_LEG_COUNT ||
            command->data[2] >= ROBOT_AXIS_COUNT) break;
        result = ESP_ERR_NOT_SUPPORTED;
        detail = command->data[1] * ROBOT_AXIS_COUNT + command->data[2];
        break;
    case AURA_CMD_CALIBRATION_RELEASE_AXIS:
        if (command->size != 3 || command->data[1] >= ROBOT_LEG_COUNT ||
            command->data[2] >= ROBOT_AXIS_COUNT) break;
        (void)robot_gait_set_calibration_test(false, (robot_leg_t)command->data[1]);
        result = robot_control_calibration_release_axis((robot_leg_t)command->data[1],
                                                        (robot_axis_type_t)command->data[2]);
        detail = command->data[1] * ROBOT_AXIS_COUNT + command->data[2];
        break;
    case AURA_CMD_CALIBRATION_CAPTURE_LIMIT:
        if (command->size != 4 || command->data[1] >= ROBOT_LEG_COUNT ||
            command->data[2] >= ROBOT_AXIS_COUNT || command->data[3] > 1) break;
        (void)robot_gait_set_calibration_test(false, (robot_leg_t)command->data[1]);
        result = robot_control_calibration_capture_limit((robot_leg_t)command->data[1],
                                                         (robot_axis_type_t)command->data[2],
                                                         command->data[3] != 0);
        if (result == ESP_OK)
            send_robot_axis_config((robot_leg_t)command->data[1],
                                   (robot_axis_type_t)command->data[2]);
        detail = command->data[1] * ROBOT_AXIS_COUNT + command->data[2];
        break;
    case AURA_CMD_CALIBRATION_LEG_TEST:
        if (command->size != 3 || command->data[1] >= ROBOT_LEG_COUNT || command->data[2] > 1) break;
        result = robot_gait_set_calibration_test(command->data[2] != 0,
                                                 (robot_leg_t)command->data[1]);
        detail = command->data[1];
        break;
    case AURA_CMD_CALIBRATION_LEG_TORQUE:
        if (command->size != 3 || command->data[1] >= ROBOT_LEG_COUNT || command->data[2] > 1) break;
        (void)robot_gait_set_calibration_test(false, (robot_leg_t)command->data[1]);
        result = robot_control_calibration_set_leg_torque((robot_leg_t)command->data[1],
                                                          command->data[2] != 0);
        detail = command->data[1];
        break;
    case AURA_CMD_CALIBRATION_LEG_PREVIEW:
        if (command->size != 3 || command->data[1] >= ROBOT_LEG_COUNT || command->data[2] > 1) break;
        // Preview is deliberately read-only. Stopping a possible old leg
        // test releases it; the following call only changes the feedback
        // scheduler and cannot write a servo register.
        (void)robot_gait_set_calibration_test(false, (robot_leg_t)command->data[1]);
        result = robot_control_calibration_set_leg_preview((robot_leg_t)command->data[1],
                                                            command->data[2] != 0);
        detail = command->data[1];
        break;
    case AURA_CMD_ROBOT_ATTITUDE_BALANCE:
        if (command->size != 2 || command->data[1] > 1) break;
        result = robot_gait_set_attitude_balance_enabled(command->data[1] != 0);
        detail = command->data[1];
        break;
    case AURA_CMD_ROBOT_ATTITUDE_LIMIT:
        if (command->size != 3) break;
        result = robot_gait_set_attitude_correction_limit_cdeg(get_u16(command->data + 1));
        detail = result == ESP_OK ? (uint8_t)(get_u16(command->data + 1) / 100U) : 0;
        break;
    case AURA_CMD_ROBOT_CONFIGS:
        // This is a read-only request. Accept a future desktop client that
        // appends optional request metadata instead of leaving its calibration
        // workflow waiting forever for a result.
        if (command->size < 1) break;
        // A backup request must be inert: it neither stops a leg test nor
        // writes a byte to NVS. It simply sends the records currently used.
        for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg)
            for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis)
                send_robot_axis_config((robot_leg_t)leg, (robot_axis_type_t)axis);
        uint8_t profile_frame[AURA_FRAME_SIZE];
        for (int gait = ROBOT_GAIT_TROT; gait <= ROBOT_GAIT_TRIPOD; ++gait) {
            make_robot_gait_profile_frame(profile_frame, (robot_gait_mode_t)gait);
            notify(telemetry_handle, profile_frame, sizeof(profile_frame), aura_network_connected());
        }
        make_robot_motion_tuning_frame(profile_frame);
        notify(telemetry_handle, profile_frame, sizeof(profile_frame), aura_network_connected());
        result = ESP_OK;
        detail = ROBOT_LEG_COUNT * ROBOT_AXIS_COUNT;
        break;
    default:
        break;
    }
    send_result(operation, result, detail);
}

static void command_task(void *argument)
{
    (void)argument;
    aura_command_t command;
    while (!shutdown_requested) {
        // Desktop commands are configuration/telemetry work.  The pad and
        // physical Create safety switch run in controller_task below, so a
        // TCP client or a busy application can never govern locomotion.
        if (xQueueReceive(command_queue, &command, pdMS_TO_TICKS(10)) == pdTRUE &&
            !shutdown_requested)
            process_command(&command);
    }
    command_task_handle = NULL;
    vTaskDelete(NULL);
}

// The only runtime source of real gait input is a direct Classic-HID report
// from the paired DualSense.  Keep it at a higher priority than networking
// and independent of the TCP command/telemetry path: Aura must keep its
// 50 Hz planner and controller alive with the Mac entirely absent.
static void controller_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    while (!shutdown_requested) {
        dualsense_poll();
        process_create_arm_switch();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
    controller_task_handle = NULL;
    vTaskDelete(NULL);
}

static void telemetry_task(void *argument)
{
    (void)argument;
    unsigned tick = 0;
    while (!shutdown_requested) {
        if (aura_network_connected()) {
            uint8_t frame[AURA_FRAME_SIZE];
            // 50 Hz telemetry keeps the desktop simulation and current trace
            // in step with live controls. Each frame is only 22 bytes on TCP,
            // and MSG_DONTWAIT drops a stalled desktop client instead of
            // accumulating latency on the shared Wi-Fi / Bluetooth radio.
            make_power_frame(frame);
            memcpy(last_power, frame, sizeof(last_power));
            notify(telemetry_handle, frame, sizeof(frame), true);
            make_tof_frame(frame);
            notify(telemetry_handle, frame, sizeof(frame), true);
            make_dualsense_frame(frame);
            notify(telemetry_handle, frame, sizeof(frame), true);
            make_imu_frame(frame);
            notify(telemetry_handle, frame, sizeof(frame), true);
            make_robot_attitude_frame(frame);
            notify(telemetry_handle, frame, sizeof(frame), true);
            make_robot_contact_frame(frame);
            notify(telemetry_handle, frame, sizeof(frame), true);
            make_robot_safety_fault_frame(frame);
            notify(telemetry_handle, frame, sizeof(frame), true);
            make_robot_status_frame(frame);
            notify(telemetry_handle, frame, sizeof(frame), true);
            make_robot_pose_frame(frame);
            notify(telemetry_handle, frame, sizeof(frame), true);
            make_robot_body_reference_frame(frame);
            notify(telemetry_handle, frame, sizeof(frame), true);
            // Board-owned profiles synchronize independently of the 50 Hz
            // controller. The desktop never feeds targets back to Aura.
            if (tick % 50 == 0) {
                for (int gait = ROBOT_GAIT_TROT; gait <= ROBOT_GAIT_TRIPOD; ++gait) {
                    make_robot_gait_profile_frame(frame, (robot_gait_mode_t)gait);
                    notify(telemetry_handle, frame, sizeof(frame), true);
                }
                make_robot_motion_tuning_frame(frame);
                notify(telemetry_handle, frame, sizeof(frame), true);
            }
            // All twelve commanded and measured axis states reach the desktop
            // every control period. The board remains the sole source of them.
            for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
                make_robot_leg_frame(frame, (robot_leg_t)leg);
                notify(telemetry_handle, frame, sizeof(frame), true);
            }
            if (tick % 10 == 0) {
                make_servo_capacity_frame(frame);
                notify(telemetry_handle, frame, sizeof(frame), true);
            }
            if (tick % 50 == 0) {
                make_tof_diagnostics_frame(frame);
                notify(telemetry_handle, frame, sizeof(frame), true);
                send_dualsense_status();
                // Device discoveries are useful only while an inquiry is
                // running.  Re-sending the complete unchanged list while the
                // controller is connected caused needless link/UI churn.
                dualsense_snapshot_t controller;
                dualsense_get_snapshot(&controller);
                if (controller.state == DUALSENSE_STATE_SCANNING)
                    send_dualsense_devices();
            }
            ++tick;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    telemetry_task_handle = NULL;
    vTaskDelete(NULL);
}

static void queue_command(const uint8_t *data, uint16_t size)
{
    if (!command_queue || !data || !size || size > AURA_FRAME_SIZE) return;
    if (data[0] == AURA_CMD_CALIBRATION_CAPTURE_ZERO || data[0] == AURA_CMD_ROBOT_CONFIGS)
        ESP_LOGW(TAG, "RX command 0x%02x, %u bytes", data[0], (unsigned)size);
    aura_command_t command = {.size = (uint8_t)size};
    memcpy(command.data, data, size);
    const BaseType_t queued = is_motor_stop(&command)
                                ? xQueueSendToFront(command_queue, &command, 0)
                                : xQueueSend(command_queue, &command, pdMS_TO_TICKS(20));
    if (queued != pdTRUE) ESP_LOGW(TAG, "Aura command queue full");
}

static esp_err_t start_bluetooth_stack(void)
{
    esp_bt_controller_config_t controller_configuration = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_LOGW(TAG, "BT stage: controller init (Wi-Fi + Classic HID)");
    esp_err_t result = esp_bt_controller_init(&controller_configuration);
    if (result != ESP_OK) return result;
    controller_running = true;

    result = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (result != ESP_OK) return result;

    esp_bluedroid_config_t bluedroid_configuration = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    result = esp_bluedroid_init_with_cfg(&bluedroid_configuration);
    if (result != ESP_OK) return result;
    result = esp_bluedroid_enable();
    if (result != ESP_OK) return result;
    bluedroid_running = true;

    result = esp_bt_gap_set_device_name(AURA_NAME);
    if (result != ESP_OK) return result;
    return dualsense_init();
}

static void stop_bluetooth_stack(void)
{
    if (bluedroid_running) {
        dualsense_deinit();
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED)
            esp_bluedroid_disable();
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED)
            esp_bluedroid_deinit();
        bluedroid_running = false;
    }
    if (controller_running) {
        if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
            esp_bt_controller_disable();
        if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED)
            esp_bt_controller_deinit();
        controller_running = false;
    }

}

esp_err_t aura_radio_init(void)
{
    if (radio_running) return ESP_OK;
    shutdown_requested = false;
    memset(last_power, 0, sizeof(last_power));

    // Keep room for a complete, deliberate 12-axis commissioning write while
    // preserving a bounded command backlog for physical control.
    command_queue = xQueueCreate(16, sizeof(aura_command_t));
    if (!command_queue) return ESP_ERR_NO_MEM;

    ESP_LOGW(TAG, "radio features: Wi-Fi=%d DualSense=%d", AURA_ENABLE_WIFI, AURA_ENABLE_DUALSENSE);
    esp_err_t result = AURA_ENABLE_DUALSENSE ? start_bluetooth_stack() : ESP_OK;
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth stack: %s", esp_err_to_name(result));
        stop_bluetooth_stack();
        vQueueDelete(command_queue);
        command_queue = NULL;
        return result;
    }
    result = AURA_ENABLE_WIFI ? aura_network_start(queue_command) : ESP_OK;
    if (result != ESP_OK) {
        stop_bluetooth_stack();
        vQueueDelete(command_queue); command_queue = NULL;
        return result;
    }
    if (xTaskCreate(controller_task, "aura_controller", 4096, NULL, 9,
                    &controller_task_handle) != pdPASS ||
        xTaskCreate(command_task, "aura_commands", 4096, NULL, 5,
                    &command_task_handle) != pdPASS ||
        (AURA_ENABLE_WIFI && xTaskCreate(telemetry_task, "aura_telemetry", 3072, NULL, 4,
                    &telemetry_task_handle) != pdPASS)) {
        shutdown_requested = true;
        if (controller_task_handle) vTaskDelete(controller_task_handle);
        if (command_task_handle) vTaskDelete(command_task_handle);
        if (telemetry_task_handle) vTaskDelete(telemetry_task_handle);
        controller_task_handle = NULL;
        command_task_handle = NULL;
        telemetry_task_handle = NULL;
        aura_network_stop();
        stop_bluetooth_stack();
        vQueueDelete(command_queue);
        command_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    radio_running = true;
    ESP_LOGW(TAG, "Configured radio services started");
    return ESP_OK;
}

bool aura_radio_is_running(void)
{
    return radio_running;
}

esp_err_t aura_radio_deinit(void)
{
    if (!radio_running) return ESP_OK;
    shutdown_requested = true;
    aura_network_stop();
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    while ((controller_task_handle || command_task_handle || telemetry_task_handle) &&
           (int32_t)(deadline - xTaskGetTickCount()) > 0)
        vTaskDelay(pdMS_TO_TICKS(10));
    if (controller_task_handle) { vTaskDelete(controller_task_handle); controller_task_handle = NULL; }
    if (command_task_handle) { vTaskDelete(command_task_handle); command_task_handle = NULL; }
    if (telemetry_task_handle) { vTaskDelete(telemetry_task_handle); telemetry_task_handle = NULL; }
    stop_bluetooth_stack();
    if (command_queue) { vQueueDelete(command_queue); command_queue = NULL; }
    radio_running = false;
    return ESP_OK;
}
