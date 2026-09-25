#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#include "dualsense.h"
#include "app_state.h"
#include "joint_trajectory.h"
#include "robot_control.h"
#include "robot_gait.h"
#include "robot_gait_profile.h"
#include "robot_locomotion.h"
#include "robot_predictive_support.h"
#include "robot_body_trajectory.h"
#include "robot_balance.h"
#include "robot_static_gait_profile.h"
#include "robot_static_balance.h"
#include "robot_gait_sync.h"
#include "robot_contact.h"
#include "robot_kinematics.h"
#include "robot_odometry.h"
#include "mpu6050.h"

#define ROBOT_GAIT_PERIOD_MS 20
#define ROBOT_GAIT_VIRTUAL_INPUT_TIMEOUT_US 300000
#define ROBOT_PI 3.14159265358979323846f
#define ROBOT_GAIT_SPEED_RAW 3400
#define ROBOT_GAIT_ACCELERATION 254
// Aura's calibrated links have ample clearance at this standing height. The
// old 118 mm pose forced a deeply folded knee and made the usable gait stroke
// unnecessarily small.
#define ROBOT_DEFAULT_BODY_HEIGHT_MM 180
#define ROBOT_MIN_BODY_HEIGHT_MM 120
#define ROBOT_MAX_BODY_HEIGHT_MM 250
#define ROBOT_BODY_HEIGHT_ADJUST_MM 35.0f
#define ROBOT_GAIT_SINGLE_FOOT_DEFAULT_STRIDE_MM 42
#define ROBOT_GAIT_SINGLE_FOOT_DEFAULT_LIFT_MM 18
#define ROBOT_GAIT_SINGLE_FOOT_MIN_STRIDE_MM 20
#define ROBOT_GAIT_SINGLE_FOOT_MAX_STRIDE_MM 75
#define ROBOT_GAIT_SINGLE_FOOT_MIN_LIFT_MM 8
#define ROBOT_GAIT_SINGLE_FOOT_MAX_LIFT_MM 55
#define ROBOT_GAIT_SINGLE_FOOT_DEFAULT_FREQUENCY_CENTI_HZ 45
#define ROBOT_GAIT_SINGLE_FOOT_MIN_FREQUENCY_CENTI_HZ 30
// At 50 Hz, a 1 Hz four-leg crawl gives the vertical lift only two or three
// target samples.  The foot then appears to jab the floor even with a smooth
// geometric path.  This is a commissioning-safe ceiling; it can be raised
// only after the physical gait has a verified contact controller.
#define ROBOT_GAIT_SINGLE_FOOT_MAX_FREQUENCY_CENTI_HZ 55
// Extra lateral offset after the physical ab/ad link.  It widens the final
// footprint, not a fictitious servo-width or mechanical-clearance value.
#define ROBOT_GAIT_DEFAULT_LATERAL_STANCE_MM 25
#define ROBOT_GAIT_MIN_LATERAL_STANCE_MM 0
#define ROBOT_GAIT_MAX_LATERAL_STANCE_MM 80
#define ROBOT_GAIT_NVS_NAMESPACE "robot_gait"
#define ROBOT_GAIT_NVS_HEIGHT_KEY "height_mm"
#define ROBOT_GAIT_NVS_STRIDE_KEY "step_mm"
#define ROBOT_GAIT_NVS_LIFT_KEY "lift_mm"
#define ROBOT_GAIT_NVS_FREQUENCY_KEY "freq_chz"
#define ROBOT_GAIT_NVS_PROFILES_KEY "gaitprof1"
#define ROBOT_GAIT_NVS_MOTION_TUNING_KEY "motiont1"
#define ROBOT_GAIT_NVS_COM_FORWARD_KEY "com_x_mm"
#define ROBOT_GAIT_NVS_COM_LEFT_KEY "com_z_mm"
#define ROBOT_GAIT_NVS_SUPPORT_MARGIN_KEY "support_mm"
#define ROBOT_GAIT_NVS_LATERAL_STANCE_KEY "stance_mm"
// v3 deliberately starts enabled: standing balance is a normal part of
// Aura's arm path.  Earlier builds could persist a disabled experimental
// switch, which left an otherwise healthy IMU ignored after a restart.  The
// old key is deliberately not migrated.
#define ROBOT_GAIT_NVS_ATTITUDE_ENABLE_KEY "imu_bal3"
#define ROBOT_GAIT_NVS_ATTITUDE_LIMIT_KEY "imu_lim_cd"
// During a one-foot transfer the projected centre of mass must remain inside
// the triangle formed by the other three feet. This is an edge-distance
// margin, not an arbitrary fraction of a geometric centroid.
#define ROBOT_STATIC_DEFAULT_SUPPORT_MARGIN_MM 15
#define ROBOT_STATIC_MIN_SUPPORT_MARGIN_MM 5
#define ROBOT_STATIC_MAX_SUPPORT_MARGIN_MM 45
#define ROBOT_STATIC_COM_OFFSET_LIMIT_MM 120
#define ROBOT_STATIC_REFERENCE_SETTLED_RAD (2.0f * ROBOT_PI / 180.0f)
#define ROBOT_STATIC_FEEDBACK_SETTLED_RAD (5.0f * ROBOT_PI / 180.0f)
#define ROBOT_STATIC_SETTLED_TICKS 5
// A ST3215 sample for one axis arrives roughly every 60 ms across all twelve
// axes.  That is adequate for this deliberately slow, one-foot crawl.  Do
// not apply event-driven touchdown to trot/run until they have a faster,
// per-leg contact source.
#define ROBOT_STATIC_CONTACT_TOUCHDOWN_START 0.84f
#define ROBOT_STATIC_CONTACT_HOLD_EPSILON 0.00025f
// CoM lies slightly below the body-frame origin. This is the physical
// battery/electronics plane used when IMU attitude is projected onto ground.
#define ROBOT_BODY_COM_VERTICAL_OFFSET_MM (-25.0f)
#define ROBOT_ATTITUDE_MAX_ERROR_RAD (25.0f * ROBOT_PI / 180.0f)
#define ROBOT_ATTITUDE_DEFAULT_MAX_CORRECTION_CDEG 2000
#define ROBOT_ATTITUDE_MIN_MAX_CORRECTION_CDEG 300
#define ROBOT_ATTITUDE_ABSOLUTE_MAX_CORRECTION_CDEG 2000
#define ROBOT_ATTITUDE_CDEG_TO_RAD (ROBOT_PI / 18000.0f)
// Standing balance follows a slow change of the floor one-for-one.  While a
// leg is in transfer, a smaller gain and a lower body-pose rate prevent the
// IMU loop from fighting the scheduled foot path.  The moving gain itself is
// persisted below so it can be commissioned from Aura.
#define ROBOT_ATTITUDE_STANDING_PROPORTIONAL_GAIN 1.0f
#define ROBOT_ATTITUDE_STANDING_SLEW_RAD_S (35.0f * ROBOT_PI / 180.0f)
#define ROBOT_ATTITUDE_STANDING_ACCEL_RAD_S2 (220.0f * ROBOT_PI / 180.0f)
#define ROBOT_ATTITUDE_MOVING_SLEW_RAD_S (35.0f * ROBOT_PI / 180.0f)
#define ROBOT_ATTITUDE_MOVING_ACCEL_RAD_S2 (220.0f * ROBOT_PI / 180.0f)
#define ROBOT_ATTITUDE_FRESH_US 120000
#define ROBOT_MOTION_TUNING_DEFAULT_ATTITUDE_GAIN_PER_MILLE 450U
#define ROBOT_MOTION_TUNING_MIN_ATTITUDE_GAIN_PER_MILLE 100U
#define ROBOT_MOTION_TUNING_MAX_ATTITUDE_GAIN_PER_MILLE 1000U
#define ROBOT_MOTION_TUNING_DEFAULT_COM_VELOCITY_MM_S 85U
#define ROBOT_MOTION_TUNING_MIN_COM_VELOCITY_MM_S 20U
#define ROBOT_MOTION_TUNING_MAX_COM_VELOCITY_MM_S 160U
#define ROBOT_MOTION_TUNING_DEFAULT_COM_ACCELERATION_MM_S2 360U
#define ROBOT_MOTION_TUNING_MIN_COM_ACCELERATION_MM_S2 60U
#define ROBOT_MOTION_TUNING_MAX_COM_ACCELERATION_MM_S2 900U
#define ROBOT_TRIPOD_TEST_LIFT_MM 30.0f
#define ROBOT_TRIPOD_BODY_LIFT_MM 6.0f
#define ROBOT_TRIPOD_STAGE_RATE_HZ 0.85f

// These limit the moving setpoint, not the servo's own speed or acceleration.
// Match them to the ST3215 command sent below: 3400 encoder ticks/s and
// acceleration 254.  That keeps the 50 Hz target stream continuous without
// artificially slowing a servo below the fastest factory command we issue.
// 4095 ticks equals one full revolution; the acceleration register is in
// 100-tick/s² units for this profile.
#define ROBOT_JOINT_MAX_VELOCITY_RAD_S \
    (2.0f * ROBOT_PI * (float)ROBOT_GAIT_SPEED_RAW / 4095.0f)
#define ROBOT_JOINT_MAX_ACCELERATION_RAD_S2 \
    (2.0f * ROBOT_PI * (float)ROBOT_GAIT_ACCELERATION * 100.0f / 4095.0f)

typedef struct {
    bool connected;
    bool has_input;
    bool r1;
    bool l1;
    bool cross;
    bool options;
    float forward;
    float lateral;
    float turn;
    float height;
} gait_input_t;

static SemaphoreHandle_t gait_mutex;
static TaskHandle_t gait_task;
static robot_gait_snapshot_t state;
static robot_gait_virtual_input_t virtual_input;
// UART virtual sticks are a commissioning aid only.  They must never leave
// the autonomous robot waiting for a computer after a terminal disappears.
// A current physical DualSense report always wins, and an unattended virtual
// command automatically returns control to the pad after this deadline.
static int64_t virtual_input_deadline_us;
static struct {
    bool enabled;
    robot_leg_t leg;
    float phase;
} calibration_test;
static bool r1_was_pressed, l1_was_pressed, cross_was_pressed, options_was_pressed;
static float phase;
static gait_input_t filtered_drive;
static uint32_t jump_ticks_remaining;
static joint_trajectory_t joint_trajectory[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT];
static bool trajectory_initialized;
// Separate from the full 12-axis trajectory flag. A calibration test must
// seed only its chosen leg and must never make a later full-arm transition
// skip its own safe 12-servo trajectory initialization.
static bool calibration_trajectory_initialized;
static robot_geometry_t geometry;
static robot_odometry_t odometry;
static int16_t nominal_body_height_mm = ROBOT_DEFAULT_BODY_HEIGHT_MM;
// The gait scheduler owns a complete persisted profile for every moving gait.
// Index zero (Stanie) remains unused so its index matches robot_gait_mode_t.
static robot_locomotion_profile_t gait_profiles[ROBOT_GAIT_CLIMB + 1];
// Separate key/record leaves all previously calibrated profiles byte-compatible.
static robot_locomotion_profile_t tripod_walk_profile;
static bool tripod_walk_enabled;
static robot_leg_t tripod_walk_excluded = ROBOT_LEG_RIGHT_REAR;
static float tripod_walk_entry;
static mpu6050_attitude_t balance_imu;
static robot_gait_motion_tuning_t motion_tuning = {
    .moving_attitude_gain_per_mille = ROBOT_MOTION_TUNING_DEFAULT_ATTITUDE_GAIN_PER_MILLE,
    .com_max_velocity_mm_s = ROBOT_MOTION_TUNING_DEFAULT_COM_VELOCITY_MM_S,
    .com_max_acceleration_mm_s2 = ROBOT_MOTION_TUNING_DEFAULT_COM_ACCELERATION_MM_S2,
};
// Actual projected CoM relative to the frame origin.  A centered value is the
// safe first-boot default; the commissioning UI persists the measured offset.
static int16_t static_com_forward_mm;
static int16_t static_com_left_mm;
static uint16_t static_support_margin_mm = ROBOT_STATIC_DEFAULT_SUPPORT_MARGIN_MM;
// Persisted while disarmed. Changing this never writes a target to a servo.
static int16_t lateral_stance_mm = ROBOT_GAIT_DEFAULT_LATERAL_STANCE_MM;
static uint16_t attitude_max_correction_cdeg = ROBOT_ATTITUDE_DEFAULT_MAX_CORRECTION_CDEG;
static bool static_crawl_active;
static bool static_crawl_waiting_for_settle;
static uint8_t static_crawl_settled_ticks;
static int static_crawl_input_slot = -1;
static float static_crawl_forward;
static float static_crawl_lateral;
static float static_crawl_stride_mm;
static int static_crawl_plan_slot = -1;
static bool static_crawl_support_safe;
static robot_vec3_t static_crawl_from_body_shift;
static robot_vec3_t static_crawl_to_body_shift;
// Dynamic gaits use one phase plan for feet and the floating body reference.
// `dynamic_balance_shift` is the VPSP position target for this phase, not a
// separate balance controller.
static robot_vec3_t dynamic_balance_shift;
static robot_body_trajectory_t body_trajectory;
static float tripod_body_height_mm;
static struct {
    bool enabled;
    bool reference_valid;
    // At arm time torque is still off, so the torso can be sagged or held by
    // an operator.  Capture the level reference only after the commanded
    // neutral stance has physically settled.
    bool reference_pending;
    uint8_t reference_settled_ticks;
    bool active;
    float reference_roll_radians;
    float reference_pitch_radians;
    float correction_roll_radians;
    float correction_pitch_radians;
} attitude_control = {
    .enabled = true,
};
// The posture controller uses the same bounded-acceleration generator as a
// joint command. A rate limiter alone changes velocity instantaneously when
// IMU error changes, which appears as a kick at the foot during a gait.
static joint_trajectory_t attitude_trajectory[2];
static robot_contact_estimate_t contact_observer[ROBOT_LEG_COUNT];

typedef enum {
    ROBOT_TRIPOD_IDLE = 0,
    ROBOT_TRIPOD_SHIFT,
    ROBOT_TRIPOD_LIFT,
    ROBOT_TRIPOD_HOLD,
    ROBOT_TRIPOD_LOWER,
    ROBOT_TRIPOD_RETURN,
} robot_tripod_state_t;

// The test removes the rear-right foot from support. It remains torque-held
// above the floor rather than being released, so it cannot drop or fold under
// the chassis during a balance check.
static const robot_leg_t tripod_test_leg = ROBOT_LEG_RIGHT_REAR;
static robot_tripod_state_t tripod_state;
static float tripod_stage_progress;
static uint8_t tripod_settled_ticks;
static bool tripod_exit_requested;
// 0xff means the last arm preflight passed or has not yet run.
static uint8_t last_arm_preflight_slot = 0xff;

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float wrap_radians(float value)
{
    while (value > ROBOT_PI) value -= 2.0f * ROBOT_PI;
    while (value < -ROBOT_PI) value += 2.0f * ROBOT_PI;
    return value;
}

static int16_t radians_to_cdeg_signed(float value)
{
    if (!isfinite(value)) return 0;
    const float cdeg = clampf(value * 18000.0f / ROBOT_PI, -32768.0f, 32767.0f);
    return (int16_t)lroundf(cdeg);
}

static float stick(uint8_t value)
{
    const float normalized = clampf(((float)value - 128.0f) / 127.0f, -1.0f, 1.0f);
    return fabsf(normalized) < 0.07f ? 0.0f : normalized;
}

static gait_input_t read_input(void)
{
    dualsense_snapshot_t pad = {0};
    dualsense_get_snapshot(&pad);
    const bool connected = pad.state == DUALSENSE_STATE_CONNECTED;
    const bool physical_input_available = connected && pad.has_input;

    // The physical controller is the normal, standalone source.  It is read
    // first, so neither a UART experiment nor a former desktop session can
    // silently take ownership of a running robot.
    if (physical_input_available) {
        return (gait_input_t){
            .connected = true,
            .has_input = true,
            .r1 = (pad.buttons[1] & 0x02u) != 0,
            .l1 = (pad.buttons[1] & 0x01u) != 0,
            .cross = (pad.buttons[0] & 0x20u) != 0,
            .options = (pad.buttons[1] & 0x20u) != 0,
            .forward = -stick(pad.left_y),
            .lateral = stick(pad.left_x),
            .turn = stick(pad.right_x),
            .height = -stick(pad.right_y),
        };
    }

    const int64_t now_us = esp_timer_get_time();
    if (virtual_input.enabled && now_us < virtual_input_deadline_us) {
        const float forward = clampf((float)virtual_input.forward_milli / 1000.0f, -1.0f, 1.0f);
        const float lateral = clampf((float)virtual_input.lateral_milli / 1000.0f, -1.0f, 1.0f);
        const float turn = clampf((float)virtual_input.turn_milli / 1000.0f, -1.0f, 1.0f);
        const float height = clampf((float)virtual_input.height_milli / 1000.0f, -1.0f, 1.0f);
        return (gait_input_t){
            .connected = true,
            .has_input = fabsf(forward) > 0.02f || fabsf(lateral) > 0.02f ||
                         fabsf(turn) > 0.02f || fabsf(height) > 0.02f,
            .forward = forward, .lateral = lateral, .turn = turn, .height = height,
        };
    }

    // This is deliberately a local state transition inside gait_mutex.  No
    // target is written here; it only removes an expired commissioning input.
    if (virtual_input.enabled) {
        virtual_input = (robot_gait_virtual_input_t){0};
        virtual_input_deadline_us = 0;
        ESP_LOGI("robot_gait", "virtual input expired; returning control to DualSense");
    }
    return (gait_input_t){
        .connected = connected,
        .has_input = connected && pad.has_input,
        .r1 = (pad.buttons[1] & 0x02u) != 0,
        .l1 = (pad.buttons[1] & 0x01u) != 0,
        .cross = (pad.buttons[0] & 0x20u) != 0,
        .options = (pad.buttons[1] & 0x20u) != 0,
        .forward = connected ? -stick(pad.left_y) : 0.0f,
        .lateral = connected ? stick(pad.left_x) : 0.0f,
        .turn = connected ? stick(pad.right_x) : 0.0f,
        .height = connected ? -stick(pad.right_y) : 0.0f,
    };
}

static const robot_locomotion_profile_t *gait_profile(robot_gait_mode_t gait)
{
    if (gait == ROBOT_GAIT_TRIPOD) return &tripod_walk_profile;
    if (gait < ROBOT_GAIT_TROT || gait > ROBOT_GAIT_CLIMB)
        return NULL;
    return &gait_profiles[gait];
}

static float gait_hz(robot_gait_mode_t gait)
{
    return robot_locomotion_frequency_hz(gait_profile(gait));
}

static float gait_duty(robot_gait_mode_t gait)
{
    return robot_locomotion_duty_factor(gait_profile(gait));
}

static float static_crawl_preload_fraction(void)
{
    const float duty = clampf(gait_duty(state.active_gait), 0.0f, 1.0f);
    // One full cycle consists of four leg slots. `1-duty` is the airborne
    // share for each leg, therefore the same share occupies 4× that fraction
    // inside its one slot. The profile validator keeps this below one slot.
    return clampf(1.0f - 4.0f * (1.0f - duty), 0.02f, 0.60f);
}

static robot_gait_mode_t next_gait(robot_gait_mode_t gait)
{
    return (robot_gait_mode_t)robot_locomotion_next_mode((uint8_t)gait);
}

static float leg_phase_offset(int leg, robot_gait_mode_t gait)
{
    return robot_locomotion_leg_phase_offset((uint8_t)gait, (robot_leg_t)leg);
}

static float gait_max_stride_mm(robot_gait_mode_t gait)
{
    const robot_locomotion_profile_t *profile = gait_profile(gait);
    return profile ? (float)profile->stride_mm : 0.0f;
}

static robot_vec3_t standing_foot_for_leg(robot_leg_t leg)
{
    robot_vec3_t foot = robot_kinematics_neutral_foot(&geometry, leg);
    const bool left = leg == ROBOT_LEG_LEFT_FRONT || leg == ROBOT_LEG_LEFT_REAR;
    // A modest lateral spread widens the support polygon without pretending
    // that servo-case width or construction clearance is a kinematic link.
    foot.z += left ? (float)lateral_stance_mm : -(float)lateral_stance_mm;
    return foot;
}

// Rotate the calibrated CoM by the same torso orientation used for all four
// IK targets, then project it onto the ground plane. Translation is excluded:
// support planners return precisely that missing x/z torso translation.
static robot_planar_point_t projected_com_for_body_pose(float roll_radians,
                                                         float pitch_radians)
{
    const float c_roll = cosf(roll_radians), s_roll = sinf(roll_radians);
    const float c_pitch = cosf(pitch_radians), s_pitch = sinf(pitch_radians);
    const float local_x = (float)static_com_forward_mm;
    const float local_y = ROBOT_BODY_COM_VERTICAL_OFFSET_MM;
    const float local_z = (float)static_com_left_mm;
    // World rotation is Rz(pitch) * Rx(roll), the inverse of the transform
    // in body_pose_foot_target().
    const float rolled_y = c_roll * local_y - s_roll * local_z;
    const float rolled_z = s_roll * local_y + c_roll * local_z;
    return (robot_planar_point_t){
        .x = c_pitch * local_x - s_pitch * rolled_y,
        .z = rolled_z,
    };
}

typedef struct {
    bool enabled;
    // False means the current three-foot polygon cannot provide the required
    // interior margin. In that case every foot stays planted.
    bool support_safe;
    robot_leg_t swing_leg;
    robot_vec3_t body_shift;
    float forward;
    float lateral;
} static_crawl_plan_t;

static int static_crawl_slot_for_leg(robot_leg_t leg)
{
    // Rear right, left front, left rear, right front. Each next support
    // triangle overlaps the preceding one in two feet.
    switch (leg) {
    case ROBOT_LEG_RIGHT_REAR: return 0;
    case ROBOT_LEG_LEFT_FRONT: return 1;
    case ROBOT_LEG_LEFT_REAR: return 2;
    case ROBOT_LEG_RIGHT_FRONT: return 3;
    default: return 0;
    }
}

static robot_leg_t static_crawl_leg_for_slot(int slot)
{
    static const robot_leg_t order[] = {
        ROBOT_LEG_RIGHT_REAR, ROBOT_LEG_LEFT_FRONT,
        ROBOT_LEG_LEFT_REAR, ROBOT_LEG_RIGHT_FRONT,
    };
    return order[slot & 3];
}

static int static_crawl_slot(float global_phase);

static float static_crawl_leg_phase(robot_leg_t leg, float global_phase)
{
    // One complete phase is a four-leg sequence. A particular leg's cycle
    // starts at its own quarter of that sequence, not at every quarter.
    float local = global_phase - 0.25f * (float)static_crawl_slot_for_leg(leg);
    local -= floorf(local);
    return local;
}

static robot_vec3_t static_crawl_unshifted_foot(robot_leg_t leg, float global_phase,
                                                  float forward, float lateral, float stride)
{
    const float cycle_phase = static_crawl_leg_phase(leg, global_phase);
    const float stroke = robot_static_stance_stroke_fraction_timed(
        cycle_phase, static_crawl_preload_fraction()) * stride;
    robot_vec3_t foot = standing_foot_for_leg(leg);
    foot.x += stroke * forward;
    foot.z += stroke * lateral;
    return foot;
}

static bool static_crawl_safe_body_translation(robot_leg_t excluded,
                                                    float global_phase,
                                                    float forward, float lateral,
                                                    float stride,
                                                    robot_planar_point_t projected_com,
                                                    robot_vec3_t *out_translation)
{
    if (!out_translation) return false;
    *out_translation = (robot_vec3_t){0};
    robot_planar_point_t support[3] = {0};
    int count = 0;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        if (leg == excluded) continue;
        // These are the three *current* planted-foot locations before the
        // torso is translated. Including the stance stroke keeps the support
        // polygon valid while Aura is actually moving, not only at neutral.
        const robot_vec3_t foot = static_crawl_unshifted_foot((robot_leg_t)leg,
                                                               global_phase,
                                                               forward, lateral, stride);
        support[count++] = (robot_planar_point_t){.x = foot.x, .z = foot.z};
    }

    // `robot_static_support_plan` plans the world location of the *actual*
    // projected CoM.  The gait frame origin can differ from it because the
    // battery and electronics are not necessarily centered.  Convert the
    // desired CoM location to the required torso-frame translation below.
    // The caller supplies the CoM after the current IMU body pose. This
    // keeps the static support triangle and attitude correction in one plan.
    robot_static_support_plan_t support_plan = {0};
    if (count != 3 || !robot_static_support_plan(support, projected_com,
                                                  (float)static_support_margin_mm,
                                                  &support_plan))
        return false;
    *out_translation = (robot_vec3_t){
        .x = support_plan.com_target.x - projected_com.x,
        .z = support_plan.com_target.z - projected_com.z,
    };
    return true;
}

static void static_crawl_prepare_slot_plan(float global_phase,
                                          robot_planar_point_t projected_com)
{
    const int slot = static_crawl_slot(global_phase);
    if (slot != static_crawl_plan_slot) {
        // A slot begins at a true phase boundary. Preserve the previous endpoint
        // as the next preload origin so a new support triangle never introduces a
        // discontinuous torso target.
        static_crawl_from_body_shift = static_crawl_plan_slot < 0
            ? (robot_vec3_t){0} : static_crawl_to_body_shift;
        static_crawl_plan_slot = slot;
    }

    // Re-evaluate the destination every 20 ms with the IMU-projected CoM.
    // The foot path remains continuous, while a new roll/pitch correction
    // cannot leave the physical CoM outside this slot's support triangle.
    static_crawl_support_safe = static_crawl_safe_body_translation(
        static_crawl_leg_for_slot(slot), global_phase,
        static_crawl_forward, static_crawl_lateral, static_crawl_stride_mm,
        projected_com, &static_crawl_to_body_shift);
}

static static_crawl_plan_t static_crawl_plan(float global_phase)
{
    const int slot = static_crawl_slot(global_phase);
    const float local = global_phase * 4.0f - floorf(global_phase * 4.0f);
    const robot_static_step_profile_t step = robot_static_step_profile_timed(
        local, static_crawl_preload_fraction());
    static_crawl_plan_t plan = {
        .enabled = true,
        .support_safe = static_crawl_support_safe && static_crawl_plan_slot == slot,
        .swing_leg = static_crawl_leg_for_slot(slot),
        .body_shift = {
            .x = static_crawl_from_body_shift.x +
                 (static_crawl_to_body_shift.x - static_crawl_from_body_shift.x) *
                 step.body_shift_fraction,
            .z = static_crawl_from_body_shift.z +
                 (static_crawl_to_body_shift.z - static_crawl_from_body_shift.z) *
                 step.body_shift_fraction,
        },
        .forward = static_crawl_forward,
        .lateral = static_crawl_lateral,
    };
    return plan;
}

static bool static_crawl_leg_is_swing(robot_leg_t leg, float global_phase)
{
    const float cycle_phase = static_crawl_leg_phase(leg, global_phase);
    return robot_static_leg_airborne_timed(cycle_phase, static_crawl_preload_fraction());
}

static robot_vec3_t static_crawl_foot_target(robot_leg_t leg, float global_phase,
                                               const static_crawl_plan_t *plan, float stride)
{
    if (!plan || !plan->support_safe) return standing_foot_for_leg(leg);
    const float cycle_phase = static_crawl_leg_phase(leg, global_phase);
    const float preload = static_crawl_preload_fraction();
    const robot_static_step_profile_t step = robot_static_step_profile_timed(
        cycle_phase * 4.0f, preload);
    const float stroke = robot_static_stance_stroke_fraction_timed(
        cycle_phase, preload) * stride;
    const robot_vec3_t neutral = standing_foot_for_leg(leg);
    robot_vec3_t target = neutral;
    // The support transfer remains explicit, but the airborne portion is now
    // the same MIT-style p0 to pf cubic Bezier as every other gait. It has no
    // vertical/horizontal corner to fight the rigid IMU torso pose.
    if (leg == plan->swing_leg && step.airborne) {
        const float slot_phase = cycle_phase * 4.0f;
        const float swing_phase = clampf((slot_phase - preload) / (1.0f - preload),
                                         0.0f, 1.0f);
        const robot_vec3_t liftoff = {
            .x = neutral.x - 0.5f * stride * plan->forward,
            .y = 0.0f,
            .z = neutral.z - 0.5f * stride * plan->lateral,
        };
        const robot_vec3_t touchdown = {
            .x = neutral.x + 0.5f * stride * plan->forward,
            .y = 0.0f,
            .z = neutral.z + 0.5f * stride * plan->lateral,
        };
        target = robot_locomotion_swing_bezier(
            liftoff, touchdown, swing_phase,
            (float)gait_profiles[state.active_gait].step_height_mm);
    } else {
        target.x += stroke * plan->forward;
        target.z += stroke * plan->lateral;
    }
    // This is a continuous, time-scheduled world foot path. Contact sensing
    // is diagnostic only: it must not freeze a foot or replace this trajectory.
    return target;
}

static int static_crawl_slot(float global_phase)
{
    return ((int)floorf(global_phase * 4.0f)) & 3;
}

static void static_crawl_latch_input(const gait_input_t *input, float global_phase,
                                     float stride)
{
    if (!input) return;
    static_crawl_input_slot = static_crawl_slot(global_phase);
    static_crawl_forward = input->forward;
    static_crawl_lateral = input->lateral;
    static_crawl_stride_mm = stride;
}

static bool static_crawl_is_settled(float reference_error, bool has_feedback,
                                    float feedback_error)
{
    return reference_error <= ROBOT_STATIC_REFERENCE_SETTLED_RAD &&
           (!has_feedback || feedback_error <= ROBOT_STATIC_FEEDBACK_SETTLED_RAD);
}

static void static_crawl_advance_phase(float phase_rate, const gait_input_t *input,
                                       float stride, float reference_error, bool has_feedback,
                                       float feedback_error)
{
    if (static_crawl_waiting_for_settle) {
        if (static_crawl_is_settled(reference_error, has_feedback, feedback_error)) {
            if (static_crawl_settled_ticks < UINT8_MAX) ++static_crawl_settled_ticks;
            if (static_crawl_settled_ticks >= ROBOT_STATIC_SETTLED_TICKS) {
                static_crawl_waiting_for_settle = false;
                static_crawl_settled_ticks = 0;
            }
        } else {
            static_crawl_settled_ticks = 0;
        }
        return;
    }

    const float advance = gait_hz(state.active_gait) * phase_rate *
                          ((float)ROBOT_GAIT_PERIOD_MS / 1000.0f);
    const float slot_phase = phase * 4.0f - floorf(phase * 4.0f);
    const float transition = robot_static_next_transition_timed(
        slot_phase, static_crawl_preload_fraction());
    const float distance_to_transition = (transition - slot_phase) * 0.25f;
    if (advance >= distance_to_transition) {
        phase += fmaxf(0.0f, distance_to_transition);
        if (phase >= 1.0f) phase -= 1.0f;
        static_crawl_waiting_for_settle = true;
        static_crawl_settled_ticks = 0;
    } else {
        phase += advance;
        if (phase >= 1.0f) phase -= 1.0f;
    }

    const int slot = static_crawl_slot(phase);
    if (slot != static_crawl_input_slot) static_crawl_latch_input(input, phase, stride);
}

static float smootherstep(float value)
{
    const float u = clampf(value, 0.0f, 1.0f);
    return u * u * u * (u * (u * 6.0f - 15.0f) + 10.0f);
}

static bool tripod_test_is_active(void)
{
    return tripod_state != ROBOT_TRIPOD_IDLE;
}

static void tripod_set_state(robot_tripod_state_t next)
{
    tripod_state = next;
    tripod_stage_progress = 0.0f;
    tripod_settled_ticks = 0;
    ESP_LOGI("robot_gait", "tripod test state=%d, unsupported leg=%s",
             (int)next, robot_leg_name(tripod_test_leg));
}

static robot_vec3_t tripod_test_target(robot_leg_t leg)
{
    robot_vec3_t support_shift = {0};
    const bool support_safe = static_crawl_safe_body_translation(tripod_test_leg,
                                                                  0.0f, 0.0f, 0.0f, 0.0f,
                                                                  (robot_planar_point_t){
                                                                      .x = (float)static_com_forward_mm,
                                                                      .z = (float)static_com_left_mm,
                                                                  },
                                                                  &support_shift);
    if (!support_safe) return standing_foot_for_leg(leg);
    float shift_fraction = 0.0f;
    float lift_fraction = 0.0f;
    switch (tripod_state) {
    case ROBOT_TRIPOD_SHIFT:
        shift_fraction = smootherstep(tripod_stage_progress);
        break;
    case ROBOT_TRIPOD_LIFT:
        shift_fraction = 1.0f;
        lift_fraction = smootherstep(tripod_stage_progress);
        break;
    case ROBOT_TRIPOD_HOLD:
        shift_fraction = 1.0f;
        lift_fraction = 1.0f;
        break;
    case ROBOT_TRIPOD_LOWER:
        shift_fraction = 1.0f;
        lift_fraction = 1.0f - smootherstep(tripod_stage_progress);
        break;
    case ROBOT_TRIPOD_RETURN:
        shift_fraction = 1.0f - smootherstep(tripod_stage_progress);
        break;
    case ROBOT_TRIPOD_IDLE:
    default:
        break;
    }
    robot_vec3_t target = standing_foot_for_leg(leg);
    target.x -= support_shift.x * shift_fraction;
    target.z -= support_shift.z * shift_fraction;
    if (leg == tripod_test_leg) target.y = ROBOT_TRIPOD_TEST_LIFT_MM * lift_fraction;
    return target;
}

static float tripod_test_body_lift_mm(void)
{
    switch (tripod_state) {
    case ROBOT_TRIPOD_LIFT:
        return ROBOT_TRIPOD_BODY_LIFT_MM * smootherstep(tripod_stage_progress);
    case ROBOT_TRIPOD_HOLD:
    case ROBOT_TRIPOD_LOWER:
        return ROBOT_TRIPOD_BODY_LIFT_MM;
    case ROBOT_TRIPOD_RETURN:
        return ROBOT_TRIPOD_BODY_LIFT_MM * (1.0f - smootherstep(tripod_stage_progress));
    case ROBOT_TRIPOD_IDLE:
    case ROBOT_TRIPOD_SHIFT:
    default:
        return 0.0f;
    }
}

static void tripod_test_tick(float reference_error)
{
    if (!tripod_test_is_active()) return;
    if (tripod_state == ROBOT_TRIPOD_HOLD) {
        if (tripod_exit_requested) {
            tripod_exit_requested = false;
            tripod_set_state(ROBOT_TRIPOD_LOWER);
        }
        return;
    }

    tripod_stage_progress = fminf(1.0f, tripod_stage_progress +
                                  ROBOT_TRIPOD_STAGE_RATE_HZ *
                                  ((float)ROBOT_GAIT_PERIOD_MS / 1000.0f));
    if (tripod_stage_progress < 1.0f) return;

    // Feedback samples are addressed and asynchronous. Waiting for the
    // maximum error across all twelve samples can otherwise freeze a balance
    // test after its commanded trajectory has already settled.
    if (reference_error > ROBOT_STATIC_REFERENCE_SETTLED_RAD) {
        tripod_settled_ticks = 0;
        return;
    }
    if (tripod_settled_ticks < UINT8_MAX) ++tripod_settled_ticks;
    if (tripod_settled_ticks < ROBOT_STATIC_SETTLED_TICKS) return;

    switch (tripod_state) {
    case ROBOT_TRIPOD_SHIFT:
        tripod_set_state(ROBOT_TRIPOD_LIFT);
        break;
    case ROBOT_TRIPOD_LIFT:
        if (tripod_exit_requested) {
            tripod_exit_requested = false;
            tripod_set_state(ROBOT_TRIPOD_LOWER);
        } else {
            tripod_set_state(ROBOT_TRIPOD_HOLD);
        }
        break;
    case ROBOT_TRIPOD_LOWER:
        tripod_set_state(ROBOT_TRIPOD_RETURN);
        break;
    case ROBOT_TRIPOD_RETURN:
        tripod_set_state(ROBOT_TRIPOD_IDLE);
        break;
    case ROBOT_TRIPOD_HOLD:
    case ROBOT_TRIPOD_IDLE:
    default:
        break;
    }
}

static bool set_targets_for_leg(robot_leg_t leg, robot_vec3_t target, float body_height)
{
    float angles[ROBOT_AXIS_COUNT] = {0};
    if (!robot_kinematics_inverse(&geometry, leg, target, body_height, angles)) return false;
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis)
        state.target_cdeg[leg][axis] = (int16_t)lroundf(angles[axis] * 18000.0f / ROBOT_PI);
    return true;
}

static void initialize_trajectories(const robot_model_t *model)
{
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
            const robot_axis_state_t *servo = &model->axis[leg][axis];
            const float initial = servo->present ? servo->measured_radians : servo->target_radians;
            joint_trajectory_reset(&joint_trajectory[leg][axis], initial);
        }
    }
    trajectory_initialized = true;
}

static void tracking_errors(const robot_model_t *model, float *reference_error,
                            bool *has_feedback, float *feedback_error)
{
    *reference_error = 0.0f;
    *feedback_error = 0.0f;
    *has_feedback = false;
    if (!trajectory_initialized) return;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
            const float reference = (float)state.target_cdeg[leg][axis] * ROBOT_PI / 18000.0f;
            *reference_error = fmaxf(*reference_error,
                                     fabsf(reference - joint_trajectory[leg][axis].position));
            const robot_axis_state_t *servo = &model->axis[leg][axis];
            if (servo->config.servo_id != ROBOT_SERVO_ID_UNASSIGNED && servo->present) {
                *has_feedback = true;
                *feedback_error = fmaxf(*feedback_error,
                                        fabsf(servo->target_radians - servo->measured_radians));
            }
        }
    }
}

static robot_vec3_t foot_target_for_leg(robot_leg_t leg, float raw_phase, float duty,
                                         bool active, bool stepping_in_place, bool jumping,
                                         bool spin, bool turning, const gait_input_t *input,
                                         float stride)
{
    (void)turning; // No threshold/special arc formula in the continuous twist.
    (void)duty; // The authoritative profile is also consumed by the body plan.
    const robot_locomotion_profile_t *profile = gait_profile(state.active_gait);
    if (!active || jumping || !profile) return standing_foot_for_leg(leg);
    const robot_body_trajectory_request_t request = {
        .gait = (uint8_t)state.active_gait, .profile = *profile, .geometry = geometry,
        .lateral_stance_mm = lateral_stance_mm, .forward = input->forward,
        .lateral = input->lateral, .turn = input->turn,
        .stride_mm = stride, .stepping_in_place = stepping_in_place, .spin = spin,
        .excluded_leg = tripod_walk_excluded,
    };
    return robot_body_trajectory_foot(&request, leg,
        raw_phase - leg_phase_offset(leg, state.active_gait));
}

static bool motion_tuning_valid(const robot_gait_motion_tuning_t *tuning)
{
    return tuning &&
           tuning->moving_attitude_gain_per_mille >=
               ROBOT_MOTION_TUNING_MIN_ATTITUDE_GAIN_PER_MILLE &&
           tuning->moving_attitude_gain_per_mille <=
               ROBOT_MOTION_TUNING_MAX_ATTITUDE_GAIN_PER_MILLE &&
           tuning->com_max_velocity_mm_s >= ROBOT_MOTION_TUNING_MIN_COM_VELOCITY_MM_S &&
           tuning->com_max_velocity_mm_s <= ROBOT_MOTION_TUNING_MAX_COM_VELOCITY_MM_S &&
           tuning->com_max_acceleration_mm_s2 >=
               ROBOT_MOTION_TUNING_MIN_COM_ACCELERATION_MM_S2 &&
           tuning->com_max_acceleration_mm_s2 <=
               ROBOT_MOTION_TUNING_MAX_COM_ACCELERATION_MM_S2;
}

static bool legacy_profile_defaults(const robot_locomotion_profile_t profiles[ROBOT_GAIT_CLIMB + 1])
{
    // Upgrade only an untouched set from the former release. A user-created
    // profile is commissioning data and must never be overwritten on boot.
    return profiles &&
           profiles[ROBOT_GAIT_TROT].stride_mm == 70 &&
           profiles[ROBOT_GAIT_TROT].step_height_mm == 42 &&
           profiles[ROBOT_GAIT_TROT].frequency_centi_hz == 85 &&
           profiles[ROBOT_GAIT_TROT].duty_percent == 58 &&
           profiles[ROBOT_GAIT_CRAWL].stride_mm == 42 &&
           profiles[ROBOT_GAIT_CRAWL].step_height_mm == 18 &&
           profiles[ROBOT_GAIT_CRAWL].frequency_centi_hz == 45 &&
           profiles[ROBOT_GAIT_CRAWL].duty_percent == 82 &&
           profiles[ROBOT_GAIT_RUN].stride_mm == 80 &&
           profiles[ROBOT_GAIT_RUN].step_height_mm == 52 &&
           profiles[ROBOT_GAIT_RUN].frequency_centi_hz == 120 &&
           profiles[ROBOT_GAIT_RUN].duty_percent == 52 &&
           profiles[ROBOT_GAIT_CLIMB].stride_mm == 45 &&
           profiles[ROBOT_GAIT_CLIMB].step_height_mm == 65 &&
           profiles[ROBOT_GAIT_CLIMB].frequency_centi_hz == 45 &&
           profiles[ROBOT_GAIT_CLIMB].duty_percent == 82;
}

// VPSP supplies a gait-phase reference; the reduced body planner projects
// it onto scheduled support and solves c'' = g/h (c - p) over the full cycle.
// IK receives c(phase) minus the configured, rotated CoM offset. This is a
// feed-forward body trajectory plus the existing IMU orientation loop, not
// the force-controlled MPC of Cheetah. See docs/locomotion-model.md.
static robot_vec3_t dynamic_balance_desired(float global_phase, bool active,
                                             bool stepping_in_place, bool spin,
                                             bool turning, const gait_input_t *input,
                                             float stride,
                                             robot_planar_point_t projected_com, float body_height)
{
    (void)turning; // The common twist handles straight motion and turns.
    state.body_preview_active = false;
    if (!active || !input) return (robot_vec3_t){0};

    const robot_locomotion_profile_t *profile = gait_profile(state.active_gait);
    const robot_body_trajectory_request_t request = {
        .gait = (uint8_t)state.active_gait, .profile = *profile, .geometry = geometry,
        .com_height_mm = body_height + ROBOT_BODY_COM_VERTICAL_OFFSET_MM,
        .lateral_stance_mm = lateral_stance_mm, .forward = input->forward,
        .lateral = input->lateral, .turn = input->turn,
        .stride_mm = stride, .stepping_in_place = stepping_in_place, .spin = spin,
        .excluded_leg = tripod_walk_excluded,
        .body_height_mm = body_height,
        .com_offset_x_mm = projected_com.x, .com_offset_z_mm = projected_com.z,
        .joint_velocity_limit = ROBOT_JOINT_MAX_VELOCITY_RAD_S,
        .joint_acceleration_limit = ROBOT_JOINT_MAX_ACCELERATION_RAD_S2,
    };
    // Unlike p_VPSP itself, this periodic CoM reference includes the
    // acceleration required to stay supported on the scheduled diagonal.
    // Feet and torso are evaluated at the SAME phase; no causal body lag.
    if (robot_body_trajectory_build(&body_trajectory, &request)) {
        const robot_body_trajectory_sample_t body =
            robot_body_trajectory_sample(&body_trajectory, global_phase);
        state.body_preview_active = true;
        return (robot_vec3_t){.x = body.position_mm.x - projected_com.x,
                              .z = body.position_mm.z - projected_com.z};
    }
    // The caller holds the last complete frame when no valid path exists.
    return (robot_vec3_t){0};
}

static void dynamic_balance_step(robot_vec3_t desired, bool enabled)
{
    // The preview already supplies a continuous position, velocity and
    // acceleration. An extra causal filter would put the body behind the
    // shared contact schedule again.
    dynamic_balance_shift = enabled ? desired : (robot_vec3_t){0};
}

static void update_attitude_reference_capture(bool armed, bool operator_idle,
                                              float reference_error, bool has_feedback,
                                              float feedback_error)
{
    if (!armed || !attitude_control.enabled) {
        attitude_control.reference_pending = false;
        attitude_control.reference_settled_ticks = 0;
        return;
    }
    if (!attitude_control.reference_pending) return;

    // Do not define "level" while the operator is already requesting a
    // step, or while the trajectory / actual joints are still moving toward
    // their torque-on neutral targets. This makes the reference independent
    // of the loose-servo pose that existed before Create.
    const bool settled = operator_idle && trajectory_initialized &&
        reference_error <= ROBOT_STATIC_REFERENCE_SETTLED_RAD &&
        (!has_feedback || feedback_error <= ROBOT_STATIC_FEEDBACK_SETTLED_RAD);
    if (!settled) {
        attitude_control.reference_settled_ticks = 0;
        return;
    }
    if (attitude_control.reference_settled_ticks < UINT8_MAX)
        ++attitude_control.reference_settled_ticks;
    if (attitude_control.reference_settled_ticks < ROBOT_STATIC_SETTLED_TICKS) return;

    mpu6050_attitude_t measured = {0};
    mpu6050_get_attitude(&measured);
    const int64_t now_us = esp_timer_get_time();
    const bool fresh = measured.valid && measured.sample_time_us > 0 &&
        now_us >= measured.sample_time_us &&
        now_us - measured.sample_time_us <= ROBOT_ATTITUDE_FRESH_US;
    if (!fresh) return;
    attitude_control.reference_roll_radians = measured.roll_radians;
    attitude_control.reference_pitch_radians = measured.pitch_radians;
    attitude_control.reference_valid = true;
    attitude_control.reference_pending = false;
    attitude_control.reference_settled_ticks = 0;
    attitude_control.correction_roll_radians = 0.0f;
    attitude_control.correction_pitch_radians = 0.0f;
    joint_trajectory_reset(&attitude_trajectory[0], 0.0f);
    joint_trajectory_reset(&attitude_trajectory[1], 0.0f);
    ESP_LOGI("robot_gait", "IMU attitude reference captured after neutral stance settled");
}

static void update_attitude_controller(bool armed, bool moving)
{
    mpu6050_attitude_t measured = {0};
    mpu6050_get_attitude(&measured);
    const int64_t now_us = esp_timer_get_time();
    const bool fresh = measured.valid && measured.sample_time_us > 0 &&
                       now_us >= measured.sample_time_us &&
                       now_us - measured.sample_time_us <= ROBOT_ATTITUDE_FRESH_US;
    balance_imu = measured;
    state.attitude_valid = fresh;
    state.attitude_control_enabled = attitude_control.enabled;
    state.attitude_reference_valid = attitude_control.reference_valid;
    state.attitude_reference_pending = attitude_control.reference_pending;
    state.attitude_roll_cdeg = fresh ? radians_to_cdeg_signed(measured.roll_radians) : 0;
    state.attitude_pitch_cdeg = fresh ? radians_to_cdeg_signed(measured.pitch_radians) : 0;
    state.attitude_reference_roll_cdeg = attitude_control.reference_valid
        ? radians_to_cdeg_signed(attitude_control.reference_roll_radians) : 0;
    state.attitude_reference_pitch_cdeg = attitude_control.reference_valid
        ? radians_to_cdeg_signed(attitude_control.reference_pitch_radians) : 0;

    const bool usable = armed && attitude_control.enabled &&
                        attitude_control.reference_valid && fresh;
    if (!usable) {
        attitude_control.active = false;
        attitude_control.correction_roll_radians = 0.0f;
        attitude_control.correction_pitch_radians = 0.0f;
        joint_trajectory_reset(&attitude_trajectory[0], 0.0f);
        joint_trajectory_reset(&attitude_trajectory[1], 0.0f);
    } else {
        const float roll_error = wrap_radians(attitude_control.reference_roll_radians -
                                              measured.roll_radians);
        const float pitch_error = wrap_radians(attitude_control.reference_pitch_radians -
                                               measured.pitch_radians);
        // Keep a bounded corrective command even beyond the nominal
        // envelope. Dropping it suddenly to zero during a fall was a kick.
        {
            const float correction_limit = (float)attitude_max_correction_cdeg *
                                           ROBOT_ATTITUDE_CDEG_TO_RAD;
            const float proportional_gain = moving
                ? (float)motion_tuning.moving_attitude_gain_per_mille / 1000.0f
                : ROBOT_ATTITUDE_STANDING_PROPORTIONAL_GAIN;
            // Standard angular PD; 80 ms derivative time is a commissioning
            // default for these position servos, not a gain copied from MIT's
            // torque controller. The known-good standing controller is intact.
            const float kd = moving ? 0.08f : 0.0f;
            const float desired_roll = robot_balance_posture_target(roll_error,
                measured.roll_rate_rad_s, proportional_gain, kd, correction_limit);
            const float desired_pitch = robot_balance_posture_target(pitch_error,
                measured.pitch_rate_rad_s, proportional_gain, kd, correction_limit);
            const joint_trajectory_limits_t posture_limits = {
                .maximum_velocity = moving ? ROBOT_ATTITUDE_MOVING_SLEW_RAD_S
                                            : ROBOT_ATTITUDE_STANDING_SLEW_RAD_S,
                .maximum_acceleration = moving ? ROBOT_ATTITUDE_MOVING_ACCEL_RAD_S2
                                                : ROBOT_ATTITUDE_STANDING_ACCEL_RAD_S2,
            };
            attitude_control.correction_roll_radians = joint_trajectory_step(
                &attitude_trajectory[0], desired_roll,
                (float)ROBOT_GAIT_PERIOD_MS / 1000.0f, posture_limits);
            attitude_control.correction_pitch_radians = joint_trajectory_step(
                &attitude_trajectory[1], desired_pitch,
                (float)ROBOT_GAIT_PERIOD_MS / 1000.0f, posture_limits);
            attitude_control.active = true;
        }
    }
    state.attitude_control_active = attitude_control.active;
    state.attitude_correction_roll_cdeg = radians_to_cdeg_signed(
        attitude_control.correction_roll_radians);
    state.attitude_correction_pitch_cdeg = radians_to_cdeg_signed(
        attitude_control.correction_pitch_radians);
    state.attitude_max_correction_cdeg = attitude_max_correction_cdeg;
}

typedef struct {
    // Translation and attitude describe one desired rigid torso pose in the
    // level-world frame. The gait creates world foot paths; this transform
    // expresses each of those paths in the moving torso frame for one IK
    // solve. It is the only place where gait and balance are combined.
    float x_mm;
    float z_mm;
    float roll_radians;
    float pitch_radians;
} robot_body_pose_t;

static robot_vec3_t body_pose_foot_target(robot_vec3_t world_target, float body_height,
                                          robot_body_pose_t pose)
{
    const float c_pitch = cosf(pose.pitch_radians), s_pitch = sinf(pose.pitch_radians);
    const float c_roll = cosf(pose.roll_radians), s_roll = sinf(pose.roll_radians);
    // `world_target` is the gait's continuous path on the level ground
    // frame. First remove the complete horizontal torso shift, then apply a
    // single inverse roll/pitch about the torso. Doing these together avoids
    // the old non-commuting "balance shift, then IMU correction" commands.
    const robot_vec3_t relative = {
        .x = world_target.x - pose.x_mm,
        .y = world_target.y - body_height,
        .z = world_target.z - pose.z_mm,
    };
    // R^-1 = Rx(-roll) * Rz(-pitch) in Aura's X-forward/Y-up/Z-left frame.
    const float x1 = c_pitch * relative.x + s_pitch * relative.y;
    const float y1 = -s_pitch * relative.x + c_pitch * relative.y;
    const float z1 = relative.z;
    return (robot_vec3_t){
        .x = x1,
        .y = c_roll * y1 + s_roll * z1 + body_height,
        .z = -s_roll * y1 + c_roll * z1,
    };
}

static bool gait_leg_scheduled_swing(robot_leg_t leg, bool motion_active, bool static_crawl)
{
    if (!motion_active) return false;
    if (state.active_gait == ROBOT_GAIT_TRIPOD)
        return robot_locomotion_tripod_phase(leg, tripod_walk_excluded, phase,
                                             &tripod_walk_profile).scheduled_swing;
    if (static_crawl) return static_crawl_leg_is_swing(leg, phase);
    const float raw_phase = fmodf(phase + leg_phase_offset(leg, state.active_gait), 1.0f);
    return raw_phase >= gait_duty(state.active_gait);
}

static bool estimate_swing_body(const robot_model_t *feedback,
                                const robot_vec3_t planned_feet[4],
                                const bool swing[4], robot_balance_pose_t commanded,
                                robot_balance_pose_t *measured)
{
    if (!state.attitude_valid || !attitude_control.active || !attitude_control.reference_valid) return false;
    const float roll = -wrap_radians(balance_imu.roll_radians - attitude_control.reference_roll_radians);
    const float pitch = -wrap_radians(balance_imu.pitch_radians - attitude_control.reference_pitch_radians);
    if (fabsf(roll) > ROBOT_ATTITUDE_MAX_ERROR_RAD || fabsf(pitch) > ROBOT_ATTITUDE_MAX_ERROR_RAD) return false;
    robot_vec3_t feet[4] = {0}; bool support[4] = {0};
    const int64_t now = esp_timer_get_time();
    for (int leg=0; leg<4; ++leg) {
        if (swing[leg]) continue;
        float q[3]; bool fresh = true;
        for (int axis=0; axis<3; ++axis) {
            const robot_axis_state_t *servo = &feedback->axis[leg][axis];
            const int64_t age = now - servo->feedback_time_us;
            fresh &= servo->present && servo->feedback_time_us > 0 && age >= 0 && age < 120000;
            // Addressed TTL reads have different timestamps. Align to this
            // tick using measured speed, bounded to one normal bus sweep.
            const float dt = clampf((float)age * 1e-6f, 0, 0.060f);
            q[axis] = servo->measured_radians + dt * servo->measured_speed_raw *
                servo->config.direction * (2.0f * ROBOT_PI / 4096.0f);
        }
        support[leg] = fresh;
        feet[leg] = robot_kinematics_forward(&geometry, (robot_leg_t)leg, q, 0);
    }
    if (!robot_balance_estimate_pose(feet, planned_feet, support, roll, pitch, measured)) return false;
    // A missed/slipping contact can invalidate the kinematic translation.
    // Reject it rather than allowing a bad base estimate to throw a swing.
    return measured->position.y > 80 && measured->position.y < 360 &&
        fabsf(measured->position.y-commanded.position.y) < 70 &&
        hypotf(measured->position.x-commanded.position.x,
               measured->position.z-commanded.position.z) < 80;
}

static bool gait_leg_in_touchdown_window(robot_leg_t leg, bool motion_active,
                                         bool static_crawl)
{
    if (!gait_leg_scheduled_swing(leg, motion_active, static_crawl)) return false;
    if (state.active_gait == ROBOT_GAIT_TRIPOD) {
        if (leg == tripod_walk_excluded) return false;
        return robot_locomotion_tripod_phase(leg, tripod_walk_excluded, phase,
                                             &tripod_walk_profile).swing_phase >= 0.70f;
    }
    if (static_crawl) {
        const float slot_phase = static_crawl_leg_phase(leg, phase) * 4.0f;
        return slot_phase >= ROBOT_STATIC_CONTACT_TOUCHDOWN_START;
    }
    const float raw_phase = fmodf(phase + leg_phase_offset(leg, state.active_gait), 1.0f);
    const float duty = gait_duty(state.active_gait);
    return raw_phase >= duty + (1.0f - duty) * 0.70f;
}

static void update_contact_observers(const robot_model_t *model, bool armed, bool motion_active,
                                     bool static_crawl, bool jumping, bool tripod_active)
{
    app_state_snapshot_t imu = {0};
    app_state_get(&imu);
    const float acceleration_norm = sqrtf(imu.imu.accel_g[0] * imu.imu.accel_g[0] +
                                          imu.imu.accel_g[1] * imu.imu.accel_g[1] +
                                          imu.imu.accel_g[2] * imu.imu.accel_g[2]);
    const float specific_force_error = imu.imu_error == ESP_OK && imu.imu.sample_count
        ? fabsf(acceleration_norm - 1.0f) : 0.0f;

    state.contact_feedback_available = false;
    state.expected_contact_mask = 0;
    state.detected_contact_mask = 0;
    state.early_touchdown_mask = 0;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        const robot_axis_state_t *hip = &model->axis[leg][ROBOT_AXIS_HIP];
        const robot_axis_state_t *knee = &model->axis[leg][ROBOT_AXIS_KNEE];
        const robot_axis_state_t *abduction = &model->axis[leg][ROBOT_AXIS_ABDUCTION];
        const bool feedback_valid = hip->present && knee->present && abduction->present;
        const bool scheduled_swing = gait_leg_scheduled_swing((robot_leg_t)leg, motion_active,
                                                               static_crawl);
        const bool expected_stance = !jumping && !scheduled_swing &&
            (!tripod_active || leg != (int)tripod_test_leg);
        const bool touchdown_window = gait_leg_in_touchdown_window((robot_leg_t)leg,
                                                                    motion_active, static_crawl);
        const robot_contact_input_t input = {
            .armed = armed,
            .expected_stance = expected_stance,
            .touchdown_window = touchdown_window,
            .feedback_valid = feedback_valid,
            .hip_load_raw = hip->measured_load_raw,
            .knee_load_raw = knee->measured_load_raw,
            .hip_current_raw = hip->measured_current_raw,
            .knee_current_raw = knee->measured_current_raw,
            .hip_tracking_error_radians = hip->target_radians - hip->measured_radians,
            .knee_tracking_error_radians = knee->target_radians - knee->measured_radians,
            .imu_specific_force_error_g = specific_force_error,
        };
        robot_contact_update(&contact_observer[leg], &input);
        state.contact_confidence_percent[leg] = contact_observer[leg].confidence_percent;
        if (feedback_valid) state.contact_feedback_available = true;
        if (expected_stance) state.expected_contact_mask |= 1U << leg;
        if (contact_observer[leg].contact) state.detected_contact_mask |= 1U << leg;
        if (contact_observer[leg].early_touchdown) state.early_touchdown_mask |= 1U << leg;
    }
}

static void update_odometry(const robot_model_t *model, bool armed, bool active,
                            bool jumping, bool solve_yaw, float body_height)
{
    if (!armed) {
        robot_odometry_reset(&odometry);
        state.odometry_valid = false;
        state.odometry_x_mm = 0.0f;
        state.odometry_z_mm = 0.0f;
        state.odometry_yaw_radians = 0.0f;
        return;
    }
    robot_vec3_t feet[ROBOT_LEG_COUNT] = {0};
    bool contact[ROBOT_LEG_COUNT] = {false};
    bool any_feedback = false;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        float angles[ROBOT_AXIS_COUNT] = {0};
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
            const robot_axis_state_t *servo = &model->axis[leg][axis];
            angles[axis] = servo->present ? servo->measured_radians : servo->target_radians;
            any_feedback = any_feedback || servo->present;
        }
        feet[leg] = robot_kinematics_forward(&geometry, (robot_leg_t)leg, angles, body_height);
        const float leg_phase = fmodf(phase + leg_phase_offset(leg, state.active_gait), 1.0f);
        const bool static_crawl = state.active_gait == ROBOT_GAIT_CRAWL && active && !solve_yaw;
        contact[leg] = !jumping && (!active ||
            (state.active_gait == ROBOT_GAIT_TRIPOD
                ? robot_locomotion_tripod_phase((robot_leg_t)leg, tripod_walk_excluded,
                    phase, &tripod_walk_profile).support_contact
                : static_crawl ? !static_crawl_leg_is_swing((robot_leg_t)leg, phase)
                              : leg_phase < gait_duty(state.active_gait)));
    }
    if (!any_feedback) return;
    robot_odometry_update(&odometry, feet, contact, solve_yaw);
    state.odometry_valid = odometry.valid;
    state.odometry_x_mm = odometry.x_mm;
    state.odometry_z_mm = odometry.z_mm;
    state.odometry_yaw_radians = odometry.yaw_radians;
}

static void initialize_calibration_trajectory(robot_leg_t leg, const robot_model_t *model)
{
    if (!model || leg < 0 || leg >= ROBOT_LEG_COUNT) return;
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        const robot_axis_state_t *servo = &model->axis[leg][axis];
        joint_trajectory_reset(&joint_trajectory[leg][axis],
                               servo->present ? servo->measured_radians : servo->target_radians);
    }
    calibration_trajectory_initialized = true;
}

static void calibration_tracking_errors(robot_leg_t leg, const robot_model_t *model,
                                        float *reference_error, bool *has_feedback,
                                        float *feedback_error)
{
    *reference_error = 0.0f;
    *has_feedback = false;
    *feedback_error = 0.0f;
    if (!model || !calibration_trajectory_initialized) return;
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        const float reference = (float)state.target_cdeg[leg][axis] * ROBOT_PI / 18000.0f;
        *reference_error = fmaxf(*reference_error,
                                 fabsf(reference - joint_trajectory[leg][axis].position));
        const robot_axis_state_t *servo = &model->axis[leg][axis];
        if (servo->present) {
            *has_feedback = true;
            *feedback_error = fmaxf(*feedback_error,
                                    fabsf(servo->target_radians - servo->measured_radians));
        }
    }
}

static void calibration_test_tick(const gait_input_t *input)
{
    // This is the normal Trot pipeline reduced to one physical leg:
    // foot path -> inverse kinematics -> bounded trajectory -> sync write.
    // It therefore does not use the former independent direct joint test.
    const float magnitude = fminf(1.0f, sqrtf(input->forward * input->forward +
                                               input->lateral * input->lateral));
    const bool active = input->connected && magnitude > 0.02f;
    const bool left = calibration_test.leg == ROBOT_LEG_LEFT_FRONT ||
                      calibration_test.leg == ROBOT_LEG_LEFT_REAR;
    const int16_t knee_reference = left ? 9000 : -9000;

    robot_model_t feedback = {0};
    robot_control_snapshot(&feedback);
    if (!calibration_trajectory_initialized)
        initialize_calibration_trajectory(calibration_test.leg, &feedback);

    float reference_error = 0.0f, feedback_error = 0.0f;
    bool has_feedback = false;
    calibration_tracking_errors(calibration_test.leg, &feedback, &reference_error,
                                &has_feedback, &feedback_error);
    const float phase_rate = robot_gait_phase_rate(reference_error, has_feedback, feedback_error);
    if (active) {
        calibration_test.phase += gait_hz(ROBOT_GAIT_TROT) * phase_rate *
                                  ((float)ROBOT_GAIT_PERIOD_MS / 1000.0f);
        calibration_test.phase -= floorf(calibration_test.phase);
    }

    memset(state.target_cdeg, 0, sizeof(state.target_cdeg));
    state.selected_gait = ROBOT_GAIT_TROT;
    state.active_gait = active ? ROBOT_GAIT_TROT : ROBOT_GAIT_STAND;
    state.spin_mode = false;
    state.jumping = false;
    state.tracking_limited = active && phase_rate < 0.995f;

    bool solved = false;
    if (active) {
        const float raw_phase = fmodf(calibration_test.phase +
                                      leg_phase_offset(calibration_test.leg, ROBOT_GAIT_TROT), 1.0f);
        const float stride = 58.0f * fmaxf(0.25f, magnitude);
        const robot_vec3_t target = foot_target_for_leg(calibration_test.leg, raw_phase,
                                                          gait_duty(ROBOT_GAIT_TROT), true,
                                                          false, false, false, false,
                                                          input, stride);
        solved = set_targets_for_leg(calibration_test.leg, target, 118.0f);
    }
    if (!solved) {
        state.target_cdeg[calibration_test.leg][ROBOT_AXIS_ABDUCTION] = 0;
        state.target_cdeg[calibration_test.leg][ROBOT_AXIS_HIP] = 0;
        state.target_cdeg[calibration_test.leg][ROBOT_AXIS_KNEE] = knee_reference;
    }

    int16_t command_cdeg[ROBOT_AXIS_COUNT] = {0};
    for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
        const float reference = (float)state.target_cdeg[calibration_test.leg][axis] *
                                ROBOT_PI / 18000.0f;
        const float command = joint_trajectory_step(&joint_trajectory[calibration_test.leg][axis],
                                                     reference,
                                                     (float)ROBOT_GAIT_PERIOD_MS / 1000.0f,
                                                     (joint_trajectory_limits_t){
                                                         .maximum_velocity = ROBOT_JOINT_MAX_VELOCITY_RAD_S,
                                                         .maximum_acceleration = ROBOT_JOINT_MAX_ACCELERATION_RAD_S2,
                                                     });
        command_cdeg[axis] = (int16_t)lroundf(command * 18000.0f / ROBOT_PI);
    }
    (void)robot_control_calibration_drive_leg_profile(calibration_test.leg, command_cdeg,
                                                      ROBOT_GAIT_SPEED_RAW,
                                                      ROBOT_GAIT_ACCELERATION);

    state.controller_connected = input->connected;
    state.controller_has_input = input->has_input;
    state.virtual_input = false;
    state.odometry_valid = false;
    state.phase_milli = (uint16_t)lroundf(calibration_test.phase * 1000.0f);
    state.input_forward_milli = (int16_t)lroundf(input->forward * 1000.0f);
    state.input_lateral_milli = (int16_t)lroundf(input->lateral * 1000.0f);
    state.input_turn_milli = 0;
    state.body_height_mm = 118;
    state.phase_rate_percent = (uint8_t)lroundf(phase_rate * 100.0f);
    ++state.tick_count;
}

static void gait_tick(void)
{
    gait_input_t input = read_input();
    if (calibration_test.enabled) {
        state.body_preview_active = false;
        state.body_shift_x_mm = state.body_shift_z_mm = 0;
        state.swing_balance_active = false;
        state.constrained_leg_mask = 0;
        calibration_test_tick(&input);
        return;
    }
    if (!input.connected || !input.has_input) {
        filtered_drive = (gait_input_t){0};
    } else {
        filtered_drive.forward = robot_locomotion_filter_command(filtered_drive.forward,input.forward,0.02f);
        filtered_drive.lateral = robot_locomotion_filter_command(filtered_drive.lateral,input.lateral,0.02f);
        filtered_drive.turn = robot_locomotion_filter_command(filtered_drive.turn,input.turn,0.02f);
        filtered_drive.height = robot_locomotion_filter_command(filtered_drive.height,input.height,0.02f);
    }
    input.forward=filtered_drive.forward; input.lateral=filtered_drive.lateral;
    input.turn=filtered_drive.turn; input.height=filtered_drive.height;
    if (input.r1 && !r1_was_pressed) {
        state.selected_gait = next_gait(state.selected_gait);
        tripod_walk_entry = 0;
    }
    tripod_walk_enabled = state.selected_gait == ROBOT_GAIT_TRIPOD;
    if (input.l1 && !l1_was_pressed) state.spin_mode = !state.spin_mode;
    if (input.cross && !cross_was_pressed && !tripod_test_is_active() && !tripod_walk_enabled) jump_ticks_remaining = 33;
    r1_was_pressed = input.r1;
    l1_was_pressed = input.l1;
    cross_was_pressed = input.cross;

    const float magnitude = fminf(1.0f, sqrtf(input.forward * input.forward +
                                               input.lateral * input.lateral + input.turn * input.turn));
    const bool stepping_in_place = (state.selected_gait == ROBOT_GAIT_TROT || tripod_walk_enabled) && magnitude < 0.02f;
    const bool jumping = jump_ticks_remaining != 0;
    const bool spin = state.spin_mode && fabsf(input.turn) > 0.02f && !jumping;
    state.active_gait = state.selected_gait == ROBOT_GAIT_STAND && magnitude > 0.02f
                        ? ROBOT_GAIT_TROT : state.selected_gait;
    const bool motion_requested = !jumping && state.active_gait != ROBOT_GAIT_STAND &&
                                  (stepping_in_place || magnitude > 0.02f || spin);
    const bool operator_idle = magnitude < 0.02f && !spin && !jumping;
    // A newly armed robot must establish its level reference from the actual
    // neutral stance, not from the loose pre-arm pose. It therefore holds
    // the stand until that short capture sequence has completed.
    const bool active = motion_requested && !attitude_control.reference_pending;
    const bool turning = fabsf(input.turn) > 0.02f && !stepping_in_place && !spin;
    const float scale = stepping_in_place ? 1.0f : fmaxf(0.25f, magnitude);
    const float stride = gait_max_stride_mm(state.active_gait) * scale;

    robot_model_t feedback = {0};
    robot_control_snapshot(&feedback);
    const bool armed = robot_control_is_armed();
    if (!armed) {
        trajectory_initialized = false;
        tripod_walk_entry = 0;
        dynamic_balance_shift = (robot_vec3_t){0};
        if (tripod_test_is_active()) {
            tripod_state = ROBOT_TRIPOD_IDLE;
            tripod_stage_progress = 0.0f;
            tripod_settled_ticks = 0;
            tripod_exit_requested = false;
        }
    }
    if (armed && !trajectory_initialized) initialize_trajectories(&feedback);
    float reference_error = 0.0f, feedback_error = 0.0f;
    bool has_feedback = false;
    tracking_errors(&feedback, &reference_error, &has_feedback, &feedback_error);
    update_attitude_reference_capture(armed, operator_idle, reference_error,
                                      has_feedback, feedback_error);
    // This reads only the latest MPU observer state. The 100 Hz telemetry
    // task owns I2C sampling; the 50 Hz gait loop never blocks on the sensor.
    // `active` is established from the pad before this point. The attitude
    // loop changes to moving gains on that same tick as the gait generator.
    update_attitude_controller(armed, active);
    const float phase_rate = armed ? robot_gait_phase_rate(reference_error, has_feedback, feedback_error) : 1.0f;

    if (input.options && !options_was_pressed && !tripod_walk_enabled) {
        if (!armed) {
            ESP_LOGW("robot_gait", "Options ignored: robot is disarmed");
        } else if (magnitude > 0.02f || jumping) {
            ESP_LOGW("robot_gait", "Options ignored: centre both sticks before tripod test");
        } else if (tripod_test_is_active()) {
            tripod_exit_requested = true;
            ESP_LOGI("robot_gait", "tripod test exit queued");
        } else {
            tripod_exit_requested = false;
            tripod_body_height_mm = (float)nominal_body_height_mm;
            tripod_set_state(ROBOT_TRIPOD_SHIFT);
        }
    }
    options_was_pressed = input.options;

    const bool tripod_active = tripod_test_is_active();
    if (tripod_active) state.active_gait = ROBOT_GAIT_STAND;
    const bool motion_active = active && !tripod_active;
    const bool use_static_crawl = motion_active &&
        (state.active_gait == ROBOT_GAIT_CRAWL || state.active_gait == ROBOT_GAIT_CLIMB) &&
        !spin && !turning;
    if (use_static_crawl && !static_crawl_active) {
        // Start from the present four-foot support instead of assuming that a
        // previous gait cycle had already shifted the body to a triangle.
        phase = 0.0f;
        static_crawl_waiting_for_settle = false;
        static_crawl_settled_ticks = 0;
        static_crawl_plan_slot = -1;
        static_crawl_support_safe = false;
        static_crawl_from_body_shift = (robot_vec3_t){0};
        static_crawl_to_body_shift = (robot_vec3_t){0};
        static_crawl_latch_input(&input, phase, stride);
    }
    static_crawl_active = use_static_crawl;
    if (!use_static_crawl) {
        static_crawl_waiting_for_settle = false;
        static_crawl_settled_ticks = 0;
        static_crawl_input_slot = -1;
        static_crawl_plan_slot = -1;
        static_crawl_support_safe = false;
    }

    // Contact estimates remain telemetry for diagnosis and fault handling.
    // They never change phase, torso position, or a Cartesian foot target.
    update_contact_observers(&feedback, armed, motion_active, use_static_crawl, jumping,
                             tripod_active);

    float effective_frequency=gait_hz(state.active_gait);
    bool timed_path_feasible=true;
    const bool use_timed_path=motion_active && !use_static_crawl && !tripod_active && !jumping;
    if (use_timed_path) {
        // Solve the shared period BEFORE advancing phase. The twelve joint
        // velocity/acceleration bounds apply to one clock, not twelve delayed
        // stop-at-each-point trajectories. This computation stays on ESP.
        (void)dynamic_balance_desired(phase,true,stepping_in_place,spin,turning,&input,stride,
            projected_com_for_body_pose(attitude_control.active ? attitude_control.correction_roll_radians : 0,
                                        attitude_control.active ? attitude_control.correction_pitch_radians : 0),
            nominal_body_height_mm+input.height*ROBOT_BODY_HEIGHT_ADJUST_MM);
        timed_path_feasible=body_trajectory.valid;
        effective_frequency=timed_path_feasible ? body_trajectory.effective_frequency_hz : 0;
    }
    state.effective_frequency_centi_hz=(uint16_t)lroundf(effective_frequency*100);
    if (use_static_crawl) {
        static_crawl_advance_phase(phase_rate, &input, stride, reference_error, has_feedback,
                                   feedback_error);
    } else if (motion_active && (!tripod_walk_enabled || tripod_walk_entry >= 1.0f)) {
        // Dynamic gaits own a fixed 50 Hz clock. Servo feedback is not a
        // contact sensor and must not dilate a phase shared by torso and feet.
        const float requested_advance = effective_frequency *
                                       ((float)ROBOT_GAIT_PERIOD_MS / 1000.0f);
        // Trot and Run retain a deterministic 50 Hz phase clock.  The
        // addressed ST3215 feedback arrives asynchronously across twelve
        // axes, so it is useful for early-touchdown latching but too delayed
        // to pause a dynamic gait without introducing cadence holes.
        phase += requested_advance;
        phase -= floorf(phase);
    }
    state.contact_control_active = false;
    state.contact_touchdown_waiting = false;

    if (tripod_walk_enabled && active && armed && timed_path_feasible) {
        if (tripod_walk_entry == 0) phase = 0;
        tripod_walk_entry = fminf(1, tripod_walk_entry + 0.02f / 1.2f);
    }
    if (tripod_active) tripod_test_tick(reference_error);

    const float requested_body_height = (float)nominal_body_height_mm +
                                        input.height * ROBOT_BODY_HEIGHT_ADJUST_MM;
    const float body_height = tripod_active
        ? tripod_body_height_mm + tripod_test_body_lift_mm() : requested_body_height;
    const float jump_u = jumping ? (float)(33 - jump_ticks_remaining) / 32.0f : 0.0f;
    const float jump_lift = jumping ? 52.0f * sinf(ROBOT_PI * jump_u) : 0.0f;
    if (jump_ticks_remaining) --jump_ticks_remaining;
    const float planned_roll = attitude_control.active
        ? attitude_control.correction_roll_radians : 0.0f;
    const float planned_pitch = attitude_control.active
        ? attitude_control.correction_pitch_radians : 0.0f;
    const robot_planar_point_t projected_com = projected_com_for_body_pose(
        planned_roll, planned_pitch);
    if (use_static_crawl) static_crawl_prepare_slot_plan(phase, projected_com);
    const static_crawl_plan_t static_plan = use_static_crawl
        ? static_crawl_plan(phase) : (static_crawl_plan_t){0};
    const bool use_dynamic_balance = armed && motion_active && !use_static_crawl &&
                                     !tripod_active && !jumping;
    dynamic_balance_step(dynamic_balance_desired(phase, use_dynamic_balance,
                                                 stepping_in_place, spin, turning,
                                                 &input, stride, projected_com, body_height),
                         use_dynamic_balance);

    // There is one floating-body reference per tick: static crawl supplies a
    // three-foot support transfer and Trot/Run supply the VPSP translation.
    // Orientation has one source only: the measured MPU6050 attitude loop.
    robot_vec3_t planned_body_shift = static_plan.enabled
        ? static_plan.body_shift
        : (use_dynamic_balance ? dynamic_balance_shift : (robot_vec3_t){0});
    if (tripod_walk_enabled && tripod_walk_entry < 1) {
        const float transfer = smootherstep(fminf(1, tripod_walk_entry * 2));
        planned_body_shift.x *= transfer; planned_body_shift.z *= transfer;
    }
    state.body_shift_x_mm = planned_body_shift.x;
    state.body_shift_z_mm = planned_body_shift.z;
    const robot_body_pose_t body_pose = {
        .x_mm = planned_body_shift.x,
        .z_mm = planned_body_shift.z,
        .roll_radians = planned_roll,
        .pitch_radians = planned_pitch,
    };

    robot_vec3_t world_feet[4]; bool swing[4];
    for (int leg=0; leg<4; ++leg) {
        const float raw_phase = fmodf(phase + leg_phase_offset(leg, state.active_gait), 1.0f);
        world_feet[leg] = tripod_active
            ? tripod_test_target((robot_leg_t)leg)
            : static_plan.enabled
                ? static_crawl_foot_target((robot_leg_t)leg, phase, &static_plan, stride)
                : foot_target_for_leg((robot_leg_t)leg, raw_phase, gait_duty(state.active_gait), motion_active,
                                      stepping_in_place, jumping, spin, turning, &input, stride);
        if (tripod_walk_enabled && tripod_walk_entry < 1 && leg == tripod_walk_excluded)
            world_feet[leg].y *= smootherstep(fmaxf(0, tripod_walk_entry*2-1));
        swing[leg] = gait_leg_scheduled_swing((robot_leg_t)leg, motion_active, use_static_crawl);

    }
    const robot_balance_pose_t commanded_pose = {
        .position={body_pose.x_mm, body_height+jump_lift, body_pose.z_mm},
        .roll=body_pose.roll_radians, .pitch=body_pose.pitch_radians};
    robot_balance_pose_t measured_pose;
    const bool swing_balance = armed && motion_active && !jumping && !tripod_active &&
        (!tripod_walk_enabled || tripod_walk_entry >= 1) &&
        estimate_swing_body(&feedback, world_feet, swing, commanded_pose, &measured_pose);
    state.swing_balance_active = swing_balance;
    state.tripod_walk_enabled = tripod_walk_enabled;
    state.tripod_walk_excluded = tripod_walk_excluded;
    state.constrained_leg_mask = 0;
    float frame[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT];
    // A failed IK solve retains that leg's previous complete target.
    for (int leg=0;leg<ROBOT_LEG_COUNT;++leg) for (int axis=0;axis<ROBOT_AXIS_COUNT;++axis)
        frame[leg][axis]=feedback.axis[leg][axis].target_radians;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        robot_vec3_t target = body_pose_foot_target(world_feet[leg], body_height + jump_lift, body_pose);
        if (swing_balance && swing[leg]) {
            target = robot_balance_swing_target(world_feet[leg], measured_pose);
            target.y += body_height + jump_lift; // IK API includes nominal torso height
        }
        if (!set_targets_for_leg((robot_leg_t)leg, target, body_height + jump_lift)) {
            state.constrained_leg_mask |= 1U << leg;
            continue;
        }
        for (int axis=0; axis<ROBOT_AXIS_COUNT; ++axis) {
            const robot_axis_config_t *config = &feedback.axis[leg][axis].config;
            if (state.target_cdeg[leg][axis] < config->minimum_cdeg ||
                state.target_cdeg[leg][axis] > config->maximum_cdeg)
                state.constrained_leg_mask |= 1U << leg;
        }
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
            const robot_axis_config_t *config=&feedback.axis[leg][axis].config;
            const float low=config->minimum_cdeg*ROBOT_PI/18000.0f;
            const float high=config->maximum_cdeg*ROBOT_PI/18000.0f;
            const float reference=clampf(state.target_cdeg[leg][axis]*ROBOT_PI/18000.0f,low,high);
            const joint_trajectory_limits_t limits={ROBOT_JOINT_MAX_VELOCITY_RAD_S,
                                                     ROBOT_JOINT_MAX_ACCELERATION_RAD_S2};
            float command=reference;
            if (armed) {
                command = use_timed_path && timed_path_feasible
                    ? joint_trajectory_track(&joint_trajectory[leg][axis],reference,0.02f,limits)
                    : joint_trajectory_step(&joint_trajectory[leg][axis],reference,0.02f,limits);
                if (command<low || command>high) {
                    command=clampf(command,low,high);
                    joint_trajectory_reset(&joint_trajectory[leg][axis],command);
                }
            }
            frame[leg][axis]=command;
        }
    }
    if (!timed_path_feasible) { state.constrained_leg_mask=0x0f; trajectory_initialized=false; }
    else if (robot_control_set_frame(frame, ROBOT_GAIT_SPEED_RAW, ROBOT_GAIT_ACCELERATION)!=ESP_OK)
        ++state.target_frame_drops;
    state.jumping = tripod_active ? false : jumping;
    state.tracking_limited = (use_static_crawl || tripod_active) ? phase_rate < 0.995f
        : use_timed_path && effective_frequency < gait_hz(state.active_gait)*0.995f;
    state.controller_connected = input.connected;
    state.controller_has_input = input.has_input;
    state.virtual_input = virtual_input.enabled;
    state.phase_milli = (uint16_t)lroundf(phase * 1000.0f);
    state.input_forward_milli = (int16_t)lroundf(input.forward * 1000.0f);
    state.input_lateral_milli = (int16_t)lroundf(input.lateral * 1000.0f);
    state.input_turn_milli = (int16_t)lroundf(input.turn * 1000.0f);
    state.body_height_mm = (int16_t)lroundf(body_height + jump_lift);
    const float applied_phase_rate = (use_static_crawl || tripod_active) ? phase_rate
        : use_timed_path ? effective_frequency / fmaxf(0.01f,gait_hz(state.active_gait)) : 1.0f;
    state.phase_rate_percent = (uint8_t)lroundf(applied_phase_rate * 100.0f);
    update_odometry(&feedback, armed, motion_active, tripod_active ? false : jumping,
                    spin || turning,
                    body_height + jump_lift);
    ++state.tick_count;
}

static void gait_task_fn(void *unused)
{
    (void)unused;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            const int64_t start=esp_timer_get_time();
            gait_tick();
            state.planner_last_us=(uint32_t)(esp_timer_get_time()-start);
            if (state.planner_last_us>state.planner_max_us) state.planner_max_us=state.planner_last_us;
            if (state.planner_last_us>ROBOT_GAIT_PERIOD_MS*1000) ++state.planner_overruns;
            xSemaphoreGive(gait_mutex);
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ROBOT_GAIT_PERIOD_MS));
    }
}

esp_err_t robot_gait_init(void)
{
    if (gait_mutex) return ESP_OK;
    gait_mutex = xSemaphoreCreateMutex();
    if (!gait_mutex) return ESP_ERR_NO_MEM;
    memset(&state, 0, sizeof(state));
    state.selected_gait = ROBOT_GAIT_CRAWL;
    state.active_gait = ROBOT_GAIT_CRAWL;
    for (int gait = ROBOT_GAIT_TROT; gait <= ROBOT_GAIT_CLIMB; ++gait)
        gait_profiles[gait] = robot_locomotion_default_profile((uint8_t)gait);
    tripod_walk_profile = robot_locomotion_default_profile(ROBOT_GAIT_TRIPOD);
    geometry = robot_kinematics_default_geometry();
    // A failed or absent preference is harmless: a conservative, known
    // standing pose remains available for first boot and commissioning.
    nvs_handle_t nvs = 0;
    bool migrate_fast_profile_defaults = false;
    int16_t saved_height = ROBOT_DEFAULT_BODY_HEIGHT_MM;
    if (nvs_open(ROBOT_GAIT_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_i16(nvs, ROBOT_GAIT_NVS_HEIGHT_KEY, &saved_height) == ESP_OK &&
            saved_height >= ROBOT_MIN_BODY_HEIGHT_MM &&
            saved_height <= ROBOT_MAX_BODY_HEIGHT_MM)
            nominal_body_height_mm = saved_height;
        int16_t saved_stride = ROBOT_GAIT_SINGLE_FOOT_DEFAULT_STRIDE_MM;
        if (nvs_get_i16(nvs, ROBOT_GAIT_NVS_STRIDE_KEY, &saved_stride) == ESP_OK &&
            saved_stride >= ROBOT_GAIT_SINGLE_FOOT_MIN_STRIDE_MM &&
            saved_stride <= ROBOT_GAIT_SINGLE_FOOT_MAX_STRIDE_MM)
            gait_profiles[ROBOT_GAIT_CRAWL].stride_mm = (uint16_t)saved_stride;
        int16_t saved_lift = ROBOT_GAIT_SINGLE_FOOT_DEFAULT_LIFT_MM;
        if (nvs_get_i16(nvs, ROBOT_GAIT_NVS_LIFT_KEY, &saved_lift) == ESP_OK &&
            saved_lift >= ROBOT_GAIT_SINGLE_FOOT_MIN_LIFT_MM &&
            saved_lift <= ROBOT_GAIT_SINGLE_FOOT_MAX_LIFT_MM)
            gait_profiles[ROBOT_GAIT_CRAWL].step_height_mm = (uint16_t)saved_lift;
        int16_t saved_frequency = ROBOT_GAIT_SINGLE_FOOT_DEFAULT_FREQUENCY_CENTI_HZ;
        if (nvs_get_i16(nvs, ROBOT_GAIT_NVS_FREQUENCY_KEY, &saved_frequency) == ESP_OK &&
            saved_frequency >= ROBOT_GAIT_SINGLE_FOOT_MIN_FREQUENCY_CENTI_HZ &&
            saved_frequency <= ROBOT_GAIT_SINGLE_FOOT_MAX_FREQUENCY_CENTI_HZ)
            gait_profiles[ROBOT_GAIT_CRAWL].frequency_centi_hz = (uint16_t)saved_frequency;

        // First firmware using profiles migrates the commissioned Krok 1×
        // values above. Later boots replace all four profiles atomically from
        // one validated blob.
        robot_locomotion_profile_t stored_profiles[ROBOT_GAIT_CLIMB + 1] = {0};
        size_t stored_profiles_size = sizeof(stored_profiles);
        if (nvs_get_blob(nvs, ROBOT_GAIT_NVS_PROFILES_KEY, stored_profiles,
                         &stored_profiles_size) == ESP_OK &&
            stored_profiles_size == sizeof(stored_profiles)) {
            bool profiles_valid = true;
            for (int gait = ROBOT_GAIT_TROT; gait <= ROBOT_GAIT_CLIMB; ++gait)
                profiles_valid &= robot_locomotion_profile_valid((uint8_t)gait,
                                                                   &stored_profiles[gait]);
            if (profiles_valid) {
                memcpy(gait_profiles, stored_profiles, sizeof(gait_profiles));
                // The prior release stored its untouched factory profiles in
                // NVS. Upgrade exactly that set so this release's larger
                // speed/lift envelope actually becomes active after flashing.
                // Any edited value makes this false and is left intact.
                migrate_fast_profile_defaults = legacy_profile_defaults(gait_profiles);
                if (migrate_fast_profile_defaults) {
                    for (int gait = ROBOT_GAIT_TROT; gait <= ROBOT_GAIT_CLIMB; ++gait)
                        gait_profiles[gait] = robot_locomotion_default_profile((uint8_t)gait);
                }
            }
        }
        robot_gait_motion_tuning_t stored_motion_tuning = {0};
        size_t stored_motion_tuning_size = sizeof(stored_motion_tuning);
        if (nvs_get_blob(nvs, ROBOT_GAIT_NVS_MOTION_TUNING_KEY, &stored_motion_tuning,
                         &stored_motion_tuning_size) == ESP_OK &&
            stored_motion_tuning_size == sizeof(stored_motion_tuning) &&
            motion_tuning_valid(&stored_motion_tuning))
            motion_tuning = stored_motion_tuning;
        int16_t saved_com_forward = 0;
        if (nvs_get_i16(nvs, ROBOT_GAIT_NVS_COM_FORWARD_KEY, &saved_com_forward) == ESP_OK &&
            saved_com_forward >= -ROBOT_STATIC_COM_OFFSET_LIMIT_MM &&
            saved_com_forward <= ROBOT_STATIC_COM_OFFSET_LIMIT_MM)
            static_com_forward_mm = saved_com_forward;
        int16_t saved_com_left = 0;
        if (nvs_get_i16(nvs, ROBOT_GAIT_NVS_COM_LEFT_KEY, &saved_com_left) == ESP_OK &&
            saved_com_left >= -ROBOT_STATIC_COM_OFFSET_LIMIT_MM &&
            saved_com_left <= ROBOT_STATIC_COM_OFFSET_LIMIT_MM)
            static_com_left_mm = saved_com_left;
        int16_t saved_margin = ROBOT_STATIC_DEFAULT_SUPPORT_MARGIN_MM;
        if (nvs_get_i16(nvs, ROBOT_GAIT_NVS_SUPPORT_MARGIN_KEY, &saved_margin) == ESP_OK &&
            saved_margin >= ROBOT_STATIC_MIN_SUPPORT_MARGIN_MM &&
            saved_margin <= ROBOT_STATIC_MAX_SUPPORT_MARGIN_MM)
            static_support_margin_mm = (uint16_t)saved_margin;
        int16_t saved_lateral_stance = ROBOT_GAIT_DEFAULT_LATERAL_STANCE_MM;
        if (nvs_get_i16(nvs, ROBOT_GAIT_NVS_LATERAL_STANCE_KEY, &saved_lateral_stance) == ESP_OK &&
            saved_lateral_stance >= ROBOT_GAIT_MIN_LATERAL_STANCE_MM &&
            saved_lateral_stance <= ROBOT_GAIT_MAX_LATERAL_STANCE_MM)
            lateral_stance_mm = saved_lateral_stance;
        uint8_t saved_attitude_enable = 0;
        if (nvs_get_u8(nvs, ROBOT_GAIT_NVS_ATTITUDE_ENABLE_KEY,
                       &saved_attitude_enable) == ESP_OK)
            attitude_control.enabled = saved_attitude_enable != 0;
        int16_t saved_attitude_limit = ROBOT_ATTITUDE_DEFAULT_MAX_CORRECTION_CDEG;
        if (nvs_get_i16(nvs, ROBOT_GAIT_NVS_ATTITUDE_LIMIT_KEY, &saved_attitude_limit) == ESP_OK &&
            saved_attitude_limit >= ROBOT_ATTITUDE_MIN_MAX_CORRECTION_CDEG &&
            saved_attitude_limit <= ROBOT_ATTITUDE_ABSOLUTE_MAX_CORRECTION_CDEG)
            attitude_max_correction_cdeg = (uint16_t)saved_attitude_limit;
        robot_locomotion_profile_t three_profile;
        size_t three_size = sizeof(three_profile);
        if (nvs_get_blob(nvs, "triprof1", &three_profile, &three_size) == ESP_OK &&
            three_size == sizeof(three_profile) && robot_locomotion_profile_valid(5, &three_profile))
            tripod_walk_profile = three_profile;
        uint8_t three_config = 3;
        if (nvs_get_u8(nvs, "trimode1", &three_config) == ESP_OK && (three_config & 0x7c) == 0) {
            tripod_walk_enabled = (three_config & 0x80) != 0;
            tripod_walk_excluded = (robot_leg_t)(three_config & 3);
        }
        if (tripod_walk_enabled) state.selected_gait = ROBOT_GAIT_TRIPOD;
        state.tripod_walk_enabled = tripod_walk_enabled;
        state.tripod_walk_excluded = tripod_walk_excluded;
        nvs_close(nvs);
    }
    if (migrate_fast_profile_defaults) {
        nvs_handle_t writable_nvs = 0;
        esp_err_t migration_result = nvs_open(ROBOT_GAIT_NVS_NAMESPACE, NVS_READWRITE,
                                              &writable_nvs);
        if (migration_result == ESP_OK) {
            migration_result = nvs_set_blob(writable_nvs, ROBOT_GAIT_NVS_PROFILES_KEY,
                                            gait_profiles, sizeof(gait_profiles));
            // Retain a coherent set of the earlier per-crawl keys too. They
            // exist only for firmware rollback, but must not silently restore
            // the legacy slow profile on a later boot.
            const robot_locomotion_profile_t *crawl = &gait_profiles[ROBOT_GAIT_CRAWL];
            if (migration_result == ESP_OK)
                migration_result = nvs_set_i16(writable_nvs, ROBOT_GAIT_NVS_STRIDE_KEY,
                                               (int16_t)crawl->stride_mm);
            if (migration_result == ESP_OK)
                migration_result = nvs_set_i16(writable_nvs, ROBOT_GAIT_NVS_LIFT_KEY,
                                               (int16_t)crawl->step_height_mm);
            if (migration_result == ESP_OK)
                migration_result = nvs_set_i16(writable_nvs, ROBOT_GAIT_NVS_FREQUENCY_KEY,
                                               (int16_t)crawl->frequency_centi_hz);
            if (migration_result == ESP_OK) migration_result = nvs_commit(writable_nvs);
            nvs_close(writable_nvs);
        }
        if (migration_result == ESP_OK)
            ESP_LOGI("robot_gait", "upgraded untouched gait defaults to fast profile set");
        else
            ESP_LOGW("robot_gait", "fast gait default migration was not persisted: %s",
                     esp_err_to_name(migration_result));
    }
    return ESP_OK;
}

static bool arm_preflight_stance(const robot_model_t *model, float body_height,
                                 uint8_t *failed_slot)
{
    if (failed_slot) *failed_slot = 0xff;
    if (!model || !robot_model_is_complete(model)) return false;
    for (int leg = 0; leg < ROBOT_LEG_COUNT; ++leg) {
        float angles[ROBOT_AXIS_COUNT] = {0};
        const robot_vec3_t neutral = standing_foot_for_leg((robot_leg_t)leg);
        if (!robot_kinematics_inverse(&geometry, (robot_leg_t)leg, neutral,
                                      body_height, angles)) {
            if (failed_slot) *failed_slot = (uint8_t)(leg * ROBOT_AXIS_COUNT);
            return false;
        }
        for (int axis = 0; axis < ROBOT_AXIS_COUNT; ++axis) {
            const robot_axis_state_t *state_axis = &model->axis[leg][axis];
            const float cdeg = angles[axis] * 18000.0f / ROBOT_PI;
            if (!isfinite(cdeg) || cdeg < (float)state_axis->config.minimum_cdeg ||
                cdeg > (float)state_axis->config.maximum_cdeg) {
                if (failed_slot) *failed_slot = (uint8_t)(leg * ROBOT_AXIS_COUNT + axis);
                return false;
            }
        }
    }
    return true;
}

esp_err_t robot_gait_arm(void)
{
    if (!gait_mutex) return ESP_ERR_INVALID_STATE;
    if (robot_control_is_armed()) return ESP_OK;
    if (robot_control_calibration_is_active()) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    const float body_height = (float)nominal_body_height_mm;
    xSemaphoreGive(gait_mutex);
    robot_model_t model = {0};
    robot_control_snapshot(&model);
    uint8_t failed_slot = 0xff;
    if (!arm_preflight_stance(&model, body_height, &failed_slot)) {
        last_arm_preflight_slot = failed_slot;
        const int leg = failed_slot == 0xff ? -1 : failed_slot / ROBOT_AXIS_COUNT;
        const int axis = failed_slot == 0xff ? -1 : failed_slot % ROBOT_AXIS_COUNT;
        ESP_LOGE("robot_gait", "arm rejected: L%d A%d cannot reach %.0f mm neutral stance within saved limits",
                 leg, axis, body_height);
        return ESP_ERR_NOT_SUPPORTED;
    }
    // Verify that an attitude sample exists before torque-on. The reference
    // itself is captured later by the 50 Hz gait task, after the neutral
    // standing pose has settled; before torque-on the body can be sagged and
    // is not a trustworthy definition of level.
    if (attitude_control.enabled) {
        mpu6050_attitude_t measured = {0};
        mpu6050_get_attitude(&measured);
        const int64_t now_us = esp_timer_get_time();
        const bool fresh = measured.valid && measured.sample_time_us > 0 &&
            now_us >= measured.sample_time_us &&
            now_us - measured.sample_time_us <= ROBOT_ATTITUDE_FRESH_US;
        if (!fresh) {
            ESP_LOGE("robot_gait", "arm rejected: MPU6050 attitude is not fresh");
            return ESP_ERR_INVALID_STATE;
        }
        if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
        attitude_control.reference_valid = false;
        attitude_control.reference_pending = true;
        attitude_control.reference_settled_ticks = 0;
        attitude_control.correction_roll_radians = 0.0f;
        attitude_control.correction_pitch_radians = 0.0f;
        attitude_control.active = false;
        joint_trajectory_reset(&attitude_trajectory[0], 0.0f);
        joint_trajectory_reset(&attitude_trajectory[1], 0.0f);
        xSemaphoreGive(gait_mutex);
    }
    last_arm_preflight_slot = 0xff;
    const esp_err_t result = robot_control_arm();
    if (result != ESP_OK && attitude_control.enabled &&
        xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        attitude_control.reference_pending = false;
        attitude_control.reference_settled_ticks = 0;
        xSemaphoreGive(gait_mutex);
    }
    return result;
}

esp_err_t robot_gait_set_body_height_mm(int16_t height_mm)
{
    if (!gait_mutex || height_mm < ROBOT_MIN_BODY_HEIGHT_MM ||
        height_mm > ROBOT_MAX_BODY_HEIGHT_MM)
        return ESP_ERR_INVALID_ARG;
    // Height selection is setup, never an in-motion pose command. Requiring a
    // disarmed controller avoids moving a loaded leg just by editing a field.
    if (robot_control_is_armed() || robot_control_calibration_is_active())
        return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    nvs_handle_t nvs = 0;
    esp_err_t result = nvs_open(ROBOT_GAIT_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (result == ESP_OK) {
        result = nvs_set_i16(nvs, ROBOT_GAIT_NVS_HEIGHT_KEY, height_mm);
        if (result == ESP_OK) result = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (result == ESP_OK) nominal_body_height_mm = height_mm;
    xSemaphoreGive(gait_mutex);
    return result;
}

int16_t robot_gait_get_body_height_mm(void)
{
    if (!gait_mutex || xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(5)) != pdTRUE)
        return ROBOT_DEFAULT_BODY_HEIGHT_MM;
    const int16_t height_mm = nominal_body_height_mm;
    xSemaphoreGive(gait_mutex);
    return height_mm;
}

esp_err_t robot_gait_set_locomotion_profile(robot_gait_mode_t gait,
                                            const robot_locomotion_profile_t *profile)
{
    if (!gait_mutex || gait < ROBOT_GAIT_TROT || gait > ROBOT_GAIT_TRIPOD ||
        !robot_locomotion_profile_valid((uint8_t)gait, profile))
        return ESP_ERR_INVALID_ARG;
    // Profile changes are planning data. The current profile must remain
    // invariant for an armed step, otherwise a normal UI edit could alter a
    // loaded foot's touchdown point.
    if (robot_control_is_armed() || robot_control_calibration_is_active())
        return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    robot_locomotion_profile_t candidate[ROBOT_GAIT_CLIMB + 1];
    memcpy(candidate, gait_profiles, sizeof(candidate));
    if (gait <= ROBOT_GAIT_CLIMB) candidate[gait] = *profile;

    nvs_handle_t nvs = 0;
    esp_err_t result = nvs_open(ROBOT_GAIT_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (result == ESP_OK) {
        result = gait == ROBOT_GAIT_TRIPOD
            ? nvs_set_blob(nvs, "triprof1", profile, sizeof(*profile))
            : nvs_set_blob(nvs, ROBOT_GAIT_NVS_PROFILES_KEY, candidate, sizeof(candidate));
        // Keep the original commissioning keys current for a safe downgrade
        // to the prior firmware. They describe only the one-foot profile.
        if (result == ESP_OK && gait == ROBOT_GAIT_CRAWL)
            result = nvs_set_i16(nvs, ROBOT_GAIT_NVS_STRIDE_KEY, (int16_t)profile->stride_mm);
        if (result == ESP_OK && gait == ROBOT_GAIT_CRAWL)
            result = nvs_set_i16(nvs, ROBOT_GAIT_NVS_LIFT_KEY, (int16_t)profile->step_height_mm);
        if (result == ESP_OK && gait == ROBOT_GAIT_CRAWL)
            result = nvs_set_i16(nvs, ROBOT_GAIT_NVS_FREQUENCY_KEY,
                                 (int16_t)profile->frequency_centi_hz);
        if (result == ESP_OK) result = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (result == ESP_OK) {
        if (gait == ROBOT_GAIT_TRIPOD) tripod_walk_profile = *profile;
        else memcpy(gait_profiles, candidate, sizeof(gait_profiles));
    }
    xSemaphoreGive(gait_mutex);
    return result;
}

bool robot_gait_get_locomotion_profile(robot_gait_mode_t gait,
                                       robot_locomotion_profile_t *profile)
{
    if (!profile || !gait_mutex || gait < ROBOT_GAIT_TROT || gait > ROBOT_GAIT_TRIPOD ||
        xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(5)) != pdTRUE)
        return false;
    *profile = *gait_profile(gait);
    xSemaphoreGive(gait_mutex);
    return true;
}

esp_err_t robot_gait_set_motion_tuning(const robot_gait_motion_tuning_t *tuning)
{
    if (!gait_mutex || !motion_tuning_valid(tuning)) return ESP_ERR_INVALID_ARG;
    // These are gains and trajectory limits, not a live balance command.
    // Keep them immutable throughout a loaded gait cycle.
    if (robot_control_is_armed() || robot_control_calibration_is_active())
        return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    nvs_handle_t nvs = 0;
    esp_err_t result = nvs_open(ROBOT_GAIT_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (result == ESP_OK) {
        result = nvs_set_blob(nvs, ROBOT_GAIT_NVS_MOTION_TUNING_KEY, tuning, sizeof(*tuning));
        if (result == ESP_OK) result = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (result == ESP_OK) motion_tuning = *tuning;
    xSemaphoreGive(gait_mutex);
    return result;
}

bool robot_gait_get_motion_tuning(robot_gait_motion_tuning_t *tuning)
{
    if (!tuning || !gait_mutex || xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(5)) != pdTRUE)
        return false;
    *tuning = motion_tuning;
    xSemaphoreGive(gait_mutex);
    return true;
}

esp_err_t robot_gait_set_single_foot_tuning(int16_t stride_mm, int16_t lift_mm,
                                             uint16_t frequency_centi_hz)
{
    // Compatibility for existing desktop clients: Krok 1× is now simply the
    // crawl profile. The unified setter also persists it with the other gaits.
    robot_locomotion_profile_t profile = robot_locomotion_default_profile(ROBOT_GAIT_CRAWL);
    (void)robot_gait_get_locomotion_profile(ROBOT_GAIT_CRAWL, &profile);
    profile.stride_mm = stride_mm > 0 ? (uint16_t)stride_mm : 0;
    profile.step_height_mm = lift_mm > 0 ? (uint16_t)lift_mm : 0;
    profile.frequency_centi_hz = frequency_centi_hz;
    return robot_gait_set_locomotion_profile(ROBOT_GAIT_CRAWL, &profile);
}

esp_err_t robot_gait_set_static_balance(int16_t com_forward_mm,
                                        int16_t com_left_mm,
                                        uint16_t support_margin_mm)
{
    if (!gait_mutex || com_forward_mm < -ROBOT_STATIC_COM_OFFSET_LIMIT_MM ||
        com_forward_mm > ROBOT_STATIC_COM_OFFSET_LIMIT_MM ||
        com_left_mm < -ROBOT_STATIC_COM_OFFSET_LIMIT_MM ||
        com_left_mm > ROBOT_STATIC_COM_OFFSET_LIMIT_MM ||
        support_margin_mm < ROBOT_STATIC_MIN_SUPPORT_MARGIN_MM ||
        support_margin_mm > ROBOT_STATIC_MAX_SUPPORT_MARGIN_MM)
        return ESP_ERR_INVALID_ARG;
    // This is commissioning data, never a motion request.
    if (robot_control_is_armed() || robot_control_calibration_is_active())
        return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    nvs_handle_t nvs = 0;
    esp_err_t result = nvs_open(ROBOT_GAIT_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (result == ESP_OK) {
        result = nvs_set_i16(nvs, ROBOT_GAIT_NVS_COM_FORWARD_KEY, com_forward_mm);
        if (result == ESP_OK) result = nvs_set_i16(nvs, ROBOT_GAIT_NVS_COM_LEFT_KEY, com_left_mm);
        if (result == ESP_OK) result = nvs_set_i16(nvs, ROBOT_GAIT_NVS_SUPPORT_MARGIN_KEY,
                                                    (int16_t)support_margin_mm);
        if (result == ESP_OK) result = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (result == ESP_OK) {
        static_com_forward_mm = com_forward_mm;
        static_com_left_mm = com_left_mm;
        static_support_margin_mm = support_margin_mm;
    }
    xSemaphoreGive(gait_mutex);
    return result;
}

esp_err_t robot_gait_set_lateral_stance_mm(int16_t stance_mm)
{
    if (!gait_mutex || stance_mm < ROBOT_GAIT_MIN_LATERAL_STANCE_MM ||
        stance_mm > ROBOT_GAIT_MAX_LATERAL_STANCE_MM)
        return ESP_ERR_INVALID_ARG;
    // A footprint change is commissioning data. It is deliberately rejected
    // whenever torque or a calibration action owns a physical axis.
    if (robot_control_is_armed() || robot_control_calibration_is_active())
        return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    nvs_handle_t nvs = 0;
    esp_err_t result = nvs_open(ROBOT_GAIT_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (result == ESP_OK) {
        result = nvs_set_i16(nvs, ROBOT_GAIT_NVS_LATERAL_STANCE_KEY, stance_mm);
        if (result == ESP_OK) result = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (result == ESP_OK) lateral_stance_mm = stance_mm;
    xSemaphoreGive(gait_mutex);
    return result;
}

int16_t robot_gait_get_lateral_stance_mm(void)
{
    if (!gait_mutex || xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(5)) != pdTRUE)
        return ROBOT_GAIT_DEFAULT_LATERAL_STANCE_MM;
    const int16_t stance_mm = lateral_stance_mm;
    xSemaphoreGive(gait_mutex);
    return stance_mm;
}

esp_err_t robot_gait_set_attitude_balance_enabled(bool enabled)
{
    if (!gait_mutex) return ESP_ERR_INVALID_STATE;
    // The switch is intentionally a commissioning decision. It cannot change
    // an already armed robot's target frame.
    if (robot_control_is_armed() || robot_control_calibration_is_active())
        return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    nvs_handle_t nvs = 0;
    esp_err_t result = nvs_open(ROBOT_GAIT_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (result == ESP_OK) {
        result = nvs_set_u8(nvs, ROBOT_GAIT_NVS_ATTITUDE_ENABLE_KEY, enabled ? 1 : 0);
        if (result == ESP_OK) result = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (result == ESP_OK) {
        attitude_control.enabled = enabled;
        attitude_control.reference_valid = false;
        attitude_control.reference_pending = false;
        attitude_control.reference_settled_ticks = 0;
        attitude_control.active = false;
        attitude_control.correction_roll_radians = 0.0f;
        attitude_control.correction_pitch_radians = 0.0f;
        joint_trajectory_reset(&attitude_trajectory[0], 0.0f);
        joint_trajectory_reset(&attitude_trajectory[1], 0.0f);
    }
    xSemaphoreGive(gait_mutex);
    return result;
}

esp_err_t robot_gait_set_attitude_correction_limit_cdeg(uint16_t centidegrees)
{
    if (!gait_mutex || centidegrees < ROBOT_ATTITUDE_MIN_MAX_CORRECTION_CDEG ||
        centidegrees > ROBOT_ATTITUDE_ABSOLUTE_MAX_CORRECTION_CDEG)
        return ESP_ERR_INVALID_ARG;
    if (robot_control_is_armed() || robot_control_calibration_is_active())
        return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    nvs_handle_t nvs = 0;
    esp_err_t result = nvs_open(ROBOT_GAIT_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (result == ESP_OK) {
        result = nvs_set_i16(nvs, ROBOT_GAIT_NVS_ATTITUDE_LIMIT_KEY, (int16_t)centidegrees);
        if (result == ESP_OK) result = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (result == ESP_OK) attitude_max_correction_cdeg = centidegrees;
    xSemaphoreGive(gait_mutex);
    return result;
}

uint8_t robot_gait_last_arm_preflight_slot(void)
{
    return last_arm_preflight_slot;
}

esp_err_t robot_gait_start(void)
{
    if (!gait_mutex) return ESP_ERR_INVALID_STATE;
    if (gait_task) return ESP_OK;
    return xTaskCreate(gait_task_fn, "robot_gait", 4096, NULL, 8, &gait_task) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void robot_gait_snapshot(robot_gait_snapshot_t *snapshot)
{
    if (!snapshot || !gait_mutex || xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
    *snapshot = state;
    snapshot->calibration_test = calibration_test.enabled;
    snapshot->calibration_leg = calibration_test.leg;
    xSemaphoreGive(gait_mutex);
}

esp_err_t robot_gait_set_virtual_input(const robot_gait_virtual_input_t *input)
{
    if (!input || !gait_mutex) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    virtual_input = *input;
    virtual_input.forward_milli = (int16_t)lroundf(clampf((float)virtual_input.forward_milli, -1000.0f, 1000.0f));
    virtual_input.lateral_milli = (int16_t)lroundf(clampf((float)virtual_input.lateral_milli, -1000.0f, 1000.0f));
    virtual_input.turn_milli = (int16_t)lroundf(clampf((float)virtual_input.turn_milli, -1000.0f, 1000.0f));
    virtual_input.height_milli = (int16_t)lroundf(clampf((float)virtual_input.height_milli, -1000.0f, 1000.0f));
    virtual_input_deadline_us = virtual_input.enabled
        ? esp_timer_get_time() + ROBOT_GAIT_VIRTUAL_INPUT_TIMEOUT_US : 0;
    xSemaphoreGive(gait_mutex);
    return ESP_OK;
}

esp_err_t robot_gait_clear_virtual_input(void)
{
    const robot_gait_virtual_input_t input = {0};
    return robot_gait_set_virtual_input(&input);
}

esp_err_t robot_gait_set_calibration_test(bool enabled, robot_leg_t leg)
{
    if (!gait_mutex || leg < 0 || leg >= ROBOT_LEG_COUNT) return ESP_ERR_INVALID_ARG;
    if (enabled && robot_control_is_armed()) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    const bool stopping = calibration_test.enabled && (!enabled || calibration_test.leg != leg);
    const robot_leg_t previous = calibration_test.leg;
    calibration_test.enabled = enabled;
    calibration_test.leg = leg;
    calibration_test.phase = 0.0f;
    calibration_trajectory_initialized = false;
    xSemaphoreGive(gait_mutex);
    if (stopping) (void)robot_control_calibration_release_leg(previous);
    // A regular calibration command calls this only to stop a running leg
    // test. Do not release manually armed calibration servos when no test was
    // active; the explicit "Rozbrój tę nogę" control owns that decision.
    if (!enabled) return ESP_OK;
    return ESP_OK;
}

esp_err_t robot_gait_set_tripod_walk(bool enabled, robot_leg_t excluded)
{
    if (!gait_mutex || excluded < 0 || excluded >= ROBOT_LEG_COUNT) return ESP_ERR_INVALID_ARG;
    if (robot_control_is_armed() || robot_control_calibration_is_active()) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(gait_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    nvs_handle_t nvs;
    esp_err_t result = nvs_open(ROBOT_GAIT_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (result == ESP_OK) {
        result = nvs_set_u8(nvs, "trimode1", (enabled ? 0x80 : 0) | (uint8_t)excluded);
        if (result == ESP_OK) result = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (result == ESP_OK) {
        tripod_walk_enabled = enabled; tripod_walk_excluded = excluded;
        tripod_walk_entry = 0; phase = 0;
        state.selected_gait = enabled ? ROBOT_GAIT_TRIPOD : ROBOT_GAIT_CRAWL;
        state.tripod_walk_enabled = enabled; state.tripod_walk_excluded = excluded;
    }
    xSemaphoreGive(gait_mutex);
    return result;
}
