#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "robot_model.h"
#include "robot_locomotion.h"

typedef enum {
    ROBOT_GAIT_STAND = 0,
    ROBOT_GAIT_TROT = 1,
    ROBOT_GAIT_CRAWL = 2,
    ROBOT_GAIT_RUN = 3,
    ROBOT_GAIT_CLIMB = 4,
    ROBOT_GAIT_TRIPOD = 5,
} robot_gait_mode_t;

// A UART-only commissioning source. It uses the same gait, kinematics and
// safety path as a real controller; it never bypasses the actuator layer.
typedef struct {
    bool enabled;
    int16_t forward_milli;
    int16_t lateral_milli;
    int16_t turn_milli;
    int16_t height_milli;
} robot_gait_virtual_input_t;

// Parameters used only by the moving body controller.  They are deliberately
// separate from standing balance: a robot that is already walking must not
// receive the same aggressive posture target as a robot standing on an
// inclined board.  All fields are persisted and accepted only while torque is
// off, so changing a slider cannot alter a loaded foot.
typedef struct {
    // Proportional IMU correction during a gait, in parts per thousand.
    // 1000 is one-to-one attitude correction; 450 is the conservative default.
    uint16_t moving_attitude_gain_per_mille;
    // Bounded trajectory of the VPSP CoM target, in millimetres per second
    // and millimetres per second squared.
    uint16_t com_max_velocity_mm_s;
    uint16_t com_max_acceleration_mm_s2;
} robot_gait_motion_tuning_t;

// This snapshot is calculated on Aura at 50 Hz.  The desktop receives it as
// telemetry and must never be the source of physical servo targets.
typedef struct {
    robot_gait_mode_t selected_gait;
    robot_gait_mode_t active_gait;
    bool spin_mode;
    bool jumping;
    // Feedback has not stopped the planner; it only reduces phase speed while
    // the physical joints are materially behind their generated trajectories.
    bool tracking_limited;
    bool controller_connected;
    bool controller_has_input;
    bool virtual_input;
    // True only while commissioning one selected leg from the paired pad.
    // This is telemetry state; it does not alter any actuator configuration.
    bool calibration_test;
    robot_leg_t calibration_leg;
    bool odometry_valid;
    uint16_t phase_milli;
    bool body_preview_active;
    bool swing_balance_active;
    uint8_t constrained_leg_mask;
    bool tripod_walk_enabled;
    robot_leg_t tripod_walk_excluded;
    float body_shift_x_mm;
    float body_shift_z_mm;
    int16_t target_cdeg[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT];
    int16_t input_forward_milli;
    int16_t input_lateral_milli;
    int16_t input_turn_milli;
    int16_t body_height_mm;
    // Filtered MPU6050 pose and the bounded body-attitude correction. Angles
    // use Aura's body frame and centidegrees; correction is zero until an
    // operator enables it while disarmed, then arms from a stable reference.
    bool attitude_valid;
    bool attitude_control_enabled;
    bool attitude_control_active;
    bool attitude_reference_valid;
    bool attitude_reference_pending;
    int16_t attitude_roll_cdeg;
    int16_t attitude_pitch_cdeg;
    int16_t attitude_reference_roll_cdeg;
    int16_t attitude_reference_pitch_cdeg;
    int16_t attitude_correction_roll_cdeg;
    int16_t attitude_correction_pitch_cdeg;
    uint16_t attitude_max_correction_cdeg;
    // Proprioceptive contact observer. In the quasistatic one-foot crawl it
    // also gates touchdown; faster gaits remain observational. Bits use the
    // robot-leg order LF, RF, LR, RR.
    bool contact_feedback_available;
    bool contact_control_active;
    bool contact_touchdown_waiting;
    uint8_t expected_contact_mask;
    uint8_t detected_contact_mask;
    uint8_t early_touchdown_mask;
    uint8_t contact_confidence_percent[ROBOT_LEG_COUNT];
    float odometry_x_mm;
    float odometry_z_mm;
    float odometry_yaw_radians;
    uint32_t tick_count;
    uint32_t planner_last_us, planner_max_us, planner_overruns, target_frame_drops;
    uint8_t phase_rate_percent;
    uint16_t effective_frequency_centi_hz;
} robot_gait_snapshot_t;

esp_err_t robot_gait_init(void);
esp_err_t robot_gait_start(void);
void robot_gait_snapshot(robot_gait_snapshot_t *snapshot);
// Sets the neutral standing height used for inverse kinematics and arm
// preflight. It is persisted in NVS and is intentionally available only
// while torque is off: changing it never writes a servo target.
esp_err_t robot_gait_set_body_height_mm(int16_t height_mm);
int16_t robot_gait_get_body_height_mm(void);
// Parameters of the three-support, one-foot transfer gait. They are saved
// before arming and never alter an active trajectory.
esp_err_t robot_gait_set_single_foot_tuning(int16_t stride_mm, int16_t lift_mm,
                                             uint16_t frequency_centi_hz);
// Independent profiles for Trot, one-foot crawl, Run and Climb. Aura accepts
// edits only while disarmed, persists them locally, and uses them in the ESP
// planner. The desktop has no role in the real-time gait loop.
esp_err_t robot_gait_set_locomotion_profile(robot_gait_mode_t gait,
                                            const robot_locomotion_profile_t *profile);
bool robot_gait_get_locomotion_profile(robot_gait_mode_t gait,
                                       robot_locomotion_profile_t *profile);

// Moving-balance tuning is commissioning data, never an actuator command.
// The ESP planner reads it at 50 Hz; the desktop only edits and displays it.
esp_err_t robot_gait_set_motion_tuning(const robot_gait_motion_tuning_t *tuning);
bool robot_gait_get_motion_tuning(robot_gait_motion_tuning_t *tuning);

// Static crawl keeps the *actual* projected centre of mass inside the
// three-foot support triangle before a foot may lift.  These setup values are
// measured from the geometric frame origin in millimetres (forward, left) and
// are accepted only with torque off.  Saving them never sends a servo target.
esp_err_t robot_gait_set_static_balance(int16_t com_forward_mm,
                                        int16_t com_left_mm,
                                        uint16_t support_margin_mm);
// Extra lateral placement of each foot after the physical ab/ad link.  The
// value is persisted only while disarmed and never writes a servo target.
esp_err_t robot_gait_set_lateral_stance_mm(int16_t stance_mm);
int16_t robot_gait_get_lateral_stance_mm(void);
// Enables the measured-body-attitude loop for the next arm cycle. This is a
// commissioning setting: it is persisted only while disarmed and never sends
// an actuator command by itself.
esp_err_t robot_gait_set_attitude_balance_enabled(bool enabled);
esp_err_t robot_gait_set_attitude_correction_limit_cdeg(uint16_t centidegrees);
// Read-only kinematic preflight followed by the usual torque-on sequence.
// It refuses arming when the saved per-axis calibration cannot represent the
// normal standing pose; no servo target is written on a failed preflight.
esp_err_t robot_gait_arm(void);
uint8_t robot_gait_last_arm_preflight_slot(void);
esp_err_t robot_gait_set_virtual_input(const robot_gait_virtual_input_t *input);
esp_err_t robot_gait_clear_virtual_input(void);
// Calibration test drives one selected leg from the already paired DualSense.
// It runs on Aura's 50 Hz loop and is only available while the 12-axis gait
// controller is disarmed.
esp_err_t robot_gait_set_calibration_test(bool enabled, robot_leg_t leg);

// Configuration only, accepted while disarmed. Create is still the only pad
// arming action. R1 also reaches this mode as index 5 in its normal cycle.
esp_err_t robot_gait_set_tripod_walk(bool enabled, robot_leg_t excluded);
