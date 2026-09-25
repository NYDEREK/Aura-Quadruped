#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "robot_model.h"

// The real-time layer owns the 50 Hz actuator output. It has no knowledge of
// Bluetooth, Wi-Fi, SceneKit, or Gazebo; those systems submit axis targets to
// this interface.
esp_err_t robot_control_init(void);
esp_err_t robot_control_start(void);
esp_err_t robot_control_assign_axis(robot_leg_t leg, robot_axis_type_t axis,
                                    const robot_axis_config_t *config);
esp_err_t robot_control_clear_axis(robot_leg_t leg, robot_axis_type_t axis);
esp_err_t robot_control_save_configuration(void);
esp_err_t robot_control_load_configuration(void);
esp_err_t robot_control_set_axis_target(robot_leg_t leg, robot_axis_type_t axis,
                                        float radians, uint16_t speed_raw,
                                        uint8_t acceleration);
esp_err_t robot_control_arm(void);
esp_err_t robot_control_disarm(void);
bool robot_control_is_armed(void);

// Latched emergency stop raised by the per-servo stall monitor. The monitor is
// active only during normal 12-axis motion, never while commissioning a leg.
typedef struct {
    bool latched;
    robot_leg_t leg;
    robot_axis_type_t axis;
    uint8_t servo_id;
    int16_t current_raw;       // ST3215: one raw unit is 6.5 mA
    int16_t load_raw;          // signed PWM/load feedback; 1000 = 100 %
    uint16_t tracking_error_cdeg;
    uint8_t confirmations;
} robot_safety_fault_t;

bool robot_control_has_safety_fault(void);
void robot_control_get_safety_fault(robot_safety_fault_t *fault);
// Requires the robot to remain disarmed. It only clears the latched report;
// it never enables torque or sends a position command.
esp_err_t robot_control_clear_safety_fault(void);
// True while a commissioning operation owns a calibration axis or while the
// read-only TTL calibration preview is active. The DualSense Create shortcut
// must never arm the complete robot during either state.
bool robot_control_calibration_is_active(void);
esp_err_t robot_control_read_axis(robot_leg_t leg, robot_axis_type_t axis);

// Commissioning is deliberately separate from the normal 12-axis planner.
// These calls only work while the robot is disarmed and affect the selected
// physical axis (or the three axes of one selected leg). They preserve the
// same angle-to-tick mapping and software safety limits as normal motion.
// reference_cdeg is the model angle represented by the manually placed pose.
// Hip and ab/ad use 0°, while each knee's folded reference uses +/-90°.
esp_err_t robot_control_calibration_capture_zero(robot_leg_t leg, robot_axis_type_t axis,
                                                 int16_t reference_cdeg);
esp_err_t robot_control_calibration_release_axis(robot_leg_t leg, robot_axis_type_t axis);
esp_err_t robot_control_calibration_capture_limit(robot_leg_t leg, robot_axis_type_t axis,
                                                  bool maximum);
esp_err_t robot_control_calibration_nudge_axis(robot_leg_t leg, robot_axis_type_t axis,
                                               int16_t target_cdeg);
// Reverses the logical sense while the joint is physically held at its known
// reference angle. This only changes the mapping/NVS record; it releases
// torque and deliberately never sends a position command.
esp_err_t robot_control_calibration_flip_direction(robot_leg_t leg, robot_axis_type_t axis,
                                                   int16_t reference_cdeg);
// Repairs a sign that an earlier calibration build flipped without rebasing
// its centre. The desired sign is retained; torque stays off and no target is
// sent while the coordinate system and limits are rebuilt around the reference.
esp_err_t robot_control_calibration_rebase_direction(robot_leg_t leg, robot_axis_type_t axis,
                                                     int16_t reference_cdeg);
// Safe direction-test transaction: return with the old map to the known
// reference, verify the feedback, reverse/rebase, then execute the same
// positive probe with the new map.
esp_err_t robot_control_calibration_reverse_probe(robot_leg_t leg, robot_axis_type_t axis,
                                                  int16_t reference_cdeg, int16_t probe_delta_cdeg);
esp_err_t robot_control_calibration_drive_leg(robot_leg_t leg,
                                              const int16_t target_cdeg[ROBOT_AXIS_COUNT]);
// Same selected-leg-only path with the normal gait profile. Used by the
// commissioning Trot so its smoothness matches the assembled robot.
esp_err_t robot_control_calibration_drive_leg_profile(robot_leg_t leg,
                                                      const int16_t target_cdeg[ROBOT_AXIS_COUNT],
                                                      uint16_t speed_raw, uint8_t acceleration);
// Hold or release exactly the selected three axes. This never invokes the
// normal 12-servo arm path, so a single assembled leg can be commissioned.
esp_err_t robot_control_calibration_set_leg_torque(robot_leg_t leg, bool enabled);
// Enables addressed encoder polling for exactly one released leg. This is a
// read-only commissioning aid: it never enables torque and never writes a
// target position to a servo.
esp_err_t robot_control_calibration_set_leg_preview(robot_leg_t leg, bool enabled);
esp_err_t robot_control_calibration_release_leg(robot_leg_t leg);
void robot_control_snapshot(robot_model_t *model);

// One coherent planner frame; servo output cannot observe half-updated legs.
esp_err_t robot_control_set_frame(const float radians[ROBOT_LEG_COUNT][ROBOT_AXIS_COUNT],
                                  uint16_t speed_raw, uint8_t acceleration);
