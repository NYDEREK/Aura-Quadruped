#!/bin/bash
# Host tests: each tests/test_*.c is linked with the portable (ESP-free)
# modules of main/ and run on the Mac.   tools/run_host_tests.sh
cd "$(dirname "$0")/.." || exit 1
OUT=$(mktemp -d)
PURE="main/robot_kinematics.c main/robot_locomotion.c main/robot_body_trajectory.c main/robot_gait_profile.c
      main/robot_predictive_support.c main/robot_balance.c main/joint_trajectory.c main/robot_odometry.c
      main/attitude_filter.c main/battery_gauge.c main/robot_walk_balance.c main/robot_state_estimator.c
      main/robot_static_balance.c main/robot_static_gait_profile.c main/robot_gait_sync.c main/robot_contact.c"
failed=0
for test in tests/test_*.c; do
    name=$(basename "$test" .c)
    extra=""
    case $name in
        test_robot_model) extra="main/robot_model.c" ;;
        test_servo_protocol) extra="main/servo_protocol.c" ;;
    esac
    if ! cc -std=c11 -O2 -w -I main -I tests "$test" $PURE $extra -lm -o "$OUT/$name" 2>"$OUT/$name.err"; then
        echo "BUILD FAILED $name"; grep -m3 error "$OUT/$name.err"; failed=1
    elif "$OUT/$name" >"$OUT/$name.out" 2>&1; then
        echo "PASS $name"
    else
        echo "FAIL $name"; tail -3 "$OUT/$name.out"; failed=1
    fi
done
rm -rf "$OUT"
exit $failed
