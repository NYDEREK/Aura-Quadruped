#!/usr/bin/env python3
"""Portable planner/protocol regressions. Does not open ports or command hardware."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parent.parent
planner = ['robot_body_trajectory', 'robot_locomotion', 'robot_gait_profile',
           'robot_predictive_support', 'robot_kinematics']
cases = {
    'robot_body_trajectory': planner,
    'robot_timed_path': planner + ['joint_trajectory'],
    'robot_balance': planner + ['robot_balance'],
    'robot_gait_foundations': ['joint_trajectory', 'robot_gait_profile', 'robot_locomotion',
        'robot_predictive_support', 'robot_gait_sync', 'robot_kinematics',
        'robot_static_gait_profile', 'robot_static_balance'],
    'robot_model': ['robot_model'],
    'joint_trajectory': ['joint_trajectory'],
    'attitude_filter': ['attitude_filter'],
    'servo_protocol': ['servo_protocol'],
    'dualsense_report': [],
    'robot_odometry': ['robot_odometry'],
}
with tempfile.TemporaryDirectory(prefix='aura-host-tests-') as temporary:
    for name, units in cases.items():
        binary = str(pathlib.Path(temporary) / name)
        subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-I', 'main',
                        f'tests/test_{name}.c', *[f'main/{unit}.c' for unit in units], '-lm',
                        '-o', binary], cwd=root, check=True)
        subprocess.run([binary], cwd=root, check=True)
    binary = str(pathlib.Path(temporary) / 'telemetry')
    subprocess.run(['xcrun', 'swiftc', '-O', 'desktop/MainBoardControl/RobotTelemetry.swift',
                    'tests/test_robot_telemetry.swift', '-o', binary], cwd=root, check=True)
    subprocess.run([binary], cwd=root, check=True)
print(f'{len(cases) + 1} suites passed; no hardware accessed.')
