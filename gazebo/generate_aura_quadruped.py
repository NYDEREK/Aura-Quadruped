#!/usr/bin/env python3
"""Generate the Aura quadruped as an SDF world for Gazebo Sim 8.

Gazebo uses x forward, y left, z up.  Every leg contains exactly three
revolute joints: ab/ad (roll) about local x, then hip and knee (pitch) about
local y.  There is intentionally no steering / yaw joint about z.
"""

from pathlib import Path

OUT = Path(__file__).with_name("aura_quadruped.sdf")
LEGS = (("lf", 0.130, 0.075, 1), ("rf", 0.130, -0.075, -1),
        ("lr", -0.130, 0.075, 1), ("rr", -0.130, -0.075, -1))


def inertial(mass: float) -> str:
    # Conservative diagonal tensors for the initial physics model.
    i = mass * 0.0012
    return f"""<inertial><mass>{mass}</mass><inertia>
      <ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz>
      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
    </inertia></inertial>"""


def box_link(name: str, pose: str, size: str, mass: float, color: str) -> str:
    return f"""<link name=\"{name}\"><pose>{pose}</pose>{inertial(mass)}
      <visual name=\"visual\"><geometry><box><size>{size}</size></box></geometry>
        <material><diffuse>{color}</diffuse><ambient>{color}</ambient></material></visual>
      <collision name=\"collision\"><geometry><box><size>{size}</size></box></geometry>
        <surface><friction><ode><mu>0.85</mu><mu2>0.85</mu2></ode></friction></surface></collision>
    </link>"""


def controller(joint: str) -> str:
    return f"""<plugin filename=\"gz-sim-joint-position-controller-system\"
      name=\"gz::sim::systems::JointPositionController\">
      <joint_name>{joint}</joint_name><topic>/aura/{joint}/cmd_pos</topic>
      <p_gain>65</p_gain><i_gain>0.02</i_gain><d_gain>3.5</d_gain>
      <i_max>0.3</i_max><i_min>-0.3</i_min><cmd_max>18</cmd_max><cmd_min>-18</cmd_min>
    </plugin>"""


def joint(name: str, parent: str, child: str, pose: str, axis: str, lower: float, upper: float) -> str:
    return f"""<joint name=\"{name}\" type=\"revolute\"><pose>{pose}</pose>
      <parent>{parent}</parent><child>{child}</child><axis><xyz>{axis}</xyz>
      <limit><lower>{lower}</lower><upper>{upper}</upper><effort>18</effort><velocity>8</velocity></limit>
      <dynamics><damping>0.12</damping><friction>0.02</friction></dynamics></axis></joint>"""


def leg(name: str, x: float, y: float, side: int) -> str:
    shoulder_z, hip_offset, upper, lower = 0.360, 0.042, 0.140, 0.140
    hip_y = y + side * hip_offset
    abd = f"{x} {y + side * hip_offset / 2} {shoulder_z} 0 0 0"
    upper_pose = f"{x} {hip_y} {shoulder_z - upper / 2} 0 0 0"
    lower_pose = f"{x} {hip_y} {shoulder_z - upper - lower / 2} 0 0 0"
    foot_pose = f"{x} {hip_y} 0.025 0 0 0"
    abd_link = box_link(f"{name}_abduction_link", abd, "0.030 0.084 0.030", 0.10, "0.82 0.68 0.10 1")
    upper_link = box_link(f"{name}_upper_link", upper_pose, "0.032 0.032 0.140", 0.19, "0.08 0.80 0.78 1" if side > 0 else "0.96 0.38 0.14 1")
    lower_link = box_link(f"{name}_lower_link", lower_pose, "0.027 0.027 0.140", 0.13, "0.08 0.80 0.78 1" if side > 0 else "0.96 0.38 0.14 1")
    foot = f"""<link name=\"{name}_foot\"><pose>{foot_pose}</pose>{inertial(0.035)}
      <visual name=\"visual\"><geometry><sphere><radius>0.022</radius></sphere></geometry>
      <material><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>
      <collision name=\"collision\"><geometry><sphere><radius>0.022</radius></sphere></geometry>
      <surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision></link>"""
    # x is forward, y is left, z is up.  Roll around x spreads a leg
    # laterally; pitch around y moves it fore/aft.  z would be steering yaw,
    # which this robot does not have.
    abd_joint = joint(f"{name}_abduction", "body", f"{name}_abduction_link", f"{x} {y} {shoulder_z} 0 0 0", "1 0 0", -0.78, 0.78)
    hip_joint = joint(f"{name}_hip", f"{name}_abduction_link", f"{name}_upper_link", f"{x} {hip_y} {shoulder_z} 0 0 0", "0 1 0", -1.75, 1.57)
    knee_joint = joint(f"{name}_knee", f"{name}_upper_link", f"{name}_lower_link", f"{x} {hip_y} {shoulder_z - upper} 0 0 0", "0 1 0", -2.80, 0.20)
    foot_joint = f"""<joint name=\"{name}_foot_fixed\" type=\"fixed\">
      <pose>{x} {hip_y} {shoulder_z - upper - lower} 0 0 0</pose>
      <parent>{name}_lower_link</parent><child>{name}_foot</child></joint>"""
    plugins = "\n".join(controller(f"{name}_{axis}") for axis in ("abduction", "hip", "knee"))
    return "\n".join((abd_link, upper_link, lower_link, foot, abd_joint, hip_joint, knee_joint, foot_joint, plugins))


def main() -> None:
    body = box_link("body", "0 0 0.400 0 0 0", "0.338 0.150 0.080", 2.0, "0.10 0.16 0.27 1")
    legs = "\n".join(leg(*spec) for spec in LEGS)
    OUT.write_text(f"""<?xml version=\"1.0\"?>
<sdf version=\"1.10\"><world name=\"aura_quadruped_world\">
  <gravity>0 0 -9.81</gravity>
  <physics name=\"dart\" type=\"dart\"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename=\"gz-sim-physics-system\" name=\"gz::sim::systems::Physics\"/>
  <plugin filename=\"gz-sim-scene-broadcaster-system\" name=\"gz::sim::systems::SceneBroadcaster\"/>
  <plugin filename=\"gz-sim-user-commands-system\" name=\"gz::sim::systems::UserCommands\"/>
  <plugin filename=\"gz-sim-sensors-system\" name=\"gz::sim::systems::Sensors\"><render_engine>ogre2</render_engine></plugin>
  <light type=\"directional\" name=\"sun\"><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows><direction>-0.4 0.3 -0.85</direction></light>
  <model name=\"ground\"><static>true</static><link name=\"link\"><collision name=\"collision\"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry><surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision><visual name=\"visual\"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry><material><diffuse>0.12 0.12 0.14 1</diffuse></material></visual></link></model>
  <model name=\"aura\">{body}\n{legs}
    <plugin filename=\"gz-sim-joint-state-publisher-system\" name=\"gz::sim::systems::JointStatePublisher\"><topic>/aura/joint_states</topic></plugin>
  </model>
</world></sdf>""")
    print(OUT)


if __name__ == "__main__":
    main()
