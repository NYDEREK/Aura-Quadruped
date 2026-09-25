# Aura robot-control architecture

The control model is deliberately independent from the user interface, radio
transport, simulator, and physical servo bus.

```text
DualSense → robot_gait (Aura, 50 Hz)
             │ IK target + speed limit
             ▼
  robot_model: Servo → Axis → Leg → Robot (Aura)
             │ raw ST3215 commands and feedback
             ▼
      robot_control (50 Hz, disarmed by default)
             │
       ┌─────┴─────┐
       ▼           ▼
 ST3215 TTL     Wi-Fi telemetry → Aura desktop display
```

## Blocks

- **Servo** is the physical ST3215 identity, its direction, electronic centre,
  mechanical software range, acceleration and native speed limit.
- **Axis** owns exactly one servo and is one of `abduction`, `hip`, `knee`.
  A servo cannot be assigned to a second axis.
- **Leg** owns three axes in the fixed order abduction → hip → knee.
- **Robot** owns LF, RF, LR and RR legs, for 12 axes total.
- **Controller** is the only code allowed to write movement packets. At 50 Hz it
  encodes every configured position-axis target and sends one Feetech
  `SYNC_WRITE` broadcast. All joints therefore receive their next target from
  the same packet.
- **Gait planner** runs on Aura, not on the desktop. It interprets DualSense
  input, handles R1 gait selection, L1 spin mode and × jump state, then solves
  the three-axis inverse kinematics for all four legs. A separate bounded
  velocity/acceleration trajectory block turns each IK reference into the next
  continuous joint setpoint before it is submitted to the controller.
- **Odometry** runs on Aura at the same 50 Hz rate. During a planned stance it
  binds each measured foot pose to a world point and solves the body’s planar
  translation from those contacts. A yaw estimate is enabled only during a
  commanded turn or in-place rotation, so encoder latency cannot invent a
  turn while walking straight.
- **Desktop application** writes the axis mapping, displays Aura's 50 Hz target
  stream and displays the latest returned servo measurements. It never
  generates a physical target packet.

## Safety state

The axis mapping is persisted in Aura's NVS after each accepted edit, so it
survives a restart without the computer. New or invalid NVS records resolve to
an empty disarmed model. Mapping an ID never enables torque. A complete set of
12 distinct axes plus an explicit arm action is required before the controller
sends any physical position packet. Disarm clears the armed state before it
sends torque-off commands, so a failed bus transaction cannot lead to another
scheduled motion packet.

## Units

High-level angles use radians, centred on the physical axis zero. The model
converts that to 0…4095 ST3215 ticks only at the hardware boundary. A direction
of `+1` or `-1`, per axis, makes left/right mirrored mounting explicit. The
speed field is retained in the ST3215's native 0…3400 unit until each servo
model is calibrated against measured motion; it is never mislabeled as degrees
per second.

## Feedback and phase timing

The same axis state stores measured position, speed, load, voltage,
temperature, current and moving flag. `robot_control_read_axis` updates this
state from the physical servo; the desktop frame shows the last valid
measurement beside Aura's current target. Feedback does not stop at every
waypoint: when a measured joint error grows beyond the commissioning band,
the static one-foot gait smoothly reduces or, for a large backlog, pauses the
phase while the joint trajectories continue to catch up. Dynamic profiles
keep a shared deterministic phase for feet and torso; asynchronous TTL reads
do not pause their clock. Before torque is enabled Aura reads all 12 positions and seeds each
trajectory from those fresh encoders, preventing a stale target from producing
an arming jump.

The world position is odometry, not an external measurement: it is useful for
the desktop camera and simulation, but needs foot-contact sensing and a
working IMU to correct slip and accumulated drift on the assembled robot.

## Three-leg configuration and balance telemetry

Command `0x2d enabled:u8 excluded_leg:u8` selects three-legged walking.
`enabled` is 0/1, legs are FL=0, FR=1, RL=2, RR=3. It is accepted only while
disarmed and outside calibration. It persists planning configuration and
never enables torque. Profile 5 uses the existing profile command/readback;
profiles 1–4 and servo-calibration NVS formats are unchanged.

The existing 20-byte body-reference event `0x31` now uses reserved bytes:
14 three-leg mode enabled, 15 excluded leg, 16 measured-pose swing correction
active, 17 leg bitmask of unreachable/clipped targets, 18 extension version=1.
Older firmware leaves these bytes zero; the desktop disables the new mode
controls until version 1 is reported. All corrections run on ESP32 without
the app. See `locomotion-model.md` for equations and limitations.
