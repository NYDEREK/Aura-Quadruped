# Aura quadruped — Gazebo / DART

This is a real rigid-body simulation, not the desktop app's drawing.  Gazebo
solves gravity, collisions, friction, joint limits, and actuator effort through
the DART physics engine.

The model has four legs (`lf`, `rf`, `lr`, `rr`) and exactly these three axes
per leg, in Gazebo coordinates (`x` forward, `y` left, `z` up):

| Joint | Axis | Purpose |
|---|---:|---|
| `*_abduction` | `x` | leg roll / lateral displacement |
| `*_hip` | `y` | hip pitch, forward–back motion |
| `*_knee` | `y` | knee pitch |

There is no `z` yaw / steering joint.

The editable dimensions are at the beginning of `generate_aura_quadruped.py`:
the body footprint, upper and lower leg length, hip offset, masses, friction,
joint ranges, and controller gains.  Regenerate the SDF after changing them.

## Run

In two terminals from the project root:

```zsh
./gazebo/launch.sh
./gazebo/run_controller.sh
```

The first starts Gazebo's DART server.  The second sends the neutral standing
pose to all 12 joints at 50 Hz.  Use `./gazebo/run_controller.sh --demo` only
to verify the bounded motion of the real axes; it is a mechanical test, not a
claimed walking controller.

## Current visual-client limitation

On this Mac, the locally installed conda-forge Gazebo 8 GUI cannot load its
OGRE renderer correctly.  The model, transport topics, and DART physics server
work headlessly and have been tested.  The next useful step is to calibrate
the model dimensions and contact parameters, then implement an inverse
kinematics + contact-aware gait controller before reconnecting the desktop
app and DualSense input.
