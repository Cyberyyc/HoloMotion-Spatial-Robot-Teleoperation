# HoloMotion · Spatial Robot Teleoperation

**Hand-driven Cartesian teleoperation through a head-aligned spatial reference frame.**

Selected HoloLens 2 / MRTK runtime modules for transmitting human hand-motion intent to a ROS-connected robot arm. The control loop uses relative hand displacement and quaternion rotation rather than absolute headset-world coordinates.

## Interaction pipeline

```text
Left-hand pinch → clutch / reference capture
Right-hand metacarpal pose → head-aligned relative motion
                          → jump rejection + positional smoothing
                          → Cartesian displacement + quaternion increment
                          → ROS /arm/arm_follow
```

## Core sources

- `Runtime/hand_loc_new.cs`: clutch state, reference-frame capture, relative hand pose, smoothing, jump rejection, and ROS publishing.
- `Runtime/ROS_ImageMsg.cs`: ROS camera-image display.
- `Runtime/Messages/`: generated C# definitions for the arm command and its position/rotation fields.
- `RosMessages/msg/`: matching ROS message definitions.

## Control contract

The left thumb/middle-finger pinch toggles control with a 1 s cooldown. The right middle-metacarpal pose supplies motion. Translation is expressed relative to the captured headset yaw frame, filtered with coefficient 0.15, and scaled by 0.5. Position jumps above 3 cm are rejected for that sample. Orientation is transmitted as a relative quaternion.

`unity_robotics_demo_msgs/Arm_move` contains position, quaternion orientation, legacy Euler rotation, `init_pose`, and a six-element hand flag array. The quaternion fields are populated by the main controller; the legacy Euler fields remain at their defaults. Inactive control publishes an initialization/reset message. Topic and filtering settings are exposed in the Inspector.

## Integration

Use Unity 2021.3 or 2022.3, MRTK 2.8, Mixed Reality OpenXR, and [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector). Copy `Runtime` into the Unity project's Assets folder, configure the ROS endpoint with ROSConnection, and attach `hand_loc_new` to an active scene object. Use a ROS message package named `unity_robotics_demo_msgs` with the included definitions; the repository already includes the matching generated Unity classes.

The ROS-side robot controller consumes the displacement, rotation increment, and initialization flag. Set robot workspace limits and enable the robot through its own controller before a teleoperation session.

## Repository scope

Core interaction, message, and feedback sources. Unity caches, imported SDKs, robot models, recordings, scenes, and build outputs are excluded.

Application sources are MIT licensed; MRTK and Unity Robotics remain external dependencies under their respective licenses.
