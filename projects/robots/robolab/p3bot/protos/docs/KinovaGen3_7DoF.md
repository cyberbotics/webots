# KinovaGen3_7DoF

Designed by **Kinova Robotics**, the **Kinova Gen3 7 DoF** is a lightweight 7-degree-of-freedom robotic manipulator designed for research, service robotics and advanced manipulation tasks.
It features a compact design, high precision joints and an integrated parallel gripper, making it suitable for mobile manipulation platforms and fixed-base applications.

More information about the Kinova Gen3 manipulator can be found on the official
[Kinova website](https://www.kinovarobotics.com/product/gen3-robot) or in the
[Kinova Gen3 technical documentation](https://www.kinovarobotics.com/resources).

## KinovaGen3_7DoF PROTO

Derived from [`Solid`](https://cyberbotics.com/doc/reference/solid).

```proto
KinovaGen3_7DoF {
    SFVec3f translation 0 0 0
    SFRotation rotation 0 0 1 0
    SFString name "KinovaGen3_7DoF"
    SFBool invertWrist FALSE
    SFString prefix "Right_"
    SFFloat armsMinPosition -0.032
    SFFloat armsMaxPosition 0.01
}
```

## KinovaGen3_7DoF Field Summary

- **`translation`**  
  Defines the initial position of the manipulator base.

- **`rotation`**  
  Defines the initial orientation of the manipulator.

- **`name`**  
  Defines the name of the arm instance.

- **`invertWrist`**  
  If set to `TRUE`, inverts the wrist orientation to support mirrored configurations.

- **`prefix`**  
  Prefix applied to all actuator, motor, and sensor names.  
  Useful when multiple arms are used in the same robot.

- **`armsMinPosition`**  
  Minimum linear position of the gripper fingers.

- **`armsMaxPosition`**  
  Maximum linear position of the gripper fingers.

## Description

The **Kinova Gen3 7DoF** is a **7-degree-of-freedom articulated robotic manipulator** designed for research and advanced manipulation tasks.  
This Webots PROTO provides a detailed physical and kinematic model, including realistic inertial properties, joint limits, and actuator constraints.

The model includes an integrated **two-finger parallel gripper** actuated through linear joints.

## Kinematics and Actuation

The manipulator consists of:

- **7 revolute joints**, each driven by a `RotationalMotor`
- **Position sensors** on all joints for precise state feedback
- Realistic **joint limits**, **maximum velocities**, and **torque constraints**

All joint and sensor names are automatically generated using the provided `prefix` field, enabling seamless multi-arm configurations.

## Wrist Configuration

The wrist can be configured using the `invertWrist` field:

- `FALSE` (default): Standard wrist orientation
- `TRUE`: Inverted wrist orientation, useful for mirrored left-arm setups

This feature is particularly useful when mounting two symmetric arms on a mobile platform.

## Gripper

The end-effector consists of a **parallel two-finger gripper**:

- Fingers are actuated via **linear motors**
- Independent position sensors are provided for each finger
- Opening range is defined by `armsMinPosition` and `armsMaxPosition`

The gripper geometry includes collision shapes to ensure correct physical interaction.

## Physics and Collision

Each link of the manipulator:

- Has a dedicated **bounding object**
- Includes accurate **mass**, **center of mass**, and **inertia matrix**
- Supports physically realistic simulation behavior

This makes the Kinova Gen3 suitable for:
- Manipulation research
- Grasp planning
- Force-aware interaction experiments