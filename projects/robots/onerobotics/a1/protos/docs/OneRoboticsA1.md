# OneRobotics A1

The [OneRobotics A1](http://www.onerobot.com/) is a compact fixed-base robotic arm with seven revolute joints. This
model represents the independent right-arm revision published by OneRobotics.

The model was converted from the published
[`a1_r.urdf`](https://github.com/katazen/onerobot_h1/blob/ecf530911284ba0e559f7a24dc222fd8e60d31ed/source/h1_reach/h1_reach/assets/urdf/A1_2026/a1_r.urdf).
The visual meshes, kinematics, joint limits, masses, centers of mass, and inertia matrices come from source commit
[`ecf530911284ba0e559f7a24dc222fd8e60d31ed`](https://github.com/katazen/onerobot_h1/commit/ecf530911284ba0e559f7a24dc222fd8e60d31ed).
Collision meshes were approximated with boxes, mesh filenames were changed to lowercase, and the source repository's
published hardware motor limits were applied.

The robot assets are © 2026 OneRobotics and licensed under the
[Creative Commons Attribution 4.0 International license](https://creativecommons.org/licenses/by/4.0/).

## OneRoboticsA1 PROTO

Derived from [`Robot`](https://cyberbotics.com/doc/reference/robot).

```proto
OneRoboticsA1 {
  SFVec3f    translation       0 0 0
  SFRotation rotation          0 0 1 0
  SFString   name              "OneRobotics A1"
  SFString   controller        "<generic>"
  MFString   controllerArgs    []
  SFString   window            "<generic>"
  SFString   customData        ""
  SFBool     supervisor        FALSE
  SFBool     synchronization   TRUE
  SFBool     selfCollision     FALSE
  MFNode     endEffectorSlot   []
  SFBool     staticBase        TRUE
}
```

### Field Summary

- `translation`: position of the arm base.
- `rotation`: orientation of the arm base.
- `name`: name of the `Robot` node.
- `controller`: controller used by the arm.
- `controllerArgs`: arguments passed to the controller.
- `window`: robot window shown for the arm.
- `customData`: application-specific robot data.
- `supervisor`: whether the controller has supervisor privileges.
- `synchronization`: whether the controller is synchronized with the simulation.
- `selfCollision`: whether collisions between the arm's links are enabled.
- `endEffectorSlot`: nodes attached to the Link7 flange frame.
- `staticBase`: whether the arm base is fixed to the static environment.

## Devices

Each joint has one `RotationalMotor` named `joint1-a1_r` through `joint7-a1_r` and one `PositionSensor` using the
same name followed by `_sensor`.

| Joint | Minimum position [rad] | Maximum position [rad] | Maximum velocity [rad/s] | Maximum torque [N m] |
| --- | ---: | ---: | ---: | ---: |
| `joint1-a1_r` | -1.04 | 3.14 | 2.6179938779914944 | 26.859 |
| `joint2-a1_r` | -3.14 | 0.26 | 2.6179938779914944 | 26.859 |
| `joint3-a1_r` | -2.76 | 2.76 | 2.6179938779914944 | 26.859 |
| `joint4-a1_r` | -1.92 | 1.92 | 12.566370614359172 | 5.975 |
| `joint5-a1_r` | -2.23 | 2.23 | 12.566370614359172 | 5.975 |
| `joint6-a1_r` | -1.57 | 1.57 | 12.566370614359172 | 5.975 |
| `joint7-a1_r` | -2.76 | 2.76 | 12.566370614359172 | 5.975 |

## Known Limitations

- This independent-arm revision does not include a gripper. Attach an end effector using `endEffectorSlot`.
- The Link7 mass and inertia are explicitly marked as a placeholder in the source URDF and should be replaced when
  measured flange data becomes available.
- Collision geometry uses axis-aligned boxes for stable and efficient contact simulation; it is less detailed than the
  visual meshes.
