## PAL Robotics' TIAGo Steel

%robot tiago-steel images/robots/tiago-steel/model.thumbnail.png

Designed by PAL Robotics, TIAGo Steel is a two-wheel human-like robot with a torso, a head and one articulated arm finish by a parallel gripper as end-effector.
It is completely ROS based, fully customizable and expandable with extra sensors and devices like other end-effector (Hey5 hand).
TIAGo's software has out-of-the box extras available like navigation upgrade, human-robot interaction skills or teleoperation applications.

More information on the TIAGo robot can be found [here](http://pal-robotics.com/robots/tiago/).

### TiagoSteel PROTO

Derived from [Robot](../reference/robot.md).

```
TiagoSteel {
  SFVec3f     translation        0 0 0
  SFRotation  rotation           1 0 0 -1.5708
  SFString    name               "TIAGo Steel"
  SFString    controller         "tiagoSteel"
  SFString    controllerArgs     ""
  SFString    customData         ""
  SFBool      supervisor         FALSE
  SFBool      synchronization    TRUE
}
```

> **File location**: "WEBOTS\_HOME/projects/robots/pal_robotics/tiago/protos/TiagoSteel.proto"

### Sample

You will find the following sample in this folder: "WEBOTS\_HOME/projects/robots/pal_robotics/tiago/worlds".

#### tiago\_steel.wbt

![tiago_steel.wbt.png](images/robots/tiago-steel/tiago_steel.wbt.thumbnail.jpg) This simulation shows a TIAGo Steel robot following a predefined path on a chequered parquetry.
