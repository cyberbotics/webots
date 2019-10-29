## PAL Robotics' TIAGo Iron

%robot tiago-iron images/robots/tiago-iron/model.thumbnail.png

Designed by PAL Robotics, TIAGo Iron is a two-wheel human-like robot with a torso and a head but no articulated arm.
The model is a modular mobile platform that allows human-robot interaction. It is completely ROS based and customizable with accessories to adapt to any specific need.
TIAGo's software has out-of-the box extras available like navigation upgrade and human-robot interaction skills.

More information on the TIAGo robot can be found [here](http://pal-robotics.com/robots/tiago/).

### TiagoIron PROTO

Derived from [Robot](../reference/robot.md).

```
TiagoIron {
  SFVec3f     translation        0 0 0
  SFRotation  rotation           1 0 0 -1.5708
  SFString    name               "TIAGo Iron"
  SFString    controller         "tiagoIron"
  SFString    controllerArgs     ""
  SFString    customData         ""
  SFBool      supervisor         FALSE
  SFBool      synchronization    TRUE
}
```

> **File location**: "WEBOTS\_HOME/projects/robots/pal_robotics/tiago/protos/TiagoIron.proto"

### Sample

You will find the following sample in this folder: "WEBOTS\_HOME/projects/robots/pal_robotics/tiago/worlds".

#### tiago\_iron.wbt

![tiago_iron.wbt.png](images/robots/tiago-iron/tiago_iron.wbt.thumbnail.jpg) This simulation shows a TIAGo Iron robot following a predefined path on a chequered parquetry.
