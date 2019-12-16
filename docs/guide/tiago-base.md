## PAL Robotics' TIAGo Base

%robot tiago-base images/robots/tiago_base/model.thumbnail.png

Designed by PAL Robotics, TIAGo Base is a two-wheeled robot.
The model is a modular mobile platform that automates deliveries in industrial environment. It is completely ROS based and customizable with accessories to adapt to any specific need.

More information on the TIAGo Base robot can be found on their [website](http://pal-robotics.com/robots/tiago-base/) or in the [technical specifications datasheet](http://pal-robotics.com/wp-content/uploads/2019/06/TIAGo-Base_Datasheet.pdf).

### TiagoBase PROTO

Derived from [Robot](../reference/robot.md).

```
TiagoBase {
  SFVec3f      translation     0 0 0
  SFRotation   rotation        1 0 0 -1.5708
  SFString     name            "TIAGo Base"
  SFString     controller      "tiagoBase"
  SFString     controllerArgs  ""
  SFString     customData      ""
  SFBool       supervisor      FALSE
  SFBool       synchronization TRUE
  SFBool       selfCollision   FALSE
  SFString     model           "TIAGo Base"
  SFString     description     "two-wheeled robot designed by PAL Robotics"
  MFNode       bodySlot        []
  MFNode       lidarSlot       []
}
```

> **File location**: "[WEBOTS\_HOME/projects/robots/pal\_robotics/tiago\_base/protos/TiagoBase.proto](https://github.com/cyberbotics/webots/tree/master/projects/robots/pal_robotics/tiago_base/protos/TiagoBase.proto)"

#### TiagoBase Field Summary

- `bodySlot`:  Extends the robot with new nodes (such as the `TiagoBody` for example).
- `lidarSlot`: Extends the robot with a lidar.

### Sample

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/pal\_robotics/tiago\_base/worlds](https://github.com/cyberbotics/webots/tree/master/projects/robots/pal_robotics/tiago_base/worlds)".

#### tiago\_base.wbt

![tiago_base.wbt.png](images/robots/tiago_base/tiago_base.wbt.thumbnail.jpg) This simulation shows a Tiago Base moving in an industrial environment using a Braitenberg algorithm using the information received by its lidar.
