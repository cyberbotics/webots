## PAL Robotics' TIAGo Base

%robot tiago-base images/robots/tiago-base/model.thumbnail.png

Designed by PAL Robotics, TIAGo Base is a two-wheel robot.
The model is a modular mobile platform that automates deliveries in industrial environment. It is completely ROS based and customizable with accessories to adapt to any specific need.
TIAGo's software has out-of-the box extras available like navigation upgrade.

More information on the TIAGo base robot can be found [here](http://pal-robotics.com/robots/tiago-base/)

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
  SFString     model           "TIAGo Base"
  SFString     description     "two-wheel robot designed by PAL Robotics"
  MFNode       lidarExtension  []
  MFNode       bodyExtension   []
}
```

> **File location**: "WEBOTS\_HOME/projects/robots/pal_robotics/tiago/protos/TiagoBase.proto"

#### TiagoBase Field Summary

- `lidarExtension`: Defines the lidar used {Sick TIM551 or Hokuyo URG 04LX_UG01}.
- `bodyExtension`: Defines body extension used {TiagoBody, Fixed shelves, Safety box, roller conveyor, boxes, lifter}.

### Sample

You will find the following sample in this folder: "WEBOTS\_HOME/projects/robots/pal_robotics/tiago/worlds".

#### tiago\_base.wbt

![tiago_base.wbt.png](images/robots/tiago-base/tiago_base.wbt.thumbnail.jpg) This simulation shows a TIAGo Base robot following a predefined path on a chequered parquetry.
