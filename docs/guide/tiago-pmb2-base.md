## PAL Robotics' TIAGo PMB2 Base

%robot tiago-pmb2-base images/robots/tiago_pmb2-base/model.thumbnail.png

Designed by PAL Robotics, TIAGo PMB2 Base is a two-wheeled robot.
The model is a modular mobile platform that automates deliveries in industrial environment.
It is completely ROS based and customizable with accessories to adapt to any specific need.

More information on the TIAGo PMB2 Base robot can be found on their [website](http://pal-robotics.com/robots/tiago-base/) or in the [technical specifications datasheet](https://pal-robotics.com/wp-content/uploads/2021/10/Datasheet_TIAGo-Base-2021.pdf).

### TiagoPmb2Base PROTO

Derived from [Robot](../reference/robot.md).

```
TiagoPmb2Base {
  SFVec3f      translation      0 0 0.095
  SFRotation   rotation         0 0 1 0
  SFString     name             "TIAGo PMB2 Base"
  SFString     controller       "tiago_pmb2_base"
  MFString     controllerArgs   []
  SFString     window          "<generic>"
  SFString     customData       ""
  SFBool       supervisor       FALSE
  SFBool       synchronization  TRUE
  SFBool       selfCollision    FALSE
  SFString     model            "PAL Robotics - TIAGo Base"
  MFNode       bodySlot         []
  MFNode       lidarSlot        []
}
```

> **File location**: "[WEBOTS\_HOME/projects/robots/pal\_robotics/tiago\_pmb2\_base/protos/TiagoPmb2Base.proto]({{ url.github_tree }}/projects/robots/pal_robotics/tiago_pmb2_base/protos/TiagoPmb2Base.proto)"

#### TiagoPmb2Base Field Summary

- `bodySlot`:  Extends the robot with new nodes (such as the `TiagoBody` for example).
- `lidarSlot`: Extends the robot with a lidar.

### Sample

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/pal\_robotics/tiago\_pmb2\_base/worlds]({{ url.github_tree }}/projects/robots/pal_robotics/tiago_pmb2_base/worlds)".

#### tiago\_pmb2\_base.wbt

![tiago_pmb2_base.wbt.png](images/robots/tiago_base/tiago_pmb2_base.wbt.thumbnail.jpg) This simulation shows a TIAGo PMB2 Base moving in an industrial environment using a Braitenberg algorithm using the information received by its lidar.
