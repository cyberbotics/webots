## PAL Robotics' TIAGo OMNI Base

%robot tiago-omni-base images/robots/tiago_omni-base/model.png

Designed by PAL Robotics, TIAGo OMNI Base is a mecanum omnidirectional robot.
The model is a modular mobile platform that automates deliveries in industrial environment.
It is completely ROS based and customizable with accessories to adapt to any specific need.

More information on the TIAGo OMNI Base robot can be found on their [website](http://pal-robotics.com/robots/tiago-base/) or in the [technical specifications datasheet](https://pal-robotics.com/wp-content/uploads/2022/11/Datasheet-TIAGo-OMNI-Base.pdf).

### TiagoOmniBase PROTO

Derived from [Robot](https://cyberbotics.com/doc/reference/robot).

```
TiagoOmniBase {
  SFVec3f      translation      0 0 0
  SFRotation   rotation         0 0 1 0
  SFString     name             "TIAGo OMNI Base"
  SFString     controller       "tiago_omni_base"
  MFString     controllerArgs   []
  SFString     window          "<generic>"
  SFString     customData       ""
  SFBool       supervisor       FALSE
  SFBool       synchronization  TRUE
  SFBool       selfCollision    FALSE
  SFString     model            "PAL Robotics - TIAGo OMNI Base"
  MFNode       bodySlot         []
  MFNode       lidarSlot        []
}
```

> **File location**: "[WEBOTS\_HOME/projects/robots/pal\_robotics/tiago\_omni\_base/protos/TiagoOmniBase.proto]({{ url.github_tree }}/projects/robots/pal_robotics/tiago_omni_base/protos/TiagoOmniBase.proto)"

#### TiagoPmbBase Field Summary

- `bodySlot`:  Extends the robot with new nodes (such as the `TiagoBody` for example).
- `lidarSlot`: Extends the robot with a front lidar sensor.

### Sample

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/pal\_robotics/tiago\_omni\_base/worlds]({{ url.github_tree }}/projects/robots/pal_robotics/tiago_omni_base/worlds)".

#### tiago\_omni\_base.wbt

![tiago_omni_base.wbt.png](images/robots/tiago_base/tiago_omni_base.wbt.png) This simulation shows a TIAGo OMNI Base moving in an industrial environment using a Braitenberg algorithm using the information received by its lidar.
