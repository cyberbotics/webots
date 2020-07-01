## PAL Robotics' TIAGo Steel

%robot tiago-steel images/robots/tiago_steel/model.thumbnail.png

Designed by PAL Robotics, TIAGo Steel is a two-wheeled human-like robot with a torso, a head and one articulated arm finished by a parallel gripper as end-effector.
It is completely ROS based, fully customizable and expandable with extra sensors and devices like other end-effector (Hey5 hand).

More information on the TIAGo Steel robot can be found on their [website](http://pal-robotics.com/robots/tiago/) or in the [technical specifications datasheet](http://pal-robotics.com/wp-content/uploads/2019/07/Datasheet_TIAGo_Complete.pdf).

### TiagoSteel PROTO

Derived from [Robot](../reference/robot.md).

```
TiagoSteel {
  SFVec3f     translation      0 0 0
  SFRotation  rotation         1 0 0 -1.5708
  SFString    name             "TIAGo Steel"
  SFString    controller       "tiagoSteel"
  MFString    controllerArgs   []
  SFString    customData       ""
  SFBool      supervisor       FALSE
  SFBool      synchronization  TRUE
  SFBool      selfCollision    FALSE
  MFNode      lidarSlot        []
}
```

> **File location**: "[WEBOTS\_HOME/projects/robots/pal\_robotics/tiago\_steel/protos/TiagoSteel.proto](https://github.com/cyberbotics/webots/tree/master/projects/robots/pal_robotics/tiago_steel/protos/TiagoSteel.proto)"

### Sample

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/pal\_robotics/tiago\_steel/worlds](https://github.com/cyberbotics/webots/tree/master/projects/robots/pal_robotics/tiago_steel/worlds)".

#### tiago\_steel.wbt

![tiago_steel.wbt.png](images/robots/tiago_steel/tiago_steel.wbt.thumbnail.jpg) This simulation shows a Tiago Steel making hello in an industrial environment.
It can also been controlled using the keyboard arrows.
