## PAL Robotics' TIAGo++

%robot tiago++ images/robots/tiago++/model.thumbnail.png

Designed by PAL Robotics, TIAGo++ is a two-wheeled human-like robot with two articulated arms finished by parallel grippers as end-effector.
The model includes two articulated arms with 7-DoF to perform coordinated dual-arm actions.
As the TIAGo model, this bi-manual version is completely ROS based, fully customizable and expandable with extra sensors and devices like end-effector (parallel gripper or Hey5 hand).
TIAGo's software has out-of-the box extras available like navigation upgrade, human-robot interaction skills or teleoperation applications.

More information on the TIAGo++ robot can be found on their [website](http://blog.pal-robotics.com/tiago-bi-manual-robot-research/) or in the [technical specifications datasheet](http://pal-robotics.com/wp-content/uploads/2019/07/Datasheet_TIAGo_Complete.pdf).

### Tiago++ PROTO

Derived from [Robot](../reference/robot.md).
```
Tiago++ {
  SFVec3f     translation        0 0 0
  SFRotation  rotation           1 0 0 -1.5708
  SFString    name               "TIAGo++"
  SFString    controller         "tiago++"
  SFString    controllerArgs     ""
  SFString    customData         ""
  SFBool      supervisor         FALSE
  SFBool      synchronization    TRUE
  MFNode      endEffectorRight   TiagoGripper{}
  MFNode      endEffectorLeft    TiagoGripper{}
}
```
> **File location**: "WEBOTS\_HOME/projects/robots/pal_robotics/tiago++/protos/Tiago++.proto"

#### Tiago++ Field Summary

- `endEffectorRight`: Defines the end-effector used by default but it can be changed with TiagoRightHey5.
- `endEffectorLeft`:  Defines the end-effector used by default but it can be changed with TiagoLeftHey5.

### Sample

You will find the following sample in this folder: "WEBOTS\_HOME/projects/robots/pal\_robotics/tiago++/worlds".

#### tiago++.wbt

![tiago++.wbt.png](images/robots/tiago++/tiago++.wbt.thumbnail.jpg) This simulation shows a TIAGo++ robot that automatically moves straight ahead but can also be controlled using the keyboard arrows.
