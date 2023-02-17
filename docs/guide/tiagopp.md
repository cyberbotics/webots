## PAL Robotics' TIAGopp

%robot tiagopp images/robots/tiagopp/model.thumbnail.png

Designed by PAL Robotics, TIAGopp is a two-wheeled human-like robot with two articulated arms finished by parallel grippers as end-effector.
The model includes two articulated arms with 7-DoF to perform coordinated dual-arm actions.
As the TIAGo model, this bi-manual version is completely ROS based, fully customizable and expandable with extra sensors and devices like end-effector (parallel gripper or Hey5 hand).

More information on the TIAGopp robot can be found on their [website](http://blog.pal-robotics.com/tiago-bi-manual-robot-research/) or in the [technical specifications datasheet](http://pal-robotics.com/wp-content/uploads/2019/07/Datasheet_TIAGo_Complete.pdf).

### Movie Presentation

![youtube video](https://www.youtube.com/watch?v=2KYpuaREQm0)

### Tiagopp PROTO

Derived from [Robot](../reference/robot.md).
```
Tiagopp {
  SFVec3f     translation           0 0 0.095
  SFRotation  rotation              0 0 1 0
  SFString    name                  "TIAGopp"
  SFString    controller            "tiagopp"
  MFString    controllerArgs        []
  SFString    customData            ""
  SFBool      supervisor            FALSE
  SFBool      synchronization       TRUE
  SFBool      selfCollision         FALSE
  MFNode      lidarSlot             []
  SFNode      endEffectorRightSlot  TiagoGripper { name "right" }
  SFNode      endEffectorLeftSlot   TiagoGripper { name "left" }
}
```
> **File location**: "[WEBOTS\_HOME/projects/robots/pal\_robotics/tiagopp/protos/Tiagopp.proto]({{ url.github_tree }}/projects/robots/pal_robotics/tiagopp/protos/Tiagopp.proto)"

#### Tiagopp Field Summary

- `endEffectorRightSlot`: Extends the right arm with new nodes (such as the `TiagoRightHey5` for example).
- `endEffectorLeftSlot`: Extends the left arm with new nodes (such as the `TiagoLeftHey5` for example).

### Sample

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/pal\_robotics/tiagopp/worlds]({{ url.github_tree }}/projects/robots/pal_robotics/tiagopp/worlds)".

#### tiagopp.wbt

![tiagopp.wbt.png](images/robots/tiagopp/tiagopp.wbt.thumbnail.jpg) This simulation shows a Tiagopp making hello with both arms in an industrial environment. It can also be controlled using the arrows on the keyboard.
