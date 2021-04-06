## Niryo's Ned

%robot ned images/robots/ned/model.thumbnail.png

"Ned" is a 6-axis collaborative robot developed by [Niryo](https://niryo.com/). It is  based on open-source technologies designed for Education, Professional Training and Research.

### Ned PROTO

Derived from [Robot](../reference/robot.md).

```
PROTO NedGripper {
  field  SFVec3f     translation     0 0 0              # Is `Transform.translation`.
  field  SFRotation  rotation        0 1 0 0            # Is `Transform.rotation`.
  field  SFString    name            "NedGripper"       # Is `Robot.name`.
  field  SFString    controller      "ned_c_controller" # Is `Robot.controller`.
  field  MFString    controllerArgs  []                 # Is `Robot.controllerArgs`.
  field  SFString    customData      ""                 # Is `Robot.customData`.
  field  SFBool      supervisor      FALSE              # Is `Robot.supervisor`.
  field  SFBool      synchronization TRUE               # Is `Robot.synchronization`.
  field  SFBool      selfCollision   FALSE              # Is `Robot.selfCollision`.
}
```

> **File location**: "WEBOTS\_HOME/projects/robots/niryo/ned/protos/NedGripper.proto"

### Samples

You will find the following sample in the folder: "WEBOTS\_HOME/projects/robots/niryo/ned/worlds".

#### Ned.wbt

![ned.wbt.png](images/robots/ned/ned.wbt.thumbnail.jpg) This simulation shows Ned in a working environment. You can control the robot with your keyboard, launch a demo and run a pick and place. Feel free to use this simulation in order to make your own world and controller with Ned. 

