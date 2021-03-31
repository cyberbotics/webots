## Niryo's Ned

%robot Ned images/robots/ned/model.thumbnail.png

"Ned" is a 6-axis collaborative robot developed by [Niryo](https://niryo.com/). It is  based on open-source technologies designed for Education, Professional Training and Research.

### Ned PROTO

Derived from [Robot](https://cyberbotics.com/doc/reference/robot).

```
PROTO NedGripper [
  field  SFVec3f     translation     0 0 0
  field  SFRotation  rotation        0 1 0 0
  field  SFString    name            "NedGripper"
  field  SFString    controller      "ned_c_controller"
  field  MFString    controllerArgs  []
  field  SFString    customData      ""
  field  SFBool      supervisor      FALSE
  field  SFBool      synchronization TRUE
  field  SFBool      selfCollision   FALSE
]
```

> **File location**: "WEBOTS\_HOME/projects/robots/niryo/ned/protos/NedGripper.proto"

### Samples

You will find the following sample in the folder: "WEBOTS\_HOME/projects/robots/niryo/ned/worlds".

#### Ned.wbt

![ned.wbt.png]() This simulation shows Ned in a working environment. You can control the robot with your keyboard, launch a demo and run a pick and place. Feel free to use this simulation in order to make your own world and controller with Ned. 

