## F&P Robotics P-Rob 3

%robot p-rob3 images/robots/p-rob3/model.png

The [P-Rob 3](https://www.fp-robotics.com/en/p-rob) is a lightweight and soft covered collaborative robotic arm developed by [F&P Robotics](https://www.fp-robotics.com/en/).

### P-Rob3 PROTO

Derived from [Robot](../reference/robot.md).

```
PROTO P-Rob3 [
  SFVec3f    translation     0 0 0
  SFRotation rotation        1 0 0 -1.57079969
  SFString   name            "P-Rob3"
  SFString   controller      "void"
  SFString   controllerArgs  ""
  SFBool     supervisor      FALSE
  SFBool     synchronization TRUE
  SFBool     selfCollision   TRUE
  SFColor    mainColor       0.98 0.98 0.98
  SFColor    secondaryColor  0.036 0.3 0.615
  MFNode     toolSlot        []
  SFBool     staticBase      TRUE
]
```

> **File location**: "[WEBOTS\_HOME/projects/robots/fp\_robotics/p-rob3/protos/P-Rob3.proto](https://github.com/cyberbotics/webots/tree/master/projects/robots/fp_robotics/p-rob3/protos/P-Rob3.proto)"

#### P-Rob3 Field Summary

- `mainColor`: Defines the main color fo the robot.

- `secondaryColor`: Defines the secondary color of the robot.

- `toolSlot`: Extends the arm hand with new nodes.

- `staticBase`: Defines if the robot base should be pinned to the static environment.

### Samples

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/fp\_robotics/p-rob3/worlds/prob3.wbt](https://github.com/cyberbotics/webots/tree/master/projects/robots/fp_robotics/p-rob3/worlds/prob3.wbt)".

#### prob3.wbt

TODO...
