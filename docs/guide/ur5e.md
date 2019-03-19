## Universal Robot UR5e

%robot ur5e images/robots/ur5e/model.png

The [Universal Robot UR5e](https://www.universal-robots.com/products/ur5-robot/) is a flexible collaborative robot arm with 6 degree of freedom.

### Movie Presentation

![youtube video](https://www.youtube.com/watch?v=d6NJLFg1x9M)  TODO

### UR5e PROTO

Derived from [Robot](../reference/robot.md).

```
UR5e {
  SFVec3f    translation     0 0 0
  SFRotation rotation        1 0 0 4.712388966
  SFString   name            "UR5e"
  SFString   controller      "void"
  SFString   controllerArgs  ""
  SFBool     synchronization TRUE
  SFBool     selfCollision   TRUE
  MFNode     toolSlot        []
  SFBool     staticBase      TRUE
}
```

> **File location**: "WEBOTS\_HOME/projects/robots/universal_robots/ur5e/protos/UR5e.proto"

#### UR5e Field Summary

- `toolSlot`: Extend the robot with new nodes at the end of the arm.

- `staticBase`: Defines if the robot base should be pinned to the static environment.

### Samples

You will find the following sample in this folder: "WEBOTS\_HOME/projects/robots/universal_robots/ur5e/worlds".

#### ur5e.wbt

TODO.
