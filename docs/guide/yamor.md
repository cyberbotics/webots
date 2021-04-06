## BioRob Yamor

%robot yamor images/robots/yamor/model.thumbnail.png

The "Yamor" robot is a modular robot developed by the [EPFL BioRob laboratory](https://biorob.epfl.ch/).

### Yamor PROTO

Derived from [Robot](../reference/robot.md).

```
Yamor {
  SFVec3f    translation     0 0 0
  SFRotation rotation        0 1 0 0
  SFString   name            "Yamor"
  SFString   controller      "yamor"
  MFString   controllerArgs  []
  SFString   customData      ""
  SFBool     supervisor      FALSE
  SFBool     synchronization TRUE
  MFNode     extensionSlot   []
}
```

> **File location**: "WEBOTS\_HOME/projects/robots/epfl/biorob/protos/Yamor.proto"

#### Yamor Field Summary

- `extensionSlot`: Extends the robot with new nodes in the extension slot.

### Samples

You will find the following sample in this folder: "WEBOTS\_HOME/projects/robots/epfl/biorob/worlds".

#### yamor.wbt

![yamor.wbt.png](images/robots/yamor/yamor.wbt.thumbnail.jpg) In this example, eight Yamor robot "modules" attach to and detach from each other using [Connector](../reference/connector.md) devices.
Connector devices are used to simulate the mechanical connections of docking systems.
Each module is controlled by an instance of the same robot controller, which can only connect or disconnect from another module and move its rotational motor.
From these simple behaviors, a global behavior emerges.
The resulting robot (composed of all the modules) moves forward as a worm.
From times to times, a module is dropped, showing the robustness of the locomotion behavior of such a system.
