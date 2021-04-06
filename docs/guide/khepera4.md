## K-Team's Khepera IV

%robot khepera4 images/robots/khepera4/model.thumbnail.png

The "Khepera IV" robot is a two-wheeled robot produced by [K-Team](https://www.k-team.com/mobile-robotics-products/khepera-iv).
It is mounted by multiple sensors including 8 distance sensors.

### Movie Presentation

![youtube video](https://www.youtube.com/watch?v=RVOwk3FkIWo)

### Khepera4 PROTO

Derived from [Robot](../reference/robot.md).

```
Khepera4 {
  SFVec3f    translation         0 0 0
  SFRotation rotation            0 1 0 0
  SFString   name                "Khepera IV"
  SFString   controller          "khepera4"
  MFString   controllerArgs      []
  SFString   customData          ""
  SFBool     supervisor          FALSE
  SFBool     synchronization     TRUE
  SFString   bodyMaterial        "default"
  SFString   wheelMaterial       "default"
  SFString   casterWheelMaterial "khepera4 caster wheel"
  SFInt32    emitterChannel      1
  SFInt32    receiverChannel     1
  MFNode     turretSlot          []
}
```

> **File location**: "WEBOTS\_HOME/projects/robots/k-team/khepera4/protos/Khepera4.proto"

#### Khepera4 Field Summary

- `bodyMaterial`: Defines the `contactMaterial` field of the body [Solid](../reference/solid.md).

- `wheelMaterial`: Defines the `contactMaterial` field of the wheel [Solid](../reference/solid.md) nodes.

- `casterWheelMaterial`: Defines the `contactMaterial` field of the caster wheel [Solid](../reference/solid.md).

- `emitterChannel`: Defines the `channel` field of the [Emitter](../reference/emitter.md).

- `receiverChannel`: Defines the `channel` field of the [Receiver](../reference/receiver.md).

- `turretSlot`: Extends the robot with new nodes in the turret slot.

### Samples

You will find the following sample in this folder: "WEBOTS\_HOME/projects/robots/k-team/khepera4/worlds":

#### khepera4.wbt

![khepera4.wbt.png](images/robots/khepera4/khepera4.wbt.thumbnail.jpg) This simulation shows a Khepera IV moving in a square arena.
Its camera is enabled.
