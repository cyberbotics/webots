## Fujitsu's HOAP-2

%robot hoap2 images/robots/hoap2/model.thumbnail.png

The [Fujitsu HOAP-2](https://en.wikipedia.org/wiki/HOAP) (HOAP for "Humanoid for Open Architecture Platform") is an humanoid robot of 48 cm with 25 degrees of freedom.

### Movie Presentation

![youtube video](https://www.youtube.com/watch?v=xuVxFqmRl2g)

### Hoap2 PROTO

Derived from [Robot](../reference/robot.md).

```
Hoap2 {
  SFVec3f    translation         0 0.29 0
  SFRotation rotation            0 1 0 0
  SFString   name                "HOAP-2"
  SFString   controller          "hoap2"
  MFString   controllerArgs      []
  SFString   customData          ""
  SFBool     supervisor          FALSE
  SFBool     synchronization     TRUE
  SFColor    diffuseColor        0.65 0.65 0.65
  SFFloat    shininess           0.8
  SFColor    specularColor       0.65 0.65 0.65
  SFString   contactMaterial     "default"
  SFString   footContactMaterial "default"
  MFNode     extensionSlot       []
}
```

> **File location**: "WEBOTS\_HOME/projects/robots/fujitsu/hoap2/protos/Hoap2.proto"

#### Hoap2 Field Summary

- `diffuseColor`: Defines the `diffuseColor` field of the main [Material](../reference/material.md).

- `shininess`: Defines the `shininess` field of the main [Material](../reference/material.md).

- `specularColor`: Defines the `specularColor` field of the main [Material](../reference/material.md).

- `contactMaterial`: Defines the `contactMaterial` field of all the [Solid](../reference/solid.md) nodes except the feet.

- `footContactMaterial`: Defines the `contactMaterial` field of the foot [Solid](../reference/solid.md) nodes.

- `extensionSlot`: Extends the robot with new nodes in the extension slot.

### Samples

You will find some samples in this folder: "WEBOTS\_HOME/projects/robots/fujitsu/hoap2/worlds".

### hoap2\_sumo.wbt

![hoap2_sumo.wbt.png](images/robots/hoap2/hoap2_sumo.wbt.thumbnail.jpg) In this example, a HOAP-2 robot performs the Shiko dance (the dance sumos perform before a combat).
This robot is equipped with `TouchSensors` on the soles of its feet; it measures and logs the pressure exerted by its body on the ground.

### hoap2\_walk.wbt

![hoap2_walk.wbt.png](images/robots/hoap2/hoap2_walk.wbt.thumbnail.jpg) In this example, a HOAP-2 robot walks straight forward on a tatami.
This robot is equipped with `TouchSensors` on the soles of its feet; it measures and logs the pressure exerted by its body on the ground.
