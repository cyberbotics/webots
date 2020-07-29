## KONDO's KHR-2HV

%robot khr-2hv images/robots/khr-2hv/model.thumbnail.png

The "Kondo KHR-2HV" is an humanoid robot with 17 degrees of freedom.

### Movie Presentation

![youtube video](https://www.youtube.com/watch?v=AtaGm9nR-EM)

### Khr2hv PROTO

Derived from [Robot](../reference/robot.md).

```
Khr2hv {
  SFVec3f    translation     0 0.215 0
  SFRotation rotation        1 0 0 -1.5708
  SFString   name            "KHR-2HV"
  SFString   controller      "khr-2hv_demo"
  MFString   controllerArgs  []
  SFString   contactMaterial "default"
  SFString   customData      ""
  SFBool     supervisor      FALSE
  SFBool     synchronization TRUE
  MFNode     bodySlot        []
}
```

> **File location**: "WEBOTS\_HOME/projects/robots/kondo/khr-2hv/protos/Khr2hv.proto"

#### Khr2hv Field Summary

- `bodySlot`: Extends the robot with new nodes in the body slot.

### Samples

You will find the following sample in this folder: "WEBOTS\_HOME/projects/robots/kondo/khr-2hv/worlds".

#### khr-2hv.wbt

![khr-2hv.wbt.png](images/robots/khr-2hv/khr-2hv.wbt.thumbnail.jpg) In this simulation, the KHR-2HV robot plays several open-loop motions.
Once this automatic behavior is completed, you can move the robot using the computer keyboard (please refer to the instruction displayed in the Webots console).
