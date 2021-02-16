## TransmissionJoint

Derived from [HingeJoint](hingejoint.md).

```
TransmissionJoint {
  SFNode     jointParameters2  NULL        # {HingeJointParameters, PROTO}
  SFFloat    multiplier        1           # (-inf, inf)
  SFFloat    backlash          0           # (-inf, inf)
  SFNode     startPoint        NULL        # {Solid, Slot, PROTO}
  SFFloat    position2         0           # [0, inf)
}
```

### Description

TODO: ADD IMAGE

The [TransmissionJoint](#transmissionjoint) node can be used to model a mechanical transmission.
It inherits [Joint](joint.md)'s `jointParameters` field.
This field can be filled with a [HingeJointParameters](hingejointparameters.md) only.

### Field Summary

- `jointParameters`: This field specifies a [HingeJointParameters](hingejointparameters.md) node.
It describes the axis and anchor points of the driving joint.

- `jointParameters2`: This field specifies a [HingeJointParameters](hingejointparameters.md) node.
It describes the axis and anchor points of the driven joint.

- `multiplier`: This field specifies the mechanical advantage of the transmission. If a positive value is provided

- `backlash`: This field defines the clearance in the mesh of the wheels of the transmission. The backlash is defined as the maximum
distance that the geometric contact point can travel without any actual contact or transfer of power between the wheels. For example,
if a wheel of radius *r1* is driving another wheel of radius *r2* and there is an amount of backlash equal to *b* in their meshes, the
assuming the driving wheel would come to a halt instantaneously, then the driven one would continue to turn for another *b/r2* radians
until it contacts again.

- `startPoint`: this field specifies which [Solid](solid.md) will be on the driving side of the joint.
It must be either a [Solid](solid.md) child, or a reference to an existing [Solid](solid.md), i.e. a [SolidReference](solidreference.md).
Alternatively, a [Slot](slot.md) node can be inserted in the `startPoint` field, but this [Slot](slot.md) should be connected to another [Slot](slot.md) whose `endPoint` is either a [Solid](solid.md) or a [SolidReference](solidreference.md).

- `position2`: This field is not visible from the Scene Tree, see [joint's hidden position field](joint.md#joints-hidden-position-fields).
