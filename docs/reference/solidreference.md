## SolidReference

```
SolidReference {
  SFString solidName ""   # any string
}
```

### Description

A [SolidReference](#solidreference) can be used inside the `endPoint` field of a [Joint](joint.md) node to refer either to an existing [Solid](solid.md) or to the static environment.
Mechanical loops can be closed this way.
The only constraint when referring to a [Solid](solid.md) is that both [Solid](solid.md) and [Joint](joint.md) must be descendants of a common upper [Solid](solid.md).

### Field Summary

- `solidName`: This field specifies either the static environment (if the value is `<static environment>`) or the name of an existing [Solid](solid.md) node to be linked with the [Joint](joint.md)'s closest upper [Solid](solid.md) node.
Referring to the [Joint](joint.md) closest upper [Solid](solid.md) node or to a [Solid](solid.md) node which has no common upper [Solid](solid.md) with the [Joint](joint.md) is prohibited.

### Mechanical Loop Example

The [SolidReference](solidreference.md) node is particularly useful to create closed mechanical loops, here is a very simple example:

```
HingeJoint {
  jointParameters HingeJointParameters {
    anchor 0 0 1
  }
  endPoint Solid {
    name "front axle"
  }
}
HingeJoint {
  jointParameters HingeJointParameters {
    anchor 0 0 -1
  }
  endPoint SolidReference {
    solidName "front axle"
  }
}
```
