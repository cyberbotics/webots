## HingeJointParameters

Derived from [JointParameters](jointparameters.md).

```
HingeJointParameters {
  SFVec3f anchor                    0 0 0   # any vector
  SFVec3f axis                      1 0 0   # unit axis
  SFFloat suspensionSpringConstant  0       # [0, inf)
  SFFloat suspensionDampingConstant 0       # [0, inf)
  SFVec3f suspensionAxis            1 0 0   # unit axis
}
```

### Description

The [HingeJointParameters](#hingejointparameters) node can be used to specify the hinge rotation axis and various joint parameters (e.g., angular position, stop angles, spring and damping constants etc.) related to this rotation axis.

### Field Summary

- `anchor`: This field specifies the anchor position, i.e. a point through which the hinge axis passes.
Together with the `axis` field inherited from the [JointParameters](jointparameters.md) node, the `anchor` field determines the hinge rotation axis in a unique way.
It is expressed in relative coordinates with respect to the closest upper [Transform](transform.md) node's frame.

- `suspensionSpringConstant`: This field specifies the suspension spring constant along the suspension axis.

- `suspensionDampingConstant`: This field specifies the suspension damping constant along the suspension axis.

- `suspensionAxis`: This field specifies the direction of the suspension axis.

The `suspensionSpringConstant` and `suspensionDampingConstant` fields can be used to add a linear spring and/or damping behavior *along* the axis defined in `suspensionAxis`.
These fields are described in more detail in [JointParameters](jointparameters.md)'s ["Springs and Dampers"](jointparameters.md#springs-and-dampers) section.
