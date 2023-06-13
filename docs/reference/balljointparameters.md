## BallJointParameters

Derived from [JointParameters](jointparameters.md).

```
BallJointParameters {
  SFVec3f anchor  0 0 0   # any vector
}
```

### Description

The [BallJointJointParameters](#balljointparameters) node can be used to specify the parameters of a ball joint.
It contains the anchor position, i.e. the coordinates of the point where bodies under a ball joint constraints are kept attached.
It can be used in the jointParameters field of [BallJoint](balljoint.md) only.

### Field Summary

- `anchor`: This field specifies the anchor position expressed in relative coordinates with respect to the center of the closest upper [Pose](pose.md) node's frame.
