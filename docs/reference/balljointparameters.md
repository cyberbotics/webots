## BallJointParameters

```
BallJointParameters {
  SFVec3f anchor          0 0 0   # any vector
  SFFloat springConstant  0       # [0, inf)
  SFFloat dampingConstant 0       # [0, inf)
}
```

### Description

The [BallJointJointParameters](#balljointparameters) node can be used to specify the parameters of a ball joint.
It contains the anchor position, i.e. the coordinates of the point where bodies under a ball joint constraints are kept attached.
It can be used in the jointParameters field of [BallJoint](balljoint.md) only.

### Field Summary

- `anchor`: This field specifies the anchor position expressed in relative coordinates with respect to the center of the closest upper [Transform](transform.md) node's frame.

- `springConstant` and `dampingConstant`: These fields specify the uniform amount of rotational spring and damping effect around each of the frame axis of the [BallJoint](balljoint.md)'s closest upper [Transform](transform.md) (see [JointParameters](jointparameters.md)'s ["Springs and Dampers"](jointparameters.md#springs-and-dampers) section for more information on these constants).
This is can be useful to simulate a retraction force that pulls the [BallJoint](balljoint.md) solid `endPoint` back towards its initial orientation.
