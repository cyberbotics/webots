## Joint

```
Joint {
  SFNode jointParameters NULL   # {JointParameters, HingeJointParameters, PROTO}
  SFNode endPoint        NULL   # {Solid, Slot, PROTO}
}
```

### Description

The [Joint](#joint) node is an abstract node (not instantiated) whose derived classes model various types of mechanical joints: hinge ([HingeJoint](hingejoint.md)), slider ([SliderJoint](sliderjoint.md)), ball joint ([BallJoint](balljoint.md)), hinge2 ([Hinge2Joint](hinge2joint.md)).
Apart from the ball joint, joints can be motorized and endowed with [PositionSensor](positionsensor.md) nodes.

The [Joint](#joint) node creates a link between its [Solid](solid.md) parent and the [Solid](solid.md) placed into its ` endPoint` field.
Using a [SolidReference](solidreference.md) inside `endPoint` enables you to close mechanical loops within a [Robot](robot.md) or a passive mechanical system.

### Field Summary

- `jointParameters`: this field optionally specifies a [JointParameters](jointparameters.md) node or one of its derived classes.
These nodes contain common joint parameters such as position, stops, anchor or axis if existing.
This field must be filled with an [HingeJointParameters](hingejointparameters.md) node for an [HingeJoint](hingejoint.md) or an [Hinge2Joint](hinge2joint.md), with a [JointParameters](jointparameters.md) node for a [SliderJoint](sliderjoint.md) (anchor-less) and with a [BallJointParameters](balljointparameters.md) node for a [BallJoint](balljoint.md).

    For an [Hinge2Joint](hinge2joint.md), the `jointParameters` field is related to the first rotation axis while an additional field called `jointParameters2` refers to the second rotation axis.

    For a [BallJoint](balljoint.md), the `jointParameters` field is related to the first rotation axis while two additional fields called `jointParameters2` and `jointParameters3` refer to the second and third rotation axes.

    3D-vector parameters (e.g `axis, anchor`) are always expressed in relative coordinates with respect to the closest upper [Pose](pose.md)'s frame using the meter as unit. If the `jointParameters` field is not specified, parameters are set with the default values defined in the corresponding parameter node.

- `endPoint`: this field specifies which [Solid](solid.md) will be subjected to the joint constraints.
It must be either a [Solid](solid.md) child, or a reference to an existing [Solid](solid.md), i.e. a [SolidReference](solidreference.md).
Alternatively, a [Slot](slot.md) node can be inserted in the `endPoint` field, but this [Slot](slot.md) should be connected to another [Slot](slot.md) whose `endPoint` is either a [Solid](solid.md) or a [SolidReference](solidreference.md).

### Joint's Hidden Position Fields

If the `jointParameters` is set to NULL, joint positions are then not visible from the Scene Tree.
In this case Webots keeps track of the initial positions of [Joint](#joint) nodes by means of hidden position fields.
These fields, which are not visible from the Scene Tree, are used to store inside the world file the current joint positions when the simulation is saved.
As a result joint positions are restored when reloading the simulation just the same way they would be if [JointParameters](jointparameters.md) nodes were used.

For [HingeJoint](hingejoint.md) and [SliderJoint](sliderjoint.md) nodes containing no [JointParameters](jointparameters.md), Webots uses the hidden field named `position`.
For a [Hinge2Joint](hingejoint.md) node, an additional hidden field named `position2` is used to store the joint position with respect to the second axis.
For a [BallJoint](balljoint.md) node, an additional hidden field named `position3` is used to store the joint position with respect to the third axis.
