## Pose

Derived from [Group](group.md).

```
Pose {
  SFVec3f    translation     0 0 0     # any vector
  SFRotation rotation        0 0 1 0   # unit axis, (-inf, inf) angle
  SFFloat    translationStep 0.01
  SFFloat    rotationStep    0.261799387
}
```

Direct derived nodes: [Solid](solid.md), [Transform](transform.md), [Fluid](fluid.md).

### Description

The [Pose](#pose) node is a grouping node that defines a coordinate system for its children that is relative to the coordinate systems of its parent.

### Field Summary

- The `translation` field defines the translation from the parent coordinate system to the children's coordinate system.

- The `rotation` field defines an arbitrary rotation of the children's coordinate system with respect to the parent coordinate system.
This field contains four floating point values: *rx, ry, rz* and α.
The first three numbers, *rx ry rz*, define a normalized vector giving the direction of the axis around which the rotation must be carried out.
The fourth value, α, specifies the rotation angle around the axis in radians.
When α is zero, no rotation is carried out.
All the values of the rotation field can be positive or negative.
Note however that the length of the 3D vector *rx ry rz* must be normalized (i.e. its length is 1.0), otherwise the outcome of the simulation is undefined.

    For example, a rotation of &pi;/2 radians around the z-axis is represented like
    this:

        rotation 0 0 1 1.5708

    A rotation of &pi; radians around an axis located exactly between the *x* and
    y-axis is represented like this:

        rotation 0.7071 0.7071 0 3.1416

    And finally, note that these two rotations are identical:

        rotation 0 1 0 -1.5708
        rotation 0 -1 0 1.5708

- The `translationStep` and `rotationStep` fields defines the minimum step size used by the translation and rotation handles appearing in the 3D view when the object is selected.
If they are set to 0, then the step is disabled and translation and rotation are continuous.
