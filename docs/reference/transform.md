## Transform

Derived from [Group](group.md).

```
Transform {
  SFVec3f    translation 0 0 0     # any vector
  SFRotation rotation    0 1 0 0   # unit axis, (-inf, inf) angle
  SFVec3f    scale       1 1 1     # any vector
}
```

Direct derived nodes: [Solid](solid.md).

### Description

The [Transform](#transform) node is a grouping node that defines a coordinate system for its children that is relative to the coordinate systems of its parent.

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

- The `scale` field specifies a possibly non-uniform scale.
Only positive values are permitted; non-positive values scale are automatically reset to 1.
Graphical objects support any positive non-uniform scale whereas physical objects are subjected to restrictions.
This is so because scaled geometries must remain admissible for the physics engine collision detection.
Restrictions for `Geometries` placed inside `boundingObjects` are as follows: [Sphere](sphere.md)s and [Capsule](capsule.md)s only support uniform scale; the scale coordinates x and z of a `Transform` with a [Cylinder](cylinder.md) descendant must be the same.
For the remaining `Geometries`, the scale is not restricted.
The `scale` fields of a [Solid](solid.md) node and its derived nodes must be uniform, i.e., of the form *x x x* so as to comply with the physics engine.
For such nodes a positive `scale` field initially set to *x y z* is automatically reset to *x x x*.
The same holds for a `Transform` placed inside a `boundingObject` and with a `Sphere` or a `Capsule` descendant.
In the case of a `Cylinder`, *x y z* will be reset to *x z x*.
If some value changes within one of the previous constrained scale fields, the two others are actuated using the new value and the corresponding constraint rule.

> **Note**: If a `Transform` is named using the [DEF](def-and-use.md) keyword and later referenced inside a `boundingObject` with a USE statement, the constraint corresponding to its first `Geometry` descendant applies to the `scale` fields of the defining `Transform` and of all its further references.
