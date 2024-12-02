## PointSet

```
PointSet {
  SFNode  color   NULL  # {Color, PROTO}
  SFNode  coord   NULL  # {Coordinate, PROTO}
}
```

### Description

The [PointSet](#pointset) node represents a set of 3D points specified in the `coord` field.
[PointSet](#pointset) nodes don't support [PBRAppearance](pbrappearance.md).
Instead, you should use an [Appearance](appearance.md) node in the [Shape](shape.md) containing the [PointSet](#pointset) geometry or leave the `appearance` field to NULL.
[PointSet](#pointset) nodes are not lit, not texture-mapped and they do not cast or receive shadows.
[PointSet](#pointset) nodes cannot be used for collision detection (boundingObject).

### Field Summary

The `color` field optionally contains a [Color](color.md) node which defines the color of each point.
If the `color` field is not NULL, the [Color](color.md) node should have the same number of items as the [Coordinate](coordinate.md) node in the `coord` field.
If the `color` field is NULL, the `emissiveColor` of the [Material](material.md) will be used to define the color of the points.

The `coord` field contains a [Coordinate](coordinate.md) node that specifies the list of 3D points.
