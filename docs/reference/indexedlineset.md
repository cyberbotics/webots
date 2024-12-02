## IndexedLineSet

```
IndexedLineSet {
  SFNode  coord      NULL   # {Coordinate, PROTO}
  MFInt32 coordIndex [ ]    # [-1, inf)
}
```

The [IndexedLineSet](#indexedlineset) node represents a 3D geometry formed by constructing polylines from 3D vertices specified in the `coord` field.
[IndexedLineSet](#indexedlineset) uses the indices in its `coordIndex` field to specify the polylines by connecting vertices from the `coord` field.
An index of "-1" indicates that the current polyline has ended and the next one begins.
The last polyline may be (but does not have to be) followed by a "-1".
[IndexedLineSet](#indexedlineset) is specified in the local coordinate system and is affected by the transformations of its ancestors.

The `coord` field contains a [Coordinate](coordinate.md) node that specifies the 3D vertices of the line set.

[IndexedLineSet](#indexedlineset) nodes don't support [PBRAppearance](pbrappearance.md).
You may however set an [Appearance](appearance.md) node in the [Shape](shape.md) containing the [IndexedLineSet](#indexedlineset) geometry.
[IndexedLineSet](#indexedlineset) nodes are not lit, not texture-mapped and they do not cast or receive shadows.
[IndexedLineSet](#indexedlineset) nodes cannot be used for collision detection (boundingObject).
