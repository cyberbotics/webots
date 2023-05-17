## Transform

Derived from [Pose](pose.md).

```
Transform {
  SFVec3f    scale           1 1 1     # any vector
}
```

Direct derived nodes: none.

### Description

The [Transform](#transform) node is a grouping node that defines a coordinate system for its `children` that is relative to the coordinate systems of its parent.
The `children` list of a [Transform](#transform) node may only contain [Transform](#transform), [Pose](pose.md), [Group](group.md), [Shape](shape.md) or [CadShape](cadshape.md) nodes.
It cannot contain any [Solid](solid.md) or `Solid` derived nodes.

### Field Summary

- The `scale` field specifies a possibly non-uniform scale.
Only non-zero values are permitted; zero values are automatically reset to 1.
