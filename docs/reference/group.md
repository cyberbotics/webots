## Group

```
Group {
  MFNode children [ ]    # {node, PROTO}
}
```

Direct derived nodes: [Pose](pose.md), [Billboard](billboard.md).

A [Group](#group) node contains `children` nodes without introducing a new transformation.
It is equivalent to a [Pose](pose.md) node with a `translation` and a `rotation` set to zero (default values).

In the standard case, the list of `children` of a [Group](#group) node may contain different types of node:
- [Group](#group)
- [Pose](pose.md)
- [Transform](transform.md)
- [Shape](shape.md)
- [CadShape](cadshape.md)
- [Solid](solid.md)
- Any node derived from [Solid](solid.md)

However, if the [Group] node is inside the `boundingObject` of a [Solid](solid.md) node, [special rules](solid.md#how-to-use-the-boundingobject-field) apply.
