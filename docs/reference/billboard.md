## Billboard

```
Billboard {
  MFNode children [ ]    # {node, PROTO}
}
```

A [Billboard](#billboard) node contains `children` nodes that rotates automatically to face the viewpoint.
It is otherwise similar to a [Group](group.md) node.

When a `node` is added to the `children` of a [Billboard](#billboard), it will be placed on the viewpoint coordinates.
To see the new `node`, it should be translated negatively along the z-axis.

The objects contained in billboard are not seen by [Cameras](camera.md).

[Shapes][shape.md] that are inside a [Billboard](#billboard) cannot cast or receive shadows.
