## Billboard

```
Billboard {
  MFNode children [ ]    # {node, PROTO}
}
```

A [Billboard](#billboard) node contains `children` nodes that rotate and translate automatically to follow the viewpoint and face it. It could be use to display information in the main view regardless of the position and orientation of the viewpoint, like a cockpit or the score of a soccer match.
It is otherwise similar to a [Group](group.md) node.

When a `node` is added to the `children` of a [Billboard](#billboard), it will be placed on the viewpoint coordinates.
To see the new `node`, it should be translated negatively along the z-axis.

The objects contained in billboard are not seen by [Cameras](camera.md).

[Shapes][shape.md] that are inside a [Billboard](#billboard) cannot cast or receive shadows.
