## Billboard

```
Billboard {
  MFNode children [ ]    # {node, PROTO}
}
```

A [Billboard](#Billboard) node contains `children` nodes that rotates automatically to face the users.
It is otherwise similar to a [Group](group.md) node.

When a `node` is added to the `children` of a billboard, it will be placed on the viewpoint coordinates.
To see the new `node`, it should be translated negatively along the z-axis.
