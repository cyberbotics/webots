## Color

```
Color {
  MFColor color [ ]   # any color
}
```

This node defines a set of RGB colors to be used in the fields of another node.

[Color](#color) nodes are only used to specify multiple colors for a single geometric shape, such as colors for the faces or vertices of an [ElevationGrid](elevationgrid.md).
A [Material](material.md) node is used to specify the overall material parameters of a geometric node.
If both a [Material](material.md) node and a [Color](#color) node are specified for a geometric shape, the colors shall replace the diffuse component of the material.

RGB or RGBA textures take precedence over colors; specifying both an RGB or RGBA texture and a Color node for a geometric shape will result in the Color node being ignored.
