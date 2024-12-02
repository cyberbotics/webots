## TextureCoordinate

```
TextureCoordinate {
  MFVec2f point [ ]   # (-inf, inf)
}
```

The [TextureCoordinate](#texturecoordinate) node specifies a set of 2D texture coordinates used by vertex-based `Geometry` nodes (e.g., [IndexedFaceSet](indexedfaceset.md)) to map textures to vertices.
Textures are two-dimensional color functions that, given a coordinate pair *(s,t)*, return a color value *color(s,t)*.
Texture map values ([ImageTexture](imagetexture.md)) range from 0.0 to 1.0 along the s and t axes.
Texture coordinates identify a location (and thus a color value) in the texture map.
The horizontal coordinate *s* is specified first, followed by the vertical coordinate *t*.
