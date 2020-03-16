## Mesh

```
Mesh {
  field MFString url [ ]
}
```

### Description

The [Mesh](#mesh) node represents a 3D shape imported from an external file created from a 3D modeling programs.
The [Mesh](#mesh) node can be used either as a graphical or as a collision detection primitive (in a boundingObject).
Currently, the following formats are supported:
  - blend
  - dae
  - fbx
  - obj
  - stl

If the file contains more than one meshes, they will be merged into a single one.

### Field Summary

The `url` field defines the path to the 3D file.
The file should be specified with a relative or absolute path.
The same search algorithm as for [ImageTexture](imagetexture.md) is used (cf. [this section](imagetexture.md#search-rule-of-the-texture-path)).
