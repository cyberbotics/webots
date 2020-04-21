## Mesh

```
Mesh {
  field MFString url [ ]
}
```

### Description

The [Mesh](#mesh) node represents a 3D shape imported from an external file created by a 3D modeling software.
The [Mesh](#mesh) node can be used either as a graphical or as a collision detection primitive (in a boundingObject).
Currently, the following formats are supported:
  - [3D Studio mesh](https://wiki.fileformat.com/3d/3ds) files (.3ds).
  - [Blender](https://www.blender.org/) files (.blend).
  - [Biovision Hierarchy](https://en.wikipedia.org/wiki/Biovision_Hierarchy) files (.bvh).
  - [Collada](https://en.wikipedia.org/wiki/COLLADA) files (.dae).
  - [Filmbox](https://en.wikipedia.org/wiki/FBX) files (.fbx).
  - [STL](https://en.wikipedia.org/wiki/STL_(file_format)) files (.stl).
  - [Wavefront](https://wiki.fileformat.com/3d/obj) files (.obj).
  - [X3D](https://www.web3d.org/getting-started-x3d) files (.x3d).

If the file contains more than one mesh, the meshes will be merged into a single one.

### Field Summary

The `url` field defines the path to the 3D file.
The file should be specified with a relative or absolute path.
The same search algorithm as for [ImageTexture](imagetexture.md) is used (cf. [this section](imagetexture.md#search-rule-of-the-texture-path)).
