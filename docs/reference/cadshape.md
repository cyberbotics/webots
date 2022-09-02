
## CadShape

```
CadShape {
  MFString url         []
  SFBool   ccw         TRUE
  SFBool   castShadows TRUE
  SFBool   isPickable  TRUE
}
```

The [CadShape](#cadshape) node renders a Collada (.dae) or Wavefront (.obj) object imported from an external file.
Both Collada and Wavefront files can include 3D geometries and appearances, be it materials or references to external textures, therefore the [CadShape](#cadshape) node builds both.
If the file references external textures, these should be placed relative to the collada/wavefront file itself as indicated (i.e., in the location expressed within the .dae/.obj file), no additional search is performed for these textures.

### Field Summary

- The `url` field defines the URL of the 3D file.
If the `url` value starts with `http://` or `https://`, Webots will get the file from the web.
Otherwise, the URL must specify a relative path.
Absolute paths are also supported, but not recommended as they are not portable.

> **Note**: if the URL of the file starts with `http://` or `https://`, then any implicit textures referenced by this file will also be assumed to be available at an URL relative to it.
For example, assume a collada file `MyCollada.dae` references a texture `./textures/texture.png` relative to its location, then if the collada file is provided as a remote file of URL like:
*https://raw.githubusercontent.com/cyberbotics/master/meshes/MyCollada.dae*
it follows that the corresponding texture will be downloaded from:
*https://raw.githubusercontent.com/cyberbotics/master/meshes/textures/texture.png*

- The `ccw` field indicates whether the vertices of the faces are ordered in a counter-clockwise direction.
Changing this value will flip the faces of the meshes.

- The `castShadows` field allows the user to turn on (TRUE) or off (FALSE) shadows casted by this shape.
However, shapes containing more than 65535 vertices will ignore this field and won't cast any shadow to save performance.

> **Note**: Objects cast shadows only if the world contains at least one [Light](light.md) node with `castShadows` field set to TRUE and if shadows are not disabled in the application preferences.

- The `isPickable` field defines if the object is detected (TRUE) or not (FALSE) when clicking on the 3D scene.
When navigating the 3D view, if the mouse hovers over a pickable object the viewpoint will rotate around it.
Additionally, when a pickable shape is a children of a [Solid](solid.md) node (or any of its descendants), clicking on it with the mouse will display the manipulators that allow it to be translated and rotated, whereas a non-pickable object will not.
