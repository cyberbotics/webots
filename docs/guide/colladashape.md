
## ColladaShape

```
ColladaShape {
  MFString url         []
  SFBool   ccw         TRUE
  SFBool   castShadows TRUE
  SFBool   isPickable  TRUE
}
```

The [ColladaShape](#colladashape) node represents a Collada object (*.dae) imported from an external file.
Collada files can include both 3D geometries and appearances, be it materials or references to external textures, therefore the [ColladaShape](#colladashape) node builds both.
If the file references external textures, these should be placed relative to the collada file itself as indicated, no search is performed.


### Field Summary

- The `url` field defines the url of the 3D file.
If the `url` value starts with `http://` or `https://`, Webots will get the file from the web.
Otherwise, the file should be specified with a relative path.

- The `ccw` field indicates whether the vertices of the faces are ordered in a counter-clockwise direction.

- The `castShadows` field allows the user to turn on (TRUE) or off (FALSE) shadows casted by this shape.
However, shapes containing more than 65535 vertices will ignore this field and won't cast any shadow to save performance.

- The `isPickable` field defines if the object is detected (TRUE) or not (FALSE) when clicking on the 3D scene.
When navigating the 3D view, if the mouse hovers over a pickable object the viewpoint will rotate around it.
Additionally, when a pickable shape is a children of a [Solid](solid.md) node (or any of its descendants), clicking on it with the mouse will display the manipulators that allow it to be translated and rotated, whereas a non-pickable object will not.

> **Note**: Objects cast shadows only if the world contains at least one [Light](light.md) node with `castShadows` field set to TRUE and if shadows are not disabled in the application preferences.
