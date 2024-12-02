## Material

```
Material {
  SFFloat ambientIntensity 0.2          # [0, 1]
  SFColor diffuseColor     0.8 0.8 0.8  # any color
  SFColor emissiveColor    0 0 0        # any color
  SFFloat shininess        0.2          # [0, 1]
  SFColor specularColor    0 0 0        # any color
  SFFloat transparency     0            # [0, 1]
}
```

### Description

The [Material](#material) node specifies surface material properties for associated geometry nodes and is used by the VRML97 lighting equations during rendering.
The fields in the [Material](#material) node determine how light reflects off an object to create color.

### Field Summary

- The `ambientIntensity` field specifies how much ambient light from the various light sources in the world this surface shall reflect.
Ambient light is omni-directional and depends only on the number of light sources, not their positions with respect to the surface.
Ambient color is calculated as `ambientIntensity` x `diffuseColor`.

- The `diffuseColor` field reflects all VRML97 light sources depending on the angle of the surface with respect to the light source.
The more directly the surface faces the light, the more diffuse light reflects.

- The `emissiveColor` field models "glowing" objects.
This can be useful for displaying pre-lit models (where the light energy of the room is computed explicitly), or for displaying scientific data.

- The `specularColor` and `shininess` fields determine the specular highlights (e.g., the shiny spots on an apple).
When the angle from the light to the surface is close to the angle from the surface to the camera, the `specularColor` is added to the diffuse and ambient color calculations.
Lower shininess values produce soft glows, while higher values result in sharper, smaller highlights.

- The `transparency` field specifies how "translucent" an object must be rendered: with 0.0 (the default) the object will appear completely opaque, and with 1.0 it will appear completely transparent.
A transparent object doesn't cast or receive shadows.
Webots performs dynamic alpha sorting according to the distance between the center of the objects (the local position of the parent [Pose](pose.md)) and the viewpoint.
Some occlusion issues can occur if two transparent objects intersect each other, or if the coordinate center of a transparent object is located outside the effectively rendered polygons, or if the sizes of nearby transparent objects differ significantly.
