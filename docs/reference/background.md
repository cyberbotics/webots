## Background

```
Background {
  MFColor  skyColor   [ 0 0 0 ]  # any color
  MFString backUrl    []
  MFString bottomUrl  []
  MFString frontUrl   []
  MFString leftUrl    []
  MFString rightUrl   []
  MFString topUrl     []
  SFFloat  luminosity 1          # [0, inf)
}
```

### Description

The [Background](#background) node defines the background used for rendering the 3D world.

### Field Summary

The `skyColor` field defines the red, green and blue components of a solid color background, in the case where no valid texture is present.
Only the three first float values of the `skyColor` field are used.

The `backUrl`, `bottomUrl`, `frontUrl`, `leftUrl`, `rightUrl`, and `topUrl` fields specify a set of images that define a background panorama, between the ground/sky backdrop and the world's geometry.
The panorama consists of six images, each of which is mapped onto the faces of an infinitely large cube centered in the local coordinate system.
The images are applied individually to each face of the cube; the entire image goes on each face.
On the "front", "back", "right", and "left" faces of the cube, when viewed from the inside with the `Y-axis` up, the texture is mapped onto each face with the same orientation as the if image was displayed normally in 2D.
On the "top" face of the cube, when viewed from the inside looking up along the `+Y axis` with the `+Z axis` as the view up direction, the texture is mapped onto the face with the same orientation as the if image was displayed normally in 2D.
On the "bottom" face of the box, when viewed from the inside down the `-Y axis` with the `-Z axis` as the view up direction, the texture is mapped onto the face with the same orientation as the if image was displayed normally in 2D.

The image format supported by the url fields are: JPEG, PNG or HDR.

The `luminosity` specifies a scale factor to be applied to the light contribution of the [Background](background.md) node on [Shape](shape.md) nodes using the [PBRAppearance](pbrappearance.md).
