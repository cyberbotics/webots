## Background

```
Background {
  MFColor  skyColor            [ 0 0 0 ]  # any color
  MFString backUrl             []
  MFString bottomUrl           []
  MFString frontUrl            []
  MFString leftUrl             []
  MFString rightUrl            []
  MFString topUrl              []
  MFString backIrradianceUrl   []
  MFString bottomIrradianceUrl []
  MFString frontIrradianceUrl  []
  MFString leftIrradianceUrl   []
  MFString rightIrradianceUrl  []
  MFString topIrradianceUrl    []
  SFFloat  luminosity          1          # [0, inf)
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
On the "front", "back", "right", and "left" faces of the cube, when viewed from the inside with the y-axis up, the texture is mapped onto each face with the same orientation as the if image was displayed normally in 2D.
On the "top" face of the cube, when viewed from the inside looking up along the positive y-axis with the positive z-axis as the view up direction, the texture is mapped onto the face with the same orientation as the if image was displayed normally in 2D.
On the "bottom" face of the box, when viewed from the inside down the negative y-axis with the negative z-axis as the view up direction, the texture is mapped onto the face with the same orientation as the if image was displayed normally in 2D.
The image format supported by the `url` fields are: JPEG or PNG.

Similarly, the `*IrradianceUrl` fields specify a set of images which define a second cubemap used for the light reflections on the [PBR appearances](pbrappearance.md).
This cube map is oriented in the same way as the background panorama (cf. description above).
The image format should be [HDR](https://en.wikipedia.org/wiki/RGBE_image_format).
This format allows to have light intensities bigger than 1.0.

If a `*Url` value starts with `http://` or `https://`, Webots will get the file from the web.
Otherwise, the file should be specified with a relative path.
The same search algorithm as for [ImageTexture](imagetexture.md) is used (cf. [this section](imagetexture.md#search-rule-of-the-texture-path)).
Absolute paths work as well, but they are not recommended because they are not portable across systems.

HDR backgrounds can be found easily on the internet.
They often come as single files using an equirectangular projection.
This [set of image tools]({{ url.github_tree }}/scripts/image_tools) provides basic conversion tools to transform your background resources to Webots ones.
Several image editors such as [Gimp](https://www.gimp.org) support this format.

The `luminosity` specifies a scale factor to be applied to the light contribution of the [Background](background.md) node on [Shape](shape.md) nodes using the [PBRAppearance](pbrappearance.md).
