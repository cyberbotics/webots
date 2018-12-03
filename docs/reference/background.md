## Background

```
Background {
  MFColor skyColor  [ 0 0 0 ]   # any color
  SFNode  cubemap   NULL        # {Cubemap, PROTO}
}
```

### Description

The [Background](#background) node defines the background used for rendering the 3D world.

### Field Summary

The `skyColor` field defines the red, green and blue components of a solid color background, in the case where no valid [Cubemap](cubemap.md) node is present.
Only the three first float values of the `skyColor` field are used.

The `cubemap` field specifies a [Cubemap](cubemap.md) node to be used to create a background panorama, between the ground/sky backdrop and the world's geometry.
This panorama is also known by the more colloquial term "skybox".
The skybox consists of six images, each of which is mapped onto the faces of an infinitely large cube centered in the local coordinate system.
The images are applied individually to each face of the cube; the entire image goes on each face.
On the front, back, right, and left faces of the cube, when viewed from the inside with the Y-axis up, the texture is mapped onto each face with the same orientation as the if image was displayed normally in 2D.
On the top face of the cube, when viewed from the inside looking up along the +Y axis with the +Z axis as the view up direction, the texture is mapped onto the face with the same orientation as the if image was displayed normally in 2D.
On the bottom face of the box, when viewed from the inside down the -Y axis with the -Z axis as the view up direction, the texture is mapped onto the face with the same orientation as the if image was displayed normally in 2D.
