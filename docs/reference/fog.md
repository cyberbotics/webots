## Fog

```
Fog {
  SFColor  color           1 1 1      # any color
  SFString fogType         "LINEAR"   # {"LINEAR", "EXPONENTIAL", "EXPONENTIAL2"}
  SFFloat  visibilityRange 0          # [0, inf)
}
```

The [Fog](#fog) node provides a way to simulate atmospheric effects by blending objects with the color specified by the `color` field based on the distances of the various objects from the camera.
The distances are calculated in the coordinate space of the [Fog](#fog) node.
The `visibilityRange` specifies the distance in meters (in the local coordinate system) at which objects are totally obscured by the fog.
Objects located beyond the `visibilityRange` of the camera are drawn with a constant specified by the `color` field.
Objects very close to the viewer are blended very little with the fog `color`.
A `visibilityRange` of 0.0 disables the [Fog](#fog) node.

The `fogType` field controls how much of the fog color is blended with the object as a function of distance.
If `fogType` is "LINEAR", the amount of blending is a linear function of the distance, resulting in a depth cueing effect.
If `fogType` is "EXPONENTIAL", an exponential increase in blending is used, resulting in a more natural fog appearance.
If `fogType` is "EXPONENTIAL2", a square exponential increase in blending is used, resulting in an even more natural fog appearance (see the OpenGL documentation for more details about fog rendering).
