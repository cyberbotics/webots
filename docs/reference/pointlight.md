## PointLight

Derived from [Light](light.md).

```
PointLight {
  SFVec3f attenuation 1 0 0   # any positive vector
  SFVec3f location    0 0 0   # any vector
  SFFloat radius      100     # [0, inf)
}
```

### Description

The [PointLight](#pointlight) node specifies a point light source at a 3D location in the local coordinate system.
A point light source emits light equally in all directions.
It is possible to put a [PointLight](#pointlight) on board a mobile robot to have the light move with the robot.

A [PointLight](#pointlight) node's illumination drops off with distance as specified by three `attenuation` coefficients.
The final attenuation factor *af* is calculated as follows: *af = 1/(attenuation[0] + attenuation[1] &times; r + attenuation[2] &times; r<sup>2</sup>)*, where *r* is the distance from the light to the surface being illuminated.
The default is no attenuation.
When [PointLight](#pointlight) nodes are used together with [LightSensor](lightsensor.md), it is recommended to change the default attenuation to a more realistic `[0 0 12.56]` in order to more accurately model physical reality.
`12.56` is an approximation of 4 &times; &pi;, so that the formula relies on the surface of the sphere (4 &times; &pi; &times; r<sup>2</sup>) corresponding the light casted by the [PointLight](#pointlight) node.
Indeed, if a point source radiates light uniformly in all directions and there is no absorption, then the irradiance drops off in proportion to the square of the distance from the surface.

Contrary to the VRML97 specifications, the `attenuation` and the `ambientIntensity` fields cannot be set simultaneously.
