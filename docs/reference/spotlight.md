## SpotLight

Derived from [Light](light.md).

```
SpotLight {
  SFFloat ambientIntensity 0          # [0, 1]
  SFVec3f attenuation      1 0 0      # any positive vector
  SFFloat beamWidth        1.570796   # [0, pi/2)
  SFColor color            1 1 1      # any color
  SFFloat cutOffAngle      0.785398   # [0, pi/2)
  SFVec3f direction        0 0 -1     # any vector
  SFFloat intensity        1          # [0, 1]
  SFVec3f location         0 0 10     # any vector
  SFBool  on               TRUE       # {TRUE, FALSE}
  SFFloat radius           100        # [0, inf)
  SFBool  castShadows      FALSE      # {TRUE, FALSE}
}
```

### Description

The [SpotLight](#spotlight) node defines a light source that emits light from a specific point along a specific direction vector and constrained within a solid angle.
Spotlights may illuminate `Geometry` nodes that respond to light sources and intersect the solid angle.
Spotlights are specified in their local coordinate system and are affected by parent transformations.

The `location` field specifies a translation offset of the center point of the light source from the light's local coordinate system origin.
This point is the apex of the solid angle which bounds light emission from the given light source.
The `direction` field specifies the direction vector of the light's central axis defined in its own local coordinate system.
The `on` field specifies whether the light source emits light--if TRUE, then the light source is emitting light and may illuminate geometry in the scene, if FALSE it does not emit light and does not illuminate any geometry.
The `radius` field specifies the radial extent of the solid angle and the maximum distance from `location` that may be illuminated by the light source - the light source does not emit light outside this radius.
The `radius` must be >= 0.0.

The `cutOffAngle` field specifies the outer bound of the solid angle.
The light source does not emit light outside of this solid angle.
The `beamWidth` field specifies an inner solid angle in which the light source emits light at uniform full intensity.
The light source's emission intensity drops off from the inner solid angle (`beamWidth`) to the outer solid angle (`cutOffAngle`).
The drop off function from the inner angle to the outer angle is a linear interpolation:

```
intensity(angle) = intensity * (angle - cutOffAngle) / (beamWidth - cutOffAngle)

where intensity is the SpotLight's field value,
  intensity(angle) is the light intensity at an arbitrary
      angle from the direction vector,
  and angle ranges from beamWidth to cutOffAngle.
```

If `beamWidth` > `cutOffAngle`, then `beamWidth` is assumed to be equal to `cutOffAngle` and the light source emits full intensity within the entire solid angle defined by `cutOffAngle`.
Both `beamWidth` and `cutOffAngle` must be greater than 0.0 and less than or equal to &pi;/2.
See figure below for an illustration of the SpotLight's field semantics (note: this example uses the default attenuation).

The light's illumination falls off with distance as specified by three `attenuation` coefficients.
The attenuation factor is `1/(attenuation[0]+attenuation[1]*r+attenuation[2]*r²)`, where `r` is the distance of the light to the surface being illuminated.
The default is no attenuation.
An `attenuation` value of `0` `0` `0` is identical to `1` `0` `0`.
Attenuation values must be >= 0.0.

Contrary to the VRML97 specifications, the `attenuation` and the `ambientIntensity` fields cannot be set simultaneously.

%figure "Spot light"

![spot_light.png](images/spot_light.png)

%end
