# Gear

## Gear PROTO

Generic gear generator, inspired by Brian Paul's glxgears demo.

%figure

![Gear](images/objects/gear/Gear/model.png)

%end

Derived from [Solid](../reference/solid.md).

```
Gear {
  SFVec3f     translation            0 0 0
  SFRotation  rotation               0 1 0 0
  SFNode      appearance             PBRAppearance{baseColor 0.533333, 0.541176, 0.521569, roughness 0.5}
  SFString    name                   "gear"
  SFInt32     teeth                  8
  SFFloat     width                  0.01
  SFFloat     innerRadius            0.01
  SFFloat     pitchRadius            0.04
  SFFloat     toothDepth             0.01
  SFFloat     density                0.001
  SFBool      bodyBoundingObject     TRUE
  SFBool      teethBoundingObject    FALSE
}
```

> **File location**: "[WEBOTS\_HOME/projects/objects/gear/protos/Gear.proto]({{ url.github_tree }}/projects/objects/gear/protos/Gear.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### Gear Field Summary

- `name`: Defines the solid's name.

- `teeth`: Defines the number of teeth of the gear.

- `width`: Defines the thickness of the gear.

- `innerRadius`: Defines the radius of the central hole.

- `pitchRadius`: Defines the mid-tooth radius.

- `toothDepth`: Defines the height of the tooth profile.

- `density`: Defines the density of the gear. When setting a positive value, the physics node is enabled. When the value is zero,
the physics node is disabled.

- `bodyBoundingObject`: When enabled, the bounds of the solid are modeled by a cylinder that encompasses the entire gear.

- `teethBoundingObject`: When enabled, a spherical `boundingObject` is added to the tip of each tooth, and its radius can be
controlled by setting the `contactRadius` field.
