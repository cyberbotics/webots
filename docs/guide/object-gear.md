# Gear

## Gear PROTO

A simple generic gear generator inspired by Brian Paul's glxgears demo.

%figure

![Gear](images/objects/gear/Gear/model.png)

%end

Derived from [Solid](../reference/solid.md).

```
Gear {
  SFInt32         teeth                  8
  SFFloat         width                  0.01
  SFFloat         innerRadius            0.01
  SFFloat         pitchRadius            0.04
  SFFloat         toothDepth             0.01
  SFFloat         density                0
  SFBool          bodyBoundingObject     TRUE
  SFBool          teethBoundingObject    FALSE
  SFFloat         contactRadius          0.001
}
```

> **File location**: "[WEBOTS\_HOME/projects/objects/gear/protos/Gear.proto]({{ url.github_tree }}/projects/objects/gear/protos/Gear.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### Gear Field Summary

- `teeth`: Defines the number of teeth of the gear.

- `width`: Defines the thickness of the gear.

- `innerRadius`: Defines the radius of the central hole.

- `pitchRadius`: Defines the mid-tooth radius.

- `toothDepth`: Defines the height of the tooth profile.

- `density`: Defines the density of the gear. When setting a positive value, the physics node is enabled. When the value is zero,
the physics node is disabled.

- `bodyBoundingObject`: When enabled, the bounds of the solid are modeled by a cylinder that encapsulates the entire gear.

- `teethBoundingObject`: When enabled, a spherical `boundingObject` is added to the tip of each tooth, and its radius can be
controlled by setting the `contactRadius` field.

- `contactRadius`: Defines the radius of the spherical contact point at the tip of each tooth.
