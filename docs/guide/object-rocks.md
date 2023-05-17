# Rocks

## Rock

A rock with a flat underside.

Derived from [Solid](../reference/solid.md).

```
Rock {
  SFVec3f     translation       0 0 0
  SFRotation  rotation          0 0 1 0
  SFString    name              "rock"
  SFString    type              "regular"
  SFFloat     scale             1
  MFString    texture           "https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/default/worlds/textures/rock.jpg"
  SFColor     color             1 1 1
  SFNode      physics           NULL
  SFBool      locked            FALSE
}
```

> **File location**: "[WEBOTS\_HOME/projects/objects/rocks/protos/Rock.proto]({{ url.github_tree }}/projects/objects/rocks/protos/Rock.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### Rock Field Summary

- `type`: Defines the rock variant. This field accepts the following values: `"regular"` and `"flat"`.

- `scale`: Defines the scale of the rock.

- `texture`: Defines the texture used for the rock.

- `color`: Defines the base color of the rock.

