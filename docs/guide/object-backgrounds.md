# Backgrounds

## TexturedBackground

Background textured with a skybox.

Supported values for the `texture` field:

- dawn\_cloudy\_empty
- empty\_office
- morning\_cloudy\_empty
- noon\_building\_overcast (HDR)
- noon\_cloudy\_empty
- noon\_cloudy\_mountains
- noon\_park\_empty (HDR)
- noon\_stormy\_empty
- noon\_sunny\_empty
- noon\_sunny\_garden
- twilight\_cloudy\_empty

%figure

![TexturedBackground](images/objects/backgrounds/TexturedBackground/model.png)

%end

Derived from [Background](../reference/background.md).

```
TexturedBackground {
  SFString texture "mountains"
}
```

> **File location**: "WEBOTS\_HOME/projects/objects/backgrounds/protos/TexturedBackground.proto"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### TexturedBackground Field Summary

- `texture`: Defines the texture of the background.

## TexturedBackgroundLight

Light designed to match the skyboxes in the TexturedBackground PROTO.

Supported values for the "texture" field:

- dawn\_cloudy\_empty
- empty\_office
- morning\_cloudy\_empty
- noon\_building\_overcast
- noon\_cloudy\_empty
- noon\_cloudy\_mountains
- noon\_park\_empty
- noon\_stormy\_empty
- noon\_sunny\_empty
- noon\_sunny\_garden
- twilight\_cloudy\_empty

%figure

![TexturedBackgroundLight](images/objects/backgrounds/TexturedBackgroundLight/model.png)

%end

Derived from [DirectionalLight](../reference/directionallight.md).

```
TexturedBackgroundLight {
  SFString texture        "mountains"
  SFBool   castShadows    TRUE
}
```

> **File location**: "WEBOTS\_HOME/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### TexturedBackgroundLight Field Summary

- `texture`: Should be equivalent to the 'texture' field of the TexturedBackground.

- `castShadows`: Defines whether the light should cast shadows.

