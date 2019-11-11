# Backgrounds

## TexturedBackground

Background textured with a skybox.

Supported values for the `texture` field:

- dusk
- empty\_office
- entrance\_hall
- factory
- mars
- noon\_building\_overcast
- noon\_cloudy\_countryside
- noon\_park\_empty
- mountains

%figure

![TexturedBackground](images/objects/backgrounds/TexturedBackground/model.png)

%end

Derived from [Background](../reference/background.md).

```
TexturedBackground {
  SFString texture             "mountains"
  SFFloat  luminosity          1
  SFBool   skybox              TRUE
  SFBool   reflections         TRUE
  MFColor  fallbackSkyboxColor [0 0 0]
}
```

> **File location**: "[WEBOTS\_HOME/projects/objects/backgrounds/protos/TexturedBackground.proto](https://github.com/cyberbotics/webots/tree/master/projects/objects/backgrounds/protos/TexturedBackground.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### TexturedBackground Field Summary

- `texture`: Defines the texture of the background.

- `luminosity`: Is `Background.luminosity`.

- `skybox`: Specifies if the background is used to define the textured skybox.

- `reflections`: Specifies if the background is used in the material reflections.

- `fallbackSkyboxColor`: Defines the background color in case the skybox is not defined.

## TexturedBackgroundLight

Light designed to match the skyboxes in the TexturedBackground PROTO.

Supported values for the `texture` field:

- dusk
- empty\_office
- entrance\_hall
- factory
- mars
- noon\_building\_overcast
- noon\_cloudy\_countryside
- noon\_park\_empty
- mountains

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

> **File location**: "[WEBOTS\_HOME/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto](https://github.com/cyberbotics/webots/tree/master/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### TexturedBackgroundLight Field Summary

- `texture`: Should be equivalent to the 'texture' field of the TexturedBackground.

- `castShadows`: Defines whether the light should cast shadows.
