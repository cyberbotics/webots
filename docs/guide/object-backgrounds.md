# Backgrounds

## TexturedBackground

`TexturedBackground` provides a set of background textures, and apply them to the background skybox and to the reflections on the 3D objects.
It is designed to be used together with the [TexturedBackgroundLight](#texturedbackgroundlight) PROTO.

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
  SFString texture      "mountains"
  SFFloat  luminosity   1
  SFBool   skybox       TRUE
  SFBool   reflections  TRUE
  MFColor  skyColor     [0 0 0]
}
```

> **File location**: "[WEBOTS\_HOME/projects/objects/backgrounds/protos/TexturedBackground.proto](https://github.com/cyberbotics/webots/tree/master/projects/objects/backgrounds/protos/TexturedBackground.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### TexturedBackground Field Summary

- `texture`: Defines the texture of the background.

- `luminosity`: Is `Background.luminosity`.

- `skybox`: Specifies if the `texture` field is used to define the skybox shown in the scene background.

- `reflections`: Specifies if the `texture` field is used in the reflections of the [PBRAppearance](../reference/pbrappearance.md) nodes.

- `skyColor`: Defines the background color in case the `skybox` field is `FALSE`.

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
