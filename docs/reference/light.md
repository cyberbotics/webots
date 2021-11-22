## Light

```
Light {
  SFFloat ambientIntensity 0       # [0, 1]
  SFColor color            1 1 1   # any color
  SFFloat intensity        1       # [0, inf)
  SFBool  on               TRUE    # {TRUE, FALSE}
  SFBool  castShadows      FALSE   # {TRUE, FALSE}
}
```

Direct derived nodes: [PointLight](pointlight.md), [SpotLight](spotlight.md), [DirectionalLight](directionallight.md).

### Description

The [Light](#light) node is abstract: only derived nodes can be instantiated.
Lights have two purposes in Webots: (1) the are used to graphically illuminate objects and (2) they determine the quantity of light perceived by [LightSensor](lightsensor.md) nodes.
Except for `castShadows`, every field of a [Light](#light) node affects the light measurements made by [LightSensor](lightsensor.md) nodes.

### Field Summary

- The `intensity` field specifies the brightness of the direct emission from the light.

- The `ambientIntensity` specifies the intensity of the ambient emission from the light.
It only applies to objects using the former [Appearance](../reference/appearance.md) node and not to objects using the [PBRAppearance](../reference/pbrappearance.md) node.

- The `color` field specifies the spectral color properties of both the direct and ambient light emission as an RGB value.

- The `on` boolean value allows the user to turn the light on (TRUE) or off (FALSE).

- The `castShadows` field allows the user to turn on (TRUE) or off (FALSE) the casting of shadows for this [Light](#light).
When activated, sharp shadows are casted from and received by any renderable object except for the semi-transparent objects, and the [IndexedLineSet](indexedlineset.md) and [PoinSet](pointset.md) primitives.
An object can be semi-transparent either if its texture has an alpha channel, or if its [Material](material.md).`transparency` field is not equal to 1.
Shadows are additive (Several lights can cast shadows).
The darkness of a shadow depends on how the occluded part is lighted (either by an ambient light component or by another light).
Activating the shadows of just one [Light](#light) can have a significant impact on the global rendering performance, particularly if the world contains either lots of objects or complex meshes.
Some shadow issues can occurs in closed spaces.

### Limitation

Due to a performance issue in Firefox on Windows and macOS (see comment of this [issue](https://github.com/cyberbotics/webots/issues/2691)), the number of lights is limited to 48 lights of _each_ kind: [PointLight](pointlight.md), [SpotLight](spotlight.md) and [DirectionalLight](directionallight.md).

However it is possible to support more lights (up to 256 of each) if needed. The following procedure explains how to increase the lights limit:

- Build Webots from sources (instructions are available [here](https://github.com/cyberbotics/webots/wiki)).

- Modify the number of lights of `gMaxActiveDirectionalLights`, `gMaxActiveDirectionalLights` and `gMaxActiveSpotLights` in `webots/src/wren/Constants.hpp`.

- Modify the number of lights of `maxDirectionalLights`, `maxPointLights` and `maxSpotLights` in the following shaders:
  - `webots/resources/wren/shaders/pbr.frag`
  - `webots/resources/wren/shaders/pbr_stencil_diffuse_specular.frag`
  - `webots/resources/wren/shaders/phong.frag`
  - `webots/resources/wren/shaders/phong_stencil_ambient_emissive.frag`
  - `webots/resources/wren/shaders/phong_stencil_diffuse_specular.frag`
  - `webots/resources/wren/shaders/shadow_volume.vert`

- Compile Webots again.
