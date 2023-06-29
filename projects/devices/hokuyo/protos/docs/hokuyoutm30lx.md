The [Hokuyo UTM-30LX](https://www.hokuyo-aut.jp/search/single.php?serial=169) is a lidar designed for outdoor robots with a high moving speed.
The model has the following specifications:

- `field of view`: 270 [deg]
- `range`: 0.1 to 30 [m]
- `resolution`: 1080 * 0.25 [deg]
- `dimension`: 0.06 x 0.087 x 0.06 [m]
- `weight`: 0.37 [kg]

```
HokuyoUtm30lx {
  SFVec3f    translation 0 0 0
  SFRotation rotation    0 0 1 0
  SFString   name        "Hokuyo UTM-30LX"
  SFFloat    noise       0.0
  SFInt32    resolution  1080
}
```

- `resolution`: Defines the `horizontalResolution` field of the [Lidar](https://cyberbotics.com/doc/reference/lidar).
