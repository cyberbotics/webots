The [Hokuyo URG-04LX](https://www.hokuyo-aut.jp/search/single.php?serial=165) is a lidar designed for lightweight indoor robots.
The model has the following specifications:

- `field of view`: 240 [deg]
- `range`: 0.06 to 4.095 [m]
- `resolution`: 667 * 0.36 [deg]
- `dimension`: 0.05 x 0.07 x 0.05 [m]
- `weight`: 0.16 [kg]

```
HokuyoUrg04lx [
  SFVec3f    translation 0 0 0
  SFRotation rotation    0 0 1 0
  SFString   name        "Hokuyo URG-04LX"
  SFFloat    noise       0.0
  SFInt32    resolution  667
]
```

`resolution`: Defines the `horizontalResolution` field of the [Lidar](https://cyberbotics.com/doc/reference/lidar).
