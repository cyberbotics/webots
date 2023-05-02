The [Orbbec Astra](https://shop.orbbec3d.com/Astra) is a motion sensing input device.
It is modeled using a regular RGB [Camera](https://cyberbotics.com/doc/reference/camera) and a [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder) device to retrieve the depth information in meters.

Derived from [Solid](https://cyberbotics.com/doc/reference/solid).
Includes a [Camera](https://cyberbotics.com/doc/reference/camera) and a [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder).

```
Astra {
  SFVec3f    translation   0 0 0
  SFRotation rotation      0 0 1 0
  SFString   name          "Astra"
  SFFloat    colorNoise    0.0
  SFFloat    rangeNoise    0.0
}
```

- `colorNoise`: Defines the `noise` field of the [Camera](https://cyberbotics.com/doc/reference/camera).
- `rangeNoise`: Defines the `noise` field of the [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder).
