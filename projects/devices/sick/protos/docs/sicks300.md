The [SICK S300](https://www.sick.com/ag/en/opto-electronic-protective-devices/safety-laser-scanners/s300-standard/s30b-2011ba/p/p53845) is a 3 layers safety lidar. The model has the following specifications:

- `field of view`: 270 [deg]
- `range`: up to 30 [m]
- `number of layers`: 3
- `angular resolution`: 0.5 [deg]
- `resolution`: 540
- `dimension`: 0.102 x 0.152 x 0.106 [m]
- `weight`: 1.2 [kg]

```
SickS300 [
  SFVec3f    translation    0 0 0
  SFRotation rotation       0 0 1 0
  SFString   name           "Sick S300"
  SFFloat    noise          0.0
  SFInt32    resolution     540
  SFBool     enablePhysics  TRUE
]
```

The `noise` field specifies the standard deviation of the gaussian depth noise in meters.

The `resolution` field specifies the number of points returned per layer per scan.

The `enablePhysics` field specifies if the sensor should be affected by physics or not.
