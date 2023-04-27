The [Microsoft Kinect](https://en.wikipedia.org/wiki/Kinect) is a motion sensing input device.
It is modeled using a regular RGB [Camera](https://cyberbotics.com/doc/reference/camera) and a [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder) device to retrieve the depth information in meters.

Derived from [Solid](https://cyberbotics.com/doc/reference/solid).
Includes a [Camera](https://cyberbotics.com/doc/reference/camera), a [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder) and optionally a [RotationalMotor](https://cyberbotics.com/doc/reference/rotationalmotor).

```
Kinect {
  SFVec3f    translation   0 0 0
  SFRotation rotation      0 0 1 0
  SFString   name          "kinect"
  SFInt32    width         320
  SFInt32    height        190
  SFFloat    maxRange      3.5
  SFFloat    colorNoise    0.0
  SFFloat    rangeNoise    0.0
  SFString   cameraName    "kinect"
  SFString   tiltMotorName "tilt motor"
  SFBool     foot          TRUE
}
```

- `width`: Defines the `width` field of the [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder) and [Camera](https://cyberbotics.com/doc/reference/camera) nodes.
- `height`: Defines the `height` field of the [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder) and [Camera](https://cyberbotics.com/doc/reference/camera) nodes.
- `maxRange`: Defines the `maxRange` field of the [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder).
- `colorNoise`: Defines the `noise` field of the [Camera](https://cyberbotics.com/doc/reference/camera).
- `rangeNoise`: Defines the `noise` field of the [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder).
- `cameraName`: Defines the name of the [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder) and [Camera](https://cyberbotics.com/doc/reference/camera) nodes. The [RangeFinder](https://cyberbotics.com/doc/reference/rangefinder) is named `<cameraName> + ' range'` and the [Camera](https://cyberbotics.com/doc/reference/camera) `<cameraName> + ' color'`.
- `tiltMotorName`: Defines the name of the foot tilt [RotationalMotor](https://cyberbotics.com/doc/reference/rotationalmotor) (when the `foot` field is enabled).
- `foot`: Defines whether the articulated foot is present.
