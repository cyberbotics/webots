The `Robotis LDS-01` is a 1 layer lidar with a range of up to 3.5 meters and a field of view of up to 360 degrees.

```
RobotisLds01 {
  SFVec3f    translation    0 0 0.02
  SFRotation rotation       0 0 1 0
  SFString   name           "LDS-01"
  SFFloat    noise          0.0043
  SFBool     enablePhysics  TRUE
}
```

The `noise` field specifies the standard deviation of the gaussian depth noise in meters.

The `enablePhysics` field specifies if the sensor should be affected by physics (mass = 0.125 [kg]) or not.
