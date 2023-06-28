The `SICK LMS 291` is a 1 layer lidar with a range of up to 80 meters and a field of view of up to 180 degrees.

The model of the `SICK LMS 291` contains a spherical projection, a configurable fixed resolution and a configurable gaussian noise.

```
SickLms291 {
  SFVec3f    translation 0 0 0
  SFRotation rotation    0 0 1 0
  SFString   name        "Sick LMS 291"
  SFFloat    noise       0.0
  SFInt32    resolution  180
}
```

The `noise` field specifies the standard deviation of the gaussian depth noise in meters.

The `resolution` field specifies the number of points returned per layer per scan.
