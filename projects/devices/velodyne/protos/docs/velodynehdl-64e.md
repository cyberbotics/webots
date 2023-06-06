The `Velodyne HDL 64E` is a 64 layers lidar with a range of up to 120 meters and a field of view of 360 degrees, it returns 4500 points per layer per scan.

The model of the `Velodyne HDL 64` contains a gaussian noise with a standard deviation of 0.02 meter and a rotating head.

```
VelodyneHDL-64E {
  SFVec3f    translation    0 0 0
  SFRotation rotation       0 0 1 0
  SFString   name           "Velodyne HDL-32E"
  SFBool     enablePhysics  TRUE
}
```
