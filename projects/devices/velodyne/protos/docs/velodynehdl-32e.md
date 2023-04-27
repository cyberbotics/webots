The `Velodyne HDL 32E` is a 32 layers lidar with a range of up to 70 meters and a field of view of 360 degrees, it returns 4500 points per layer per scan.

The model of the `Velodyne HDL 32` contains a gaussian noise with a standard deviation of 0.02 meter and a rotating head.

```
VelodyneHDL-32E {
  SFVec3f    translation    0 0 0
  SFRotation rotation       0 0 1 0
  SFString   name           "Velodyne HDL-32E"
  SFBool     enablePhysics  TRUE
}
```
