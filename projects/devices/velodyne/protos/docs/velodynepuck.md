The `Velodyne Puck` is a 16 layers lidar with a range of up to 100 meters and a field of view of 360 degrees, it returns 3600 points per layer per scan. The `Velodyne Puck` comes in 3 different versions (selectable with the ` version`):
  - **Puck**: Default version (also known as the Velodyne VLP-16).
  - **Puck LITE**: Lighter weight version of the PUCK (590gr instead of 830gr).
  - **Puck Hi-Res**: Version with a 20° vertical FoV for a tighter layer distribution (1.33° between layers instead of 2.00°).

The model of the `Velodyne Puck` contains a spherical projection and a gaussian noise with a standard deviation of 0.03 meter.

```
VelodynePuck {
  SFVec3f    translation    0 0 0
  SFRotation rotation       0 0 1 0
  SFString   name           "Velodyne VLP-16"
  SFString   version        "Puck"
  SFBool     enablePhysics  TRUE
}
```
