The [SICK LD-MRS](https://www.sick.com/us/en/detection-and-ranging-solutions/3d-lidar-sensors/ld-mrs/c/g91913) is a multi-layer lidar designed for harsh outdoor environments.

The `SickLdMrs` PROTO contains a [Lidar](https://cyberbotics.com/doc/reference/lidar) node which covers the main usual cases.
Its reference name (to get it from the Webots API) matches directly with the `SickLdMrs.name` field.
It has the following properties:

- Its number of layers is 4 except for the `800001S01` type which is 8.
- Its horizontal scanning range is 85°, shifted horizontally by 7,5°.
- Its maximal range is 300 meters.
- Each layer is separated vertically by 0.8°.

In addition to this main [Lidar](https://cyberbotics.com/doc/reference/lidar) node, the PROTO contains a second [Lidar](https://cyberbotics.com/doc/reference/lidar) in order to model the overlapping long-range layers.
This sensor reference name is the `SickLdMrs.name` field concatenated by the ` (long range)` string.
In addition to the properties of the main lidar, it has the following properties:

- Its horizontal scanning range is 110°, shifted horizontally by 5°.
- Its number of layers is `SickLdMrs.measurementLayers` divided by 2.

The internal [Lidars](https://cyberbotics.com/doc/reference/lidar) are oriented as follows:

- Layer 0 corresponds to the bottom layer.
- First response values are corresponding to the device right.

In comparison to the real sensor, the simulated model has the following limitations:

- The scanning range resolution is constant over the entire scan.
- The vertical azimuth is constant over the entire scan.

```
SickLdMrs {
  SFVec3f    translation       0 0 0
  SFRotation rotation          0 0 1 0
  SFString   name              "Sick LD-MRS"
  SFString   type              "400001"
  SFString   angularResolution "0.5 [deg]"
  SFFloat    noise             0.001
  SFBool     enablePhysics     TRUE
}
```

The `type` field specifies the `SICK LD-MRS` type (cf. [specifications](https://www.sick.com/us/en/detection-and-ranging-solutions/3d-lidar-sensors/ld-mrs/c/g91913)).
The value could be one of the following: `400001`, `400102`, `400001S01`, `400102S01` or `800001S01`.

The `noise` field specifies the standard deviation of gaussian image noise in meters.

The `angularResolution` field specifies the vertical angular gap between two measurements.
From the `SICK LD-MRS` specification, it can be either 0.5, 0.25 or 0.125 degrees.
Internally, the `Lidar.horizontalResolution` is directly affected by this field.
The value could be one of the following: `0.5 [deg]`, `0.25 [deg]` or `0.125 [deg]`.

The `enablePhysics` field specifies if the sensor should be affected by physics (mass = 1 [kg]) or not.
