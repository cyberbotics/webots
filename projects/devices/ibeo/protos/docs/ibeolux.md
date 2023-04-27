The `Ibeo LUX` is a 4 layers lidar with a range of up to 200 meters and a field of view of up to 110 degrees, it returns 680 points per layer per scan.

The model of the `Ibeo LUX` contains a spherical projection, a fixed resolution of 0.04 meter and a gaussian noise with a standard deviation of 0.1 meter.

```
IbeoLux {
  SFVec3f    translation             0 0 0
  SFRotation rotation                0 0 1 0
  SFString   name                    "Ibeo Lux"
  SFBool     useExtendedFieldOfView  FALSE
  SFBool     fastModel               FALSE
}
```

The `IbeoLux` PROTO can either be used in normal field of view mode (80 degrees field of view) or in extended field of view mode (110 degrees field of view) depending on the value of the `useExtendedFieldOfView` field.

The `fastModel` field can be used to simplify the model of the sensor by removing the spherical projection, the noise and the limited resolution in order to speed up the simulation.
