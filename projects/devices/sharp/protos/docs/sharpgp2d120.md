The [Sharp GP2D120](https://www.pololu.com/file/0J157/GP2D120-DATA-SHEET.pdf) is a distance measuring sensor with integrated signal processing and analog voltage output.
This device outputs the voltage corresponding to the detection distance.

The model has the following specifications:

- `effective range`: 4 to 30 [cm]
- `output type`: analog
- `response time`: 39 [ms]
- `start up delay`: 44 [ms]
- `average consumption`: 33 [mA]

```
PROTO SharpGP2D120 {
  SFVec3f     translation  0 0 0
  SFRotation  rotation     0 0 1 0
  SFString    name         "Sharp's IR sensor GP2D120"
  SFString    model        "GP2D120"
}
```

The `lookupTable` field of the [DistanceSensor](https://cyberbotics.com/doc/reference/distancesensor) is already implemented according to the characteristics found in the [datasheet](https://www.pololu.com/file/0J157/GP2D120-DATA-SHEET.pdf).

The `wb_distance_sensor_get_value` function returns the voltage/intensity of the measurement. To convert these values, use the following formulas:
- Convert meters to voltage: `y(x) = 0.5131*x^(-0.5735)-0.6143`
- Convert voltage to meters: `y(x) = 0.1594*x^(-0.8533)-0.02916`
