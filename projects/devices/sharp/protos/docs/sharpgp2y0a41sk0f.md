The [Sharp GP2Y0A41SK0F](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf) is a distance measuring sensor unit, composed of an integrated combination of PSD (position sensitive detector), IR-LED (infrared emitting diode) and signal processing circuit.
The variety of the reflectivity of the object, the environmental temperature and the operating duration are not influenced easily to the distance detection because of adopting the triangulation method.
This device outputs the voltage corresponding to the detection distance.
So this sensor can also be used as a proximity sensor.

The model has the following specifications:

- `effective range`: 4 to 30 [cm]
- `output type`: analog
- `response time`: 16.5 [ms]
- `start up delay`: 26 [ms]
- `average consumption`: 12 [mA]

```
PROTO SharpGP2Y0A41SK0F {
  SFVec3f     translation  0 0 0
  SFRotation  rotation     0 0 1 0
  SFString    name         "Sharp's IR sensor GP2Y0A41SK0F"
}
```
The `lookupTable` field of the [DistanceSensor](https://cyberbotics.com/doc/reference/distancesensor) is already implemented according to the characteristics found in the [datasheet](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf).

The `wb_distance_sensor_get_value` function returns the voltage/intensity of the measurement. To convert these values, use the following formulas:
- Convert meters to voltage: `y(x) = 0.5131*x^(-0.5735)-0.6143`
- Convert voltage to meters: `y(x) = 0.1594*x^(-0.8533)-0.02916`
