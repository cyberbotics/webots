The [Sharp GP2Y0A710K0F](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a710k_e.pdf) is a distance measuring sensor unit, composed of an integrated combination of PSD (position sensitive detector), IR-LED (infrared emitting diode) and signal processing circuit.
The variety of the reflectivity of the object, the environmental temperature and the operating duration are not influenced easily to the distance detection because of adopting the triangulation method.
This device outputs the voltage corresponding to the detection distance.
So this sensor can also be used as a proximity sensor.

The model has the following specifications:

- `effective range`: 100 to 550 [cm]
- `output type`: analog
- `response time`: 21 [ms]
- `start up delay`: 26 [ms]
- `average consumption`: 30 [mA]


```
PROTO SharpGP2Y0A710K0F {
  SFVec3f     translation  0 0 0
  SFRotation  rotation     0 0 1 0
  SFString    name         "Sharp's IR sensor GP2Y0A710K0F"
}
```
The `lookupTable` field of the [DistanceSensor](https://cyberbotics.com/doc/reference/distancesensor) is already implemented according to the characteristics found in the [datasheet](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a710k_e.pdf).

The `wb_distance_sensor_get_value` function returns the voltage/intensity of the measurement. To convert these values, use the following formulas:
- Convert meters to voltage: `y(x) = 1.962*x^(-0.5214)+0.4926`
- Convert voltage to meters: `y(x) = 20.24*x^(-4.76)+0.6632`
