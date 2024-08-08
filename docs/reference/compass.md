## Compass

Derived from [Device](device.md) and [Solid](solid.md).

```
Compass {
  MFVec3f lookupTable [ ]    # lookup table
  SFBool  xAxis       TRUE   # {TRUE, FALSE}
  SFBool  yAxis       TRUE   # {TRUE, FALSE}
  SFBool  zAxis       TRUE   # {TRUE, FALSE}
  SFFloat resolution  -1     # {-1, [0, inf)}
}
```

### Description

A [Compass](#compass) node can be used to model a 1, 2 or 3-axis digital compass (magnetic sensor).
The [Compass](#compass) node returns a vector that indicates the north direction specified by the `coordinateSystem` field of the [WorldInfo](worldinfo.md) node.

### Field Summary

- `lookupTable`: This field optionally specifies a lookup table that can be used for mapping each vector component (between -1.0 and +1.0) to device specific output values.
By default the lookup table is empty and therefore no mapping is applied.
See the section on the [DistanceSensor](distancesensor.md#lookup-table) node for more explanation on how a `lookupTable` works.

- `xAxis, yAxis, zAxis`: Each of these boolean fields specifies if the computation should be enabled or disabled for the specified axis.
If one of these fields is set to FALSE, then the corresponding vector element will not be computed and it will return *NaN* (Not a Number).
For example if zAxis is FALSE, then the second element of the array returned by the `wb_compass_get_values` function will always return *NaN*.
The default is that all three axes are enabled (TRUE).
Modifying these fields makes it possible to choose between a single, dual or a three-axis digital compass and to specify which axes will be used.

- `resolution`: This field allows to define the resolution of the sensor, the resolution is the smallest change that it is able to measure.
Setting this field to -1 (default) means that the sensor has an 'infinite' resolution (it can measure any infinitesimal change).
This field accepts any value in the interval (0.0, inf).

### Compass Functions

#### `wb_compass_enable`
#### `wb_compass_disable`
#### `wb_compass_get_sampling_period`
#### `wb_compass_get_values`
#### `wb_compass_get_lookup_table_size`
#### `wb_compass_get_lookup_table`

%tab-component "language"

%tab "C"

```c
#include <webots/compass.h>

void wb_compass_enable(WbDeviceTag tag, int sampling_period);
void wb_compass_disable(WbDeviceTag tag);
int wb_compass_get_sampling_period(WbDeviceTag tag);
const double *wb_compass_get_values(WbDeviceTag tag);
int wb_compass_get_lookup_table_size(WbDeviceTag tag);
const double *wb_compass_get_lookup_table(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Compass.hpp>

namespace webots {
  class Compass : public Device {
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    const double *getValues() const;
    int getLookupTableSize() const;
    const double *getLookupTable() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Compass

class Compass (Device):
    def enable(self, samplingPeriod):
    def disable(self):
    def getSamplingPeriod(self):
    def getValues(self):
    def getLookupTable(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Compass;

public class Compass extends Device {
  public void enable(int samplingPeriod);
  public void disable();
  public int getSamplingPeriod();
  public double[] getValues();
  public double[] getLookupTable();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_compass_enable(tag, sampling_period)
wb_compass_disable(tag)
period = wb_compass_get_sampling_period(tag)
x_y_z_array = wb_compass_get_values(tag)
lookup_table_array = wb_compass_get_lookup_table(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/values` | `topic` | [`sensor_msgs::MagneticField`](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html) | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>[`geometry_msgs/Vector3`](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html) `magnetic_field`<br/>`float64[9] magnetic_field_covariance`<br/> |
| `/<device_name>/enable` | `service` | [`webots_ros::set_int`](ros-api.md#common-services) | |
| `/<device_name>/get_sampling_period` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |
| `/<device_name>/get_lookup_table` | `service` | [`webots_ros::get_float_array`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*enable, disable and read the output values of the compass device*

The `wb_compass_enable` function turns on the [Compass](#compass) measurements.
The `sampling_period` argument specifies the sampling period of the sensor and is expressed in milliseconds.
Note that the first measurement will be available only after the first sampling period elapsed.

The `wb_compass_disable` function turns off the [Compass](#compass) device.

The `wb_compass_get_sampling_period` function returns the period given into the `wb_compass_enable` function, or 0 if the device is disabled.

The `wb_compass_get_values` function returns the current [Compass](#compass) measurement.
The returned vector indicates the north direction in the coordinate system of the [Compass](#compass) device.
Here is the internal algorithm of the `wb_compass_get_values` function in pseudo-code:

```c
float[3] wb_compass_get_values() {
  float[3] n = getGlobalNorthDirection();
  n = rotateToCompassOrientation3D(n);
  n = normalizeVector3D(n);
  n[0] = applyLookupTable(n[0]);
  n[1] = applyLookupTable(n[1]);
  n[2] = applyLookupTable(n[2]);
  if (xAxis == FALSE) n[0] = 0.0;
  if (yAxis == FALSE) n[1] = 0.0;
  if (zAxis == FALSE) n[2] = 0.0;
  return n;
}
```

If the lookupTable is empty and all three xAxis, yAxis and zAxis fields are TRUE then the length of the returned vector is 1.0.

The values are returned as a 3D-vector, therefore only the indices 0, 1, and 2 are valid for accessing the vector.
Let's look at one example.
The default value of the `WorldInfo.coordinateSystem` field is `"ENU"` and therefore the north direction is along with the Y-positive axis.
Now if the [Compass](#compass) node is in *upright* position, meaning that its z-axis is aligned with the global z-axis, then the bearing angle in degrees can be computed as follows:

```c
double get_bearing_in_degrees() {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[1], north[0]);
  double bearing = (rad / M_PI) * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}
```

The `wb_compass_get_lookup_table_size` function returns the number of rows in the lookup table.

The `wb_compass_get_lookup_table` function returns the values of the lookup table.
This function returns a matrix containing exactly N * 3 values (N represents the number of mapped values obtained with the `wb_compass_get_lookup_table_size` function) that shall be interpreted as a N x 3 table.

> **Note** [C, C++]: The returned vector is a pointer to the internal values managed by the [Compass](#compass) node, therefore it is illegal to free this pointer.
Furthermore, note that the pointed values are only valid until the next call to the `wb_robot_step` or `Robot::step` functions.
If these values are needed for a longer period they must be copied.

<!-- -->

> **Note** [Python]: The `getValues` function returns the vector as a list containing three floats.
