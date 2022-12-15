## Altimeter

Derived from [Device](device.md) and [Solid](solid.md).

### Description

The [Altimeter](#altimeter) node is used to model an altimeter sensor, which can obtain information about its height above the global reference plane from the controller program.

### Field Summary

```
Altimeter {
  SFString name             "altimeter"   # used by wb_robot_get_device()
  SFFloat  accuracy         0             # [0, inf)
  SFFloat  resolution       -1            # {-1, [0, inf)}
}
```
**Note**: The above description lists only the fields specific to the Altimeter node. The complete field list can be found in the [Altimeter.wrl]({{ url.github_tree }}/resources/nodes/Altimeter.wrl) definition file.

- `name`: This field defines the string name used by `wb_robot_get_device()` to get the `WbDeviceTag` handle of this sensor. Its default value is "altimeter".

- `accuracy`: This field defines the precision of the altimeter, that is the standard deviation (expressed in meter) of the gaussian noise added to the altitude.

- `resolution`: This field allows the user to define the resolution of the sensor. The resolution is the smallest change that it is able to measure.
Setting this field to -1 (default) means that the sensor has an 'infinite' resolution (it can measure any infinitesimal change).
Otherwise this field accepts any value in the interval `[0.0, inf)`.


### Altimeter Functions

#### `wb_altimeter_enable`
#### `wb_altimeter_disable`
#### `wb_altimeter_get_sampling_period`
#### `wb_altimeter_get_value`

%tab-component "language"

%tab "C"

```c
#include <webots/altimeter.h>

void wb_altimeter_enable(WbDeviceTag tag, int sampling period)
void wb_altimeter_disable(WbDeviceTag tag)
int wb_altimeter_get_sampling_period(WbDeviceTag tag)
double wb_altimeter_get_value(WbDeviceTag tag)
```
%tab-end

%tab "C++"

```cpp
#include "webots/Altimeter.hpp>"

namespace webots {
  class Altimeter : public Device {
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    double getValue() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Altimeter

class Altimeter (Device):
    def enable(self, samplingPeriod):
    def disable(self):
    def getSamplingPeriod(self):
    def getValue(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Altimeter;

public class Altimeter extends Device {
  public void enable(int samplingPeriod);
  public void disable();
  public int getSamplingPeriod();
  public double getValue();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_altimeter_enable(tag, sampling_period)
wb_altimeter_disable(tag)
period = wb_altimeter_get_sampling_period(tag)
altitude = wb_altimeter_get_value(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/value` | `topic` | webots_ros::Float64stamped | [`Header`](https://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |
| `/<device_name>/enable` | `service` | [`webots_ros::set_int`](ros-api.md#common-services) | | 
| `/<device_name>/get_sampling_period` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*enable, disable, and read last altimeter measurement*

The `wb_altimeter_enable` function allows the user to enable altimeter measurements.

The `sampling_period` arguement specifies the sampling period of the sensor and is expressed in milliseconds.
Note that the first measurement will be available only after the first sampling period has elapsed.

The `wb_altimeter_disable` function turns the altimeter off, saving computation time.

The `wb_altimeter_get_value` function returns the last value measured by the altimeter.

The `wb_altimeter_get_sampling_period` function returns the period given into the `wb_altimeter_enable` function, or 0 if the device is disabled.
