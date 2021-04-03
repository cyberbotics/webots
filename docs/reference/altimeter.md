## Altimeter

Derived from [Device](device.md) and [Solid](solid.md).

```
Altimeter {
  SFFloat  accuracy         0             # [0, inf)
  SFFloat  noiseCorrelation 0             # [0, 1]
  SFFloat  resolution       -1            # {-1, [0, inf)}
}
```

### Description

The [Altimeter](#altimeter) node is used to model an Altimeter sensor, which can obtain information about its height above the global reference plane from the controller program.

### Field Summary

-  `accuracy`: This field defines the precision of the altimeter, that is the standard deviation (expressed in meter) of the gaussian noise added to the altitude.

- `resolution`: This field allows the user to define the resolution of the sensor. The resolution is the smallest change that it is able to measure.
Setting this field to -1 (default) means that the sensor has an 'infinite' resolution (it can measure any infinitesimal change).
This field accepts any value in the interval (0.0, inf).

### Altimeter Functions

#### `wb_altimeter_enable`
#### `wb_altimeter_disable`
#### `wb_altimeter_get_sampling_period`
#### `wb_altimeter_get_value`

%tab-component "language"

%tab "C"

```c
#include <webots/altimeter.h>

void wb_altimeter_enable(WbDeviceTag tag, int sampling period);
void wb_altimeter_disable(WbDeviceTag tag);
int wb_altimeter_get_sampling_period(WbDeviceTag tag);
double wb_altimeter_get_value(WbDeviceTag tag);
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
wb_altimeter_enable(tag, sampling_period)
wb_altimeter_disable(tag)
period = wb_altimeter_get_sampling_period(tag)
altitude = wb_altimeter_get_value(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |

