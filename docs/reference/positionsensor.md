## PositionSensor

Derived from [Device](device.md).

```
PositionSensor {
  SFFloat noise       0   # [0, inf)
  SFFloat resolution -1   # {-1, [0, inf)}
}
```

### Description

A [PositionSensor](#positionsensor) node can be used in a mechanical simulation to monitor a joint position.
The position sensor can be inserted in the `device` field of a [HingeJoint](hingejoint.md), a [Hinge2Joint](hinge2joint.md), a [SliderJoint](sliderjoint.md), or a [Track](track.md).
Depending on the [Joint](joint.md) type, it will measure the angular position in *radian* [rad] or the linear position in *meter* [m].

### Field Summary

- `noise`: This field allows to define the standard deviation of the Gaussian noise added to the sensor output.
The noise is expressed in *meter* [m] if the parent node is a [SliderJoint](sliderjoint.md) or a [Track](track.md).
It is expressed in *radian* [rad] if the parent node is a [HingeJoint](hingejoint.md) or a [Hinge2Joint](hinge2joint.md).

- `resolution`: This field allows to define the resolution of the sensor, the resolution is the smallest change that it is able to measure.
Setting this field to -1 (default) means that the sensor has an 'infinite' resolution (it can measure any infinitesimal change).
Otherwise, this field accepts values in the interval (0.0, inf).
The resolution is expressed in *meter* [m] if the parent node is a [SliderJoint](sliderjoint.md) or a [Track](track.md).
It is expressed in *radian* [rad] if the parent node is a [HingeJoint](hingejoint.md) or a [Hinge2Joint](hinge2joint.md).

### PositionSensor Functions

#### `wb_position_sensor_enable`
#### `wb_position_sensor_disable`
#### `wb_position_sensor_get_sampling_period`
#### `wb_position_sensor_get_value`
#### `wb_position_sensor_get_type`

%tab-component "language"

%tab "C"

```c
#include <webots/position_sensor.h>

void wb_position_sensor_enable(WbDeviceTag tag, int sampling_period);
void wb_position_sensor_disable(WbDeviceTag tag);
int wb_position_sensor_get_sampling_period(WbDeviceTag tag);
double wb_position_sensor_get_value(WbDeviceTag tag);
int wb_position_sensor_get_type(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <webots/PositionSensor.hpp>

namespace webots {
  class PositionSensor : public Device {
    enum {ROTATIONAL, LINEAR};

    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    double getValue() const;
    int getType() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import PositionSensor

class PositionSensor (Device):
    ROTATIONAL, LINEAR

    def enable(self, samplingPeriod):
    def disable(self):
    def getSamplingPeriod(self):
    def getValue(self):
    def getType(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.PositionSensor;

public class PositionSensor extends Device {
  public final static int ROTATIONAL, LINEAR;

  public void enable(int samplingPeriod);
  public void disable();
  public int getSamplingPeriod();
  public double getValue();
  public int getType();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
WB_ROTATIONAL, WB_LINEAR

wb_position_sensor_enable(tag, sampling_period)
wb_position_sensor_disable(tag)
period = wb_position_sensor_get_sampling_period(tag)
value = wb_position_sensor_get_value(tag)
type = wb_position_sensor_get_type(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/value` | `topic` | webots_ros::Float64Stamped | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 data` |
| `/<device_name>/enable` | `service` | [`webots_ros::set_int`](ros-api.md#common-services) | |
| `/<device_name>/get_sampling_period` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |
| `/<device_name>/get_type` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*enable, disable and read position sensor measurement*

The `wb_position_sensor_enable` function enables measurements of the joint position.
The `sampling_period` argument specifies the sampling period of the sensor and is expressed in milliseconds.
Note that the first measurement will be available only after the first sampling period elapsed.

The `wb_position_sensor_disable` function turns off the position sensor to save CPU time.

The `wb_position_sensor_get_sampling_period` function returns the period given into the `wb_position_sensor_enable` function, or 0 if the device is disabled.

The `wb_position_sensor_get_value` function returns the most recent value measured by the specified position sensor.
Depending on the type, it will return a value in *radian* [rad] (angular position sensor) or in *meter* [m] (linear position sensor).

The `wb_position_sensor_get_type` function returns the type of the position sensor.
It will return `WB_ROTATIONAL` if the sensor is associated with a [HingeJoint](hingejoint.md) or a [Hinge2Joint](hinge2joint.md) node, and `WB_LINEAR` if it is associated with a [SliderJoint](sliderjoint.md) or a [Track](track.md) node.

---

#### `wb_position_sensor_get_brake`
#### `wb_position_sensor_get_motor`

%tab-component "language"

%tab "C"

```c
#include <webots/position_sensor.h>
#include <webots/brake.h>
#include <webots/motor.h>

WbDeviceTag wb_brake_get_brake(WbDeviceTag tag);
WbDeviceTag wb_brake_get_motor(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <webots/PositionSensor.hpp>
#include <webots/Brake.hpp>
#include <webots/Motor.hpp>

namespace webots {
  class PositionSensor : public Device {
    Brake *getBrake() const;
    Motor *getMotor() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import PositionSensor, Brake, Motor

class PositionSensor (Device):
    def getBrake(self):
    def getMotor(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Brake;
import com.cyberbotics.webots.controller.Motor;

public class PositionSensor extends Device {
  public Brake getBrake();
  public Motor getMotor();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
tag = wb_brake_get_brake(tag)
tag = wb_brake_get_motor(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/get_brake_name` | `service` | [`webots_ros::get_string`](ros-api.md#common-services) | |
| `/<device_name>/get_motor_name` | `service` | [`webots_ros::get_string`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*get associated devices*

The `wb_position_sensor_get_brake` and `wb_position_sensor_get_motor` functions return the [Brake](brake.md) and [Motor](motor.md) instances defined in the same [Joint](joint.md) or [Track](track.md) `device` field.
If none is defined they return 0.
