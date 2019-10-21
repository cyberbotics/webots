## Brake

Derived from [Device](device.md).

```
Brake {
}
```

### Description

A [Brake](#brake) node can be used in a mechanical simulation in order to change the friction of a joint.
The [Brake](#brake) node can be inserted in the `device` field of a [HingeJoint](hingejoint.md), a [Hinge2Joint](hinge2joint.md), a [SliderJoint](sliderjoint.md), or a [Track](track.md).

### Brake Functions

#### `wb_brake_set_damping_constant`
#### `wb_brake_get_type`

%tab-component "language"

%tab "C"

```c
#include <webots/brake.h>

void wb_brake_set_damping_constant(WbDeviceTag tag, double damping_constant);
int wb_brake_get_type(WbDeviceTag tag);
```
%tab-end

%tab "C++"

```cpp
#include <webots/Brake.hpp>

namespace webots {
  class Brake : public Device {
    enum {ROTATIONAL, LINEAR};

    void setDampingConstant(double dampingConstant) const;
    int getType() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Brake

class Brake (Device):
    ROTATIONAL, LINEAR

    def setDampingConstant(self, dampingConstant):
    def getType(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Brake;

public class Brake extends Device {
  public final static int ROTATIONAL, LINEAR;

  public void setDampingConstant(double dampingConstant);
  public int getType();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
WB_ROTATIONAL, WB_LINEAR

wb_brake_set_damping_constant(tag, damping_constant)
type = wb_brake_get_type(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/set_damping_constant` | `service` | [`webots_ros::set_float`](ros-api.md#common-services) | |
| `/<device_name>/get_type` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*set the damping constant coefficient of the joint and get the type of brake*

The `wb_brake_set_damping_constant` function sets the value of the dampingConstant coefficient (Ns/m or Nms) of the joint.
If any dampingConstant is already set using [JointParameters](jointparameters.md) the resulting dampingConstant coefficient is the sum of the one in the [JointParameters](jointparameters.md) and the one set using the `wb_brake_set_damping_constant` function.

The `wb_brake_get_type` function returns the type of the brake.
It will return `WB_ROTATIONAL` if the sensor is associated with a [HingeJoint](hingejoint.md) or a [Hinge2Joint](hinge2joint.md) node, and `WB_LINEAR` if it is associated with a [SliderJoint](sliderjoint.md) or a [Track](track.md) node.

---

#### `wb_brake_get_motor`
#### `wb_brake_get_position_sensor`

%tab-component "language"

%tab "C"

```c
#include <webots/brake.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

WbDeviceTag wb_brake_get_motor(WbDeviceTag tag);
WbDeviceTag wb_brake_get_position_sensor(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Brake.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

namespace webots {
  class Brake : public Device {
    Motor *getMotor() const;
    PositionSensor *getPositionSensor() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Brake, Motor, PositionSensor

class Brake (Device):
    def getMotor(self):
    def getPositionSensor(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Brake;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;

public class Brake extends Device {
  public Motor getMotor();
  public PositionSensor getPositionSensor();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
tag = wb_brake_get_motor(tag)
tag = wb_brake_get_position_sensor(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/get_motor_name` | `service` | [`webots_ros::get_string`](ros-api.md#common-services) | |
| `/<device_name>/get_position_sensor_name` | `service` | [`webots_ros::get_string`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*get associated devices*

The `wb_brake_get_motor` and `wb_brake_get_position_sensor` functions return the [Motor](motor.md) and [PositionSensor](positionsensor.md) instances defined in the same [Joint](joint.md) or [Track](track.md) `device` field.
If none is defined they return 0.
