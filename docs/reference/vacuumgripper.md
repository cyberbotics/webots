## Vacuum Gripper

Derived from [Device](device.md) and [Solid](solid.md).

```
VacuumGripper {
  SFBool   isOn              FALSE         # {TRUE, FALSE}
  SFFloat  tensileStrength   -1            # {-1, [0, inf)}
  SFFloat  shearStrength     -1            # {-1, [0, inf)}
  SFInt32  contactPoints     3             # [1, inf)
}
```

### Description

The [VacuumGripper](#vacuum-gripper) node is used to simulate physical links created by vacuum suction.

The physical connection can be created and destroyed at run time by the robot's controller.
The [VacuumGripper](#vacuum-gripper) node can connect to dynamic [Solid](solid.md) nodes, e.g., for pick and place operations, as well as to the static environment, e.g., for locomotion.
Then, the detection of the presence of an object to connect to is based on collision detection.

### Field Summary

- `isOn`: represents the state of the [VacuumGripper](#vacuum-gripper).
The state can be changed through the `wb_vacuum_gripper_turn_on` and `wb_vacuum_gripper_turn_off` API functions.
The *on state* just means that the suction pump is active, but it does not indicates whether or not an actual physical link exists with an object.
The actual physical link exists only if a valid [Solid](solid.md) object collided with the [VacuumGripper](#vacuum-gripper) after the `wb_vacuum_gripper_turn_on` function was called.

  > **Note**:
If the `VacuumGripper` node is on in the .wbt file and a valid [Solid](solid.md) node is available, then the simulation will automatically connect to the [Solid](solid.md) object.
You can take advantage of this feature to start your simulation with the desired mechanical configuration.

- `tensileStrength`: maximum tensile force in *newton* [N] that the suction mechanism can withstand before it breaks.
This can be used to simulate the rupture of the suction mechanism.
The tensile force corresponds to a force that pulls the two objects apart (in the negative *x*-axes direction).
When the tensile force exceeds the tensile strength, the link breaks.
The default value -1 indicates an infinitely strong suction mechanism that does not break no matter how much force is applied.

- `shearStrength`: indicates the maximum shear force in *newtons* [N] that the suction mechanism can withstand before it breaks.
This can be used to simulate the rupture of the suction mechanism.
The `shearStrength` field specifies the ability of the suction mechanism to withstand a force that would makes them slide against each other in opposite directions (in the *yz*-plane).
The default value -1 indicates an infinitely strong suction mechanism that does not break no matter how much force is applied.

- `contactPoints`: indicates the minimum number of contact points with the [Solid](solid.md) object required to connect.


### VacuumGripper Functions

#### `wb_vacuum_gripper_enable_presence`
#### `wb_vacuum_gripper_disable_presence`
#### `wb_vacuum_gripper_get_presence_sampling_period`
#### `wb_vacuum_gripper_get_presence`

%tab-component "language"

%tab "C"

```c
#include <webots/vacuum_gripper.h>

void wb_vacuum_gripper_enable_presence(WbDeviceTag tag, int sampling_period);
void wb_vacuum_gripper_disable_presence(WbDeviceTag tag);
int wb_vacuum_gripper_get_presence_sampling_period(WbDeviceTag tag);
bool wb_vacuum_gripper_get_presence(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <webots/VacuumGripper.hpp>

namespace webots {
  class VacuumGripper : public Device {
    virtual void enablePresence(int samplingPeriod);
    virtual void disablePresence();
    int getPresenceSamplingPeriod() const;
    bool getPresence() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import VacuumGripper

class VacuumGripper (Device):
    def enablePresence(self, samplingPeriod):
    def disablePresence(self):
    def getPresenceSamplingPeriod(self):
    def getPresence(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.VacuumGripper;

public class VacuumGripper extends Device {
  public void enablePresence(int samplingPeriod);
  public void disablePresence();
  public int getPresenceSamplingPeriod();
  public boolean getPresence();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_vacuum_gripper_enable_presence(tag, sampling_period)
wb_vacuum_gripper_disable_presence(tag)
period = wb_vacuum_gripper_get_presence_sampling_period(tag)
presence = wb_vacuum_gripper_get_presence(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/presence` | `topic` | `webots_ros::BoolStamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`bool data` |
| `/<device_name>/presence_sensor/enable` | `service` | [`webots_ros::set_int`](ros-api.md#common-services) | |
| `/<device_name>/presence_sensor/get_sampling_period` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*detect the presence of a connected object*

The `wb_vacuum_gripper_enable_presence` function starts querying the presence of a linked [Solid](solid.md) object.
The `sampling_period` argument specifies the sampling period of the presence sensor.
It is expressed in milliseconds.
Note that it will be active only after the first sampling period elapsed.

The `wb_vacuum_gripper_disable_presence` function stops querying the presence sensor of a linked [Solid](solid.md) object.

The `wb_vacuum_gripper_get_presence_sampling_period` function returns the period at which the presence sensor of the linked [Solid](solid.md) object is queried.
The `wb_vacuum_gripper_get_presence` function returns the current *presence* state, it returns *TRUE* if a [Solid](solid.md) object is connected and *FALSE* otherwise.

---

#### `wb_vacuum_gripper_turn_on`
#### `wb_vacuum_gripper_turn_off`
#### `wb_vacuum_gripper_is_on`

%tab-component "language"

%tab "C"

```c
#include <webots/vacuum_gripper.h>

void wb_vacuum_gripper_turn_on(WbDeviceTag tag);
void wb_vacuum_gripper_turn_off(WbDeviceTag tag);
bool wb_vacuum_gripper_is_on(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <webots/VacuumGripper.hpp>

namespace webots {
  class VacuumGripper : public Device {
    virtual void turnOn();
    virtual void turnOff();
    virtual bool isOn()
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import VacuumGripper

class VacuumGripper (Device):
    def turnOn(self):
    def turnOff(self):
    def isOn(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.VacuumGripper;

public class VacuumGripper extends Device {
  public void turnOn();
  public void turnOff();
  public boolean isOn();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_vacuum_gripper_turn_on(tag)
wb_vacuum_gripper_turn_off(tag)
on = wb_vacuum_gripper_is_on(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/turn_on` | `service` | [`webots_ros::set_bool`](ros-api.md#common-services) | |
| `/<device_name>/is_on` | `service` | [`webots_ros::get_bool`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*create / destroy the physical connection wit a solid object*

The `wb_vacuum_gripper_turn_on` and `wb_vacuum_gripper_turn_off` functions can be used to activate or deactivate the [VacuumGripper](#vacuum-gripper) suction pump and changing the status of the `isOn` field.
This will eventually create or destroy the physical connection with a [Solid](solid.md) object.

If dynamic [Solid](solid.md) object collides with the [VacuumGripper](#vacuum-gripper) while the device is on, a physical link will be created between the [VacuumGripper](#vacuum-gripper) and the [Solid](solid.md) object.
The two connected bodies will then keep a constant distance and orientation with respect to each other from this moment on.

If the `wb_vacuum_gripper_turn_off` function is invoked while there is a physical link between the [VacuumGripper](#vacuum-gripper) and a [Solid](solid.md) object, the link will be destroyed.
