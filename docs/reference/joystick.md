## Joystick

The [Joystick](#joystick) API doesn't correspond to a Webots node.
It is a set of functions available for every [Robot](robot.md) node to read the input from a real joystick.
Therefore, the joystick functions do not require any `WbDeviceTag` parameter.

Each physical joystick can be used by one controller at a time only.
If several joysticks are connected, different controllers may be able to use a different joystick.

> **Note**: In C++, Python and Java the joystick functions are in a dedicated class called `Joystick`.
In order to get the `Joystick` instance, you should call the `getJoystick` function of the `Robot` class.

### Joystick Functions

#### `wb_joystick_enable`
#### `wb_joystick_disable`
#### `wb_joystick_get_sampling_period`

%tab-component "language"

%tab "C"

```c
#include <webots/joystick.h>

void wb_joystick_enable(int sampling_period);
void wb_joystick_disable();
int wb_joystick_get_sampling_period();
```

%tab-end

%tab "C++"

```cpp
#include <webots/Joystick.hpp>

namespace webots {
  class Joystick {
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Joystick

class Joystick:
    def enable(self, samplingPeriod):
    def disable(self):
    def getSamplingPeriod(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Joystick;

public class Joystick {
  public void enable(int samplingPeriod);
  public void disable();
  public int getSamplingPeriod();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_joystick_enable(sampling_period)
wb_joystick_disable()
period = wb_joystick_get_sampling_period()
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/joystick/enable` | `service` | [`webots_ros::set_int`](ros-api.md#common-services) | |
| `/joystick/get_sampling_period` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*enable/disable joystick*

The `wb_joystick_enable` function allows the user to enable joystick measurements.
When this function is called the first free joystick is paired with the controller.
The `sampling_period` argument specifies the sampling period of the [Joystick](#joystick) and is expressed in milliseconds.
Note that the joystick will be active only after the first sampling period elapsed.

The `wb_joystick_disable` function turns the joystick off.
The joystick is released so that it can be used by another controller.

The `wb_joystick_get_sampling_period` function returns the value previously passed to the `wb_joystick_enable` function, or 0 if the device is disabled.

---

#### `wb_joystick_is_connected`

%tab-component "language"

%tab "C"

```c
#include <webots/joystick.h>

bool wb_joystick_is_connected();
```

%tab-end

%tab "C++"

```cpp
#include <webots/Joystick.hpp>

namespace webots {
  class Joystick {
    bool isConnected() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Joystick

class Joystick:
    def isConnected(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Joystick;

public class Joystick {
  public boolean isConnected();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
connected = wb_joystick_is_connected()
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/joystick/is_connected` | `service` | [`webots_ros::get_bool`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*check if a joystick is paired with this controller*

Once the joystick is enabled, this function can be used to check if a free joystick has been paired with the controller or if no available joystick was found.

---

#### `wb_joystick_get_model`

%tab-component "language"

%tab "C"

```c
#include <webots/joystick.h>

const char *wb_joystick_get_model();
```

%tab-end

%tab "C++"

```cpp
#include <webots/Joystick.hpp>

namespace webots {
  class Joystick {
    std::string getModel() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Joystick

class Joystick:
    def getModel(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Joystick;

public class Joystick {
  public String getModel();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
model = wb_joystick_get_model()
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/joystick/get_model` | `service` | [`webots_ros::get_string`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*get the model of the currently connected joystick*

When a joystick is connected to the controller, this function returns the model of the joystick.
If no joystick is connected to the controller, a NULL pointer is returned instead.
The returned model of the joystick may looks like: `Logitech G29 Driving Force Racing Wheel USB`.

---

#### `wb_joystick_get_number_of_axes`
#### `wb_joystick_get_axis_value`

%tab-component "language"

%tab "C"

```c
#include <webots/joystick.h>

int wb_joystick_get_number_of_axes();
int wb_joystick_get_axis_value(int axis);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Joystick.hpp>

namespace webots {
  class Joystick {
    int getNumberOfAxes() const;
    int getAxisValue(int axis) const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Joystick

class Joystick:
    def getNumberOfAxes(self):
    def getAxisValue(self, axis):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Joystick;

public class Joystick {
  public int getNumberOfAxes();
  public int getAxisValue(int axis);
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
axes_number = wb_joystick_get_number_of_axes()
axis_value = wb_joystick_get_axis_value(axis)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/joystick/get_number_of_axes` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |
| `/joystick/axis<X>` | `topic` | webots_ros::Int32Stamped | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`int32 data` |

%tab-end

%end

##### Description

*get number of axes and axis value*

The `wb_joystick_get_number_of_axes` function returns the number of axes of the joystick.

The `wb_joystick_get_axis_value` function returns the current value of the axis passed as an argument.

---

#### `wb_joystick_get_number_of_povs`
#### `wb_joystick_get_pov_value`

%tab-component "language"

%tab "C"

```c
#include <webots/joystick.h>

int  wb_joystick_get_number_of_povs();
int  wb_joystick_get_pov_value(int pov);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Joystick.hpp>

namespace webots {
  class Joystick {
    int getNumberOfPovs() const;
    int getPovValue(int pov) const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Joystick

class Joystick:
    def getNumberOfPovs(self):
    def getPovValue(self, pov):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Joystick;

public class Joystick {
  public int getNumberOfPovs();
  public int getPovValue(int pov);
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
povs_number = wb_joystick_get_number_of_povs()
pov_value = wb_joystick_get_pov_value(pov)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/joystick/pov<X>` | `topic` | `webots_ros::Int32Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`int32 data` |
| `/joystick/get_number_of_povs` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*get number of povs and pov value*

The `wb_joystick_get_number_of_povs` function returns the number of point of views (POV) of the joystick.

The `wb_joystick_get_pov_value` function returns the current value of the point of views (POV) passed as an argument.

---

#### `wb_joystick_get_pressed_button`

%tab-component "language"

%tab "C"

```c
#include <webots/joystick.h>

int wb_joystick_get_pressed_button();
```

%tab-end

%tab "C++"

```cpp
#include <webots/Joystick.hpp>

namespace webots {
  class Joystick {
    int getPressedButton() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Joystick

class Joystick:
    def getPressedButton(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Joystick;

public class Joystick {
  public int getPressedButton();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
button = wb_joystick_get_pressed_button()
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/joystick/pressed_button` | `topic` | webots_ros::Int32Stamped | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`int32 data` |

%tab-end

%end

##### Description

*get the buttons pressed on the joystick*

This function allows you to read a button pressed on the joystick paired with this controller (if any).
The Webots window must be selected and the simulation must be running.
All the buttons pressed can be read by calling the `wb_joystick_get_key` function repeatedly until this function returns -1.
The returned value, if non-negative, is a button code corresponding to a button currently pressed.
If no button is currently pressed, the function will return -1.
Calling the `wb_joystick_get_key` function a second time will return either -1 or the button code of another button which is currently simultaneously pressed.
On macOS, only the first 12 buttons and first 2 axes of the joystick are taken into account.

---

#### `wb_joystick_set_constant_force`
#### `wb_joystick_set_constant_force_duration`
#### `wb_joystick_set_auto_centering_gain`
#### `wb_joystick_set_resistance_gain`
#### `wb_joystick_set_force_axis`

%tab-component "language"

%tab "C"

```c
#include <webots/joystick.h>

void wb_joystick_set_constant_force(int level);
void wb_joystick_set_constant_force_duration(double duration);
void wb_joystick_set_auto_centering_gain(double gain);
void wb_joystick_set_resistance_gain(double gain);
void wb_joystick_set_force_axis(int axis);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Joystick.hpp>

namespace webots {
  class Joystick {
    void setConstantForce(int level);
    void setConstantForceDuration(double duration);
    void setAutoCenteringGain(double gain);
    void setResistanceGain(double gain);
    void setForceAxis(int axis);
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Joystick

class Joystick:
    def setConstantForce(self, level):
    def setConstantForceDuration(self, duration):
    def setAutoCenteringGain(self, gain):
    def setResistanceGain(self, gain):
    def setForceAxis(self, axis):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Joystick;

public class Joystick {
  public void setConstantForce(int level);
  public void setConstantForceDuration(double duration);
  public void setAutoCenteringGain(double gain);
  public void setResistanceGain(double gain);
  public void setForceAxis(int axis);
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_joystick_set_constant_force(level)
wb_joystick_set_constant_force_duration(duration)
wb_joystick_set_auto_centering_gain(gain)
wb_joystick_set_resistance_gain(gain)
wb_joystick_set_force_axis(axis)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/joystick/set_constant_force` | `service` | [`webots_ros::set_int`](ros-api.md#common-services) | |
| `/joystick/set_constant_force_duration` | `service` | [`webots_ros::set_float`](ros-api.md#common-services) | |
| `/joystick/set_auto_centering_gain` | `service` | [`webots_ros::set_float`](ros-api.md#common-services) | |
| `/joystick/set_resistance_gain` | `service` | [`webots_ros::set_float`](ros-api.md#common-services) | |
| `/joystick/set_force_axis` | `service` | [`webots_ros::set_int`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*set the force feedback parameters*

The `wb_joystick_set_constant_force` function uses the joystick force feedback to add a constant force on an axis.
The joystick must support force feedback and the unit of `level` is hardware specific.

The `wb_joystick_set_constant_force_duration` function sets for how long (in seconds) a force added with the `wb_joystick_set_constant_force` function should be applied.
After this duration if no other call to `wb_joystick_set_constant_force` was done, the constant force is stopped.
This is particularly useful in case the simulation is paused to make sure the force stops too.
By default the duration is 1 second.

The `wb_joystick_set_auto_centering_gain` function sets the auto-centering gain of the force feedback.
Auto-centering is an effect that tend to align the axis with the zero position.
The joystick must support force feedback and the unit of `gain` is hardware specific.

The `wb_joystick_set_resistance_gain` function sets the resistance gain of the force feedback.
Resistance is an effect that tend to prevent the axis from moving.
The joystick must support force feedback and the unit of `gain` is hardware specific.

The `wb_joystick_set_force_axis` function defines which axis is associated to the force feedback. This is useful for the auto centering (where the force is proportional to the difference between the axis position and it's center) and for the resistance (where the force is proportional to the axis velocity).

> **Note**:
The units of the force feedback (both the level and gain) are hardware specific, it is therefore recommended to try first with a small value in order to avoid instabilities.
