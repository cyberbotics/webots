## Mouse

The [Mouse](#mouse) API provides a set of functions available by default for each [Robot](robot.md) node and it is used to read the computer mouse input.
It doesn't have any corresponding Node.

> **Note** [C++, Python, Java]: In C++, Python and Java, the mouse functions are in a dedicated class called `Mouse`.
In order to get the `Mouse` instance, you should call the `Robot.getMouse` function.

### WbMouseState

The state of a mouse is defined like this:

%tab-component "language"

%tab "C"

```c
typedef struct {
  // mouse 2D position in the 3D window
  double u;
  double v;
  // mouse 3D position
  double x;
  double y;
  double z;
  // mouse buttons state
  bool left;
  bool middle;
  bool right;
} WbMouseState;
```
%tab-end

%tab "C++"

```cpp
#include <webots/Mouse.hpp>

namespace webots {
  typedef struct {
    // mouse 2D position in the 3D window
    double u;
    double v;
    // mouse 3D position
    double x;
    double y;
    double z;
    // mouse buttons state
    bool left;
    bool middle;
    bool right;
  } MouseState;
}
```

%tab-end

%tab "Python"

```python
from controller import MouseState

class MouseState:
    @property
    u, v  # mouse 2D position in the 3D window
    @property
    x, y, z  # mouse 3D position
    @property
    left, middle, right  # mouse button state
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.MouseState;

public class MouseState {
  public double getU();
  public double getV();
  public double getX();
  public double getY();
  public double getZ();
  public boolean getLeft();
  public boolean getMiddle();
  public boolean getRight();
}
```

%tab-end

%tab "MATLAB"

```MATLAB
structs.WbMouseState.members = struct(
  'u', 'double',
  'v', 'double',
  'x', 'double',
  'y', 'double',
  'z', 'double',
  'left', 'int8',
  'middle', 'int8',
  'right', 'int8'
);
```

%tab-end

%tab "ROS"

%tab-end

> `MouseState` data is directly accessible from the related [`/mouse/mouse_get_state` service](#wb_mouse_enable).

%end

The `left`, `middle` and `right` fields are matching respectively with the left, middle and right buttons of the computer mouse.
A `true` state means the button is pressed while a `false` state means the button is released.

The `u` and `v` fields are indicating the 2D coordinate where the mouse is pointing in the 3D window.
The `u` coordinate goes from 0 on the left border of the 3D window to 1 on the right border, and the `v` coordinate goes from 0 on the top border to 1 on the bottom border.
These values may be `NaN` if the mouse is outside of the 3D window.

The `x`, `y` and `z` fields are indicating the 3D coordinate where the mouse is pointing in the 3D window.
These values may be `NaN` if not applicable, for example when the mouse is pointing to the scene background or when the 3d position is not enabled (see the [`wb_mouse_enable_3d_position` function](#wb_mouse_enable_3d_position)).

### Mouse Functions

#### `wb_mouse_enable`
#### `wb_mouse_disable`
#### `wb_mouse_get_sampling_period`
#### `wb_mouse_get_state`

%tab-component "language"

%tab "C"

```c
#include <webots/mouse.h>

void wb_mouse_enable(int sampling_period);
void wb_mouse_disable();
int wb_mouse_get_sampling_period();
WbMouseState wb_mouse_get_state();
```

%tab-end

%tab "C++"

```cpp
#include <webots/Mouse.hpp>

namespace webots {
  class Mouse {
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    MouseState getState() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Mouse

class Mouse:
    def enable(self, samplingPeriod):
    def disable(self):
    def getSamplingPeriod(self):
    def getState(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Mouse;

public class Mouse {
  public void enable(int samplingPeriod);
  public void disable();
  public int getSamplingPeriod();
  public MouseState getState();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_mouse_enable(sampling_period)
wb_mouse_disable()
period = wb_mouse_get_sampling_period()
state = wb_mouse_get_state()
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/mouse/enable` | `service` | [`webots_ros::set_int`](ros-api.md#common-services) | |
| `/mouse/get_sampling_period` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |
| `/mouse/mouse_get_state` | `service` | `webots_ros::mouse_get_state` | `uint8 ask`<br/>`---`<br/>`uint8 left`<br/>`uint8 middle`<br/>`uint8 right`<br/>`float64 u`<br/>`float64 v`<br/>`float64 x`<br/>`float64 y`<br/>`float64 z` |

%tab-end

%end

##### Description

*mouse reading function*

The state of the computer mouse can be read from a controller program while the simulation is running by using the above functions.
Firstly it is necessary to enable mouse input by calling the `wb_mouse_enable` function.
The `sampling_period` parameter is expressed in milliseconds, and defines how frequently readings are updated.
Note that the first state will be available only after the first sampling period elapsed.
After that, the state can be read by calling the `wb_mouse_get_state` function (for more details, see [`WbMouseState`](#wbmousestate)).
The `wb_mouse_disable` function should be used to stop the mouse readings.


#### `wb_mouse_enable_3d_position`
#### `wb_mouse_disable_3d_position`
#### `wb_mouse_is_3d_position_enabled`

%tab-component "language"

%tab "C"

```c
#include <webots/mouse.h>

void wb_mouse_enable_3d_position();
void wb_mouse_disable_3d_position();
bool wb_mouse_is_3d_position_enabled();
```

%tab-end

%tab "C++"

```cpp
#include <webots/Mouse.hpp>

namespace webots {
  class Mouse {
    void enable3dPosition();
    void disable3dPosition();
    bool is3dPositionEnabled() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Mouse

class Mouse:
    def enable3dPosition(self):
    def disable3dPosition(self):
    def is3dPositionEnabled(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Mouse;

public class Mouse {
  public void enable3dPosition();
  public void disable3dPosition();
  public boolean is3dPositionEnabled();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_mouse_enable_3d_position()
wb_mouse_disable_3d_position()
enabled = wb_mouse_is_3d_position_enabled()
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/mouse/mouse_enable_3d_position` | `service` | [`webots_ros::set_bool`](ros-api.md#common-services) | |
| `/mouse/mouse_disable_3d_position` | `service` | [`webots_ros::set_bool`](ros-api.md#common-services) | |
| `/mouse/mouse_is_3d_position_enabled` | `service` | [`webots_ros::get_bool`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*enable the 3d position*

In order to retrieve the 3D coordinate where the mouse is pointing in the 3D window, a picking is necessary.
Depending on the complexity of the scene and time step, this picking can considerably reduce the simulation speed.
Therefore, to avoid  unnecessary computation, picking should be explicitly enabled using the `wb_mouse_enable_3d_position` function, otherwise the `x`, `y` and `z` fields of [`WbMouseState`](#wbmousestate) are set to `NaN`.
Once enabled, it is possible to disable it using the `wb_mouse_disable_3d_position` function.
Furthermore, it is possible at any time to check if picking is enabled using the `wb_mouse_is_3d_position_enabled` function.
