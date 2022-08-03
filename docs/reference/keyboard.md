## Keyboard

The [Keyboard](#keyboard) is not a node, it is a set of functions available by default for each [Robot](robot.md) node to read the keyboard input.
It is therefore not a device and the functions do not require any `WbDeviceTag`.

> **Note** [Python]: In C++, Python and Java the keyboard functions are in a dedicated class called `Keyboard`.
In order to get the `Keyboard` instance, you should call the `getKeyboard` function of the `Robot` class.

### Keyboard Functions

#### `wb_keyboard_enable`
#### `wb_keyboard_disable`
#### `wb_keyboard_get_sampling_period`
#### `wb_keyboard_get_key`

%tab-component "language"

%tab "C"

```c
#include <webots/keyboard.h>

enum {
  WB_KEYBOARD_END,
  WB_KEYBOARD_HOME,
  WB_KEYBOARD_LEFT,
  WB_KEYBOARD_UP,
  WB_KEYBOARD_RIGHT,
  WB_KEYBOARD_DOWN,
  WB_KEYBOARD_PAGEUP,
  WB_KEYBOARD_PAGEDOWN,
  WB_KEYBOARD_NUMPAD_HOME,
  WB_KEYBOARD_NUMPAD_LEFT,
  WB_KEYBOARD_NUMPAD_UP,
  WB_KEYBOARD_NUMPAD_RIGHT,
  WB_KEYBOARD_NUMPAD_DOWN,
  WB_KEYBOARD_NUMPAD_END,
  WB_KEYBOARD_KEY,
  WB_KEYBOARD_SHIFT,
  WB_KEYBOARD_CONTROL,
  WB_KEYBOARD_ALT
};

void wb_keyboard_enable(int sampling_period);
void wb_keyboard_disable();
int wb_keyboard_get_sampling_period();
int wb_keyboard_get_key();
```

%tab-end

%tab "C++"

```cpp
#include <webots/Keyboard.hpp>

namespace webots {
  class Keyboard {
    enum {
      END, HOME, LEFT, UP, RIGHT, DOWN,
      PAGEUP, PAGEDOWN, NUMPAD_HOME, NUMPAD_LEFT,
      NUMPAD_UP, NUMPAD_RIGHT, NUMPAD_DOWN, NUMPAD_END,
      KEY, SHIFT, CONTROL, ALT
    };

    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    int getKey() const;
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Keyboard

class Keyboard:
    END, HOME, LEFT, UP, RIGHT, DOWN, PAGEUP,
    PAGEDOWN, NUMPAD_HOME, NUMPAD_LEFT, NUMPAD_UP,
    NUMPAD_RIGHT, NUMPAD_DOWN, NUMPAD_END, KEY, SHIFT,
    CONTROL, ALT

    def enable(self, samplingPeriod):
    def disable(self):
    def getSamplingPeriod(self):
    def getKey(self):
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Keyboard;

public class Keyboard {
  public final static int END, HOME, LEFT, UP, RIGHT,
    DOWN, PAGEUP, PAGEDOWN, NUMPAD_HOME, NUMPAD_LEFT,
    NUMPAD_UP, NUMPAD_RIGHT, NUMPAD_DOWN, NUMPAD_END,
    KEY, SHIFT, CONTROL, ALT;

  public void enable(int samplingPeriod);
  public void disable();
  public int getSamplingPeriod();
  public int getKey();
}
```

%tab-end

%tab "MATLAB"

```MATLAB
WB_KEYBOARD_END, WB_KEYBOARD_HOME, WB_KEYBOARD_LEFT, WB_KEYBOARD_UP,
WB_KEYBOARD_RIGHT, WB_KEYBOARD_DOWN, WB_KEYBOARD_PAGEUP, WB_KEYBOARD_PAGEDOWN,
WB_KEYBOARD_NUMPAD_HOME, WB_KEYBOARD_NUMPAD_LEFT, WB_KEYBOARD_NUMPAD_UP,
WB_KEYBOARD_NUMPAD_RIGHT, WB_KEYBOARD_NUMPAD_DOWN, WB_KEYBOARD_NUMPAD_END,
WB_KEYBOARD_KEY, WB_KEYBOARD_SHIFT, WB_KEYBOARD_CONTROL, WB_KEYBOARD_ALT

wb_keyboard_enable(sampling_period)
wb_keyboard_disable()
period = wb_keyboard_get_sampling_period()
key = wb_keyboard_get_key()
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/keyboard/key` | `topic` | webots_ros::Int32Stamped | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`int32 data` |
| `/keyboard/enable` | `service` | [`webots_ros::set_int`](ros-api.md#common-services) | |
| `/keyboard/get_sampling_period` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) | |

%tab-end

%end

##### Description

*keyboard reading function*

These functions allow a controller program to read keys pressed on the computer keyboard when the 3D window of Webots is selected and the simulation is running.
You may have to click inside the 3D window, so that it gets selected (e.g., active) and the key press events can be sent to the controller program.
First, it is necessary to enable keyboard input by calling the `wb_keyboard_enable` function.
The `sampling_period` parameter is expressed in milliseconds, and defines how frequently readings are updated.
Note that the first key will be available only after the first sampling period elapsed.
After that, values can be read by calling the `wb_keyboard_get_key` function repeatedly until this function returns -1.
The returned value, if non-negative, is a key code corresponding to a key currently pressed.
If no modifier (shift, control or alt) key is pressed, the key code is the ASCII code of the corresponding key or a special value (e.g., for the arrow keys).
However, if a modifier key was pressed, the ASCII code (or special value) can be obtained by applying a binary AND between to the `WB_KEYBOARD_KEY` mask and the returned value.
In this case, the returned value is the result of a binary OR between one of `WB_KEYBOARD_SHIFT`, `WB_KEYBOARD_CONTROL` or `WB_KEYBOARD_ALT` and the ASCII code (or the special value) of the pressed key according to which modifier key was pressed simultaneously.
If no key is currently pressed, the function will return -1.
Calling the `wb_keyboard_get_key` function a second time will return either -1 or the key code of another key which is currently simultaneously pressed.
The function can be called up to 7 times to detect up to 7 simultaneous keys pressed.
The `wb_keyboard_disable` function should be used to stop the keyboard readings.

> **Note** [C++]: The keyboard predefined values are located into a (static) enumeration of the Keyboard class.
For example, `Keyboard.CONTROL` corresponds to the <kbd>ctrl</kbd> key stroke.

<!-- -->

> **Note** [Java]: The keyboard predefined values are final integers located in the Keyboard class.
For example, <kbd>ctrl</kbd>-<kbd>B</kbd> can be tested like this:

> ```java
> int key=keyboard.getKey()
> if (key==Keyboard.CONTROL+'B')
>   System.out.Println("Ctrl+B is pressed");
> ```

<!-- -->

> **Note** [Python]: The keyboard predefined values are integers located into the Keyboard class.
For example, <kbd>ctrl</kbd>-<kbd>B</kbd> can be tested like this:

> ```python
> key=keyboard.getKey()
> if (key==Keyboard.CONTROL+ord('B')):
>   print('Ctrl+B is pressed')
> ```
