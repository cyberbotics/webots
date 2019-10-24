## LED

Derived from [Device](device.md) and [Solid](solid.md).

```
LED {
  MFColor color   [ 1 0 0 ]   # any color
  SFBool  gradual FALSE       # {TRUE, FALSE}
}
```

### Description

The [LED](#led) node is used to model a light emitting diode (LED).
The light produced by an LED can be used for debugging or informational purposes.
The resulting color is applied only on the first child of the [LED](#led) node.
If the first child is a [Shape](shape.md) node, the `emissiveColor` field of its [Material](material.md) or [PBRAppearance](pbrappearance.md) node is altered.
If the first child is a [Light](light.md) node, its `color` field is altered.
Otherwise, if the first child is a [Group](group.md) node, a recursive search is applied on this node in order to find which color field must be modified, so every [Light](light.md), [Shape](shape.md) and [Group](group.md) node is altered according to the previous rules.
In terms of [Light](light.md) nodes, [LED](#led) nodes can only accept [PointLight](pointlight.md) and [SpotLight](spotlight.md) nodes as children.

Note that [Material](material.md) and [Light](light.md) nodes that are going to be automatically altered by the [LED](#led) functionality cannot be [USE](def-and-use.md) nodes.

### Field Summary

- `color`: This defines the colors of the LED device.
When off, an LED is always black.
However, when on it may have different colors as specified by the LED programming interface.
By default, the `color` defines only one color (red), but you can change this and add extra colors that could be selected from the LED programming interface.
However, the number of colors defined depends on the value of the `gradual` field (see below).

- `gradual`: This defines the type of LED.
If set to FALSE, the LED can take any of the color values defined in the `color` list.
If set to TRUE, then the `color` list should either be empty or contain only one color value.
If the `color` list is empty, then the LED is an RGB LED and can take any color in the R8G8B8 color space (16 million possibilities).
If the `color` list contains a single color, then the LED is monochromatic, and its intensity can be adjusted between 0 (off) and 255 (maximum intensity).

### LED Functions

#### `wb_led_set`
#### `wb_led_get`

%tab-component "language"

%tab "C"

```c
#include <webots/led.h>

void wb_led_set(WbDeviceTag tag, int value);
int wb_led_get(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <webots/LED.hpp>

namespace webots {
  class LED : public Device {
    virtual void set(int value);
    int get() const;
  }
}
```

%tab-end

%tab "Python"

```python
from controller import LED

class LED (Device):
    def set(self, value):
    def get(self):
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.LED;

public class LED extends Device {
  public void set(int value);
  public int get();
}
```

%tab-end

%tab "MATLAB"

```MATLAB
wb_led_set(tag, value)
value = wb_led_get(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/set_led` | `service` | [`webots_ros::set_int`](ros-api.md#common-services) |
| `/<device_name>/get_led` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) |

%tab-end

%end

##### Description

*turn an LED on or off and read its status*

The `wb_led_set` function switches an LED on or off, possibly changing its color.
If the `value` parameter is 0, the LED is turned off.
Otherwise, it is turned on.

In the case of a non-gradual LED (`gradual` field set to FALSE), if the `value` parameter is 1, the LED is turned on using the first color specified in the `color` field of the corresponding [LED](#led) node.
If the `value` parameter is 2, the LED is turned on using the second color specified in the `color` field of the [LED](#led) node, and so on.
The `value` parameter should not be greater than the size of the `color` field of the corresponding [LED](#led) node.

In the case of a monochromatic LED (`gradual` field set to TRUE and `color` field containing exactly one color), the `value` parameter indicates the intensity of the LED in the range 0 (off) to 255 (maximum intensity).

In the case of an RGB LED (`gradual` field set to TRUE and `color` field containing an empty list), the `value` parameter indicates the RGB color of the LED in the range 0 (off or black) to 0xffffff (white).
The format is R8G8B8: The most significant 8 bits (left hand side) indicate the red level (between 0x00 and 0xff).
Bits 8 to 15 indicate the green level and the least significant 8 bits (right hand side) indicate the blue level.
For example, 0xff0000 is red, 0x00ff00 is green, 0x0000ff is blue, 0xffff00 is yellow, etc.

The `wb_led_get` function returns the value given as an argument of the last `wb_led_set` function call.
