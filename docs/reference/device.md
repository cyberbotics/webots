## Device

Abstract node.

```
Device {
}
```

### Description

This abstract node (that cannot be instantiated) represents a robot device (actuator and/or sensor).

### Device Functions

#### `wb_device_get_model`

%tab-component "language"

%tab "C"

```c
#include <webots/device.h>

const char *wb_device_get_model(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Device.hpp>

namespace webots {
  class Device {
    std::string getModel() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Device

class Device:
    def getModel(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Device;

public class Device {
  public String getModel();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
model = wb_device_get_model(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/get_model` | `service` | [`webots_ros::get_string`](ros-api.md#common-services) |

%tab-end

%end

##### Description

*returns the model string of the corresponding device*

The `wb_device_get_model` function returns the model string of the device corresponding to the WbDeviceTag given as parameter (`tag`).

This function returns NULL if the WbDeviceTag does not match a valid device, or returns an empty string if the device is not a solid device (i.e. does not have a `model` field).

> **Note** [C, C++]: The returned string is a pointer to the internal values managed by the [Device](#device) node, therefore it is illegal to free this pointer.

---

#### `wb_device_get_name`

%tab-component "language"

%tab "C"

```c
#include <webots/device.h>

const char *wb_device_get_name(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Device.hpp>

namespace webots {
  class Device {
    const std::string &getName() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Device

class Device:
    def getName(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Device;

public class Device {
  public String getName();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
name = wb_device_get_name(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/get_name` | `service` | [`webots_ros::get_string`](ros-api.md#common-services) |

%tab-end

%end

##### Description

*convert WbDeviceTag to its corresponding device name*

The `wb_device_get_name` function converts the WbDeviceTag given as parameter (`tag`) to its corresponding name.

This function returns NULL if the WbDeviceTag does not match a valid device.

> **Note** [C, C++]: The returned string is a pointer to the internal values managed by the [Device](#device) node, therefore it is illegal to free this pointer.

---

#### `wb_device_get_node_type`

%tab-component "language"

%tab "C"

```c
#include <webots/device.h>

WbNodeType wb_device_get_node_type(WbDeviceTag tag);
```

%tab-end

%tab "C++"

```cpp
#include <webots/Device.hpp>

namespace webots {
  class Device {
    int getNodeType() const;
    // ...
  }
}
```

%tab-end

%tab "Python"

```python
from controller import Device

class Device:
    def getNodeType(self):
    # ...
```

%tab-end

%tab "Java"

```java
import com.cyberbotics.webots.controller.Device;

public class Device {
  public int getNodeType();
  // ...
}
```

%tab-end

%tab "MATLAB"

```MATLAB
type = wb_device_get_node_type(tag)
```

%tab-end

%tab "ROS"

| name | service/topic | data type | data type definition |
| --- | --- | --- | --- |
| `/<device_name>/get_node_type` | `service` | [`webots_ros::get_int`](ros-api.md#common-services) |

%tab-end

%end

##### Description

*convert WbDeviceTag to its corresponding WbNodeType*

The `wb_device_get_node_type` function converts the WbDeviceTag given as parameter (`tag`) to its corresponding WbNodeType (cf. the [Supervisor API](supervisor.md)).

This function returns NULL if the WbDeviceTag does not match a valid device.
