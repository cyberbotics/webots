## ROS API

### Common Services

The following table describes the ROS services shared between the Webots devices.

| `service` name | `service` definition |
| --- | --- |
| `get_bool` | `bool ask`<br/>`---`<br/>`bool value` |
| `get_float` | `bool ask`<br/>`---`<br/>`float64 value` |
| `get_float_array` | `bool ask`<br/>`---`<br/>`float64[] values` |
| `get_int` | `bool ask`<br/>`---`<br/>`int32 value` |
| `get_string` | `bool ask`<br/>`---`<br/>`string value` |
| `get_uint64` | `bool ask`<br/>`---`<br/>`uint64 value` |
| `set_bool` | `bool value`<br/>`---`<br/>`bool success` |
| `set_float` | `float64 value`<br/>`---`<br/>`bool success` |
| `set_float_array` | `float64[] values`<br/>`---`<br/>`bool success` |
| `set_int` | `int32 value`<br/>`---`<br/>`bool success` |
| `set_string` | `string value`<br/>`---`<br/>`bool success` |

A complete list of all the available ROS services and messages can be found [here](http://docs.ros.org/kinetic/api/webots_ros/html/index-msg.html).
