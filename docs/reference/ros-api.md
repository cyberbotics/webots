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


### Webots Messages

This section describes the custom ROS message types used by Webots.

| `message` name | `message` definition |
| --- | --- |
| `webots_ros/BoolStamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`bool data` |
| `webots_ros/ContactPoint` | [`geometry_msgs/Point`](http://docs.ros.org/api/geometry_msgs/html/msg/Point.html) `point`<br/>`int32 node_id` |
| `webots_ros/Float64Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64ged data` |
| `webots_ros/Int32Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`int32 data` |
| `webots_ros/Int8Stamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`int8 data` |
| `webots_ros/RadarTarget` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`float64 distance`<br/>`float64 receivedPower`<br/>`float64 speed`<br/>`float64 azimuth` |
| `webots_ros/RecognitionObject` | [`geometry_msgs/Vector3`](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html) `position`<br/>[`geometry_msgs/Quaternion`](http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html) `orientation`<br/>[`geometry_msgs/Vector3`](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html) `position_on_image`<br/>[`geometry_msgs/Vector3`](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html) `size_on_image`<br/>`int32 number_of_colors`<br/>[`geometry_msgs/Vector3`](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html)`[]` `colors`<br/>`string model`|
| `webots_ros/RecognitionObjects` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>webots_ros/RecognitionObject[] objects` |
| `webots_ros/StringStamped` | [`Header`](http://docs.ros.org/api/std_msgs/html/msg/Header.html) `header`<br/>`string data` |
