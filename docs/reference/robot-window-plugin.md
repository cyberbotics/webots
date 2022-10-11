## Robot Window Plugin

This section describes utility functions for building a robot window plugin.

A robot window allows the programmer to efficiently create a custom user interface for a robot.
A robot window is an HTML application that communicates with the robot controller.
It is possible to build robot window plugins where the code is provided as an independent plugin instead of including it in the robot controller directly, and use the same robot window for any robot without modifying its controller.
The functions described in this section can be used in the robot window plugin code to automatically send the devices measurements to the HTML robot window.

Some instructions about how to build HTML robot windows are given in the [User Guide](../guide/controller-plugin.md#robot-window).
An example of robot window plugin is given in [`plugins/robot_windows/custom_robot_window/custom_robot_window.c`]({{ url.github_tree }}/projects/samples/howto/custom_robot_window/plugins/robot_windows/custom_robot_window/custom_robot_window.c).

### Robot Window Setup

You can set the robot window plugin by selecting the [Robot.window](robot.md) field, and hit the `Select...` button.
A dialog pops-up and lets you choose one of the robot windows available in the current project.
Choose a plugin in the dialog and the save the ".wbt" file.
For more information about the search paths for the robot window plugins please refer to the [Robot.window](robot.md) documentation.

### Default Robot Window Functions

#### `wbu_default_robot_window_configure`
#### `wbu_default_robot_window_update`
#### `wbu_default_robot_window_set_images_max_size`

```c
#include <webots/plugins/robot_window/default.h>

char *wbu_default_robot_window_configure();
char *wbu_default_robot_window_update();
char *wbu_default_robot_window_set_images_max_size(int max_width, int max_height);
```

##### Description

*send devices configure and update messages to the HTML robot window*

These functions sends messages containing the robot's devices latest measurements that can be easily passed to the HTML robot window.

The `wbu_default_robot_window_configure` function sends to the HTML robot window a configuration JSON message, prefixed with the "configure" label, containing all the setup information about the robot and its devices.
Here is the sample structure of the configure message:
```
configure {
  "type": <"Robot"|"Supervisor">,
  "name": <string>,
  "model": <string>,
  "basicTimeStep": <double>,
  "devices": [
    {
      "type": <string>,
      "name": <string>,
      "model": <string>
    }
  ]
}
```
Additionally, for each device type, specific information is provided.
* [Camera](camera.md):
    ```
  {
    "width": <double>,
    "height": <double>,
    "recognition": <0|1>,
    "segmentation": <0|1>
  }```
* [DistanceSensor](distancesensor.md):
    ```
  {
    "sensorType": <"generic"|"infra-red"|"laser"|"sonar"|"unknown">,
    "minValue": <double>,
    "maxValue": <double>,
    "aperture": <double>
  }```
* [Lidar](lidar.md):
    ```
  {
    "width": <double>,
    "height": <double>
  }```
* [Motor](motor.md):
    ```
  {
    "minPosition": <double>,
    "maxPosition": <double>,
    "maxVelocity": <double>,
    <"maxTorque"|"maxForce">: <double>
  }```
* [Radar](radar.md):
    ```
  {
    "fieldOfView": <double>,
    "minRange": <double>,
    "maxRange": <double>
  }```
* [RangeFinder](rangefinder.md):
    ```
  {
    "width": <double>,
    "height": <double>
  }```
* [TouchSensor](touchsensor.md):
    ```
  {
    "sensorType": <"bumper"|"force"|"force-3d">
  }```

The `wbu_default_robot_window_update` function sends to the HTML robot window an update JSON message, prefixed with the "update" label, containing the current devices measurements.
Here is the sample structure of the update message:
```
update {
  "time": <double>,
  "devices": [
    "<device_name>": <device update data>
  ]
}
```
Devices update data depends on the type:
* [Accelerometer](accelerometer.md), [Altimeter](altimeter.md), [Compass](compass.md), [DistanceSensor](distancesensor.md), [GPS](gps.md), [Gyro](gyro.md), [InertialUnit](inertialunit.md), [LightSensor](lightsensor.md), [Motor](motor.md), [PositionSensor](positionsensor.md), [TouchSensor](touchsensor.md):
    ```
  {
    "update": [
      {
        "time": <double>,
        "value": <double|[<double>]>
      }
    ]
  }```
* [Camera](camera.md):
    ```
  {
    "recognitionEnabled": <"true"|"false">,
    "segmentationEnabled": <"true"|"false">,
    "image": "data:image/jpg;base64," + <data>
  }```
* [Lidar](lidar.md):
    ```
  {
    "cloudPointEnabled": <"true"|"false">,
    "image": "data:image/jpg;base64,<data>"
  }```
* [Radar](radar.md):
    ```
  {
    "targets": [
      "distance": <double>,
      "azimuth": <double>
    ]
  }```
* [RangeFinder](rangefinder.md):
    ```
  {
    "image": "data:image/jpg;base64," + <data>
 }```

The `wbu_default_robot_window_set_images_max_size` function sets the maximum size for the images of rendering devices sent from the controller to the robot window.
If the size of an image is bigger than the maximum size, then the image is scaled down before it is appended to the update message.
