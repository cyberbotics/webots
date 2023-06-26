## Using ROS

### What is ROS?

[ROS (Robot Operating System)](http://www.ros.org/) is a framework for robot software development, providing operating system-like functionality on top of a heterogeneous computer cluster.
ROS was originally developed in 2007 by the Stanford Artificial Intelligence Laboratory.
As of 2008, development continues primarily at [Willow Garage](http://www.willowgarage.com/).

ROS provides standard operating system services such as hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.
It is based on a graph architecture where processing takes place in nodes that may receive, post and multiplex sensor, control, state, planning, actuator and other messages.
The library is geared towards a Unix-like system and is supported under Linux, experimental on macOS and has partial functionality under Windows.

ROS has two basic "sides": The operating system side, `ros`, as described above and ROS packages, a suite of user contributed packages (organized into sets called stacks) that implement functionalities such as simultaneous localization and mapping, planning, perception, simulation etc.

ROS is released under the terms of the BSD license, and is an open source software.
It is free for commercial and research use.
The user contributed packages are licensed under a variety of open source licenses.

### ROS for Webots

There are two ways to use ROS with Webots.

The first solution and the easiest one is to use the **standard ROS controller**.
This solution however doesn't work on Windows and macOS, it works only on Linux.
It is part of the Webots default controllers and is available in any project.
This controller can be used on any robot in Webots and acts as a ROS node, providing all the Webots functions as services or topics to other ROS nodes.

The second solution named **custom ROS controller** requires that you build your own Webots controller that will also be a ROS node using Webots and ROS libraries.
This solution works on Windows and macOS (in Python) in addition to Linux.
It is a bit more difficult to set up but allows for more flexibility.

### Standard ROS Controller

This controller uses the "libCppController" library and proposes the available Webots functionalities on the ROS network according to the robot's configuration.
Using the "roscpp" library, it provides these Webots functions mostly as ROS services and uses standard messages type to avoid dependencies on third-party packages.
The list of services and messages can be found [here](http://docs.ros.org/noetic/api/webots_ros/html/index-msg.html).

During simulation there can be multiple instances of robots or devices and other Webots applications connected to the ROS network.
Therefore the controller uses a specific syntax to declare its services or topics on the network: `[robot_name_space]/[device_name]/[service/topic_name]`.

`[robot_name_space]`: ROS namespace to group services, topics and parameters.

`[device_name]`: since the same function can refer to different devices, this field shows you which device it refers to.

`[service/topic_name]`: this field is identical or very close to the Webots function it corresponds to.
For topics, it is followed by the sampling period.
For services, it is also the name of the corresponding srv file.

> **Note**: On Windows the standard ROS controller is not available, use the [custom ROS controller](#custom-ros-controller) instead.

#### Using the Standard ROS Controller

The controller, named `ros`, is pre-compiled and you shouldn't edit it.
All you have to do is to load it in the `controller` field of your robot; you will find it in the default list of controller.
In order to use it, you will have to build a ROS node that will communicate with the robot using the different services available.
Good examples of such ROS node can be found in the [webots\_ros repository](https://github.com/cyberbotics/webots_ros), they are documented in the [webots\_ros package tutorial](http://wiki.ros.org/webots_ros/Tutorials/Sample%20Simulations).

In the [Tutorial 9](tutorial-9-using-ros.md) chapter, you will find the instructions to run a sample simulation using ROS.

In the following table you can find the list of `ros` controller arguments.

%figure "ros controller arguments"

| Argument | Description |
| -------- | ----------- |
| `--ROS_MASTER_URI=<address>` | Specify the URI address of the machine where `roscore` is running. |
| `--name=<robot_unique_name>` | Specify a predefined [robot\_unique\_name] to be used as namespace for services and topics. Note that you are then responsible for avoiding any name clashes between the different robot controllers. |
| `--synchronize`   | By default the `ros` controller is not blocking the simulation even if no ROS node is connected to it. In order to synchronize the simulation with the ROS node, the `--synchronize` argument can be specified, so that the simulation will not run as long as the robot `time_step` service is not called. |
| `--clock`   | Publish the Webots time using the `clock` topic. |
| `--use-sim-time` | Specify that the Webots time should be used as ROS time. To work correctly you should also define the `--clock` argument and set the ROS parameter `use_sim_time` to true. |
| `--auto-publish` | Force the controller to automatically enable all devices on startup and create the corresponding topics. |
| `--use-ros-control` | Initialize the `controller_manager` from the [`ros_control`](http://wiki.ros.org/ros_control). |
| `--robot-description[={robot_description_prefix}]` | Expose the `robot_description` ROS parameter that contains the URDF of the robot. The `robot_description_prefix` parameter is optional and it corresponds to the `prefix` argument of the [`wb_robot_get_urdf`](../reference/robot.md#wb_robot_get_urdf) function. |


%end


> **Note**: By default, the robot's namespace is empty and its name is followed by the ID of the process and the IP address of the computer. Setting `--name` in the controller arguments will set the robot namespace as well as the robot name.

> **Note**: `clock` topic is under global namespace. Therefore in order to avoid any conflicts, only one robot should set `--clock` flag in multirobot simulations.

If you want to access the controller from another machine and `roscore` isn't running on the same machine as Webots, you will need to edit the ROS\_MASTER\_URI variable.
This can be done by editing your environment variables, setting `--ROS_MASTER_URI=<address>` in the controller arguments (see [table](#ros-controller-arguments)) or with a `runtime.ini` file in the controller directory.
You must also be able to connect to each of the computer with  `ssh` in both ways.
As ROS uses the hostname to find other computers on the network, you must add other computers' hostname and the associated IP address to the known hosts of each computer.
You can find this list in a file named *hosts*.
On Linux distribution, you can find it directly at `/etc/hosts`; on macOS, it is located at `/private/etc/hosts`; on Windows, it is located at `C:\Windows\System32\drivers\etc\hosts`.
On Windows and macOS, this is a hidden path and you will need to search directly for this path.
The hosts file is usually protected and you will need administrator or root privileges to edit it.

### Custom ROS Controller

The standard ROS controller has been developed in order to work on every robot and for general purpose.
Sometimes, you may not be able to do what you want with this controller or it would be too complicated.
In this case, you can build your own custom ROS controller.

It is possible to implement such a ROS node in C++ using the "roscpp" library on Linux and macOS.
However, in this case, you need to setup a build configuration to handle both the "catkin\_make" from ROS and the "Makefile" from Webots to have the resulting binary linked both against the Webots "libController" and the "roscpp" library.
An example is provided [here]({{ url.github_tree }}/projects/vehicles/controllers/ros_automobile) to create a specific controller for controlling a vehicle.

An even more generic solution, is to use an [extern controller](running-extern-robot-controllers.md) and run the controller as a regular ROS node on the ROS side.
A very simple example is provided [here](https://github.com/cyberbotics/webots_ros/blob/master/scripts/ros_python.py), it is written in pure Python and should work on Windows, Linux and macOS, straight out of the box.
A launch file is available to launch Webots with the correct world file, the extern controller and a simple ROS node moving the robot straight as long as there is no obstacle (detected using the front [DistanceSensor](../reference/distancesensor.md)), it can be launched with:
```
roslaunch webots_ros webots_ros_python.launch
```

A [second more complicated example]({{ url.github_tree }}/projects/robots/universal_robots/resources/ros_package/ur_e_webots) shows how to interface a model of a Universal Robots arm in Webots with ROS using [rospy](http://wiki.ros.org/rospy).

### Importing from a ROS Package

Webots ROS can use rospack to find controllers, nodes, and PROTOS defined in different ROS packages.
In order for Webots to find them, the following needs to be added to your [package.xml](http://wiki.ros.org/catkin/package.xml):
```
<export>
    <webots_ros webots_extra_project_path="${prefix}"/>
</export>
```
Once the export tag is added, and after building and sourcing your package, launching Webots with `webots.launch` from the [webots\_ros](https://github.com/cyberbotics/webots_ros) package will allow you to use the found controllers/nodes.
