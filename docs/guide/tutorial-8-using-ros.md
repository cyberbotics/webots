## Tutorial 8: Using ROS

This tutorial explains how to use the nodes from the `webots_ros` package provided with Webots.

These examples were tested with ROS `melodic` and `kinetic` on Linux.
There is no warranty they will work if you use a different platform or an ancient distribution of ROS.

### Installing ROS

In order to use these nodes, you will first need to install the ROS framework.
To install the latest version of ROS on Ubuntu use the following commands:

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full
sudo apt-get install ros-melodic-sensor-msgs
sudo rosdep init
rosdep update
```

For more information or to install it on another platform please read [http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation).
Unless you need older version for some other application, you should choose the latest distribution (Melodic Morenia or Kinetic Kame).

If you never used the ROS framework before, it is strongly recommended to follow some tutorials from: [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials).
These tutorials will also help you set up your ROS environment and initialize your catkin workspace.

### "webots\_ros" Package Installation

You can install the [webots_ros](http://wiki.ros.org/webots\_ros) package with the following command:
```sh
sudo apt-get install ros-melodic-webots-ros
```

### Running the Nodes

You can start any simulation using ROS with the wollowing commands (here the `e_puck_line` one for example):

```sh
source /opt/ros/melodic/setup.bash
roslaunch webots_ros catch_the_bird
```
This launch file will launch Webots and start the corresponding node.

The seed of Webots' random number generator is initialized at the beginning of the simulation and not when the ROS nodes connect.
Webots has to be running for the ROS nodes to connect.
However, we cannot guarantee how long it will run before the ROS nodes connect.
Therefore, the sensor measurements and motor commands will slightly differ from one run to another, due to the noise being slightly different at the time of the connection of the ROS nodes.
This may have consequences on the behavior of the robots, thus making such simulations non fully reproducible.
You can use the '--synchronize' argument in order to make sure that Webots will not run before the ROS node connects.
This is useful to make ROS-based simulation reproducible.

If you want to use different computers for the ROS master, the Webots simulation and/or the nodes, you must be able to connect to each of them with SSH in both ways.
The hostname and IP addresses of these computers should be listed in the known hosts list of each computer and the `ROS_MASTER_URI` variable should be adjusted accordingly.

### Creating New Nodes

These examples only show a few possibilities for interfacing ROS and Webots, but you can build your own nodes to connect with Webots.
the source code of these nodes can be found on the repository of the [webots_ros package](https://github.com/omichel/webots\_ros).

The `robot_information_parser` node is the most basic one and is a good base to start building your own node.
The `complete_test` node doesn't show any particular application but contains an almost exhaustive list of the Webots API functions.

All the functions from the Webots API have their corresponding services or topics.
You can find in the [Reference Manual](../reference/nodes-and-api-functions.md) the definitions of all the services and topics associated to each device.
