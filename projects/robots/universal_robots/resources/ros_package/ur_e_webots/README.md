## Installation

To use the package you need to copy it in your catkin workspace, install the dependencies and compile it:
```shell
cd /path/to/catkin_ws/src
cp -r $WEBOTS_HOME/projects/robots/universal_robots/resources/ros_package/ur_e_webots .
cd ..
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin_make
source devel/setup.bash
```

You need to install the `universal_robots` package too in order to get the URDF definitions of the robots (used for example for RVIZ visualization and MoveIt).
Until Kinetic, you can install it using the package manager (`apt install ros-$ROS_DISTRO-universal-robot`), but from Melodic, you need to compile it from [sources](https://github.com/ros-industrial/universal_robot/tree/melodic-devel).

## Usage

Once you have started a simulation with a UR robot and set its controller to `<extern>`, you can use the following launch file to setup all the required ROS parameters and start the simulated UR robot to ROS interface:

```
roslaunch ur_e_webots ur5e_joint_limited.launch
```

You can then control the robot with MoveIt!, use the following launch file (from the `universal_robot` ROS package) to start MoveIt!:

```
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch
```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin, run the following launch file (from the `universal_robot` ROS package):

```
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```

## Multi Robots

If your simulation uses more than one robot running the `universal_robots_ros` controller, the `---node-name` controller arguments should be set to avoid a name clash between nodes.

## Acknowledgement

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a></br>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.
