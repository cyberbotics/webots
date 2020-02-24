## Usage

Once you have started a simulation with a Universal Robot and set its controller to `universal_robots_ros`, you can use the following launch file to setup all the required ROS parameters:

```
roslaunch ur_e_webots ur5e_joint_limited.launch
```

You can then control the robot with MoveIt!, use the following launch file (from the `universal_robot` ROS package) to start MoveIt!:

```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run use the following launch file (from the `universal_robot` ROS package):

```
roslaunch ur5_moveit_config moveit_rviz.lnch config:=true
```

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
