The [clearpath's Heron USV](https://robots.ros.org/clearpath-heron-usv/) is a USV platform.

### Reliability: 
  Heron is known for engineering quality and durability based on Clearpath Robotics' years of experience. This allows users to perform their tasks without worrying about reliability.
### Flexibility: 
  Heron's modular design allows it to be configured according to users' needs and task requirements. This facilitates the integration of different sensors, equipment and loads, making it suitable for a variety of applications.
### Autonomous Capabilities: 
  Heron has autonomous navigation capabilities that can perform complex tasks. This ensures it can travel designated routes and complete tasks without operator intervention.
### High Performance: 
  Heron is a high-performance platform optimized for long-range and long-duration missions. This provides the ability to cover large areas, collect data, and be continuously active throughout the mission.

### Movie Presentation

![youtube video](https://www.youtube.com/watch?v=qWRyCnJWVuM)

### Heron PROTO

Derived from [Robot](https://cyberbotics.com/doc/reference/robot).

```
Heron {
  SFVec3f    translation     0 0 0
  SFRotation rotation        0 1 0 1.5708
  SFString   name            "heron"
  SFString   controller      "heron_usv_controller"
  MFString   controllerArgs  []
  SFBool     synchronization TRUE
  SFColor    color           0.8 0.5 0.1
  MFNode     bodySlot        []
}
```

#### Heron Field Summary

- `bodySlot`: Extends the robot with new nodes.

### Samples

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/clearpath/heron/worlds]({{ url.github_tree }}/projects/robots/clearpath/heron/worlds)".

#### [heron\ocean.wbt]({{ url.github_tree }}/projects/robots/clearpath/moose/worlds/heron\ocean.wbt)

![swarm.png](images/heron/swarm.jpg) This simulation shows a basicly swarm robotics.
