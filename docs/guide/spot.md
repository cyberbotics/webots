## Boston Dynamics' Spot

%robot spot images/robots/spot/model.thumbnail.png

The "Spot" robot, previously called "SpotMini", is is a four-legged dog-like robot developed by [Boston Dynamics](https://www.bostondynamics.com/spot).

The nimble robot is 83 centimeters tall and is designed for a variety of search, inspection and delivery tasks.
It can climb stairs and traverse rough terrain with unprecedented ease, yet it is small enough to use indoors.
It is built to be a rugged (IP54 for dust and wet protection) and customizable platform.
Spot can go where wheeled robots cannot, while carrying payloads with endurance far beyond aerial drones.
The maximal speed is 1.6 m/s with a runtime of 90 minutes and the batteries are swappable.
Spot uses five stereo cameras (360 degrees vision) to avoid obstacles and people as it moves through dynamic work sites.

### Movie Presentation

![youtube video](https://www.youtube.com/watch?v=b5mVe6dk0wI)

### Spot PROTO

Derived from [Robot](../reference/robot.md).

```
Spot {
  SFVec3f     translation      0 -1.028 0.46
  SFRotation  rotation         0 1 0 0
  SFString    name             "Spot"
  SFString    model            "Boston Dynamics - Spot"
  SFString    controller       "spot_moving_demo"
  MFString    controllerArgs   []
  SFString    customData       ""
  SFBool      supervisor       FALSE
  SFBool      synchronization  TRUE
  SFBool      selfCollision    FALSE
  MFNode      extensionSlot    []
}
```

> **File location**: "[WEBOTS\_HOME/projects/robots/boston\_dynamics/spot/protos/Spot.proto](https://github.com/cyberbotics/webots/tree/master/projects/robots/boston_dynamics/spot/protos/Spot.proto)"

#### Spot Field Summary

- `extensionSlot`: Extends the robot with new nodes.

### Samples

You will find the following sample in the folder: "[$WEBOTS\_HOME/projects/robots/boston\_dynamics/spot/worlds](https://github.com/cyberbotics/webots/tree/master/projects/robots/boston_dynamics/spot/worlds)"

### [spot.wbt](https://github.com/cyberbotics/webots/tree/master/projects/robots/boston_dynamics/spot/worlds/spot.wbt)

![spot.wbt.png](images/robots/spot/spot.wbt.thumbnail.jpg) This simulation shows a Spot robot in a simple environment.
The robot is saying hello with its right leg.
