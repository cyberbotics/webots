## Festo's Robotino 3

%robot robotino3 images/robots/robotino3/model.thumbnail.png

Designed by Festo, Robotino 3 is a three-wheeled (mecanum) robot.
The model is a modular mobile platform used for educational, training and research purposes.
It is controlled by an industry-standard PC system and it is also customizable with accessories to adapt to any specific need.

More information on the Robotino 3 robot can be found on their [website](https://ip.festo-didactic.com/InfoPortal/Robotino3/Overview/EN/index.html).

### Robotino3 PROTO

Derived from [Robot](../reference/robot.md).

```
Robotino3 {
  SFVec3f     translation            0 0 0
  SFRotation  rotation               1 0 0 -1.5708
  SFString    name                   "Robotino 3"
  SFString    controller             "robotino3"
  SFString    controllerArgs         ""
  SFString    customData             ""
  SFBool      supervisor             FALSE
  SFBool      synchronization        TRUE
  SFBool      selfCollision          FALSE   # Enables/disables the detection of collisions within the robot.
  SFString    model                  "Festo - Robotino 3"
  MFNode      bodyExtension          []
  MFNode      cameraExtension        []
  MFNode      objectExtension        []
  MFNode      lidarTopExtension      []
  MFNode      lidarMiddleExtension   []
  MFNode      lidarBottomExtension   []
}
```

> **File location**: "[WEBOTS\_HOME/projects/robots/festo/robotino3/protos/Robotino3.proto](https://github.com/cyberbotics/webots/tree/master/projects/robots/festo/robotino3/protos/Robotino3.proto)"

#### Robotino 3 Field Summary

- `bodyExtension` : Extends the robot with new nodes (such as the `Robotino3Platform` for example).
- `cameraExtension` : Extends the robot with a camera (such as the `Robotino3Webcam` for example).
- `objectExtension` : Extends the robot with an extern object (such as the `Robotino3ExtObject` for example).
- `lidarTopExtension`: Extends the robot with a lidar at the robot's top.
- `lidarMiddleExtension`: Extends the robot with a lidar at the robot's middle.
- `lidarBottomExtension`: Extends the robot with a lidar at the robot's bottom.

### Sample

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/festo/robotino3/worlds](https://github.com/cyberbotics/webots/tree/master/projects/robots/festo/robotino3/worlds)".

#### robotino3.wbt

![robotino3.wbt.png](images/robots/robotino3/robotino3.wbt.thumbnail.jpg) This simulation shows a Robotino 3 moving in an industrial environment using a Braitenberg algorithm using the information received by its nine distance sensors.
