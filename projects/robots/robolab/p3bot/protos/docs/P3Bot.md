# P3Bot

Designed by the [Robolab](https://robolab.unex.es/) **research group**, **P3Bot** is a mobile manipulation platform equipped with omnidirectional wheels, multiple perception sensors, and dual articulated robotic arms.  
It is intended for research in mobile manipulation, perception, and human–robot interaction.

The robot features a four-wheel omnidirectional base, a rich sensor suite including 360° vision, stereo vision, lidar, and IMU, as well as two **Kinova Gen3 7DoF** robotic arms mounted on the upper body.

---

## P3Bot PROTO

Derived from [Robot](https://cyberbotics.com/doc/reference/robot).

```proto
P3Bot {
  SFVec3f     translation      0 0 0.03
  SFRotation  rotation         0 0 1 0
  SFString    name             "P3Bot"
  SFString    controller       "<none>"
  MFString    controllerArgs   []
  SFString    customData       ""
  SFBool      supervisor       FALSE
  SFBool      synchronization TRUE
  MFNode      extensionSlot   []
  MFNode      sensors         [ ... ]
  MFNode      wheels          [ ... ]
  MFNode      arms            [ ... ]
}
```

## P3Bot Field Summary

- **`translation`**  
  Defines the initial position of the robot in the world.

- **`rotation`**  
  Defines the initial orientation of the robot.

- **`name`**  
  Defines the name of the robot instance.

- **`controller`**  
  Defines the controller assigned to the robot.

- **`controllerArgs`**  
  Optional arguments passed to the controller.

- **`customData`**  
  Custom user-defined data string.

- **`supervisor`**  
  Enables or disables supervisor capabilities.

- **`synchronization`**  
  Enables controller synchronization with the simulation.

- **`extensionSlot`**  
  Allows extending the robot with additional nodes located at the body center.

- **`sensors`**  
  Defines the robot sensor suite.

- **`wheels`**  
  Defines the omnidirectional wheeled base.

- **`arms`**  
  Defines the articulated robotic arms mounted on the robot.


## Sensors

The **P3Bot** includes a comprehensive set of sensors for perception and navigation:

### 360° Vision System
Two cylindrical cameras providing full panoramic vision around the robot.

### Stereo Camera with Depth Sensing
A forward-facing planar camera equipped with a `RangeFinder`, suitable for stereo vision and depth perception.

### 3D Lidar
A multi-layer lidar sensor mounted on the upper body, providing dense 3D point cloud data for mapping and obstacle detection.

### IMU
An accelerometer sensor providing inertial measurements.

All sensors are grouped under the `sensors` field and can be easily extended or replaced.


## Mobile Base

The mobile base consists of **four omnidirectional wheels**, each independently actuated by a rotational motor and equipped with a position sensor.

This configuration enables:
- Holonomic motion
- Lateral movement
- Precise maneuvering in confined environments

Each wheel includes:
- A `RotationalMotor`
- A `PositionSensor`
- A collision-aware bounding object
- Physical properties for realistic dynamics


## Manipulation

The robot is equipped with **two Kinova Gen3 robotic arms**:

- **Right arm**: Default configuration
- **Left arm**: Mirrored configuration using a dedicated prefix

Both arms are fully articulated and suitable for advanced manipulation and research tasks.


## Sample Worlds

You will find sample simulations in the following folder:

    WEBOTS_HOME/projects/robots/robolab/p3bot/worlds


