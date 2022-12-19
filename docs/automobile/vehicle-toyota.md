# Toyota

## ToyotaPrius

Model of a Toyota Prius car using the Car PROTO as a base.
The contactMaterial of the wheels is "ToyotaPriusWheels".
This model was sponsored by the CTI project RO2IVSim ([http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles](http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)).

%figure

![ToyotaPrius](images/toyota/ToyotaPrius.thumbnail.png)

%end

Derived from [Robot](../reference/robot.md).

```
ToyotaPrius {
  SFVec3f    translation       0 0 0.4
  SFRotation rotation          0 0 1 0
  SFColor    color             0.1 0.15 0.18
  MFString   plate             "../textures/plate.jpg"
  SFString   engineSound       "sounds/engine.wav"
  SFString   name              "vehicle"
  SFString   controller        "<generic>"
  MFString   controllerArgs    []
  SFBool     supervisor        FALSE
  SFBool     synchronization   TRUE
  MFNode     sensorsSlotFront  []
  MFNode     sensorsSlotRear   []
  MFNode     sensorsSlotTop    []
  MFNode     sensorsSlotCenter []
  SFBool     frontSpotLights   FALSE
  SFString   window            "automobile"
}
```

> **File location**: "[WEBOTS\_HOME/projects/vehicles/protos/toyota/ToyotaPrius.proto]({{ url.github_tree }}/projects/vehicles/protos/toyota/ToyotaPrius.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### ToyotaPrius Field Summary

- `color`: Defines the car body color.

- `plate`: Defines the plate texture.

- `sensorsSlotFront`: Extends the robot with new nodes on the front of the car.

- `sensorsSlotRear`: Extends the robot with new nodes on the back of the car.

- `sensorsSlotTop`: Extends the robot with new nodes on the roof of the car.

- `sensorsSlotCenter`: Extends the robot with new nodes at the center of the car.

- `frontSpotLights`: Defines whether the car should have front spot lights.

## ToyotaPriusSimple

Simple kinematic model of the Toyota Prius to be moved with a Supervisor.
This model was sponsored by the CTI project RO2IVSim ([http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles](http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)).

%figure

![ToyotaPriusSimple](images/toyota/ToyotaPriusSimple.thumbnail.png)

%end

Derived from [Robot](../reference/robot.md).

```
ToyotaPriusSimple {
  SFVec3f    translation         0 0 0.4
  SFRotation rotation            0 0 1 0
  SFColor    color               0.1 0.15 0.18
  MFColor    recognitionColors   [ 0.1 0.15 0.18 ]
  MFString   plate               "../textures/plate.jpg"
  SFString   name                "vehicle"
  SFString   controller          "<none>"
  MFString   controllerArgs      [ ]
  SFString   window              "<none>"
  MFNode     sensorsSlotFront    [ ]
  MFNode     sensorsSlotRear     [ ]
  MFNode     sensorsSlotTop      [ ]
  MFNode     sensorsSlotCenter   [ ]
  SFBool     wheelBoundingObject FALSE
}
```

> **File location**: "[WEBOTS\_HOME/projects/vehicles/protos/toyota/ToyotaPriusSimple.proto]({{ url.github_tree }}/projects/vehicles/protos/toyota/ToyotaPriusSimple.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### ToyotaPriusSimple Field Summary

- `color`: Defines the car body color.

- `plate`: Defines the plate texture.

- `sensorsSlotFront`: Extends the robot with new nodes on the front of the car.

- `sensorsSlotRear`: Extends the robot with new nodes on the back of the car.

- `sensorsSlotTop`: Extends the robot with new nodes on the roof of the car.

- `sensorsSlotCenter`: Extends the robot with new nodes at the center of the car.

- `wheelBoundingObject`: Defines whether the wheels should have a bounding object.

## ToyotaPriusWheel

Model of the Toyota Prius wheels using the generic [VehicleWheel](vehiclewheel.md) PROTO including physic properties and textures.
The contactMaterial is "ToyotaPriusWheels".
Size: 195/65 R15.
This model was sponsored by the CTI project RO2IVSim ([http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles](http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)).

%figure

![ToyotaPriusWheel](images/toyota/ToyotaPriusWheel.thumbnail.png)

%end

Derived from [Slot](../reference/slot.md).

```
ToyotaPriusWheel {
  SFString name           "Toyota Prius wheel"
  SFNode   physics        NULL
  SFBool   boundingObject TRUE
  SFBool   wheelSide      FALSE
}
```

> **File location**: "[WEBOTS\_HOME/projects/vehicles/protos/toyota/ToyotaPriusWheel.proto]({{ url.github_tree }}/projects/vehicles/protos/toyota/ToyotaPriusWheel.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

