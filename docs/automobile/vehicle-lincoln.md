# Lincoln

## LincolnMKZ

Model of a Lincoln MKZ 2017.

%figure

![LincolnMKZ](images/lincoln/LincolnMKZ.thumbnail.png)

%end

Derived from [Robot](../reference/robot.md).

```
LincolnMKZ {
  SFVec3f    translation       0 0 0.4
  SFRotation rotation          0 0 1 0
  SFColor    color             0.541 0.541 0.541
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

> **File location**: "[WEBOTS\_HOME/projects/vehicles/protos/lincoln/LincolnMKZ.proto]({{ url.github_tree }}/projects/vehicles/protos/lincoln/LincolnMKZ.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### LincolnMKZ Field Summary

- `color`: Defines the car body color.

- `plate`: Defines the plate texture.

- `sensorsSlotFront`: Extends the robot with new nodes on the front of the car.

- `sensorsSlotRear`: Extends the robot with new nodes on the back of the car.

- `sensorsSlotTop`: Extends the robot with new nodes on the roof of the car.

- `sensorsSlotCenter`: Extends the robot with new nodes at the center of the car.

- `frontSpotLights`: Defines whether the car should have front spot lights.

## LincolnMKZSimple

Simple kinematic model of the Lincoln MKZ 2017 to be moved with a Supervisor.

%figure

![LincolnMKZSimple](images/lincoln/LincolnMKZSimple.thumbnail.png)

%end

Derived from [Robot](../reference/robot.md).

```
LincolnMKZSimple {
  SFVec3f    translation                    0 0 0.4
  SFRotation rotation                       0 0 1 0
  SFColor    color                          0.541 0.541 0.541
  MFColor    recognitionColors              [ 0.541 0.541 0.541 ]
  MFString   plate                          "../textures/plate.jpg"
  SFString   name                           "vehicle"
  SFString   controller                     "<none>"
  MFString   controllerArgs                 [ ]
  SFString   window                         "<none>"
  MFNode     sensorsSlotFront               [ ]
  MFNode     sensorsSlotRear                [ ]
  MFNode     sensorsSlotTop                 [ ]
  MFNode     sensorsSlotCenter              [ ]
  SFBool     wheelBoundingObject            FALSE
}
```

> **File location**: "[WEBOTS\_HOME/projects/vehicles/protos/lincoln/LincolnMKZSimple.proto]({{ url.github_tree }}/projects/vehicles/protos/lincoln/LincolnMKZSimple.proto)"

> **License**: Copyright Cyberbotics Ltd. Licensed for use only with Webots.
[More information.](https://cyberbotics.com/webots_assets_license)

### LincolnMKZSimple Field Summary

- `color`: Defines the car body color.

- `plate`: Defines the plate texture.

- `sensorsSlotFront`: Extends the robot with new nodes on the front of the car.

- `sensorsSlotRear`: Extends the robot with new nodes on the back of the car.

- `sensorsSlotTop`: Extends the robot with new nodes on the roof of the car.

- `sensorsSlotCenter`: Extends the robot with new nodes at the center of the car.

- `wheelBoundingObject`: Defines whether the wheels should have a bounding object.

