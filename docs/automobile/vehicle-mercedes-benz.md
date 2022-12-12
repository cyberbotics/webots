# Mercedes Benz

## MercedesBenzSprinter

Model of a Mercedes-Benz Sprinter.

%figure

![MercedesBenzSprinter](images/mercedes_benz/MercedesBenzSprinter.png)

%end

Derived from [Robot](../reference/robot.md).

```
MercedesBenzSprinter {
  SFVec3f     translation              0 0 0.4445
  SFRotation  rotation                 0 0 1 0
  SFColor     color                    0.98 0 0.1
  SFString    name                     "Mercedes-Benz Sprinter"
  SFString    controller               "<generic>"
  MFString    controllerArgs           []
  SFBool      supervisor               FALSE
  SFBool      synchronization          TRUE
  MFNode      sensorsSlotFront         []
  MFNode      sensorsSlotRear          []
  MFNode      sensorsSlotTop           []
  MFNode      sensorsSlotCenter        []
  SFBool      interior                 FALSE
  SFString    window                   "automobile"
}
```

> **File location**: "[WEBOTS\_HOME/projects/vehicles/protos/mercedes\_benz/MercedesBenzSprinter.proto]({{ url.github_tree }}/projects/vehicles/protos/mercedes_benz/MercedesBenzSprinter.proto)"

> **License**: Apache License 2.0
[More information.](http://www.apache.org/licenses/LICENSE-2.0)

## MercedesBenzSprinterSimple

Simple kinematic model of the Mercedes-Benz Sprinter to be moved with a Supervisor.

%figure

![MercedesBenzSprinterSimple](images/mercedes_benz/MercedesBenzSprinterSimple.png)

%end

Derived from [Robot](../reference/robot.md).

```
MercedesBenzSprinterSimple {
  SFVec3f    translation                    0 0 0.4
  SFRotation rotation                       0 0 1 0
  SFColor    color                          0.98 0 0.1
  MFColor    recognitionColors              [ 0.98 0 0.1 ]
  SFString   name                           "Mercedes-Benz Sprinter"
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

> **File location**: "[WEBOTS\_HOME/projects/vehicles/protos/mercedes\_benz/MercedesBenzSprinterSimple.proto]({{ url.github_tree }}/projects/vehicles/protos/mercedes_benz/MercedesBenzSprinterSimple.proto)"

> **License**: Apache License 2.0
[More information.](http://www.apache.org/licenses/LICENSE-2.0)

