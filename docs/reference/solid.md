## Solid

Derived from [Pose](pose.md).

```
Solid {
  SFString name                "solid"       # any string
  SFString model               ""            # any string
  SFString description         ""            # any string
  SFString contactMaterial     "default"     # any string
  MFNode   immersionProperties [ ]           # {ImmersionProperties, PROTO}
  SFNode   boundingObject      NULL          # {node, PROTO}
  SFNode   physics             NULL          # {Physics, PROTO}
  SFBool   locked              FALSE         # {TRUE, FALSE}
  SFFloat  radarCrossSection   0.0           # [0, 1]
  MFColor  recognitionColors   []            # any color
  # hidden fields
  SFVec3f  linearVelocity      0 0 0         # any vector
  SFVec3f  angularVelocity     0 0 0         # any vector
}
```

Direct derived nodes: [Accelerometer](accelerometer.md), [Altimeter](altimeter.md), [Camera](camera.md), [Charger](charger.md), [Compass](compass.md), [Connector](connector.md), [Display](display.md), [DistanceSensor](distancesensor.md), [Emitter](emitter.md), [GPS](gps.md), [Gyro](gyro.md), [InertialUnit](inertialunit.md), [LED](led.md), [Lidar](lidar.md), [LightSensor](lightsensor.md), [Pen](pen.md), [Radar](radar.md), [RangeFinder](rangefinder.md), [Receiver](receiver.md), [Robot](robot.md), [TouchSensor](touchsensor.md), [Track](track.md), [VacuumGripper](vacuumgripper.md).

### Description

A [Solid](#solid) node represents an object with physical properties such as dimensions, a contact material and optionally a mass.
The [Solid](#solid) class is the base class for collision-detected objects.
Robots and device classes are subclasses of the [Solid](#solid) class.
In the 3D window, [Solid](#solid) nodes can be manipulated (dragged, lifted, rotated, etc) using the mouse.
[Solid](#solid) nodes may be `children` of [Group](group.md), [Pose](pose.md), [Solid](#solid) nodes and other nodes derived from [Solid](solid.md).
They cannot be `children` of a [Transform](transform.md) node, because they don't support scaling.

### Field Summary

- `name`: name of the solid.
In derived device classes this corresponds to the device name argument used by the `wb_robot_get_device` function.
Note that the name cannot contain the colon character '`:`' and should preferably identify the solid uniquely, please refer to the [Unique Solid name](#unique-solid-name) section for further details.

- `model`: generic name of the solid (e.g., "chair").

- `description`: short description (1 line) of the solid.

- `contactMaterial`: name of the contact material.
When the `boundingObject`s of [Solid](#solid) nodes intersect, the `contactMaterial` is used to define which [ContactProperties](contactproperties.md) must be applied at the contact points.

- `immersionProperties`: list of [ immersionProperties](immersionproperties.md) nodes.
It is used to specify dynamic interactions of the [Solid](#solid) node with one or more [Fluid](fluid.md) nodes.

- `boundingObject`: the bounding object specifies the geometrical primitives used for collision detection.
If the `boundingObject` field is NULL, then no collision detection is performed and that object can pass through any other object, e.g., the floor, obstacles and other robots.
Note that if the `boundingObject` field is NULL then the `physics` field (see below) must also be NULL.
You will find more explanations about the `boundingObject` field below.

- `physics`: this field can optionally contain a [Physics](physics.md) node that is used to model the physical properties of this [Solid](#solid).
A [Physics](physics.md) node should be added when effects such as gravity, inertia, frictional and contact forces need to be simulated.
If the `physics` field is NULL then Webots simulates this object in *kinematics* mode.
Note that if this field is not NULL then the `boundingObject` field must be specified.
Please find more info in the description of the [Physics](physics.md) node.
For consecutive solids, e.g., two solids attached to each other with a joint or a solid attached to the static environment with a joint, no collision detection is performed even if the `physics` field is set.
The reason is that this type of collision detection is usually not wanted by the user, because a very accurate design of the bounding objects of the solids would be required.
To prevent two consecutive solid nodes from penetrating each other, the `minStop` and `maxStop` fields of the corresponding joint node should be adjusted accordingly.

- `locked`: if `TRUE`, the solid object cannot be moved using the mouse.
This is useful to prevent moving an object by mistake.

- `radarCrossSection`: if greater than 0 this [Solid](#solid) node is a potential target for any [Radar](radar.md) device.
Radar cross section (RCS) is the measure of a target's ability to reflect radar signals in the direction of the radar receiver, i.e. it is a measure of the ratio of backscatter density in the direction of the radar to the power density that is intercepted by the target.
Typical values are 0.01 for a bird, 1 for a human, 100 for a car and 200 for a truck.

- `recognitionColors`: if not empty, this [Solid](#solid) node may be recognized by any [Camera](camera.md) device with a [Recognition](recognition.md) node.
The colors defined in this field are returned for this object if recognized by a camera, but they have no impact on the visual appearance of the [Solid](#solid) node.

### Hidden Field Summary

- `linearVelocity` and `angularVelocity`: these fields, which aren't visible from the Scene Tree, are used by Webots when saving a world file to store the initial linear and angular velocities of a [Solid](#solid) with a non-NULL [Physics](physics.md) node.
If the [Solid](#solid) node is merged into a solid assembly (see [implicit solid merging](physics.md#implicit-solid-merging-and-joints)), then these fields will be effective only for the [Solid](#solid) at the top of the assembly.
Hidden velocity fields allow you to save and restore the dynamics of your simulation or to define initial velocities for every physical objects in the scene.

### How to use the "boundingObject" Field?

The `boundingObject` field is used to define the bounds of a [Solid](#solid) as geometrical primitives.
Each `boundingObject` can hold one or several geometrical primitives, such as [Box](box.md), [Capsule](capsule.md), [Cylinder](cylinder.md), etc.
These primitives should normally be chosen such as to represent the approximate bounds of the [Solid](#solid).
In the usual case, the graphical representation of a robot is composed of many complex shapes, e.g., [IndexedFaceSet](indexedfaceset.md)s, placed in the `children` field of the [Solid](#solid) nodes.
However this graphical representation is usually too complex to be used directly for detecting collisions.
If there are too many faces the simulation becomes slow and error-prone.
For that reason, it is useful to be able to approximate the graphical representation by simpler primitives, e.g., one or more [Box](box.md) or [Capsule](capsule.md)s, etc.
This is the purpose of the `boundingObject` field.

Various combinations of primitives can be used in a `boundingObject`: it can contain either:

1. A [Box](box.md) node.
2. A [Capsule](capsule.md) node.
3. A [Cylinder](cylinder.md) node.
4. An [ElevationGrid](elevationgrid.md) node.
5. An [IndexedFaceSet](indexedfaceset.md) node.
6. A [Plane](plane.md) node.
7. A [Sphere](sphere.md) node.
8. A [Shape](shape.md) node with one of the above nodes in its `geometry` field.
9. A [Pose](pose.md) node with one of the above nodes in its `children` field.
10. A [Group](group.md) node with several `children`, each being one of the above.

The `boundingObject`, together with the [Physics](physics.md) node, are used to compute the inertia matrix of the [Solid](#solid).
Such a computation assumes a uniform mass distribution in the primitives composing the `boundingObject`.
Note that the center of mass of the [Solid](#solid) does not depend on its `boundingObject`.
The center of mass is specified by the `centerOfMass` field of the [Physics](physics.md) node (in coordinates relative to the center of the [Solid](#solid)).

The `boundingObject` of the selected object is displayed in the 3D view by depicting the outline of the `boundingObject` shapes.
Additionally, if the `View|Optional Rendering|Show All Bounding Objects` option is enabled, all the `boundingObjects` in the scene will be visible.
The color of the `boundingObject` outline indicates the status of the object: usually the outline is represented by white lines, but these lines turn pink if the solid is colliding with another object and blue when the solid is idle, i.e., it comes to rest and it does not interact with any other active solid.

### Unique Solid Name

The capability of identifying uniquely each [Solid](#solid) node is a base requirement for other advanced features, for example to let the viewpoint follow a solid object or to store the preferences for the rendering devices overlays.
For this reason since Webots R2018a, the user is encouraged to set unique names for sibling [Solid](#solid) nodes and a warning is printed in the console if this rule is not respected.
With sibling [Solid](#solid) nodes we mean [Solid](#solid) nodes for which the upper [Solid](#solid) node is the same.
It doesn't matter if other nodes with a different type exist between the current and the upper [Solid](#solid) node.

For example, here is a sample robot definition.
```
Robot {
  children [
    DEF A Solid { }
    DEF B Solid {
      children [
        DEF C Solid {
        }
      ]
    }
    Group {
      children [
        Pose {
          children [
            DEF D Solid { }
          ]
        }
      ]
    }
    HingeJoint {
      endPoint DEF E Solid { }
    }
    Slot {
      endPoint Slot {
        endPoint DEF F Solid { }
      }
    }
  ]
}
```

In this example [Solid](#solid) nodes `A`, `B`, `D`, `E`, and `F` are sibling nodes because they have the same upper [Solid](#solid) node, i.e. the [Robot](robot.md) node.
[Solid](#solid) node `C` is not a sibling because his upper [Solid](#solid) node is `B` and not the [Robot](robot.md) node.

To smooth the creation of worlds, when a new [Solid](#solid) node is inserted from the GUI or from a [Supervisor](robot.md) controller, Webots checks the name of the just inserted node and automatically computes a unique name if needed.
The computed name contains an additional suffix `(<i>)`, where `<i>` is the smallest available positive index that produces a unique name between sibling [Solid](#solid) nodes.
For example, if sibling [Solid](#solid) nodes with the name 'solid', 'solid(1)', and 'solid(3)' already exists, the computed name for the new node will be 'solid(2)'.

Note that in this section we refer to [Solid](#solid) nodes, but this rule applies to all the [Solid](#solid) derived nodes.
Devices have the additional constraint that their `name` field value has to be unique inside the [Robot](robot.md) node.
