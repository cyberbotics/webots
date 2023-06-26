## Physics

```
Physics {
  SFFloat density       1000   # {-1, [0, inf)}
  SFFloat mass          -1     # {-1, [0, inf)}
  MFVec3f centerOfMass  [ ]    # any vector
  MFVec3f inertiaMatrix [ ]    # any two vectors
  SFNode  damping       NULL   # {Damping, PROTO}
}
```

### Description

The [Physics](#physics) node allows to specify parameters for the physics simulation engine.
[Physics](#physics) nodes are used in most Webots worlds with the exception of some purely kinematics-based simulations.
The [Physics](#physics) node specifies the mass, the center of gravity and the mass distribution, thus allowing the physics engine to create a *body* and compute realistic forces.

A [Physics](#physics) node can be placed in a [Solid](solid.md) node (or any node derived from [Solid](solid.md)).
The presence or absence of a [Physics](#physics) node in the `physics` field of a [Solid](solid.md) defines whether the [Solid](solid.md) will have a *physics* or a *kinematic* behavior.

> **Note**: In older Webots versions, `coulombFriction, bounce, bounceVelocity` and `forceDependentSlip` fields used to be specified in [Physics](#physics) nodes.
Now these values must be specified in [ContactProperties](contactproperties.md) nodes.
For compatibility reasons, these fields are still present in the [Physics](#physics) but they should no longer be used.

### Field Summary

- The `density` field can be used to define the density of the containing [Solid](solid.md).
The value of the `density` field should be -1 or a positive number expressed in *kilogram per cubic meter* [kg/m³].
A -1 value indicates that the density is not known, in this case the `mass` field (see below) must be specified.
If the `density` is specified (different from -1) then the total mass of the [Solid](solid.md) is calculated by multiplying the specified density with the total volume of the geometrical primitives composing the `boundingObject`.
Note that Webots ignores if the geometrical primitives intersect or not, the volume of each primitive is simply added to the total volume and finally multiplied by the density.

- The `mass` field can be used to specify the total mass of the containing [Solid](solid.md).
The value of the `mass` field should be -1 or a positive number expressed in *kilogram* [kg].
A -1 value indicates that the total mass is not known, in this case the `density` field (see above) must be specified.
If the mass is known, e.g., indicated in the specifications of the robot, then it is more accurate to specify the mass rather than the density.

- The `centerOfMass` field defines the position of the center of mass of the solid.
If there is no vector in the `centerOfMass` field, then the center of mass of the solid is automatically computed based on the [Solid](solid.md) `boundingObject`.
If there is one vector in the `centerOfMass` field, it determines the [Solid](solid.md) center of mass, and so the automatic computation is overriden.
This vector is expressed in meters in the relative coordinate system of the [Solid](solid.md) node.
If this vector is different from [0 0 0], then the center of mass is depicted as a dark red/green/blue cross in Webots 3D-window.

- The `inertiaMatrix` field can be used to manually specify the inertia matrix of the [Solid](solid.md).
This field can either be empty (the default) or contain exactly 2 vectors.
If this field is empty, Webots will compute the inertia matrix automatically according to the position and orientation of the geometrical primitives in `boundingObject`.

    If this field contains 2 vectors, these values specify the inertia matrix of the
    [Solid](solid.md). If the inertia matrix is specified then the `mass` field must
    also be specified. The first vector [I11, I22, I33] represents the *principals
    moments of inertia* and the second vector [I12, I13, I23] represents the
    *products of inertia*. Together these values form a 3x3 inertia matrix:

        [ I11 I12 I13 ]
        [ I12 I22 I23 ]
        [ I13 I23 I33 ]

    The Ixx values are expressed in *kilogram square meter* [kg⋅m²]. The principals moments of inertia must
    be positive. The inertia matrix is defined with respect to the `centerOfMass` of
    the [Solid](solid.md). Internally, these 6 values are passed unchanged to the
    ODE's `dMassSetParameters` function.

- The `damping` field allows to specify a [Damping](damping.md) node that defines the velocity damping parameters to be applied to the [Solid](solid.md).

### How to use Physics Nodes?

If it contains a [Physics](#physics) node, a [Solid](solid.md) object will be simulated in *physics* mode.
The *physics* simulation mode takes into account the simulation of the forces that act on the bodies and the properties of these bodies, e.g., mass and moment of inertia.
On the contrary, if its `physics` field is NULL, then the [Solid](solid.md) will be simulated in *kinematics* mode.
The *kinematics* mode simulates the objects motions without considering the forces that cause the motion.
For example in *kinematics* mode an object can reach the desired speed immediately while in *physics* mode the inertial resistance will cause this object to accelerate progressively.
It is usually not necessary to specify all the [Physics](#physics) nodes in a Webots world.
Whether to use or not a [Physics](#physics) node in a particular case depends on what aspect of the real world your want to model in your simulation.

Note that if a [Solid](solid.md) node has at least one [Solid](solid.md) ancestor node and contains a [Physics](#physics) node but none of its [Solid](solid.md) ancestor nodes contain a [Physics](#physics) node, then this node will be attached to the static environment with an ODE fixed joint and thus it won't move from its initial position.

#### In Passive Objects

If a passive object should never move during a simulation then you should leave its `physics` field empty.
In this case no contact force will be simulated on this object and hence it will never move.
This is perfect for modeling walls or the floor.
Furthermore the floor should always be designed without [Physics](#physics) node anyway, because otherwise it would fall under the action of gravity.

On the contrary, if a passive object needs to be pushed, kicked, dropped, etc. then it should have a [Physics](#physics) node.
So for example, if you want to design a soccer game where the ball needs to be kicked and roll, then you will need to add a [Physics](#physics) node to the ball.
Similarly, in a box pushing or stacking simulation, you will need to specify the [Physics](#physics) nodes for the boxes so that the friction and gravity forces are applied to these objects.

#### In Robots

Articulated robot, humanoids, vehicles and so on, are built as hierarchies of [Solid](solid.md) nodes (or subclasses of [Solid](solid.md)).
The contact and friction forces generated by legs or wheels are usually a central aspect of the simulation of robot locomotion.
Similarly, the contact and friction forces of a grasping robotic hand or gripper is crucial for the simulation of such devices.
Therefore the mechanical body parts of robots (eg., legs, wheels, arms, hands, etc) need in general to have [Physics](#physics) nodes.

> **Note**: It is possible to set the `physics` field of a [Robot](robot.md) or a top [Solid](solid.md) to `NULL` in order to pin its base to the static environment.
This can be useful for the simulation of a robot arm whose base segment is anchored in a fixed place.
More generally, you can define a larger *static base* rooted at a given top [Solid](solid.md).
Indeed you can define a subtree starting from this top [Solid](solid.md) and whose all [Solid](solid.md) nodes have no [Physics](#physics) nodes.

<!-- -->

> **Note**: The [Robot](robot.md) nodes with a differential wheels configuration are a special case: they can move even if they don't have [Physics](#physics) nodes.
That's because Webots uses a special *kinematics* algorithm for this kind of robots without [Physics](#physics).
However, if the [Physics](#physics) nodes are present then Webots uses the regular *physics* simulation algorithms.
For a [Robot](robot.md) node to be detected as a differential wheels, the following constraints must be satisfied: the robot must have 2 [HingeJoint](hingejoint.md) nodes whose axes are aligned and the [Solid](solid.md) nodes in the `endPoint` field of the joints must each have a [Cylinder](cylinder.md) node as bounding object with the same radius.

#### Implicit Solid Merging and Joints

By [Solid](solid.md) *child* of a given [Solid](solid.md) node, we mean either a node directly placed into the children list or a [Solid](solid.md) node located into the `endPoint` field of a [Joint](joint.md) placed in the children list.
We extend this definition to nested [Group](group.md)s starting from the [Solid](solid.md) children list and containing [Joint](joint.md)s or [Solid](solid.md)s.

If a [Solid](solid.md) child in the above sense is not related to its [Solid](solid.md) parent by a joint while both have a [Physics](#physics) node, they are *merged at the physics engine level*: ODE will be given only one body to represent both parent and child.
This process is recursive and stops at the highest ancestor which have a joint pointing to an upper [Solid](solid.md) or just before the highest ancestor without [Physics](#physics) node.
This way modelling a rigid assembly of [Solid](solid.md)s won't hurt physics simulation speed even if it aggregates numerous components.

> **Note**: When solid merging applies, only the highest ancestor of the rigid assembly has a body (a non null `dBodyID` in ODE terms) which holds the physical properties of the assembly.
This may impact the way you design a [physics plugin](physics-plugin.md)s.

When designing the robot tree structure, there is one important rule to remember about the [Physics](#physics) nodes: *If a Solid node has a parent and a child with a Physics node then it must also have a Physics node* (1).
A consequence of this rule is that, in a robot tree structure, only leaf nodes and nodes included in the *static basis* (see first [note](physics.md) above) can have a NULL `physics` field.
In addition, top [Robot](robot.md) nodes do usually have [Physics](#physics) because this is required to allow any of their children to use the *physics* simulation.

Note that each [Physics](#physics) node adds a significant complexity to the world: as a consequence the simulation speed decreases.
Therefore the number of [Physics](#physics) nodes should be kept as low as possible.
Fortunately, even with a complex wheeled or articulated robot some of the `physics` fields can remain empty (NULL).
This is better explained with an example.
Let's assume that you want to design an articulated robot with two legs.
Your robot model may look like this (very simplified):

```
Robot {
  ...
  children [
    DEF LEG1_HINGE HingeJoint  {
      ...
      endPoint DEF LEG1 Solid {
        physics Physics {
        }
      }
    }
    DEF LEG2_HINGE HingeJoint {
      ...
      endPoint DEF LEG2 Solid {
        physics Physics {
        }
      }
    }
  ]
  physics Physics {
  }
}
```

The legs need [Physics](#physics) nodes because the forces generated by their contact with the floor will allow the robot to move.
If you would leave the legs without [Physics](#physics), then no contact forces would be generated and therefore the robot would not move.
Now, according to rule (1), because the legs have [Physics](#physics) nodes, their parent (the [Robot](robot.md) node) must also have a [Physics](#physics) node.
If the [Physics](#physics) node of the [Robot](robot.md) was missing, the simulation would not work, the legs would fall off, etc.

Now suppose you would like to add a [Camera](camera.md) to this robot.
Let's also assume that the physical properties of this camera are not relevant for this simulation, say, because the mass of the camera is quite small and because we want to ignore potential collisions of the camera with other objects.
In this case, you should leave the `physics` field of the camera empty.
So the model with the camera would look like this:

```
Robot {
  ...
  children [
    DEF CAM Camera {
      ...
    }
    DEF LEG1_HINGE HingeJoint  {
      ...
      endPoint DEF LEG1 Solid {
        ...
        physics Physics {
        }
      }
    }
    DEF LEG2_HINGE HingeJoint {
      ...
      endPoint DEF LEG2 Solid {
        physics Physics {
        }
      }
    }
  ]
  physics Physics {
  }
}
```

Now suppose that the camera needs to be motorized, e.g., it should rotate horizontally.
Then the camera must simply be placed in the `endPoint` field of [HingeJoint](hingejoint.md) node that controls its horizontal position.
This time again, the physical properties of the camera motor are apparently unimportant.
If we assume that the mass of the camera motor is small and that its inertia is not relevant, then the camera [Physics](#physics) node can also be omitted:

```
Robot {
  ...
  children [
    DEF CAMERA_HINGE HingeJoint {
      ...
      device DEF CAM_MOTOR RotationalMotor {
          ...
      }
      endPoint DEF CAM Camera {
        ...
      }
    }
    DEF LEG1_HINGE HingeJoint  {
      ...
      endPoint DEF LEG1 Solid {
        ...
        physics Physics {
        }
      }
    }
    DEF LEG2_HINGE HingeJoint {
      ...
      endPoint DEF LEG2 Solid {
        physics Physics {
        }
      }
    }
  ]
  physics Physics {
  }
}
```

#### Devices

Most device nodes work without [Physics](#physics) node.
But a [Physics](#physics) node can optionally be used if one wishes to simulate the weight and inertia of the device.
So it is usually recommended to leave the `physics` field of a device empty, unless it represents a significant mass or volume in the simulated robot.
This is true for these devices: [Accelerometer](accelerometer.md), [Altimeter](altimeter.md), [Camera](camera.md), [Compass](compass.md), [DistanceSensor](distancesensor.md), [Emitter](emitter.md), [GPS](gps.md), [LED](led.md), [LightSensor](lightsensor.md), [Pen](pen.md), and [Receiver](receiver.md).

> **Note**: The [InertialUnit](inertialunit.md), [Connector](connector.md), and the [VacuumGripper](vacuumgripper.md) nodes work differently.
Indeed, they require the presence of a [Physics](#physics) node in their parent node to be functional.
It is also possible to specify a [Physics](#physics) node of the device but this adds an extra body to the simulation.

> The [TouchSensor](touchsensor.md) is also a special case: it needs a [Physics](#physics) node when it is used as "force" sensor; it does not necessarily need a [Physics](#physics) node when it is only used as "bumper" sensor.
