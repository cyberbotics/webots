## PROTO Design Guidelines

When creating your own PROTO nodes, we recommend that you follow these guidelines.
This ensures that your new PROTO nodes will be well designed, easy-to-use and consistent with the existing PROTO nodes provided in Webots.

### Naming

The naming of a PROTO is important and should be precise and explicit.
Therefore, you should avoid abbreviations, numbers and vague terminology when naming a PROTO.
For example `SmallWoodenChair` is better than `SmallChair`, which is better than `Chair`, which is better than `Chair7`.
PROTO names should use upper camel case, e.g., each word should begin with a capital letter with no intervening spaces.

> **Note**: The PROTO file name should match exactly the PROTO name.
For example the `SmallWoodenChair` PROTO should be defined in a file named `SmallWoodenChair.proto`.

### Orientation

PROTO nodes should be oriented in a way they appear nicely in a ENU coordinate system (which is the default in [WorldInfo](worldinfo.md)).
If an object has at least two sides defined (for example up and front) it should be oriented using the FLU system:
- **F**orward direction along the positive X-axis,
- **L**eft direction along the positive Y-axis, and
- **U**p direction along the positive Z-axis.

Typical candidates for the FLU system are a TV set, a rubber duck, or a robot.
Objects such as a flower pot don't have any front, back, right or left side.
In such cases, only the up direction should be set correctly: the upright position should be along the positive Z-axis.
Objects such as a soccer ball don't have any up, down, front, back, right or left side.
In such cases, the orientation has no importance.

### Header

The first line of a PROTO file is called the header.
It should be the following: `#VRML_SIM {{ webots.version.major }} utf8`.
`{{ webots.version.major }}` corresponds to the version of Webots needed to run this PROTO.
It should be followed by a number of comments describing the PROTO.
These comments should be formatted as described in the [example](#example) below to be displayed properly by Webots.

#### License

If a PROTO is meant to be distributed, it is important to specify the license under which it can be used.
For that purpose, the name of the license should be specified in the `license:` comment and the URL to the license file should be given in the `license url:` comment (see [example](#example) below).

#### Tags

If needed, the `tags:` comment should be properly specified.
It currently supports four possible options: `deprecated`, `hidden`, `nonDeterministic` and `no3dView` which may be used simultaneously separated with a coma:
- `deprecated` means this PROTO should not be used any more in new simulations, but is kept for backwards compatibility. When using a deprecated PROTO, Webots will display a warning message about it.
- `hidden` tells Webots not to display this PROTO in the Add Node dialog when the user wants to insert a new PROTO.
Hidden PROTO nodes are typically used as sub-PROTO nodes, that is they are used from another PROTO file, but not directly from a world file.
- `nonDeterministic` tells Webots that this PROTO may yield a different result with each execution.
This is typically the case if randomness is involved in the execution of the PROTO, which most commonly occurs if a time-based seed is used in the random process.
- `no3dView` tells [webots.cloud](https://webots.cloud/proto) not to load the 3D view when displaying the page of this PROTO.

#### Documentation

The `documentation url:` comment can be used to link to the PROTO documentation. The link will be displayed in the add node dialog and openned from the node help menu.

#### Keywords

The `keywords:` comment should be added to classify the proto correctly on [webots.cloud](https://webots.cloud/proto).
The following keywords are available:
```
robot
  - arm
  - flying
  - tracked
  - legged
  - wheeled
  - other
  - accessory
  - extension
sensor
  - camera
  - distance-sensor
  - imu
  - lidar
  - radar
  - range-finder
actuator
  - gripper
  - motor
  - other
industrial
  - cart
  - container
  - conveyor
  - ironmongery
  - light
  - plumbing
  - safety
  - tool
  - other
household
  - electronics
  - hospital
  - kitchen
  - light
  - living room
  - potted plant
  - school
  - toy
vehicle
  - car
  - farm
  - public
  - truck
  - two wheels
  - wheel
  - steering
  - other
furniture
  - bathroom
  - bedroom
  - chair
  - kitchen
  - laundry
  - living room
  - storage
  - table
building
  - business
  - civil
  - component
  - farm
  - hotel
  - house
  - industry
  - residential
  - shop
animal
  - farm
  - pet
  - wild
exterior
  - garden
  - obstacle
  - sport
  - street furniture
  - tree
traffic
  - road
  - sign
appearance
  - electronics
  - fabric
  - leather
  - metal
  - mineral
  - other
  - paint
  - plastic
  - rubber
  - wood
primitive
  - background
  - geometry
  - ground
  - joint
```

You should declare the keywords as follow:
- If the proto fits in a category which has subcategories, you have to choose a subcategory. The syntax is then: `keywords:first level category/second level category`, for example: `keywords: robot/walking`.
- If the robot fits in several categories, the different categories must be separated by a comma, for example: `keywords: vehicle/car, appearance/metal`.

#### Description

Finally, it is important to provide a short description about what the PROTO is about.
This may be very simple as in the [example](#example) below.
This information is displayed in the Add Node dialog to describe the PROTO.

### Recommended Fields for Simple PROTO Nodes

This section describes simple PROTO nodes representing real world objects, which are derived from the `Solid` node.
It doesn't cover PROTO nodes derived from other primitives (such as geometries) and complex PROTO nodes such as sensors, actuators or robots.

#### "translation" and "rotation"

Real objects can be placed at different locations and orientations in the world.
Therefore the corresponding PROTO should expose at least a `translation` and a `rotation` field to allow the users to move and rotate the object easily.

The default value for the translation should be preferably the origin (0, 0, 0) and should correspond to the object in a normal position, e.g., laying on the floor rather than sinking into the floor.
That means the origin of an object should not be at its 3D geometrical center, but rather in the middle of the surface in contact with the floor.
However, there are some exceptions to this rule:
- Legged robots should have their origin in their main body as it is difficult to know the extension of the legs of the robot towards the floor.
- Balls should have their origin at their geometrical center, as they can roll and don't have an upright position.
- Devices that are directly derived from base nodes (e.g., Lidar, Camera, etc.) can have a default translation different from `0 0 0` so that the device shape rests on the ground when created even if the sensor itself is not located at the contact surface.
- Any other object which doesn't have a clear or stable upright position.

The rotation axis should be already well positioned, e.g., usually along the Z-axis with a 0 value for the angle, e.g., `0 0 1 0`, so that when you rotate a building for example, you should simply change the angle value to have it rotate along its vertical axis.

#### "enableBoundingObject"

Bounding objects are used for collision detection.
If you believe that an object won't collide with anything, it is convenient to be able to turn off collision detection to save computation time.
Providing a PROTO with a `enableBoundingObject` boolean field deserves this purpose.
For example, in a scenario where a small robot is running on a table top in a living room, all the objects outside of the range of the robot, like the sofa, chairs, chandelier, TV set, etc. will never collide with the robot and hence could have their collision detection disabled.
However, in some other scenarios, we want that the robot can collide with a chair, because the robot is running on the floor.
Therefore such objects should have a `enableBoundingObject` field exposed to allow the users to decide whether they want to enable collision detection, depending on the specific simulation scenario.

#### "enablePhysics"

Because sometimes, we want to simulate the physical motion of an object and sometimes we want to save computation time by not simulating the physics of the object, it is good also to add a `enablePhysics` boolean field that allows the user to enable or disable physics simulation for an object.
For example, a box may be meant to be moveable by a robot or may be glued to the floor or so heavy that it won't move.
In that case, it is nice to be able to specify if we want to simulate the physical motion of that object or not.
Not simulating the physics saves computation time and increase the stability of the simulation.
However, some objects are not meant to move during a simulation.
This includes buildings, traffic signs, trees, mailboxes, etc.
Therefore such objects should not contain a `Physics` node and obviously should not expose a `enablePhysics` field.

#### Other Fields

In PROTO nodes that correspond to real world objects, it is not recommend to add fields such as `mass` or `scale`.
This is because in real life, you cannot change the mass or the size of an object.
And we don't want to encourage users of the PROTO nodes to do so, as it may lead to non-realistic models and numerical instabilities.
Some exceptions however exist.
For example, it may be convenient to add a `size` field to a Matryoshka doll PROTO.
Nevertheless, such a `size` field should be limited to a minimal and a maximal size or to a limited range of integer values.
Therefore it could be either a floating point or an integer value.

For some PROTO nodes, it could be useful to expose a `color` field.
For example, a car or a color pencil PROTO could have a `color` field exposed.
However, that should be limited to objects that are available in different colors.
For example, it should not be used for a fire hydrant (usually always red) or a fork (usually always metallic grey).
The `color` field may be specified as a `Color` node if any color is available or as a string (if only a limited number of colors is available for that object).

Depending on the object, a number of texture fields may be useful.
For example a painting PROTO could have the painting specified in a `picture` field and the frame texture specified in a `frame` field.

Generally, the number of exposed fields should be minimal in order to guarantee that the resulting object will be realistic and correspond to the original idea of the PROTO.
Also, floating point and integer values should be generally constrained between a minimum and a maximum value to ensure a realistic and stable result.

### Field Comments

If needed, a field should be carefully commented to explain its meaning following these rules:

- If the field is connected directly to a root node with an `IS`, the comment should be `# Is RootNodeType.fieldname.`.
- If the field is a slot, the comment should be `# Extends the robot with new nodes in the XXX slot.`
- In general, comments should use a conjugated verb (in most cases "Defines") and end with period (".").

### Solid Names

Be careful to avoid as much as possible `Solid.name` values which are automatically generated by Webots, such as `solid(1)`.
It is better to use more descriptive names, like `left arm pivot`.

### Example

Here is a simple example of a good PROTO declaration (the implementation is not shown):

```
#VRML_SIM {{ webots.version.major }} utf8
# license: Apache License 2.0
# license url: https://www.apache.org/licenses/LICENSE-2.0
# A color pencil

PROTO ColorPencil [
  field SFFloat    translation          0 0 0
  field SFRotation rotation             0 0 1 0
  field SFBool     enablePhysics        TRUE
  field SFBool     enableBoundingObject TRUE
  field SFColor    color                1 0 0     # defaults to red.
  field SFFloat    size                 0.2       # range in [0.02, 0.2].
]
...
```
