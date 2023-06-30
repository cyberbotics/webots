# Glossary

This glossary defines the terminology used to describe the various concepts related to a Webots simulation.

**Actuator**: An *actuator* is a *node* representing a real robotics actuator such as a motor, a LED, a radio emitter, etc.
*Actuators* may receive commands from a *controller*.
Their behavior will affect the outcome of a simulation.
Exceptionally, some *actuators* may also be *sensors* if they make sensory measurements, such as a motor than can provide force feedback.

**Ancestor**: When referring to a *node*, the *ancestor* of a *descendant* is a *node* which hierarchically contains the *descendant* in the *scene tree*, at any relative depth.

**Base node**: A *base node* is a Webots built-in node.
It is not defined in a [PROTO](proto.md) file.
All the Webots *base nodes* are depicted in the [Node Chart](node-chart.md) and described in details in the [Nodes and API functions](nodes-and-api-functions.md) section of this manual.

**Basic time step**: The *basic time step* is the time step increment used by Webots to advance the *virtual time* and perform physics simulation.
It is specified as a *field* of the [WorldInfo](worldinfo.md) *node* and is expressed in milliseconds.

**Child**: When referring to a *node*, the *child* of a *parent* is a *node* directly contained inside the *parent*, at a relative depth of one in the *scene tree*.
Note that a *child* is always a *descendant*, but a *descendant* is not necessarily a *child*.

**Controller**: A *controller* is a program controlling the behavior of a *robot* and running on its own process.
It can be written in different languages, including C, C++, Python, Java or MATLAB.
It communicates with Webots through a local pipe to read the data measured by the *sensors* of a *robot* and send commands to the *actuators* of the *robot*.

**Controller time step**: The *controller time step* is the time increment executed at each iteration of the control loop of a *controller*.
It is usually passed directly as an argument of the [`wb_robot_step`](robot.md#wb_robot_step) function.
It should be an exact multiple of the *basic time step* for an optimal simulation performance.

**Descendant**: When referring to a *node*, the *descendant* of an *ancestor* is a *node* hierarchically contained inside the *ancestor* at any relative depth in the *scene tree*.

**Device**: A *device* is either an *actuator* or a *sensor*.

**Dynamic solid**: A *dynamic solid* is a *solid* which has a [Physics](physics.md) *node* defined and can move according to the Webots dynamic physics engine (ODE).

**Field**: a *field* is an attribute of a *node*.
It has a name and a data type as defined in VRML97.
The data type can be a single item or a list of items from *node*, boolean, integer, floating point, string, 2D vector, 3D vector, color, etc.

**Kinematic solid**: A *kinematic solid* is a *solid* which has no [Physics](physics.md) *node* defined, but which can move using the Webots kinematic physics engine.

**Node**: A *node* is a component of the *scene tree*.
It defines a concept of the *world* and may refer to external resources, such as *controllers*, image textures, plugins, sounds, etc.

**Parent**: When referring to a *node", the *parent* of a *child* is a *node* containing the *child* at a relative depth of one in the *scene tree*.
Note that a *parent* is always an *ancestor*, but an *ancestor* is not necessarily a *parent*.

**Passive solid**: A *passive solid* is a *solid* which is not a *robot*, neither a *sensor* or an *actuator*.
It may be either a *kinematic* or a *dynamic solid*.

**PROTO node**: A *PROTO node* is a node that is defined in a [PROTO](proto.md) file.
*PROTO nodes* extend the list of available nodes (in addition to the base nodes) to facilitate the use of pre-defined robots, actuators, sensors, objects, etc. Webots provides several *PROTO nodes*, but users can create their own.

**Robot**: A *robot* is a [Robot](robot.md) *node* defining a robotic system.
It usually contains *sensors* and *actuators* in its *descendants*.

**Scene tree**: The *scene tree* is a hierarchical tree structure containing *nodes* that can be saved as a *world*.

**Sensor**: A *sensor* is a *node* representing a real robotics sensor such as a camera, a sonar or a gyroscope.
*Sensors* can send measurement data, such as a camera image or a distance measurement to a *controller*.

**Solid**: A *solid* is a [Solid](solid.md) *node* or any *node* inheriting from the [Solid](solid.md) *node*.
It defines a physical object, including *robots*, *sensors*, *actuators* and *passive solids*.

**Static environment**: The *static environment* is a made up of all the *solids* of a *world* that have no [Physics](physics.md) *node* and cannot move, but have a bounding object defined.
It is used to detect collisions with the *dynamic* and *kinematic solids* in order to prevent them from penetrating it.

**Supervisor**: A *supervisor* is a special *robot* defined by a [Supervisor](supervisor.md) *node* that has some extra capabilities not available in real robots.
It is therefore not realistic to model a real robot with a *supervisor*.
*Supervisors* are nevertheless very useful to script a simulation and perform various operations that a normal user could do manually from the Webots graphical user interface (like moving objects, recording trajectories, making a movie, etc.).

**Virtual time**: The *virtual time* is the simulated time of a running simulation.
It may run faster or slower than the real time, depending on the complexity of the simulation.
The "run real time" feature of Webots allows the user to synchronize the *virtual time* with the real time to ensure a simulation is running no faster than real time.

**World**: A *world* is a file defining a simulation.
It includes *nodes* organized hierarchically in a *scene tree*.
