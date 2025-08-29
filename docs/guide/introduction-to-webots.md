## Introduction to Webots

### What is Webots?

Webots is a professional mobile robot simulation software package.
It offers a rapid prototyping environment, that allows the user to create 3D virtual worlds with physics properties such as mass, joints, friction coefficients, etc.
The user can add simple passive objects or active objects called mobile robots.
These robots can have different locomotion schemes (wheeled robots, legged robots, or flying robots).
Moreover, they may be equipped with a number of sensor and actuator devices, such as distance sensors, drive wheels, cameras, motors, touch sensors, emitters, receivers, etc.
Finally, the user can program each robot individually to exhibit the desired behavior.
Webots contains a large number of robot models and controller program examples to help users get started.

Webots also contains a number of interfaces to real mobile robots, so that once your simulated robot behaves as expected, you can transfer its control program to a real robot like e-puck, DARwIn-OP, Nao, etc.
Adding new interfaces is possible through the related system.

### What can I do with Webots?

Webots is well suited for research and educational projects related to mobile robotics.
Many mobile robotics projects have relied on Webots for years in the following areas:

- Mobile robot prototyping (academic research, the automotive industry, aeronautics, the vacuum cleaner industry, the toy industry, hobbyists, etc.)
- Robot locomotion research (legged, humanoids, quadrupeds robots, etc.)
- Multi-agent research (swarm intelligence, collaborative mobile robots groups, etc.)
- Adaptive behavior research (genetic algorithm, neural networks, AI, etc.).
- Teaching robotics (robotics lectures, C/C++/Java/Python programming lectures, etc.)
- Robot contests (e.g. Robotstadium or Rat's Life)

### What do I need to know to use Webots?

You will need a minimal amount of technical knowledge to develop your own simulations:

- A basic knowledge of the C, C++, Java, Python or MATLAB programming language is necessary to program your own robot controllers.
However, even if you don't know these languages, you can still program the e-puck and Hemisson robots using a simple graphical programming language called BotStudio.
- If you don't want to use existing robot models provided within Webots and would like to create your own robot models, or add special objects in the simulated environments, you will need a basic knowledge of 3D computer graphics and VRML97 description language.
That will allow you to create 3D models in Webots or import them from 3D modeling software.

### How do I get User Support?

[Cyberbotics](https://www.cyberbotics.com/#services) provides paid user support through its Premier Service plans and Custom Development services.

Community-based support is also available in various places, however, there is no guarantee you will get an answer to your question:
- [Robotics StackExchange](https://robotics.stackexchange.com/questions/tagged/webots) with the `webots` tag is probably the best place to ask a technical question about Webots.
- [GitHub Discussions](https://github.com/cyberbotics/webots/discussions) may be used to ask questions related with the development of Webots and its documentation.
- [GitHub Issues](https://github.com/cyberbotics/webots/issues) should be used only to report bugs found in Webots.
- [Discord](https://discordapp.com/invite/nTWbN9m) can be used for quick questions and spontaneous discussions with the Webots community.

### Webots Simulation

A Webots simulation is composed of following items:

1. A Webots *world* file (.wbt) that defines one or several robots and their environment.
The .wbt file does sometimes depend on external PROTO files (.proto) and textures.
2. One or several controller programs for the above robots (in C/C++/Java/Python/MATLAB).
3. An optional physics plugin that can be used to modify Webots regular physics behavior (in C/C++).

### What is a World?

A world, in Webots, is a 3D description of the properties of robots and of their environment.
It contains a description of every object: position, orientation, geometry, appearance (like color or brightness), physical properties, type of object, etc.
Worlds are organized as hierarchical structures where objects can contain other objects (like in VRML97).
For example, a robot can contain two wheels, a distance sensor and a joint which itself contains a camera, etc.
A world file doesn't contain the controller code of the robots; it only specifies the name of the controller that is required for each robot.
Worlds are saved in ".wbt" files.
The ".wbt" files are stored in the "worlds" subdirectory of each Webots project.

### What is a Controller?

A controller is a computer program that controls a robot specified in a world file.
Controllers can be written in any of the programming languages supported by Webots: C, C++, Java, Python or MATLAB.
When a simulation starts, Webots launches the specified controllers, each as a separate process, and it associates the controller processes with the simulated robots.
Note that several robots can use the same controller code, however a distinct process will be launched for each robot.

Some programming languages need to be compiled (C and C++) other languages need to be interpreted (Python and MATLAB) and some need to be both compiled and interpreted (Java).
For example, C and C++ controllers are compiled to platform-dependent binary executables (for example ".exe" under Windows).
Python and MATLAB controllers are interpreted by the corresponding run-time systems (which must be installed).
Java controller need to be compiled to byte code (".class" files or ".jar") and then interpreted by a Java Virtual Machine.

The source files and binary files of each controller are stored together in a controller directory.
A controller directory is placed in the "controllers" subdirectory of each Webots project.

### What is a Supervisor Controller?

The [Supervisor](../reference/supervisor.md) controller is the controller of a [Robot](../reference/robot.md) whose `supervisor` field is set to `TRUE`, it can execute operations that can normally only be carried out by a human operator and not by a real robot.
The [Supervisor](../reference/supervisor.md) controller can be written in any of the above mentioned programming languages.
However, in contrast with a regular [Robot](../reference/robot.md) controller, the [Supervisor](../reference/supervisor.md) controller will have access to privileged operations.
The privileged operations include simulation control, for example, moving the robots to a random position, making a video capture of the simulation, etc.
