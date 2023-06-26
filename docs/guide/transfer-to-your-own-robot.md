## Transfer to your own Robot

In mobile robot simulation, it is often useful to transfer the results onto real mobile robots.
Webots was designed with this transfer capability in mind.
The simulation is as realistic as possible, and the programming interface can be ported or interfaced to existing, real robots.
Webots already comprises transfer systems for a number of existing robots including *e-puck*<sup>TM</sup>, *DARwIn-OP*<sup>TM</sup>, *Khepera*<sup>TM</sup> and *Hemisson*<sup>TM</sup>.
This section explains how to develop your own transfer system to your own mobile robot.

Since the simulation is only an approximation of the physics of the real robot, some tuning is always necessary when developing a transfer mechanism for a real robot.
This tuning will affect the simulated model so that it better matches the behavior of the real robot.

### Remote Control

#### Remote Control Overview

Often, the easiest way to transfer your control program to a real robot is to develop a remote control system.
In this case, your control program runs on the computer, but instead of sending commands to and reading sensor data from the simulated robot, it sends commands to and reads sensor data from the real robot.
Developing such a remote control system can be achieved in a very simple manner by writing your own implementation of the Webots API functions as a small library.
For example, you will probably have to implement the `wb_motor_set_velocity` function to send a specific command to the real robot with the wheel speeds as an argument.
This command can be sent to the real robot via the serial port of the PC, or any other PC-robot interface you have.
You will probably need to make some unit conversions, since your robot may not use the same units of measurement as the ones used in Webots.
The same applies for reading sensor values from the real robot.

#### Developing a Remote Control Plugin

Webots already provides some facilities to implement a remote control library and in particular it is possible to develop it as a controller plugin.
Once set in the corresponding field of the [Robot](../reference/robot.md) node, this remote control plugin will be executed automatically when running the controller.
Implementation details are described in [this section](controller-plugin.md#remote-control-plugin).

#### Special Functions

The `wb_robot_init` function must be the first called function.
It performs the controller library's initialization.

The `wb_robot_step` function should be called repeatedly (typically in an infinite loop).
It requests that the simulator performs a simulation step of ms milliseconds; that is, to advance the simulation by this amount of time.

The `wb_robot_cleanup` function should be called at the end of a program in order to leave the controller in a clean fashion.

#### Running your Real Robot

Once linked with your own remote control plugin, you can control your real robot by running the simulation in Webots.
It might be useful to also add a robot window (see [this section](controller-plugin.md#robot-window)) to graphically display specific sensor values, motor commands or a stop button.

Such a remote control system is designed to be implemented in C/C++ as explained in [this section](controller-plugin.md); however, it can also be implemented in other programming languages by creating a wrapper.

### Cross-Compilation

#### Cross-Compilation Overview

Developing a cross-compilation system will allow you to recompile your Webots controller for the embedded processor of your own real robot.
Hence, the source code you wrote for the Webots simulation will be executed on the real robot itself, and there is no need to have a permanent PC connection with the robot as with the remote control system.
This is only possible if the processor on your robot can be programmed respectively in C, C++, Java or Python.
It is not possible for a processor that can be programmed only in assembler or another specific language.
Webots includes the source code of such a cross-compilation system for the e-puck and the Hemisson robot.
Samples are located in the "[WEBOTS\_HOME/projects/robots]({{ url.github_tree }}/projects/robots)" directory.

#### Developing a Custom Library

Unlike the remote control system, the cross-compilation system requires the source code of your Webots controller to be recompiled using the cross-compilation tools specific to your own robot.
You will also need to rewrite the Webots include files to be specific to your own robot.
In simple cases, you can just rewrite the Webots include files you need, as in the "hemisson" example.
In more complex cases, you will also need to write some C source files to be used as a replacement for the Webots "Controller" library, but running on the real robot.
You should then recompile your Webots controller with your robot cross-compilation system and link it with your robot library.
The resulting file should be uploaded onto the real robot for local execution.

#### Examples

Webots supports cross-compilation for several existing commercial robots.
For the *e-puck*<sup>TM</sup> robot, this system is fully integrated in Webots and needs no modification in the code.
For the *Hemisson*<sup>TM</sup> robot, this system needs a few include files to replace the Webots API include files.
For the *Khepera*<sup>TM</sup> robot, a specific C library is used in addition to specific include files.

### Interpreted Language

In some cases, it may be better to implement an interpreted language system.
This is useful if your real robot already uses an interpreted language, like Basic or a graph based control language.
In this case, the transfer is very easy since you can directly transfer the code of your program that will be interpreted to the real robot.
The most difficult part may be to develop a language interpreter in C or Java to be used by your Webots controller for controlling the simulated robot.
Such an interpreted language system was developed for the *Hemisson*<sup>TM</sup> robot with the *BotStudio*<sup>TM</sup> system.
