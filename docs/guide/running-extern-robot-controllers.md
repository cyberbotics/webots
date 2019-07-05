# Running Extern Robot Controllers

This chapter describes extern robot controllers and how to use them.

## Introduction

Normally, Webots launches automatically the robot controller specified in the `controller` field of each [Robot](../reference/robot.md) node.
However, if this field is set to `<extern>`, no controller is launched and the robot will behave like if its `controller` field was an empty string, that is, the robot will not be controlled.
But as soon as a Webots controller is launched manually on the same computer, it will attempt to connect to this `<extern>` robot controller in order to control this robot.

## Usefulness

Running an extern robot controller requires that the controller is launched manually.
This may seem inconvenient, but in several cases, it turns out to be very useful, because the user has full control over the controller process.
For example, it may run it within a debugging environment, like *gdb*, a command line tool like *Python shell*, or within some Integrated Development Environment (IDE), such as *Visual C++*, *Eclipse* or *PyCharm*.
Also, the standard output and error streams (`stdout` and `stderr`) remain under the user control and are not sent to the Webots console.
It is even possible to read the standard input stream (`stdin`) like with any standard program.

## Environment Variables

In order to be able to run an extern Webots controller, a number of environment variables should be set or extended.
Please refer to the documentation of your operating system to set environment variables.

| Operating System | Controller Language  | Environment Variable     | Typical Value                                    |
|------------------|----------------------|--------------------------|--------------------------------------------------|
| Windows          | all                  | WEBOTS\_HOME             | `C:\Program Files\Webots`                        |
| Linux            | all                  | WEBOTS\_HOME             | `/usr/local/webots`                              |
| macOS            | all                  | WEBOTS\_HOME             | `/Applications/Webots`                           |
| Windows          | all                  | PATH                     | add `${WEBOTS_HOME}\msys64\mingw64\bin`          |
| Linux            | all                  | LD\_LIBRAY\_PATH         | add `${WEBOTS_HOME}/lib`                         |
| macOS            | all                  | DYLD\_LIBRARY\_PATH      | add `${WEBOTS_HOME}/lib`                         |
| all              | Python 2.7           | PYTHONPATH               | add `${WEBOTS_HOME}/lib/python27`                |
| all              | Python 3.7           | PYTHONPATH               | add `${WEBOTS_HOME}/lib/python37`                |
| all              | Python               | PYTHONIOENCODING         | `UTF-8`                                          |
| all              | MATLAB               | WEBOTS\_PROJECT          | `/my_folder/my_webots_project`                   |
| all              | MATLAB               | WEBOTS\_CONTROLLER\_NAME | `my_robot_controller.m`                          |
| all              | MATLAB               | WEBOTS\_VERSION          | `R2019a-rev1`                                    |

If a C/C++ controller is launched from a Terminal running the bash shell, it is sufficient to issue the following command to set the path to the Controller library before launching the controller:
- On Windows/MSYS2, type: `export PATH=${PATH}:/C/Program\ Files/Webots/msys64/mingw64/bin`.
- On Linux, type `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/webots/lib`.
- On macOS, type `export DYLD_LIBRARY_PATH=${DYLD_LIBRARY_PATH}:/Applications/Webots/lib`.

For a Java controller, the `-Djava.library.path=${WEBOTS_HOME}/lib/java` option should be added to the `java` command line launching the Java controller.

## Setup

Different use cases are detailed here from the most simple to the most complex:

### Single Simulation and Single Extern Robot Controller

You are running a single Webots simulation simultaneously on the same machine and this simulation has only one robot that you want to control from an external controller.
In this case, you simply need to set the `controller` field of this robot to `<extern>` and to launch the controller program from a console or from your favorite IDE.

### Single Simulation and Multiple Extern Robot Controllers

You are running a single Webots simulation simultaneously on the same machine and this simulation has several robots that you want to control from external controllers.
In this case, for each robot that you want to control externally, you should set their `controller` field to `<extern>`.
Then, in the environment from which you are going to launch the extern controller, you should define an environment variable named `WEBOTS_ROBOT_NAME` and set it to match the `name` field of the [Robot](../reference/robot.md) node you want to control.
Once this environment variable is set, you can launch your controller and it will connect to the extern robot whose `name` matches the one provided in the environment variable.
You can repeat this for the other controllers, e.g., set a different value to the `WEBOTS_ROBOT_NAME` environment variable before starting a new controller, so that it will connect to a different robot.

> **Note**: if the `WEBOTS_ROBOT_NAME` is not set, the controller will connect to the first extern robot found which is not already connected to an extern controller.

### Multiple Concurrent Simulations

If you are running multiple simulations simultaneously on the same machine, then you need to indicate to your controller to which instance of Webots it should try to connect.
This can be achieved by setting an environment variable named `WEBOTS_PID` with the PID (Process ID) of the running Webots instance to which you want to connect your controller.
If that simulation has more than one extern controller, you may also set the `WEBOTS_ROBOT_NAME` environment variable to specify the robot to which your controller should connect.

> **Note**: the environment variables can be set inside the controller program, before calling the `wb_robot_init()` function.

## Example Usage

1. Open for example the "WEBOTS\_HOME/projects/robots/softbank/nao/worlds/nao_demo.wbt" world file.
2. If the simulation was running, stop it and revert it.
3. Then, open the Nao node in the scene tree and change its controller field from `nao_demo` to `<extern>`.
4. Save the simulation, restart it and run it.
5. Open the "WEBOTS\_HOME/projects/robots/softbank/nao/controllers/nao_demo" folder from a terminal.
6. Setup environment variables needed for a C/C++ controller as explained in the above section.
7. Start the `nao_demo` controller manually from the terminal.
8. You should see the Nao robot moving in the simulation, controlled by the `nao_demo` program you just started.
