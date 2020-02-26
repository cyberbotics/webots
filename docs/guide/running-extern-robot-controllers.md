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

> **Note**: If the `robot.synchronization` field is set to `TRUE` Webots will wait for the extern controller to be launched, otherwise the simulation will run whether the controller is started or not.

## Environment Variables

In order to be able to run an extern Webots controller, a number of environment variables should be set or extended.
Please refer to the documentation of your operating system to set environment variables.

Generic Webots environment variables needed for all the controller languages:

%tab-component "os"

%tab "Windows"
| Environment Variable     | Typical Value                                                               |
|--------------------------|-----------------------------------------------------------------------------|
| WEBOTS\_HOME             | `C:\Program Files\Webots`                                                   |
| PATH                     | add `${WEBOTS_HOME}\lib\controller` and `${WEBOTS_HOME}\msys64\mingw64\bin` |

%tab-end

%tab "Linux"

| Environment Variable     | Typical Value                                    |
|--------------------------|--------------------------------------------------|
| WEBOTS\_HOME             | `/usr/local/webots`                              |
| LD\_LIBRARY\_PATH        | add `${WEBOTS_HOME}/lib/controller`              |

%tab-end

%tab "macOs"

| Environment Variable     | Typical Value                                    |
|--------------------------|--------------------------------------------------|
| WEBOTS\_HOME             | `/Applications/Webots.app`                       |
| DYLD\_LIBRARY\_PATH      | add `${WEBOTS_HOME}/lib/controller`              |

%tab-end

%end

Specific setup depending on the controller language:


%tab-component "language"

%tab "C"

No specific setup is needed.

%tab-end

%tab "C++"

No specific setup is needed.

%tab-end

%tab "Python"
| Version      | Environment Variable     | Typical Value                                    |
|--------------|--------------------------|--------------------------------------------------|
| Python 2.7   | PYTHONPATH               | add `${WEBOTS_HOME}/lib/controller/python27`     |
| Python 3.X   | PYTHONPATH               | add `${WEBOTS_HOME}/lib/controller/python3X`     |
| all          | PYTHONIOENCODING         | `UTF-8`                                          |
%tab-end

%tab "Java"

Add the `-Djava.library.path=${WEBOTS_HOME}/lib/controller/java` option to the `java` command line launching the Java controller.

%tab-end


%tab "MATLAB"

| Environment Variable     | Typical Value                                    |
|--------------------------|--------------------------------------------------|
| WEBOTS\_PROJECT          | `/my_folder/my_webots_project`                   |
| WEBOTS\_CONTROLLER\_NAME | `my_robot_controller.m`                          |
| WEBOTS\_VERSION          | `R2019a-rev1`                                    |

%tab-end

%end

## Setup

Different use cases are detailed here from the most simple to the most complex:

### Single Simulation and Single Extern Robot Controller

You are running a single Webots simulation simultaneously on the same machine and this simulation has only one robot that you want to control from an extern controller.
In this case, you simply need to set the `controller` field of this robot to `<extern>` and to launch the controller program from a console or from your favorite IDE.

### Single Simulation and Multiple Extern Robot Controllers

You are running a single Webots simulation simultaneously on the same machine and this simulation has several robots that you want to control from extern controllers.
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

### Running Extern Robot Controller with the Snap Version of Webots

In order to compile and execute extern controllers, the following environment variables should be set:
```
export WEBOTS_HOME=/snap/webots/current/usr/share/webots
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller
```

Additionally, on Ubuntu 16.04 the following environment variables should be set:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/snap/webots/current/usr/lib/x86_64-linux-gnu
export LD_PRELOAD=/snap/webots/current/usr/lib/x86_64-linux-gnu/libz.so
```

Because of the snap sand-boxing system, Webots has to use a special temporary folder to share information with robot controllers.
When you launch the snap version of Webots, the launcher computes the `WEBOTS_TMPDIR` environment variable if it is not already set.
This variable is computed from the `SNAP_USER_COMMON` environment variable which typically points to `/home/username/snap/webots/common`, a folder accessible by both Webots and your own programs.
Similarly, the libController will automatically check this folder and its contents to determine if it should use it to communicate with Webots.
It is recommended that you do not override this `WEBOTS_TMPDIR` environment variable, unless you want to experiment a different mechanism.


## Example Usage

1. Open for example the "WEBOTS\_HOME/projects/robots/softbank/nao/worlds/nao_demo.wbt" world file.
2. If the simulation was running, stop it and revert it.
3. Then, open the Nao node in the scene tree and change its controller field from `nao_demo` to `<extern>`.
4. Save the simulation, restart it and run it.
5. Open the "WEBOTS\_HOME/projects/robots/softbank/nao/controllers/nao_demo" folder from a terminal.
6. Setup environment variables needed for a C/C++ controller as explained in the above section.
7. Start the `nao_demo` controller manually from the terminal.
8. You should see the Nao robot moving in the simulation, controlled by the `nao_demo` program you just started.
