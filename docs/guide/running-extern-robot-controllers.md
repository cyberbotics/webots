# Running Extern Robot Controllers

This chapter describes extern robot controllers and how to use them.

## Introduction

Normally, Webots launches automatically the robot controller specified in the `controller` field of each [Robot](../reference/robot.md) node.
However, if this field is set to `<extern>`, no controller is launched and the robot will behave like if its `controller` field was an empty string, that is, the robot will not be controlled.
But as soon as a Webots controller is launched manually on the same computer, it will attempt to connect to this `<extern>` robot controller in order to control this robot.
It is also possible to connect `<extern>` controllers from remote computers using a TCP connection.

## Usefulness

Running an extern robot controller requires that the controller is launched manually.
This may seem inconvenient, but in several cases, it turns out to be very useful, because the user has full control over the controller process.
For example, it may run it within a debugging environment, like *gdb*, a command line tool like *$ shell*, or within some Integrated Development Environment (IDE), such as *Visual C++*, *Eclipse* or *PyCharm*.
Also, the standard output and error streams (`stdout` and `stderr`) remain under the user control and are not sent to the Webots console.
It is even possible to read the standard input stream (`stdin`) like with any standard program.
Moreover, starting external controllers remotely allows to run Webots on a different machine than the controller, which can be useful if the specifications required by the two processes are very different.

> **Note**: If the `robot.synchronization` field is set to `TRUE` Webots will wait for the extern controller to be launched, otherwise the simulation will run whether the controller is started or not.

## Launcher

Webots is distributed with a controller launcher.
It must be used to start any compatible controller file listed below:
* **C and C++ binaries**: no extension on Linux/macOS and `.exe` on Windows.
* **Python**: `.py`.
* **Java**: `.jar` and `.class`.
* **Matlab**: `.m`.

The following command line should be used to start a controller:

%tab-component "os"

%tab "Windows"

```bash
$WEBOTS_HOME\webots-controller.exe [options] [path\to\controller\file]
```

%tab-end

%tab "Linux"

```bash
$WEBOTS_HOME/webots-controller [options] [path/to/controller/file]
```

%tab-end

%tab "macOS"

```bash
$WEBOTS_HOME/webots-controller [options] [path/to/controller/file]
```

%tab-end

%end

> **Note**: The controller file path can be absolute or relative to the directory from which the launcher is started.

### Options

The following options are available when starting an extern controller with the launcher.
Concrete use cases are discussed in the next section [Setup](#setup).

```
  --help
    Display this help message and exit.

  --protocol=<ipc|tcp>
    Define the protocol to use to communicate between the controller and Webots. `ipc` is used by default. `ipc` should be used when Webots is running on the same machine as the extern controller. `tcp` should be used when connecting to a remote instance of Webots.

  --ip-address=<ip-address>
    The IP address of the remote machine on which the Webots instance is running. This option should only be used with the `tcp` protocol (i.e. remote controllers).

  --port=<port>
    Define the port to which the controller should connect. 1234 is used by default, as it is the default port for Webots. This setting allows you to connect to a specific instance of Webots if there are multiple instances running on the target machine. The port of a Webots instance can be set at its launch.

  --robot-name=<robot-name>
    Target a specific robot by specifiyng its name in case multiple robots wait for an extern controller in the Webots instance.

  --matlab-path=<matlab-path>
    For MATLAB controllers, this option allows to specify the path to the executable of a specific MATLAB version. By default, the launcher checks in the default MATLAB installation folder. See [Using Matlab](using-matlab.md) for more information.

  --stdout-redirect
    Redirect the stdout of the controllers to the Webots console.

  --stderr-redirect
    Redirect the stderr of the controllers to the Webots console.
```

## Setup

To run a local extern controller, both Webots and the controller should run from the same user account.
This is not needed for a remote extern controller where Webots and the controller run on different machines.
Different use cases are detailed here from the most simple to the most complex:

### Single Simulation and Single Local Extern Robot Controller

You are running a single Webots simulation simultaneously on the same machine and this simulation has only one robot that you want to control from an extern controller.
In this case, you simply need to set the `controller` field of this robot to `<extern>` and to launch the controller program from a console or from your favorite IDE.
Once an extern controller is connected to a robot, any other attempt to connect to that robot will be refused by Webots and the controller attempting to connect will terminate immediately.

### Single Simulation and Multiple Local Extern Robot Controllers

You are running a single Webots simulation simultaneously on the same machine and this simulation has several robots that you want to control from extern controllers.
In this case, for each robot that you want to control externally, you should set their `controller` field to `<extern>`.
Then, in the environment from which you are going to launch the extern controller, you should define an environment variable named `WEBOTS_CONTROLLER_URL` and set it to match the `name` field of the [Robot](../reference/robot.md) node you want to control.
Once this environment variable is set, you can launch your controller and it will connect to the extern robot whose `name` matches the one provided in the environment variable.
You can repeat this for the other controllers, e.g., set a different value to the `WEBOTS_CONTROLLER_URL` environment variable before starting a new controller, so that it will connect to a different robot.

### Multiple Concurrent Simulations and Single Local Extern Robot Controller

If you are running multiple simulations simultaneously on the same machine, and each simulation has only one robot that you want to control from an extern controller, then you need to indicate to the controller to which instance of Webots it should try to connect.
This can be achieved by setting the `WEBOTS_CONTROLLER_URL` environment variable to the following value: `ipc://<port>` where `<port>` is the TCP port (defined in the `--port` command line option) of the Webots instance to which you want to connect your controller.

### Multiple Concurrent Simulations and Multiple Local Extern Robot Controllers

If you are running multiple simulations simultaneously on the same machine, and each simulation has several robots that you want to control from extern controllers, then you need to indicate to each controller to which instance of Webots and to which robot it should try to connect.
This can be achieved by setting the `WEBOTS_CONTROLLER_URL` environment variable to the following value: `ipc://<port>/<robot_name>` where `<port>` is the TCP port (defined in the `--port` command line option) of the target Webots instance and `<robot_name>` is the name of the robot to which you want to connect your controller.

### Remote Extern Controllers

`<extern>` controllers can also be started from a remote machine.
In this case, on the computer running the controller, the `WEBOTS_CONTROLLER_URL` environment variable should be set to the following value: `tcp://<ip_address>:<port>/<robot_name>`.
`<ip_address>` corresponds to the IP address of the machine running Webots.
`<port>` is the TCP port (defined in the `--port` command line option) of the Webots instance to which you want to connect your controller.
Finally, `<robot_name>` is the name of the robot to which you want to connect your controller.
Note that the URL path `/<robot_name>` can be left blank and the controller will connect to the only robot with an `<extern>` controller.

It is possible to restrict the IP addresses that can connect to a Webots instance.
To do this, the allowed IP addresses can be added in the format `X.X.X.X` in the Webots preferences in the `Network` tab.
It is also possible to allow a range of addresses using a subnet mask in [CIDR](https://en.wikipedia.org/wiki/Classless_Inter-Domain_Routing) notation, with the following format: `X.X.X.X/<netmask>`.
Note that if the list is left empty, all incoming connections are allowed.

### Running Extern Robot Controller with the Snap Version of Webots

In order to compile and execute extern controllers, the following environment variables should be set:
```
export WEBOTS_HOME=/snap/webots/current/usr/share/webots
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller
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
