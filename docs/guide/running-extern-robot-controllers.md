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
It must be used to start any extern controller file.
Compatible file types are listed below:
* **Executables**: no extension on Linux/macOS and `.exe` on Windows.
* **Python**: `.py`.
* **Java**: `.jar` and `.class`.
* **Matlab**: `.m`.

The `WEBOTS_HOME` environment variable must be set to the installation folder of Webots.
For example:

```bash
export WEBOTS_HOME=/home/username/webots
```

If you are using the snap version of Webots, please refer to the corresponding section: [Running Extern Robot Controller with the Snap Version of Webots](#running-extern-robot-controller-with-the-snap-version-of-webots).

The following command line should be used to start a controller:

%tab-component "os"

%tab "Windows"

```bash
webots-controller.exe [options] path/to/controller/file [controller-args]
```

%tab-end

%tab "Linux"

```bash
$WEBOTS_HOME/webots-controller [options] path/to/controller/file [controller-args]
```

%tab-end

%tab "macOS"

```bash
$WEBOTS_HOME/Contents/MacOS/webots-controller [options] path/to/controller/file [controller-args]
```

%tab-end

%end

> **Note**: The controller file path can be absolute or relative to the directory from which the launcher is started.

### Options

The following options are available when starting an extern controller with the launcher.
Concrete use cases are discussed in the [Setup](#setup) section.

```
  --help
    Display this help message and exit.

  --protocol=<ipc|tcp>
    Define the protocol to use to communicate between the controller and Webots.
    `ipc` is used by default.
    `ipc` should be used when Webots is running on the same machine as the extern controller.
    `tcp` should be used when connecting to a remote instance of Webots.

  --ip-address=<ip-address>
    The IP address of the remote machine on which the Webots instance is running.
    This option should only be used with the `tcp` protocol (i.e. remote controllers).

  --port=<port>
    Define the port to which the controller should connect. 1234 is used by default, as it is the default port for Webots.
    This setting allows you to connect to a specific instance of Webots if there are multiple instances running on the target machine.
    The port of a Webots instance can be set at its launch.

  --robot-name=<robot-name>
    Target a specific robot by specifying its name in case multiple robots wait for an extern controller in the Webots instance.

  --interactive
    Launch MATLAB in interactive debugging mode.
    See https://cyberbotics.com/doc/guide/matlab#using-the-matlab-desktop for more information.

  --matlab-path=<matlab-path>
    For MATLAB controllers, this option allows to specify the path to the executable of a specific MATLAB version.
    By default, the launcher checks in the default MATLAB installation folder.

  --stdout-redirect
    Redirect the stdout of the controller to the Webots console.

  --stderr-redirect
    Redirect the stderr of the controller to the Webots console.
```

## Setup

To run a local extern controller, both Webots and the controller should run from the same user account.
This is not needed for a remote extern controller where Webots and the controller run on different machines.
Different use cases are detailed here from the most simple to the most complex:

### Single Simulation and Single Local Extern Robot Controller

You are running a single Webots simulation simultaneously on the same machine and this simulation has only one robot that you want to control from an extern controller.
In this case, you simply need to set the `controller` field of this robot to `<extern>` and to launch the controller with the launcher.
No specific option is needed in this case, as default parameters will automatically target the correct instance of Webots and the single available robot.
Once an extern controller is connected to the robot, any other attempt to connect to that robot will be refused by Webots and the controller attempting to connect will terminate immediately.

### Single Simulation and Multiple Local Extern Robot Controllers

You are running a single Webots simulation simultaneously on the same machine and this simulation has several robots that you want to control from extern controllers.
In this case, for each robot that you want to control externally, you should set their `controller` field to `<extern>`.
Then, you should set the `--robot-name` option of the controller launcher to match the `name` field of the [Robot](../reference/robot.md) node you want to control.
The started controller will connect to the extern robot whose `name` matches the one provided in the command line.
This operation can be repeated in a new terminal for each robot in the simulation.

### Multiple Concurrent Simulations and Single Local Extern Robot Controller

If you are running multiple simulations simultaneously on the same machine, and each simulation has only one robot that you want to control from an extern controller, then you need to indicate to the controller to which instance of Webots it should try to connect.
This can be achieved by setting the `--port` option of the launcher to the TCP port of the target Webots instance (defined with the equivalent `--port` command line option at Webots launch) to which you want to connect your controller.

### Multiple Concurrent Simulations and Multiple Local Extern Robot Controllers

If you are running multiple simulations simultaneously on the same machine, and each simulation has several robots that you want to control from extern controllers, then you need to indicate to each controller to which instance of Webots and to which robot it should try to connect.
To achieve this, simply set the launcher `--port` option to the TCP port of the target Webots instance (set with the equivalent `--port` command line option when launching Webots) and the `--robot-name` option to the name of the target robot to which you want to connect your controller.

### Remote Extern Controllers

`<extern>` controllers can also be started from a remote machine.
In this case, when starting the controller with the launcher, the `--protocol` option should be set to `tcp`.
The `--ip-address` option must be set to the IP address of the remote machine on which the target instance of Webots is running.
If multiple instances of Webots are running on the remote machine, the `--port` option must be set to the TCP port (defined with the `--port` command line option at Webots launch) of the Webots instance to which you want to connect your controller.
Finally, if the target instance contains multiple robots waiting for an extern controller connection, the `--robot-name` option can be set to the name of the robot to which you want to connect your controller.

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

### Running a MATLAB Extern Robot Controller

Matlab controllers can also be started using the launcher.
By default, the new instance of MATLAB will be running in non-interactive ("batch") mode. 
However, by providing the `--interactive` option, this can be overridden, which will cause the full desktop user interface to run. 
See [this page](matlab.md) for more details on how to debug webots controllers using the MATLAB desktop.

Regardless of mode, the launcher will look for the latest installed version of MATLAB in the following locations, depending on the OS:

- **Windows**: C:\Program Files\MATLAB\R20XXx\bin\win64\MATLAB.exe
- **Linux**: /usr/local/MATLAB/R20XXx/bin/matlab
- **MacOS**: /Applications/MATLAB_R20XXx.app

It is also possible to give a custom MATLAB installation path to the launcher, by providing the absolute path to the executable in the `--matlab-path` option.

## Example Usage

1. Set WEBOTS_HOME to the Webots installation directory, for example:

  ```bash
  export WEBOTS_HOME=/usr/local/webots
  ```

2. Open for example the "WEBOTS\_HOME/projects/robots/softbank/nao/worlds/nao_demo.wbt" world file.
3. If the simulation was running, stop it and revert it.
4. Then, open the Nao node in the scene tree and change its controller field from `nao_demo` to `<extern>`.
5. Save the simulation, restart it and run it.
6. Open a terminal and start the `nao_demo` controller with:

  ```bash
  $WEBOTS_HOME/webots-controller $WEBOTS_HOME/projects/robots/softbank/nao/controllers/nao_demo/nao_demo
  ```

  **Note**: If you need to connect to a remote Webots instance, the controller can be started the following way:

  ```bash
  $WEBOTS_HOME/webots-controller --protocol=tcp --ip-address=127.0.0.1 $WEBOTS_HOME/projects/robots/softbank/nao/controllers/nao_demo/nao_demo
  ```

  Simply replace `127.0.0.1` by the IP address of your remote machine.
7. You should see the Nao robot moving in the simulation, controlled by the `nao_demo` program you just started.
