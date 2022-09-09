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
For example, it may run it within a debugging environment, like *gdb*, a command line tool like *Python shell*, or within some Integrated Development Environment (IDE), such as *Visual C++*, *Eclipse* or *PyCharm*.
Also, the standard output and error streams (`stdout` and `stderr`) remain under the user control and are not sent to the Webots console.
It is even possible to read the standard input stream (`stdin`) like with any standard program.
Moreover, starting external controllers remotely allows to run Webots on a different machine than the controller, which can be useful if the specifications required by the two processes are very different.

> **Note**: If the `robot.synchronization` field is set to `TRUE` Webots will wait for the extern controller to be launched, otherwise the simulation will run whether the controller is started or not.

## Environment Variables

In order to be able to run an extern Webots controller, a number of environment variables should be set or extended.
Please refer to the documentation of your operating system to set environment variables.

Webots environment variables needed by extern controllers:

%tab-component "os"

%tab "Windows"
| Environment Variable                               | Typical Value                                                             |
|----------------------------------------------------|---------------------------------------------------------------------------|
| WEBOTS\_HOME                                       | `C:\Program Files\Webots`                                                 |
| Path (all controllers)                             | add `%WEBOTS_HOME%\lib\controller`                                        |
| Path (all controllers except Python >= 3.8)        | add `%WEBOTS_HOME%\msys64\mingw64\bin`                                    |
| Path (for C++, Python < 3.8, and Java controllers) | add `%WEBOTS_HOME%\msys64\mingw64\bin\cpp`                                |
| PYTHONPATH (for Python)                            | add `${WEBOTS_HOME}\lib\controller\python3X`                              |
| PYTHONIOENCODING (for Python)                      | `UTF-8`                                                                   |
| WEBOTS\_PROJECT (for MATLAB)                       | `C:\Users\MyUsername\my_folder\my_webots_project`                         |
| WEBOTS\_CONTROLLER\_NAME (for MATLAB)              | `my_robot_controller`                                                     |
| WEBOTS\_VERSION (for MATLAB)                       | `{{ webots.version.full }}`                                               |

&nbsp;

**Python**: Setting the correct python version:
When setting or extending the `PYTHONPATH` environment variable, make sure to replace the `X` in `python3X` with your python version.
This can be found by typing `python --version` in the command line.
If for example the answer is `Python 3.8.10`, `python3X` should be `python38`.

&nbsp;

**MATLAB**: Here is an example of what you should enter in the MATLAB console:

```matlab
>> setenv('WEBOTS_PROJECT', 'C:\Users\MyUsername\my_folder\my_webots_project')
>> setenv('WEBOTS_CONTROLLER_NAME', 'my_robot_controller')
>> setenv('WEBOTS_VERSION', '{{ webots.version.full }}')
>> cd(getenv('WEBOTS_HOME'))
>> cd('lib/controller/matlab')
>> launcher
```

&nbsp;

**Java**: You should add the following options to the `java` command line for launching the Java controller:
- `-classpath $WEBOTS_HOME\lib\controller\java\Controller.jar:$WEBOTS_HOME\my_project\controllers\MyController\`
- `-Djava.library.path=$WEBOTS_HOME\lib\controller\java`

For example to launch the `Driver` Java controller, type:
```bash
java -classpath $WEBOTS_HOME\lib\controller\java\Controller.jar:$WEBOTS_HOME\projects\languages\java\controllers\Driver\ -Djava.library.path=$WEBOTS_HOME\lib\controller\java Driver
```

%tab-end

%tab "Linux"

| Environment Variable                                  | Typical Value                                    |
|-------------------------------------------------------|--------------------------------------------------|
| WEBOTS\_HOME                                          | `/usr/local/webots`                              |
| LD\_LIBRARY\_PATH (not needed for Python controllers) | add `${WEBOTS_HOME}/lib/controller`              |
| PYTHONPATH (for Python)                               | add `${WEBOTS_HOME}/lib/controller/python3X`     |
| PYTHONIOENCODING (for Python)                         | `UTF-8`                                          |
| WEBOTS\_PROJECT (for MATLAB)                          | `/home/my_username/my_folder/my_webots_project`  |
| WEBOTS\_CONTROLLER\_NAME (for MATLAB)                 | `my_robot_controller`                            |
| WEBOTS\_VERSION (for MATLAB)                          | `{{ webots.version.full }}`                      |

&nbsp;

**Python**: Setting the correct python version:
When setting or extending the `PYTHONPATH` environment variable, make sure to replace the `X` in `python3X` with your python version.
This can be found by typing `python3 --version` in the terminal.
If for example the answer is `Python 3.8.10`, `python3X` should be `python38`.

&nbsp;

**MATLAB**: Here is an example of what you should enter in the MATLAB console:

```matlab
>> setenv('WEBOTS_PROJECT','/home/my_username/my_folder/my_webots_project')
>> setenv('WEBOTS_CONTROLLER_NAME', 'my_robot_controller')
>> setenv('WEBOTS_VERSION', '{{ webots.version.full }}')
>> cd(getenv('WEBOTS_HOME'))
>> cd('lib/controller/matlab')
>> launcher
```

&nbsp;

**Java**: You should add the following options to the `java` command line for launching the Java controller:
- `-classpath $WEBOTS_HOME/lib/controller/java/Controller.jar:$WEBOTS_HOME/my_project/controllers/MyController/`
- `-Djava.library.path=${WEBOTS_HOME}/lib/controller/java`

For example to launch the `Driver` Java controller, type:
```bash
java -classpath $WEBOTS_HOME/lib/controller/java/Controller.jar:$WEBOTS_HOME/projects/languages/java/controllers/Driver/ -Djava.library.path=$WEBOTS_HOME/lib/controller/java Driver
```

%tab-end

%tab "macOS"

| Environment Variable                                  | Typical Value                                              |
|-------------------------------------------------------|------------------------------------------------------------|
| WEBOTS\_HOME                                          | `/Applications/Webots.app`                                 |
| DYLD\_LIBRARY\_PATH                                   | add `${WEBOTS_HOME}/Contents/lib/controller`               |
| PYTHONPATH (for the official python.org Python)       | add `${WEBOTS_HOME}/Contents/lib/controller/python3X`      |
| PYTHONPATH (for the Homebrew Python)                  | add `${WEBOTS_HOME}/Contents/lib/controller/python3X_brew` |
| PYTHONIOENCODING (for Python)                         | `UTF-8`                                                    |
| WEBOTS\_PROJECT (for MATLAB)                          | `/Users/my_username/my_folder/my_webots_project`           |
| WEBOTS\_CONTROLLER\_NAME (for MATLAB)                 | `my_robot_controller`                                      |
| WEBOTS\_VERSION (for MATLAB)                          | `{{ webots.version.full }}`                                |

&nbsp;

**Python**: Setting the correct python version:
When setting or extending the `PYTHONPATH` environment variable, make sure to replace the `X` in `python3X` with your python version.
This can be found by typing `python3 --version` in the terminal.
If for example the answer is `Python 3.8.10`, `python3X` should be `python38`.

&nbsp;

**MATLAB**: Here is an example of what you should enter in the MATLAB console:

```matlab
>> setenv('WEBOTS_PROJECT','/Users/my_username/my_folder/my_webots_project')
>> setenv('WEBOTS_CONTROLLER_NAME', 'my_robot_controller')
>> setenv('WEBOTS_VERSION', '{{ webots.version.full }}')
>> cd(getenv('WEBOTS_HOME'))
>> cd('lib/controller/matlab')
>> launcher
```

&nbsp;

**Java**: You should add the following options to the `java` command line for launching the Java controller:
- `-classpath $WEBOTS_HOME/lib/controller/java/Controller.jar:$WEBOTS_HOME/my_project/controllers/MyController/`
- `-Djava.library.path=${WEBOTS_HOME}/lib/controller/java`

For example to launch the `Driver` Java controller, type:
```bash
java -classpath $WEBOTS_HOME/lib/controller/java/Controller.jar:$WEBOTS_HOME/projects/languages/java/controllers/Driver/ -Djava.library.path=$WEBOTS_HOME/lib/controller/java Driver
```

%tab-end

%end

&nbsp;

Also, the [runtime.ini](controller-programming.md#environment-variables) file located in the controller folder (if any) is ignored while starting an extern controller.
Therefore it may be needed to setup manually some extra environment variables which are defined in this file, like for example adding more paths in `PYTHONPATH`.

For convenience, it is also possible to set some environment variables programmatically in your controller program as the very first statements before initializing the Webots controller API.

&nbsp;

**C/C++**: You can set environment variables using `putenv("VARIABLE=VALUE")`.
It is not recommended to use `setenv()` as this function is not available on Windows.

&nbsp;

**Python**: You can set environment variables using `os.environ["VARIABLE"] = "VALUE"`.

&nbsp;

**Java**: It's a little bit [hacky and difficult, but doable](https://stackoverflow.com/a/7201825/810268) to set environment variables in Java.

&nbsp;

**MATLAB**: You can set environment variables using `setenv('VARIABLE', 'VALUE')`.

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

### Notes about the WEBOTS\_CONTROLLER\_URL Environment Variable

`WEBOTS_CONTROLLER_URL` can be left unset and the controller will connect to the only extern robot of a local Webots instance.
If this instance has several extern robots, Webots will refuse the connection and the controller will quit, displaying an error.
For remote connections, it is mandatory to specify the complete URL for the controller to find the Webots instance, but the `/<robot_name>` URL path can be left blank if only one robot has an `<extern>` controller.
If this instance has several extern robots, Webots will refuse the connection and the controller will quit, displaying an error.
If the robot name in the `WEBOTS_CONTROLLER_URL` variable contains special characters, they should be [percent encoded](https://en.wikipedia.org/wiki/Percent-encoding).
Finally, the `WEBOTS_CONTROLLER_URL` environment variable can be set inside the controller program, before calling the `wb_robot_init()` function.

&nbsp;

The following table summarizes the possible values of the `WEBOTS_CONTROLLER_URL` environment variable:

&nbsp;

| `WEBOTS_CONTROLLER_URL` value              | Typical Use Case                                                             |
|--------------------------------------------|------------------------------------------------------------------------------|
| (not defined or empty)                     | Single local instance of Webots running a single extern robot controller     |
| `<robot_name>`                             | Single local instance of Webots running multiple extern robot controllers    |
| `ipc://<port>`                             | Multiple local instances of Webots running a single extern robot controller  |
| `ipc://<port>/<robot_name>`                | Multiple local instances of Webots running multiple extern robot controllers |
| `tcp://<ip_address>:<port>`                | Remote instance(s) of Webots running a single extern robot controller        |
| `tcp://<ip_address>:<port>/<robot_name>`   | Remote instance(s) of Webots running multiple extern robot controllers       |

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
