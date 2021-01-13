## Debugging C/C++ Controllers

### Controller Processes

In the Webots environment, the Webots application and each robot C/C++ controller are executed in distinct operating system processes.
For example, when the "soccer.wbt" world is executed, there is a total of eight processes in memory; one for Webots, six for the six player robots, and one for the supervisor robot.
To debug a C/C++ controller with Microsoft Visual Studio, please see [here](using-your-ide.md#visual-studio).

When a controller process performs an illegal instruction, it is terminated by the operating system while the Webots process and the other controller processes remain active.
Although Webots is still active, the simulation blocks because it waits for data from the terminated controller.
So if you come across a situation where your simulation stops unexpectedly, but the Webots GUI is still responsive, this usually indicates that the controller has crashed .
This can easily be confirmed by listing the active processes at this moment: For example on Linux, type:

```sh
$ ps -e
...
12751 pts/1    00:00:16 webots
13294 pts/1    00:00:00 soccer_player
13296 pts/1    00:00:00 soccer_player
13297 pts/1    00:00:00 soccer_player
13298 pts/1    00:00:00 soccer_player
13299 pts/1    00:00:00 soccer_player
13300 pts/1    00:00:00 soccer_player
13301 pts/1    00:00:00 soccer_supervisor <defunct>
...
```

On macOS, use rather `ps -x` and on Windows use the *Task Manager* for this.
If one of your robot controllers is missing in the list (or appearing as *defunct*) this confirms that it has crashed and therefore blocked the simulation.
In this example the "soccer\_supervisor" has crashed.
Note that the crash of a controller is almost certainly caused by an error in the controller code, because an error in Webots would have caused Webots to crash.
Fortunately, the GNU debugger (`gdb`) can usually help finding the reason of the crash.
The following example assumes that there is a problem with the "soccer\_supervisor" controller and indicates how to proceed with the debugging.

### Using the GNU Debugger with a Controller

On Windows GDB can be installed for example from the MSYS2 environment with the `mingw-w64-x86_64-gdb` package as indicated in the [optional dependencies](https://github.com/cyberbotics/webots/wiki/Windows-Optional-Dependencies) of the [Windows installation instructions](https://github.com/cyberbotics/webots/wiki/Windows-installation).

The first step is to recompile the controller with the `debug` target, in order to add debugging information to the executable file. 
You must recompile the controller directly in a terminal, as the Webots text editor `Build` button will omit debugging information from the build:

```sh
$ make clean
$ make debug
...
```

Once you have recompiled the controller, you will need to ensure the controller of the [Robot](../reference/robot.md) node is set to be [extern](running-extern-robot-controllers.md).
If it is not, this can be set from the scene tree:
Hit the `Pause` and `Reset` buttons, set the `controller` field of the Robot node to `<extern>` and save the world file.
From a terminal, go to the folder containing your controller program and start it with `gdb`:

```sh
$ gdb my_controller
```

In `gdb`, type for example:

```sh
(gdb) break my_controller.c:50
(gdb) run
```

Then, run the Webots simulation, using the `Run` button (you may also use the `Step`, `Real-Time` or `Fast` button).
Your controller program will start controlling the extern robot in Webots.
Once the break point is reached, you will be able to query variables, setup new break points, etc.

Then, the `cont` command will instruct the debugger to resume the execution of the process.
You may also use the `step` function to proceed step-by-step.

The controller's execution can be interrupted at any time (<kbd>ctrl</kbd>-<kbd>C</kbd>), in order to query variables, set up break points, etc.
When a crash occurs, `gdb` prints a diagnostic message similar to this:

```
Program received signal SIGSEGV, Segmentation fault.
[Switching to Thread -1208314144 (LWP 16448)]
0x00cd6dd5 in _IO_str_overflow_internal () from /lib/tls/libc.so.6
```

This indicates the location of the problem.
You can examine the call stack more precisely by using the `where` command of `gdb`.
For example type:

```sh
(gdb) where
#0 0x00cd6dd5 in _IO_str_overflow_internal() from /lib/tls/libc.so.6
#1 0x00cd596f in _IO_default_xsputn_internal() from /lib/tls/libc.so.6
#2 0x00cca9c1 in _IO_padn_internal() from /lib/tls/libc.so.6
#3 0x00cb17ea in vfprintf() from /lib/tls/libc.so.6
#4 0x00ccb9cb in vsprintf() from /lib/tls/libc.so.6
#5 0x00cb8d4b in sprintf() from /lib/tls/libc.so.6
#6 0x08048972 in run(duration=0) at soccer_supervisor.c:106
#7 0x08048b0a in main() at soccer_supervisor.c:140
```

By examining carefully the call stack you can locate the source of the error.
In this example we will assume that the `sprintf` function is OK, because it is in a system library.
Therefore it seems that the problem is caused by an illegal use of the `sprintf` function in the `run` function.
The line 106 of the source file "soccer\_supervisor.c" must be examined closely.
While the controller is still in memory you can query the values of some variables in order to understand what happened.
For example, you can use the `frame` and `print` commands:

```sh
(gdb) frame 6
#6  0x08048953 in run (duration=0) at soccer_supervisor.c:106
106         sprintf(time_string, "%02d:%02d", (int) (time / 60),
 (int) time % 60);
(gdb) print time_string
$1 = 0x0
```

The `frame` command instructs the debugger to select the specified stack frame, and the `print` command prints the current value of an expression.
In this simple example we clearly see that the problem is caused by a NULL (0x0) *time\_string* argument passed to the `sprintf` function.
The next steps are to: 
1. Fix the problem
2. Recompile the controller 
3. Reload the world to give it another try.

Once it works and gives the correct output you can remove the *-g* flag from the Makefile.
