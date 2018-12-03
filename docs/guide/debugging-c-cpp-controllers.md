## Debugging C/C++ Controllers

### Controller Processes

In the Webots environment, the Webots application and each robot C/C++ controller are executed in distinct operating system processes.
For example, when the "soccer.wbt" world is executed, there is a total of eight processes in memory; one for Webots, six for the six player robots, and one for the supervisor.
To debug a C/C++ controller with Microsoft Visual Studio, please see [here](using-visual-studio-with-webots.md).

When a controller process performs an illegal instruction, it is terminated by the operating system while the Webots process and the other controller processes remain active.
Although Webots is still active, the simulation blocks because it waits for data from the terminated controller.
So if you come across a situation where your simulation stops unexpectedly, but the Webots GUI is still responsive, this usually indicates the crash of a controller.
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
If one of your robot controllers is missing in the list (or appearing as *<defunct>*) this confirms that it has crashed and therefore blocked the simulation.
In this example the "soccer\_supervisor" has crashed.
Note that the crash of a controller is almost certainly caused by an error in the controller code, because an error in Webots would have caused Webots to crash.
Fortunately, the GNU debugger (`gdb`) can usually help finding the reason of the crash.
The following example assumes that there is a problem with the "soccer\_supervisor" controller and indicates how to proceed with the debugging.

### Using the GNU Debugger with a Controller

The first step is to recompile the controller code with the *-g* flag, in order to add debugging information to the executable file.
This can be achieved by adding this line to the controller's Makefile:

```makefile
CFLAGS = -g
```

Then, you must recompile the controller, either by using the `Clean` and `Build` buttons of the Webots text editor or directly in a terminal:

```sh
$ make clean
$ make
...
```

Note that, the *-g* flag should now appear in the compilation line.
Once you have recompiled the controller, hit the `Pause` and `Reload` buttons.
This pauses the simulation and reloads the freshly compiled versions of the controller.
Now find the process ID (PID) of the "soccer\_supervisor" process, using `ps -e` (Linux) or `ps -x` (macOS), or using the *Task Manager* (Windows).
The PID is in the left-most column of output of `ps` as shown above.
Then open a terminal and start the debugger by typing:

```sh
$ gdb
...
(gdb) attach PID
...
(gdb) cont
Continuing.
```

Where PID stands for the PID of the "soccer\_supervisor" process.
The `attach` command will attach the debugger to the "soccer\_supervisor" process and interrupt its execution.
Then the `cont` command will instruct the debugger to resume the execution of the process.
On Windows you will need to install the "gdb.exe" file separately and use an MSYS console to achieve this.

Then hit the `Run` button to start the simulation and let it run until the controller crashes again.
The controller's execution can be interrupted at any time (Ctrl-C), in order to query variables, set up break points, etc.
When the crash occurs, `gdb` prints a diagnostic message similar to this:

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
The next steps are to: fix the problem, recompile the controller and reload the world to give it another try.
Once it works correctly you can remove the *-g* flag from the Makefile.
