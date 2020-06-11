## Troubleshooting

Unlike the controller code, the physics plugin code is executed in the same process and memory space as the Webots application.
Therefore, a segmentation fault in the physics plugin code will cause the termination of the Webots application.
Webots termination is often misinterpreted by users who believe that Webots is unstable, while the error is actually in the user's plugin code.
For that reason, it is important to precisely locate the crash before reporting a bug to Cyberbotics Ltd.

The following are some debugging hints that should help you find the exact location of a crash using `gdb` (the GNU Debugger).
The first step is to recompile the physics plugin with the *-g* flag, in order to add debugging information to the compiled plugin.
This can be achieved by adding this line to the plugin's "Makefile":

```makefile
CFLAGS=-g
```

Then you must rebuild the plugin using Webots Text Editor or using these commands in a terminal:

```sh
$ make clean
$ make
```

Make sure that the *-g* flag appears in the compilation line.
Once you have rebuilt the plugin, you can quit Webots, and restart it using `gdb` in a terminal, like this:

```sh
$ cd /usr/local/webots/bin
$ export LD_LIBRARY_PATH=/usr/local/webots/lib/webots:$LD_LIBRARY_PATH
$ gdb ./webots-bin
(gdb) run
```

Note that the above path corresponds to a default Webots installation on Linux: the actual path might be different depending on your specific system or installation.
The LD\_LIBRARY\_PATH environment variable indicates where to find the shared libraries that will be required by Webots.

When Webots window appears, run the simulation until it crashes, or make it crash by some manipulations if necessary.
If the plugin crahes due to a segmentation fault, `gdb` should print an error message similar to this:

```
Program received signal SIGSEGV, Segmentation fault.
[Switching to Thread -1208154400 (LWP 30524)]
0x001f5c7e in webots_physics_init (w=0xa6f8060, s=0xa6f80e0, j=0xa6f5c00)
at my_physics.c:50
50            float pos = position[0] + position[1] + position[2];
...
```

This indicates precisely the file name and line number where the problem occurred.
If the indicated file name corresponds to one of the plugin source files, then the error is located in the plugin code.
You can examine the call stack more precisely by using the `where` or the `bt` command of `gdb`.
For example:

```
(gdb) where
#0  0x001f5c7e in webots_physics_init (w=0xa6f8060, s=0xa6f80e0, j=0xa6f5c00)
at my_physics.c:50
#1  0x081a96b3 in A_PhysicsPlugin::init ()
#2  0x081e304b in A_World::preprocess ()
#3  0x081db3a6 in A_View::render ()
#4  0x081db3f3 in A_View::onPaint ()
#5  0x084de679 in wxEvtHandler::ProcessEventIfMatches ()
#6  0x084de8be in wxEventHashTable::HandleEvent ()
#7  0x084def90 in wxEvtHandler::ProcessEvent ()
#8  0x084ea393 in wxGLContext::SetCurrent ()
...
```

In this example you see that the error is located in the plugin's `webots_physics_init` function.
If the error is reported in an unknown function (and if the line number and file name are not displayed), then the crash may have occurred in Webots, or possibly in a library used by your plugin.
