## General FAQ

### How Can I Report a Bug in Webots?

If you can still start Webots, please report the bug by using Webots menu: `Help / Bug report...`.

If Webots cannot start any more, please report the bug here: [http://www.cyberbotics.com/bug](http://www.cyberbotics.com/bug).
Please include a precise description of the problem, the sequence of actions necessary to reproduce the problem.
Do also attach the world file and the controller programs necessary to reproduce it.

Before reporting a bug, please make sure that the problem is actually caused by Webots and not by your controller program.
For example, a crash of the controller process usually indicates a bug in the controller code, not in Webots.
This situation can be identified by these couple of symptoms:

1. Webots GUI is visible and responsive, but the simulation is blocked (simulation time stopped).
2. The controller process has vanished from the *Task Manager* (Windows) or is shown as *&lt;defunct&gt;* when using `ps -e` (Linux/Mac).

### Is It Possible to Use Microsoft Visual Studio to Compile My Controllers?

Yes.
However, you will need to create your own project with all the necessary options.
You will find more detailed instructions on how to do that in [this section](using-visual-studio-with-webots.md).
To create the import libraries (the "\*.lib" files in Visual Studio) from the "\*.dll" files of the lib directory of Webots, please follow the instructions provided with the documentation of your compiler.
