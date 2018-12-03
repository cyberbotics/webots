## Linux

### Window Refresh

It may happen that the main window of Webots is not refreshed properly and appears blank at startup or upon resizing or maximization.
This is caused by a conflict between the Compiz window manager and OpenGL.
Simply disabling the operating system's visual effects should fix such a problem.
You can easily disable them using some tools like *Compiz Config Settings Manager* or *Unity Tweak Tool*.

### "ssh -x"

There are known issues about running Webots over a `ssh -x` (x-tunneling) connection.
This problem is not specific to Webots but to most GLX (OpenGL on the X Window system) applications that use complex OpenGL graphics.
We think this is caused by incomplete or defective implementation of the GLX support in the graphics drivers on Linux.
It may help to run the `ssh -x` tunnel across two computers with the same graphics hardware, e.g., both NVIDIA or both AMD.
It also usually works to use Mesa OpenGL on both sides of the `ssh -x` tunnel, however this solution is extremely slow.
