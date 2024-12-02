## Linux

### "ssh -X"

There are known issues about running Webots over a `ssh -X` (X tunneling) connection.
This problem is not specific to Webots but to most GLX (OpenGL on the X Window system) applications that use complex OpenGL graphics.
We think this is caused by incomplete or defective implementation of the GLX support in the graphics drivers on Linux.
It may help to run the `ssh -X` tunnel across two computers with the same graphics hardware, e.g., both NVIDIA or both AMD.
It also usually works to use Mesa OpenGL on both sides of the `ssh -X` tunnel, however this solution is extremely slow.
