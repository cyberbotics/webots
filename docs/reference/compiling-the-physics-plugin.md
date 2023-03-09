## Compiling the Physics Plugin

When a plugin is created using the **File / New / New Physics Plugin...** menu item, Webots will automatically add a suitable ".c" or ".cpp" source file and a Makefile to the plugin's directory.
Your plugin can be compiled with Webots text editor or manually by using `gcc` and `make` commands in a terminal.
On Windows, you can also use Microsoft Visual Studio to compile the plugin.
In this case, please note that the plugin should be dynamically linked to the ODE library.
The Webots "lib" directory contains the gcc ("libode.a") and Visual Studio ("ode.lib") import libraries.
Under Linux, you don't need to link the shared library with anything.
