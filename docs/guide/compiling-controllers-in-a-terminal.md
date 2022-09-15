## Compiling Controllers in a Terminal

It is possible to compile Webots controllers in a terminal instead of using the built-in editor.
In this case you need to define the `WEBOTS_HOME` environment variable and make it point to Webots installation directory.
The `WEBOTS_HOME` variable is used to locate Webots header files and libraries in the Makefiles.
Setting an environment variable depends on the platform (and shell), here are some examples:

### macOS and Linux

These examples assume that Webots is installed in the default directory.
On Linux, type this:

```sh
$ export WEBOTS_HOME=/usr/local/webots
```

Or add this line to your "~/.bash\_profile" file.
On macOS, type this:

```sh
$ export WEBOTS_HOME=/Applications/Webots.app
$ export WEBOTS_HOME_PATH=$WEBOTS_HOME/Contents
```

Or add these lines to your "~/.profile" file.

Once these environment variables are defined, you should be able to compile in a terminal, with the `make` command.
Like with the editor buttons, it is possible to build the whole project, or only a single binary file, e.g.:

```sh
$ make
$ make clean
$ make -f Makefile_java my_robot.class
$ make my_robot.o
```

### Windows

On Windows you must use the MSYS2 terminal to compile the controllers.
MSYS2 is a UNIX-like terminal that can be used to invoke UNIX commands.
Please follow the instructions [here](https://github.com/cyberbotics/webots/wiki/Windows-installation#msys2-development-environment-and-git) to install it.

You will also have to set the `WEBOTS_HOME` environment variable to point to the installation folder of Webots, typically `C:\Program Files\Webots` and the path to the Webots binaries in the MSYS2 `PATH` environment variable:

```bash
export WEBOTS_HOME="C:\Program Files\Webots"
export PATH=$PATH:/C/Program\ Files/Webots/msys64/mingw64/bin:/C/Program\ Files/Webots/msys64/mingw64/bin
```

For convenience, the two above lines can be appended to your `~/.bash_profile` file of MSYS2.

Once MSYS2 is installed and the environment variables are defined, you should be able to compile controllers by invoking `make` in the MSYS2 terminal, e.g.:

```sh
$ make
$ make clean
```
