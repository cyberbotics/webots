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
$ export WEBOTS_HOME=/Applications/Webots
```

Or add this line to your "~/.profile" file.

Once `WEBOTS_HOME` is defined, you should be able to compile in a terminal, with the `make` command.
Like with the editor buttons, it is possible to build the whole project, or only a single binary file, e.g.:

```sh
$ make
$ make clean
$ make my_robot.class
$ make my_robot.o
```

### Windows

On Windows you must use the MSYS terminal to compile the controllers.
MSYS is a UNIX-like terminal that can be used to invoke MinGW commands.
It can be downloaded from [http://sourceforge.net](http://sourceforge.net).
You will also need to add the "bin" directory of MinGW to your *PATH* environment variable.
MinGW is located in the "mingw" subdirectory of Webots distribution.
When set correctly, the environment variable should be like this:

```sh
WEBOTS_HOME=C:\Program Files\Webots
PATH=C:\program Files\Webots\mingw\bin;C:\...
```

Once MSYS is installed and the environment variables are defined, you should be able to compile controllers by invoking `mingw32-make` in the MSYS terminal, e.g.:

```sh
$ mingw32-make
$ mingw32-make clean
$ mingw32-make my_robot.class
$ mingw32-make my_robot.o
```
