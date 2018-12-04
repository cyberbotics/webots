## Using Python

### Introduction

The Python API has been generated from the C++ API by using SWIG.
This implies that their class hierarchy, their class names and their function names are almost identical.
The Python API is currently composed of a set of about 25 classes having about 200 public functions located in the module called *controller*.
The classes are either representations of a node of the scene tree (such as Robot, LED, etc.) or utility classes (such as Motion, ImageRef, etc.).
A complete description of these functions can be found in the reference guide while the instructions about the common way to program a Python controller can be found in [this chapter](programming-fundamentals.md).

The Python API of Webots supports both Python 3.7 and Python 2.7.
On Ubuntu 18.04, it also supports Python 3.6 and on Ubuntu 16.04, it also supports Python 3.5.

### Installation

Webots starts Python using the standard `python` command line.
As a consequence, it executes the first `python` binary found in the current `PATH`.
If you want to use a different version of Python, please install it if needed and configure your environment so that it becomes the default `python` version when called from the command line in a terminal.
Alternatively, you can change the default Python command from the Webots Preferences in the General tab.
If you set it for example to `python3.7` instead of `python`, this version of python will be used (if available from the command line).
Finally, it is also possible to set a different version of Python for each robot controller by editing the `[python]` section of the `runtime.ini` file in each robot controller directory and setting the `COMMAND` value to `python3`, `python3.7` or `python2.7`, etc.
If specified in the `runtime.ini` file of a controller, this Python command will be executed instead of the default one to launch this controller.

#### Linux Installation

Most of the Linux distributions have Python 2.7 and 3.x already installed.
To check the versions of Python installed on your system, you can type in a terminal: `python --version`, `python3.6 --version`, `python2.7 --version`, `python3 --version`, etc.

#### macOS Installation

Python 2.7 installed by default.
You can install Python 3.7 from the [Python web site](https://www.python.org).
To check the versions of Python installed on your system, you can type in a terminal: `python --version`, `python3.7 --version`, `python2.7 --version`, `python3 --version`, etc.

#### Windows Installation

You should install the latest version of Python 3.7 (64 bit) or Python 2.7 (64 bit) from the official [Python website](https://www.python.org).
Then, you have to modify your `PATH` environment variable to add the path to the `python.exe` binary which is located in the main `Python37` or `Python27` installation folder.
To check this was done properly, you can open a DOS console (`CMD.EXE`) and type `python --version`.
If it displays the correct Python version, then, everything is setup properly and you should be able to run the Python example provided with Webots in the `WEBOTS_HOME/projects/languages/python/worlds/example.wbt` world file.

### Libraries

The `WEBOTS_HOME/projects/web/visual_tracking` sample simulation uses the Python [OpenCV](http://opencv.org/) and [NumPy](http://numpy.org/) packages.
These packages have to be installed on the system in order to correctly run this simulation.
Using Python *pip*, the *NumPy* package is automatically installed with *opencv-python* package.

#### Linux Libraries

Use the `pip` command to install OpenCV:

```sh
sudo apt-get install python-pip
sudo pip install opencv-python
```

#### macOS Libraries

Open a Terminal and type:
```sh
pip install opencv-python --user
```

#### Windows Libraries

Open the DOS console (CMD.EXE) and type:

```sh
PYTHON_PATH\Scripts\pip.exe install opencv-python
```

Where `PYTHON_PATH` is the path to the Python installation directory, for example `C:\Python36`.

### Use an Alternative Python Version

As explained above, the Python libraries for Webots are precompiled for Python 3.7, Python 2.7 and on Ubuntu for the default Python 3 version provided with the system.
It is possible however to use another Python version by recompiling the Webots Python libraries.
Such a task requires some knowledge in software installation, compilation from sources and Makefile.

The general idea is to walk through the following steps:

1. Install a new Python version and add the path to the new python binary in your `PATH` environment variable, so that you can execute `python --version` from a console and get the correct version number.
2. Get [SWIG](http://www.swig.org/download.html).
3. Recompile the Python wrapper located in this directory: `$WEBOTS_HOME/resources/languages/python` using its Makefile.
You may check the above Makefile in `$WEBOTS_HOME/resources/languages/Makefile` to understand how to call the Python Makefile with different options.

On Windows you will need to install [MSYS2 for x86\_64](http://www.msys2.org/) and run it in administrator mode to be able to modify files in `$WEBOTS_HOME`.
From the MSYS2 console, you will need to install at least `gcc` `make` and `swig` with the `pacman` command:
```bash
pacman -S gcc make swig
```
From here you can now continue with step 3.
