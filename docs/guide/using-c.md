## Using C

### Introduction

The C API (Application Programming Interface) is composed of a set of about 200 C functions that can be used in C or C++ controller code.
This is the low level interface with the Webots simulator; all other APIs are built over the C API.
A majority of Webots controller examples are written in C, therefore the C API is Webots de facto standard API.
Although less represented in the controller examples, the other APIs offer exactly the same functionality as the C API.

### C/C++ Compiler Installation

#### Windows Instructions

The Windows version of Webots comes with a pre-installed copy of the MinGW C/C++ compiler, so there is usually no need to install a separate compiler.
The MinGW compiler is a port of the GNU Compiler Collection (gcc) on the Windows platform.
The advantage of using the MinGW compiler will be the better portability of your controller code.
If you develop your code with MinGW it will be straightforward to recompile it on the other Webots supported platforms: macOS and Linux.
However, if you prefer to use Microsoft Visual Studio you will find instructions [here](using-your-ide.md#visual-studio).

#### macOS Instructions

In order to compile C/C++ controllers on the Mac, you will need to install Apple Xcode.
Xcode is a suite of tools, developed by Apple, for developing software for macOS.
Xcode is free and can be downloaded from the Apple App Store.
Webots will need principally the `gcc` (GNU C Compiler) and `make` commands of Xcode.
To install these commands, start Xcode and go to Xcode menu, Preferences, Downloads, Components and click `Install` for "Command Line Tools".

#### Linux Instructions

For compiling C controllers, Webots will need the GNU C Compiler and GNU Make utility.
On Linux, these tools are often pre-installed, otherwise you will need to install them separately (*gcc* and *make* packages).
For C++ you will also need the GNU C++ Compiler (*g++* package).
Optionally you can also install the GNU Debugger (*gdb* package).
