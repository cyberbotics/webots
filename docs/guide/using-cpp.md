## Using C++

### Introduction

The C++ API is a wrapper of the C API described in the previous section.
The major part of the C functions have been wrapped in a function of a specific class.
It is currently composed of a set of about 25 classes having about 200 public functions.
The classes are either representations of a node of the scene tree (such as [Robot](../reference/robot.md), [LED](../reference/led.md), etc.) or utility classes (such as Motion, ImageRef, etc.).
A complete description of these functions can be found in the reference guide while the instructions about the common way to program a C++ controller can be found in this [chapter](programming-fundamentals.md).

### C++ Compiler Installation

Please refer to the instructions for the C Compiler installation [here](using-c.md#c-cpp-compiler-installation).

### Source Code of the C++ API

The source code of the C++ API is available in the Webots release.
You may be interested in looking through the directory containing the header files ("include/controllers/cpp") in order to get the precise definition of every classes and functions although the reference guide offers a clean description of the public functions.
This directory is automatically included when the C++ controller is compiled.

For users who want to use a third-party development environment, it is useful to know that the shared library ("CppController.dll, libCppController.so", or "libCppController.dylib") is located in the "lib" subdirectory of your Webots directory.
This directory is automatically included when the C++ controller is linked.

Advanced users may want to modify the C++ API from the [C++ sources]({{ url.github_tree }}/src/controller/cpp/).
