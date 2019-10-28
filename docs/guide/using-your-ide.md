## Using Your IDE

Using an [Integrated Development Environments (IDE)](https://en.wikipedia.org/wiki/Integrated_development_environment) is for sure convenient for its features (graphical debugger, edition tools, etc.).
You may be interested in using your favorite IDE to develop a Webots controller.

A priori, Webots can support any IDE to create, build and debug its controllers.
It is simply a matter of setting up the IDE correctly to fulfill the Webots controller build rules.
These rules are:

1. The target executable file should have the same name as the controller directory, and follow strictly this path: `$WEBOTS_PROJECT/controllers/$CONTROLLER_NAME/$CONTROLLER_NAME[$EXE_EXTENSION]`.
2. The executable should be linked with the Webots controller shared library.
For C, this library is `$WEBOTS_HOME/lib/$SL_PREFIXController$SL_SUFFIX`.
    - For C++, `$WEBOTS_HOME/lib/$SL_PREFIXCppController$SL_SUFFIX` should be added to the C library.
        - For Java, `$WEBOTS_HOME/lib/java/Controller.jar` should be added to the C++ library.
        - For Python, `$WEBOTS_HOME/lib/python$PYTHON_VERSION/_controller.so` and `$WEBOTS_HOME/lib/python$PYTHON_VERSION/controller.py` should be added to the C++ library.

Where:

- `$WEBOTS_HOME` is the path to Webots.
- `$WEBOTS_PROJECT` is the path to your Webots project.
- `$CONTROLLER_NAME` is the name of your controller.
- `$EXE_EXTENSION` is the executable file suffix.
    - For C or C++: `.exe` on Windows, and nothing else where.
    - For Python: `.py`.
    - For Java: `.class` or `.jar`.
- `SL_PREFIX` is the prefix of a shared library: `lib` on Linux or macOS, and nothing on Windows.
- `SL_SUFFIX` is the suffix of a shared library: `.so` on Linux, `.dylib` on macOS and `.dll` on Windows.
- `PYTHON_VERSION` is your Python version, but concatenated (`27`, `37`, etc.).

Documenting every IDE for each OS is a huge task.
This is why we are only documenting some of them, which can serve as a reference for others.

### Visual Studio

On Windows, [Visual Studio](https://visualstudio.microsoft.com) can be used to create a C or C++ controller.

A `Visual Studio` project can be simply created using the "New Robot Controller..." wizard.
To do so, you should simply select the **Wizards / New Robot Controller...** menu item, choose the C or C++ language, and select a Visual Studio project.
The target project can be open in `Visual Studio`.

### CMake

[CMake](https://cmake.org) is a cross-platform free and open-source software tool for managing the build process of software using a compiler-independent method.
Using [its generators](https://cmake.org/cmake/help/v3.0/manual/cmake-generators.7.html), it generates native build environments, such as `XCode`, `CodeBlocks`, `Sublime Text 2` or `Eclipse` projects.
The actual build is processed in these environments.
For the `Visual Studio` target, prefer the [solution above](#visual-studio).

As a template, you could copy the following `CMakeLists.txt` file to your controller directory.
This template is only a sample, it may be adapted depending on your CMake target.

```cmake
cmake_minimum_required(VERSION 3.0)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

# Link with the Webots controller library.
IF (WIN32)
  link_directories($ENV{WEBOTS_HOME}/msys64/mingw64/bin)
ELSE()
  link_directories($ENV{WEBOTS_HOME}/lib)
ENDIF()
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES})

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
```

Then, create a `build` directory, create the native project, and build it in the target IDE.
For example:

```shell
# export WEBOTS_HOME=...
cd $WEBOTS_PROJECT/controllers/$CONTROLLER_NAME
# edit CMakeLists.txt
mkdir build
cd build
cmake .. -G "Unix Makefiles"
make
```

### Qt Creator

[Qt Creator](https://www.qt.io) is a cross-platform IDE supporting among others the C++ language.

As a template, you could copy the following `qmake.pro` file in your controller directory and open this project in Qt Creator:

```qmake
# Setup global paths. The WEBOTS_HOME environment variable should be setup.
WEBOTS_HOME_PATH = $$(WEBOTS_HOME)
CONTROLLER_PATH = $$PWD
CONTROLLER_NAME = $$basename(CONTROLLER_PATH)

# Get C or C++ sources in the current directory (only).
SOURCES = $$files(*.c, true)
SOURCES += $$files(*.cpp, true)
HEADERS = $$files(*.h, true)

# Setup the project.
# Its name is defined to be the controller directory name.
TARGET = $$CONTROLLER_NAME
DESTDIR = $$CONTROLLER_PATH
QMAKE_TARGET = $$CONTROLLER_NAME

# Do not link with the Qt libraries :-)
CONFIG -= qt

# Link with the Webots controller library.
INCLUDEPATH += $$WEBOTS_HOME_PATH/include/controller/c $$WEBOTS_HOME_PATH/include/controller/cpp
win32 {
  CONFIG += console
  LIBS += -L$$WEBOTS_HOME_PATH/msys64/mingw64/bin -lController -lCppController
}
unix {
  LIBS += -L$$WEBOTS_HOME_PATH/lib -lController -lCppController
}
macx {
  CONFIG -= app_bundle
  CONFIG += sdk_no_version_check
  LIBS += -L$$WEBOTS_HOME_PATH/lib -lController -lCppController
}
```
