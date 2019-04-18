## Using Your IDE

Webots basically supports any [Integrated Development Environments (IDE)](https://en.wikipedia.org/wiki/Integrated_development_environment) to build its controllers.
It is simply a question of configuring your IDE correctly to fulfill the Webots controller build rules.
These rules are:

1. The resulting executable file should have the same name as the controller directory: `$YOUR_WEBOTS_PROJECT/controllers/$YOUR_CONTROLLER_NAME/$YOUR_CONTROLLER_NAME[$EXTENSION]`.
`$EXTENSION` may be `.exe`, `.py`, `.class`, etc. depending on your controller language and operating system.
2. The executable should be linked with the Webots controller library.
For example, this library is `$WEBOTS_HOME/lib/libController.so` for C on `linux` or `$WEBOTS_HOME/lib/python37/controller.py` for Python on `macOS`.

Some IDE comes with interpreters or compiler tool chain which are incompatible with the precompiled Webots controller libraries.
This may render the IDE integration much more complex.
For example, the Visual Studio C++ compiler is not compatible with the MINGW gcc compiler used to precompile the C++ Webots controller library.

Documenting every IDE for each OS is a huge task.
This is why we are only documenting some of them, which can serve as a reference for others.

### Visual Studio

On Windows, Visual Studio can be used to create a C or C++ controller.

A Visual Studio project can be simply created using the "New Robot Controller..." wizard.
To do so, you should simply select the **Wizards / New Robot Controller...** menu item, choose the C or C++ language, and select a Visual Studio project.

### CMake

[CMake](https://cmake.org) is a cross-platform free and open-source software tool for managing the build process of software using a compiler-independent method.
Using [its generators](https://cmake.org/cmake/help/v3.0/manual/cmake-generators.7.html), it generates native build environments, such as `XCode`, `CodeBlocks`, `Sublime Text 2` or `Eclipse` projects.
The actual build is processed in these environments.

As a template, you could copy the following `CMakeLists.txt` file to your controller directory:

```cmake
cmake_minimum_required(VERSION 3.0)

get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)  # controller directory name

project(${PROJECT})

IF (WIN32)
  link_directories($ENV{WEBOTS_HOME}/msys64/mingw64/bin)
ELSE()
  link_directories($ENV{WEBOTS_HOME}/lib)
ENDIF()

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)

add_executable(${PROJECT} ${SOURCES})

target_link_libraries(${PROJECT} ${LIBRARIES})

add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
```

Then, create a `build` directory, create the native project, and build it in the target IDE.
For example:

```shell
cd $YOUR_WEBOTS_PROJECT/controllers/$YOUR_CONTROLLER_NAME
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
WEBOTS_HOME_PATH = $$(WEBOTS_HOME)
CONTROLLER_PATH = $$PWD
CONTROLLER_NAME = $$basename(CONTROLLER_PATH)

SOURCES = $$files(*.c, true)
SOURCES += $$files(*.cpp, true)
HEADERS = $$files(*.h, true)

TARGET = $$CONTROLLER_NAME
DESTDIR = $$CONTROLLER_PATH
QMAKE_TARGET = $$CONTROLLER_NAME

CONFIG -= qt

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
