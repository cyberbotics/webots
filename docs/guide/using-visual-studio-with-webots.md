## Using Visual Studio with Webots

### Introduction

Microsoft Visual Studio is an integrated development environment (IDE) for C/C++ available on the Windows platform.
On Windows, Visual Studio is a possible alternative to using Webots built-in Makefile and gcc (MinGW) compiler.
Visual Studio can be used to develop controllers using Webots C or C++ API.
The developer must choose one of these two APIs as they cannot be used together in controller code.
The C API is composed of ".h" files that contains flat C functions that can be used in C or C++ controllers.
The C++ API is composed of ".hpp" files that contain C++ classes and methods that can be used in C++ controllers only.
In principle any C or C++ controller from the Webots distribution can be turned into a Visual Studio project.

### Robot Controller Wizard

Since Webots R2018b, a Visual Studio option is offered in the **Wizards / New Robot Controller...** menu after you choose the C or C++ language on Windows.
This wizard creates a Visual Studio project for your robot controller, so that you don't need to configure it manually as described in the next section.

### Configuration

When creating a Webots controller with Visual Studio, it is necessary to specify the path to Webots ".h" and/or ".hpp" files.
It is also necessary to configure the linker to use the "Controller.lib" import library from Webots distribution.
The "Controller.lib" files is needed to link with the "Controller.dll" file that must be used by the controller in order to communicate with Webots.

The following procedure (Visual Studio 2008 Express) explains how to create a Visual Studio project for a Webots controller.
Note that the resulting ".exe" file must be launched by Webots; it cannot be run from Visual Studio.

1. Copy a Webots project from Webots distribution to your "Documents" folder, or create an empty project directory using Webots menu: `Wizard / New Project Directory...` Either way, the project directory must contain the "controllers" and "worlds" subdirectories.

2. Start Visual Studio and select: `File / New / Project...`.
Then choose these settings:

        Project type: General
        Template: Empty Project
        Name: MyController (for example)
        Location: C:\Users\MyName\Documents\MyProject\controllers (for example)

    Where "MyController" is the name of a new or already existing controller
    directory, and where "Location" must indicate the "controllers" subdirectory of
    your Webots project directory.

3. Then you can add a C or C++ source file to your project: Choose either: `Project / Add Existing Item` or `Project / Add New Item / C++ File (.cpp)`.
In the second case you can copy the content of one of the C/C++ examples of Webots distribution.

    Note that if you copied C code from Webots examples to Visual Studio, it is highly
    recommended to change the source file extension from .c to .cpp. The reason is
    that Webots examples are written for the gcc compiler which uses a more modern
    version of the C language than Visual Studio. By changing the file extension to
    .cpp you will instruct Visual Studio to compile the file in C++ mode (/TP) which is
    more tolerant with gcc code. If you don't do it, you may run into error messages
    like these:

        MyController.c(24): error C2275: 'WbDeviceTag' : illegal use of
          this type as an expression
        MyController.c(24): error C2146: syntax error : missing ';' before
          identifier 'ir0'
        ...

4. Now we can set up the project configuration for Webots.
Select the `Project / Properties` menu.
In the `Property Pages`, in the `Configuration Properties`, enter following configuration:

        C/C++ > General > Additional Include Directories:
          C:\Program Files\Webots\include\controller\c

    This will tell Visual Studio where to find Webots C API (.h files).

    By default Visual Studio places the .exe file in a "Debug" or "Release"
    subdirectory. However, in order to be executed by Webots, the .exe file must be
    placed directly at the root of the "MyController" directory. So, in this example
    the .exe should be there: "MyProject\controllers\MyController\MyController.exe".
    Consequently the linker output file should be configured like this:

        Linker > General > Output File: $(ProjectName).exe

    Now we need to tell Visual Studio to use the "Controller.lib" import library:

        Linker > Input > Additional Dependencies:
          Controller.lib
        Linker > General > Additional Library Directories:
          C:\Program Files\Webots\msys64\mingw64\lib\

    Note that with old versions of Visual Studio, the default target is a 32-bit binary.
    In case you are compiling the controller as a 32-bit binary, you will need to link it with the 32-bit version of the Controller library instead:

        Linker > General > Additional Library Directories:
          C:\Program Files\Webots\msys64\mingw32\lib\

5. If you want to use the C API, you should skip step 5 and go directly to step 6.
If you want to use the C++ API follow these instructions:

    In `Property Pages`, in the `Configuration Properties`, add the path to Webots
    .hpp files:

        C/C++ > General > Additional Include Directories:
          C:\Program Files\Webots\include\controller\c
          C:\Program Files\Webots\include\controller\cpp

    Now you should have the path to both the .h and the .hpp files.

    Then you need to add Webots C++ wrappers to your project. The C++ wrappers are
    .cpp files that implement the interface between the C++ API and the C API. You
    can proceed like this:

    In Visual Studio, in the `Solution Explorer`: right-mouse-click on the `Sources
    Files` folder, then select `Add / New Filter`. This should create a `NewFilter1`
    subfolder in your `Sources Files` folder. Then select the `NewFilter1` and with
    the right-mouse-button: choose the `Add / Existing Item...` menu. In the file
    dialog, go to the "C:\Program Files\Webots\resources\languages\cpp" directory,
    then select all the .cpp files (but no other file) in that directory and hit the
    `Add` button. This should add the "Accelerometer.cpp, Camera.cpp, Compass.cpp",
    etc. source files to your project.

6. Now you should be able to build your controller with the `Build / Build MyController` menu item (or the F7 key).
This should generate the "MyProject\controllers\MyController\MyController.exe" file.

7. Now we can switch to Webots in order to test the .exe controller.
Start Webots and verify that your robot is associated with the correct controller: In the `Scene tree`, expand the robot node and check the `controller` field.
It should be: `controller "MyController"`.
Otherwise you should change it: hit the `...` (ellipsis) button, this opens a selection dialog.
In the selection dialog choose "MyController".
Then hit the `Save` button in Webots' main window.
Finally you can hit the `Run` button to start the simulation.
At this point the simulation should be using your Visual Studio controller.

8. If you want to debug your controller with Visual Studio you can *attach* the debugger to the running controller process.
Proceed like this: In Webots, hit the `Pause` button then the `Reload` button.
Then, in Visual Studio, use the `Debug / Attach to Process...` menu.
In the dialog choose the `MyController.exe_webots` process.
Still in Visual Studio, you can now add breakpoints and watches in the controller code.
Then, in Webots, hit the `Run` button to resume the simulation.
Now the controller should pause when it reaches one of your breakpoints.

### Link with the Webots Libraries

Webots contains several `C` or `C++` libraries based on the `libController` or `libCppController` libraries (e.g. the `vehicle libraries`, `DARwIn-OP library`, `youBot library`, etc.).

The precompiled `C` libraries are released with their corresponding Visual Studio `.lib` file (the linker to the `.dll` file) to facilitate their integration into a Visual Studio project.
They are located in the same directory as the library source, where the `.dll` file is generated.

**Note**: The chosen architecture (32 or 64-bit) should match with Visual Studio solution platform and the path to the `Controller.lib` library.

However there is no precompiled `C++` libraries for Visual Studio, because the `gcc` compiler tool chain embedded in Webots is incompatible with Visual Studio.
To use the `C++` libraries with your project, their source files should be compiled directly in your project, exactly as for the `libCppController` library (cf. instructions above).

For example, to add the `C++ vehicle libraries`:

- Add the `C`, `car` and `driver` precompiled libraries (`Linker > Input > Additional Dependencies`):

    - `car.lib`
    - `driver.lib`

- Add the following `C++` source files to your project (`Add / New Filter` and `Add / Existing Item...`):

    - `$(WEBOTS_HOME)/projects/default/libraries/vehicle/cpp/car/src/*.cpp`
    - `$(WEBOTS_HOME)/projects/default/libraries/vehicle/cpp/driver/src/*.cpp`

- If the include files are not located in `$(WEBOTS_HOME)/include/controller/c` or `$(WEBOTS_HOME)/include/controller/cpp` (which is the case for the vehicle libraries), they should be added to your project (`C/C++ > General > Additional Include Directories`).
