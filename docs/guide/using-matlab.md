## Using MATLAB

### Introduction to MATLAB

*MATLAB*<sup>TM</sup> is a numerical computing environment and an interpreted programming language.
It allows easy matrix manipulation, plotting of functions and data, implementation of algorithms and creation of user interfaces.
You can get more information on the official [MathWorks website](http://www.mathworks.com) .
MATLAB is widely used in robotics in particular for its *Image Processing, Neural Networks* and *Genetics Algorithms* toolboxes.
Webots allows to directly use MATLAB scripts as robot controller programs for your simulations.
Using the MATLAB interface, it becomes easy to visualize controller or supervisor data, for example, processed images, sensor readings, the performance of an optimization algorithm, etc., while the simulation is running.
In addition, it becomes possible to reuse your existing MATLAB code directly in Webots.

### MATLAB Installation

In order to use MATLAB controllers in Webots, the MATLAB software must be installed (a MATLAB license is required).
Webots {{ webots.version.full }} supports only 64bits MATLAB versions from 2015b.

On Windows the [MATLAB MinGW-w64 C/C++ Compiler](https://fr.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-compiler) needs to be installed in addition to MATLAB.

Webots must be able to access the "matlab" executable (usually a script) in order to run controller m-files.
Webots looks for the "matlab" executable in every directory of your *PATH* (or *Path* on Windows) environment variable.
Note that this is similar to calling "matlab" from a terminal (or *Command Prompt* on Windows), therefore, if MATLAB can be started from a terminal then it can also be started from Webots.

On Windows, the MATLAB installer will normally add MATLAB's bin directories to your *Path* environment variable, so usually, Webots will be able to locate MATLAB after a standard installation.
However, in case it does not work, please make sure that your *Path* environment variable contains this directory (or something slightly different, according to your MATLAB version): `C:\Program Files\MATLAB\R2020b\bin`.
With older versions of MATLAB (e.g., R0217b), your *Path* environment variable should also include: `C:\Program Files\MATLAB\R2017b\bin\win64`.


On Linux, the MATLAB installer normally suggests to add a symlink to the "matlab" startup script in the "/usr/local/bin" directory.
This is a good option to make "matlab" globally accessible.
Otherwise you can create the link at anytime afterwards with this shell command (please change according to your actual MATLAB installation directory and version):

```sh
$ sudo ln -s /usr/local/MATLAB/R2020b/bin/matlab /usr/local/bin/matlab
```

Similarly, on macOS, if Webots is unable to find the "matlab" startup script then you should add a symlink in "/usr/bin":

```sh
$ sudo ln -s /Applications/MATLAB_R2020b.app/bin/matlab /usr/local/bin/matlab
```

### How to Run the Examples?

To test MATLAB in Webots, start Webots and open the "WEBOTS\_HOME/projects/languages/matlab/worlds/e-puck\_matlab.wbt" or "[WEBOTS\_HOME/projects/robots/softbank/nao/worlds/nao\_matlab.wbt](https://github.com/cyberbotics/webots/tree/master/projects/robots/softbank/nao/worlds/nao_matlab.wbt)" world file.
Webots automatically starts MATLAB when it detects an m-file in a controller directory.
Note that the m-file must be named after its directory in order to be identified as a controller file by Webots.
So, for example, if the directory is named "my\_controller", then the controller m-file must be named "my\_controller/my\_controller.m".

No special initialization code is necessary in the controller m-file.
In fact Webots calls an intermediate "launcher.m" file that sets up the Webots controller environment and then calls the controller m-file.
In particular the "launcher.m" file loads the library for communicating with Webots and adds the path to API m-files.
The MATLAB API m-files are located in the "lib/matlab" directory of Webots distribution.
These are readable source files; please report any problem, or possible improvement about these files.

### Display Information to Webots Console

On Linux and macOS, the MATLAB output is redirected as is to the Webots console.
This means you can use all the MATLAB display features including the `disp` and `display` (omitting the semicolon character at the end of a statement.).

On Windows, the MATLAB output is not redirected to the Webots console.
The `wb_console_print(text, stream)` function should be used to display some text in the Webots console.
The second argument (`stream`) can be either `WB_STDOUT` or `WB_STDERR` depending on which stream you would like to write.

In order to create a cross-platform controller, it is recommended to use the `wb_console_print(text, stream)` on every OS.

### Compatibility Issues

We recommend to use the latest MATLAB version on an up-to-date operating system.

Note that 64-bit versions of Webots are not compatible with 32-bit versions of MATLAB.
Webots comes only in 64-bit flavors and therefore it can only inter-operate with a 64 bit version of MATLAB.

On some platforms the MATLAB interface needs `perl` and `gcc` to be installed separately.
These tools are required because MATLAB's `loadlibrary` function will need to recompile Webots header files on the fly.
According to MATLAB's documentation this will be the case on 64-bit systems, and hence we advice 64-bit Webots users (on Linux) to make sure that these packages are installed on their systems.

On some macOS systems the MATLAB interface will work only if you install the Xcode development environment, because `gcc` is required.
An error message like this one, is a symptom of the above described problem:

```
error using ==> calllib
Method was not found.

error in ==> launcher at 66
calllib('libController','wb_robot_init');
```
