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

Webots must be able to access the MATLAB executable (usually a script) in order to execute the controller m-files.
The absolute path to the executable can be changed in the Webots preferences, in the General tab.
When Webots is started for the first time, the path is defined to the default installation directory of MATLAB, depending on the OS:

- *Windows*: C:\Program Files\MATLAB\R20XXx\bin\win64\MATLAB.exe
- *Linux*: /usr/local/MATLAB/R20XXx/bin/matlab
- *MacOS*: /Applications/MATLAB_R20XXx.app

By default, the most recent version is chosen.
If MATLAB is installed at a custom path or if another version must be used, the absolute path can be easily filled in.
On Windows, there are two MATLAB.exe files: one is located in "bin\MATLAB.exe" and the other one in "bin\win64\MATLAB.exe".
"bin\win64\MATLAB.exe" should be used in the preferences, because "bin\MATLAB.exe" is just a launcher that causes problems with stdout/stderr streams and with the termination of the process.

### How to Run the Examples?

To test MATLAB in Webots, start Webots and open the "WEBOTS\_HOME/projects/languages/matlab/worlds/e-puck\_matlab.wbt" or "[WEBOTS\_HOME/projects/robots/softbank/nao/worlds/nao\_matlab.wbt]({{ url.github_tree }}/projects/robots/softbank/nao/worlds/nao_matlab.wbt)" world file.
Webots automatically starts MATLAB when it detects an m-file in a controller directory.
Note that the m-file must be named after its directory in order to be identified as a controller file by Webots.
So, for example, if the directory is named "my\_controller", then the controller m-file must be named "my\_controller/my\_controller.m".
Additionally, the m-file should have the necessary `function` declaration on the first line, matching the controller name.

No special initialization code is necessary in the controller m-file.
In fact Webots calls an intermediate "launcher.m" file that sets up the Webots controller environment and then calls the controller m-file.
In particular the "launcher.m" file loads the library for communicating with Webots and adds the path to API m-files.
The MATLAB API m-files are located in the "lib/matlab" directory of Webots distribution.
These are readable source files; please report any problem, or possible improvement about these files.

### Display Information to Webots Console

The MATLAB output is redirected as is to the Webots console.
This means you can use all the MATLAB display features including the `disp` and `display` (omitting the semicolon character at the end of a statement.).

Additionally, the `wb_console_print(text, stream)` function could be used to display some text in the Webots console.
The second argument (`stream`) can be either `WB_STDOUT` or `WB_STDERR` depending on which stream you would like to write.

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
