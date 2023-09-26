## MATLAB

The *MATLAB*<sup>TM</sup> API for Webots is very similar to the C API.
The functions names are identical, only the type and number of parameters differs slightly in some cases.
The MATLAB functions and prototypes are described in Webots [Reference Manual](../reference/index.md).
Note that unlike with the C API, there are no `wb_robot_init` and `wb_robot_cleanup` functions in the MATLAB API.
The necessary initialization and cleanup are automatically carried out respectively before entering and after leaving the controller code.

You should implement your controller as a [function](https://www.mathworks.com/help/matlab/matlab_prog/scripts-and-functions.html) by adding the necessary `function` declaration followed by the name of the controller at the start of the file.

If the MATLAB code uses graphics, it is necessary to call the `drawnow` command somewhere in the control loop in order to flush the graphics.

Here is a simple MATLAB controller example:

```MATLAB
function simple_example

TIME_STEP = 32;

my_led = wb_robot_get_device('my_led');
my_sensor = wb_robot_get_device('my_sensor');

wb_distance_sensor_enable(my_sensor, TIME_STEP);

while wb_robot_step(TIME_STEP) ~= -1
  % read the sensors
  val = wb_distance_sensor_get_value(my_sensor);

  % Process sensor data here

  % send actuator commands
  wb_led_set(my_led, 1);

  % uncomment the next line if there's graphics to flush
  % drawnow;
end
```

### Debugging Using the MATLAB Desktop

For each controller written using MATLAB, Webots will start a new instance of MATLAB to act as an interpreter.
In order to avoid cluttering the desktop with too many windows, Webots starts each instance of MATLAB in non-interactive mode.
This means that MATLAB starts without the user interface which keeps the memory usage low; this is particularly useful in multi-robot experiments.
Any output to stdout (such as `disp` or `fprintf`) will also be redirected to the Webots console.

If you would like to use the MATLAB desktop to interact with your controller, you will need to run it in `<extern>` mode with the appropriate additional argument. 
You can read more about that [here](running-extern-robot-controllers.md).

**Note**: This is equivalent to inserting the command `keyboard` in your controller code, but this is strongly discouraged since it will cause an error during non-interactive execution of the code.

Running an external controller in interactive mode will automatically place a breakpoint at the first line of your controller. 
Once MATLAB desktop has initialized, it will halt the execution of the controller and give control to the keyboard (`K>>` prompt).
MATLAB also opens your controller m-file in its editor and indicates that the execution is stopped at the breakpoint.

At this point, the controller m-file can be debugged interactively, i.e., it is possible to continue the execution step-by-step, set break points, watch variable, etc. 
You can use the navigation buttons in the Editor Toolstrip such as Continue/Pause, Step, and Quit Debugging to control the execution.
While running, the controller will run normally until it terminates or reaches another breakpoint.
While paused, the current values of the controller variables are shown in the MATLAB workspace, and the Command Window becomes available.
You can read more about debugging MATLAB code on the [MathWorks homepage](https://www.mathworks.com/help/matlab/matlab_prog/debugging-process-and-features.html).

While paused (or after the controller has been terminated), the connection with Webots remains active.
Therefore it becomes possible to issue Webots commands directly on the MATLAB prompt, for example you can interactively issue commands to query the sensors, etc.:

```MATLAB
K>> wb_robot_step(1000);
K>> wb_gps_get_values(gps)

ans =

    0.0001    0.0030   -0.6425
```

The execution of the controller can be terminated with <kbd>Ctrl</kbd>+<kbd>C</kbd> key combination, or by quitting the debugger.
However, since all controllers are functions, their Workspace (local variables) will be lost after termination.
It is possible to re-run the controller by calling `launcher` from MATLAB prompt.
Note that this will restart the controller only, not the whole simulation, so the current robot and motor positions will be preserved.
If you want to restart the whole simulation you need to use the `Reload` button in Webots as usual.
