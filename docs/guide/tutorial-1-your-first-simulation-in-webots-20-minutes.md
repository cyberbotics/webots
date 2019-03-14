## Tutorial 1: Your First Simulation in Webots (20 Minutes)

The objective of this first tutorial is to familiarize yourself with the user interface and with the basic concepts of Webots.
You will create your first simulation containing a simple environment: a light, an arena with floor and walls, a predefined robot (e-puck) and a controller program that will make the robot move (see [figure](#what-you-should-see-at-the-end-of-the-first-tutorial)).

%figure "What you should see at the end of the first tutorial."
![tutorial_e-puck.png](images/tutorial_e-puck.png)
%end

### Start Webots

If not already done, download and install Webots, following these [installation instructions](installing-webots.md).

> **Hands on #1**: Start Webots by double-clicking on its icon (or invoking it from a command line in a Terminal).
If you are running Webots for the first time on this computer, you may be prompted to select a graphical theme.
You may also be invited to follow the Webots guided tour, go ahead and close the guided tour.

If you never saw it, please take some time to view the demos featured in the guided tour.
They are telling a lot about the possibilities of Webots.
The guided tour is also available from the *Help* menu of Webots.

### Create a New World

A **World** is a file containing information like where the objects are, what they look like, how they interact with each other, what is the color of the sky, how is defined the gravity, friction, masses of the objects, etc.
It defines the initial state of a simulation.
The different objects are called **Nodes** and are organized hierarchically in a **Scene Tree**.
Therefore, a node may contain sub-nodes.
A world is stored in a file having the `.wbt` extension.
The file format is derived from the **VRML97** language, and is human readable.
The world files must be stored directly in a directory called `worlds`.

> **Hands on #2**: Pause the current simulation by clicking on the `Pause` button of the 3D view.
The simulation is paused if the virtual time counter on the main toolbar is stopped.
Create a new project from the `Wizards` menu by selecting the `New Project Directory...` menu item.
Follow the instructions.
Name the project directory `my_first_simulation` instead of the proposed `my_project`.
Name the world file `my_first_simulation.wbt` instead of the proposed `empty.wbt`.
Click all the tick boxes, including the "Add a rectangle arena" which is not ticked by default.

Webots displays a list of directories and files it just created.
This corresponds to the [standard file hierarchy of a Webots project](the-standard-file-hierarchy-of-a-project.md).

Congratulation, you just created your very first Webots world!
The 3D view should display a square arena with a checkered floor.
You can change the viewpoint of the 3D view by using the mouse buttons (left button, right button and the wheel).

Webots nodes stored in world files are organized in a tree structure called the **scene tree**.
The scene tree can be viewed in two subwindows of the main window: the 3D view (at the center of the main window) is the 3D representation of the scene tree and the scene tree view (on the left) is the hierarchical representation of the scene tree.
The scene tree view is where the nodes and fields can be modified.
It should currently list the following nodes:

- [WorldInfo](../reference/worldinfo.md): contains global parameters of the simulation.
- [Viewpoint](../reference/viewpoint.md): defines the main viewpoint camera parameters.
- [TexturedBackground](object-backgrounds.md#texturedbackground): defines the background of the scene (you should see mountains far away if you rotate a little bit the viewpoint)
- [TexturedBackroundLight](object-backgrounds.md#texturedbackgroundlight): defines the light associated with the above background.
- [RectangleArena](object-floors.md#rectanglearena): define the only object you see so far in this scene.

Each node has some customizable properties called **Fields**.
Let's modify these fields to change the rectangle arena:

> **Hands on #3**: Double-click on the `RectangleArena` node in the scene tree.
This should open the node and display its fields.
Select the `floorTileSize` field and set its value to `0.25 0.25` instead of `0.5 0.5`.
You should see the effect immediately in the 3D view.
Select the `wallHeight` field and change its value to `0.05` instead of `0.1`.
The wall of the arena should now be lower.

In the Scene Tree view, the fields are displayed in a different color (depending on the theme) if they differ from their default values.
Now, we would like to add some objects:

> **Hands on #4**: Double-click on the `RectangleArena` in the scene tree to close it and select it.
Click on the `Add` button (plus sign) at the top of the scene tree.
In the open dialog box, choose `PROTO (Webots) / objects / factory / containers / WoodenBox (Solid)`.
A big box should appear in the middle of the arena.
Double-click on it in the scene tree to open its fields.
Change its `size` to `0.1 0.1 0.1` instead of `0.6 0.6 0.6`.
Change its `translation` to `0 0.05 0` instead of `0 0.3 0`.
Alternatively, you may use the green arrow that appears in the 3D view to adjust its `translation.y` field.
Now shift-click and drag the box in the 3D view and move it in some corner of the arena.
Select the box and press Ctrl-C, Ctrl-V to copy and paste it.
Shift-click and drag the new box to move it at some different location.
Create a third box this way.
Move the boxes, so that no box is at the center of the arena.
You may also use the green rotation arrows to rotate the boxes along the vertical axis.
This can be done also by shift-click and drag with the right mouse button.
Alternatively, you can change the angle of the `rotation` field of the `WoodenBox` nodes in the scene tree.
Once you are satisfied with the result, save the world using the save button.

Using the translation and rotation handles to move objects is explained in [this section](the-3d-window.md#axis-aligned-handles).
Now your world should look like this:

%figure "What you should at this point of the tutorial."
![tutorial_1.4.png](images/tutorial_1.4.png)
%end

### Add an e-puck Robot

The e-puck is a small robot having differential wheels, 10 [LEDs](../reference/led.md), and several sensors including 8 [DistanceSensors](../reference/distancesensor.md) and a [Camera](../reference/camera.md).
In this tutorial we are only interested in using its wheels.
We will learn how to use other capabilities in the next tutorials.

Now, we are going to add an e-puck model to the world.
Make sure that the simulation is paused and that the virtual time elapsed is 0.
If this is not the case, reset the simulation with the `Reset` button (rewind).

When a Webots world is modified with the intention of being saved, it is fundamental that the simulation is first paused and reloaded to its initial state, i.e. the virtual time counter on the main toolbar should show 0:00:00:000.
Otherwise at each save, the position of each 3D objects can accumulate errors.
Therefore, any modification of the world should be performed in that order: **pause, reset, modify and save the simulation**.

We don't need to create the e-puck robot from scratch, we will just have to import a `E-puck` node.
This node is actually a [PROTO](../reference/proto.md) node, like the `RectangleArena` or the `WoodenBox` we introduced before.

> **Hands on #5**: Select the last node `WoodenBox` of the scene tree view.
Click on the `Add` button (plus sign) at the top of the scene tree view.
In the dialog box, choose `PROTO (Webots) / robots / gctronic / e-puck / E-puck (Robot)`.
An e-puck robot should appear in the middle of the arena.
Move and rotate this robot, the same way you did it with the boxes.
Save the simulation and press the `Run real-time` button (right arrow).

The robot should move, blink LEDs and avoid obstacles.
That's because it has a default controller with that behavior.
Now, while the simulation is running, let's play with the physics:

> **Hands on #6**: Apply a force to the robot by pressing *Alt + left-click + drag*.
On Linux, you should also press the *Ctrl* key in addition to *Alt + left-click + drag*.
It is not possible to apply a force to a `WoodenBox` node, because by default, they have no mass and are considered as glued on the floor.
To enable physics on the `WoodenBox` nodes, you should set their `mass` field to a certain value (for example 0.2 kg).
Once this is done, should be able to apply a force on them as well.

The simulation may be paused (pause button), run step-by-step (step button), in real time (right arrow button), in run (double right arrow button) or in fast (triple right arrow button) modes.
Starting the simulation by pressing the `Run` button will make Webots running the simulation as fast as possible.
In order to obtain a real-time simulation speed, the `Real-Time` button needs to be pressed.

Now we are going to modify the world and decrease the step of the physics simulation: this will increase the accuracy of the simulation.

> **Hands on**: In the Scene Tree view, expand the [WorldInfo](../reference/worldinfo.md) node (the first node).
Set its `basicTimeStep` field to *16*.
Then save the simulation.

Just after you add the E-puck node, a black window appears in the upper-left corner of the 3D view.
It shows the content of [Camera](../reference/camera.md) nodes, but it will stay black until not explicitly used during a simulation.
The camera can be resized by dragging the marked corner or hidden by clicking the "x" in the top-right of the camera window.

> **Hands on**: In this tutorial we will not use the [Camera](../reference/camera.md) devices of the E-puck.
So we can hide the window by clicking the "x" on the camera window.
Don't forget to reload the world before hiding the camera and to save it after the modifications.

### Create a New Controller

We will now program a simple controller that will just make the robot move forwards.
As there is no obstacle, the robot will move forwards for ever.
Firstly we will create and edit the C controller, then we will link it to the robot.

A **controller** is a program that defines the behavior of a robot.
Webots controllers can be written in the following programming languages: C, C++, Java, Python, MATLAB, etc.
Note that C, C++ and Java controllers need to be compiled before they can be run as robot controllers.
Python and MATLAB controllers are interpreted languages so they will run without being compiled.
The `controller` field of a robot specifies which controller is currently linked with to it.
Please take notice that a controller can be used by several robots, but a robot can only use one controller at a time.
Each robot controller is executed in a separate child process spawned by Webots.
Controllers don't share the same address space, and they can run on different processor cores.
Other languages than C are available but may require a setup.
Please refer to the language chapter to setup other languages (see [this chapter](language-setup.md)).

> **Hands on**: Create a new C controller called *e-puck\_go\_forward* using the `Wizards / New Robot Controller...` menu.
This will create a new "e-puck\_go\_forward" directory in "my\_webots\_projects/tutorials/controllers".
Select the option offering you to open the source file in the text editor.

The new C source file is displayed in Webots text editor window.
This C file can be compiled without any modification, however the code has no real effect.
We will now link the E-puck node with the new controller before modifying it.

> **Hands on**: Link the `E-puck` node with the *e-puck\_go\_forward* controller.
This can be done in the Scene Tree view by selecting the `controller` field of the E-puck node, then use the field editor at the bottom of the Scene Tree view: press the `Select...` button and then select *e-puck\_go\_forward* in the list.
Once the controller is linked, save the world.
Modify the program by inserting an include statement (`#include <webots/motor.h>`), getting the motor devices (`WbDeviceTag motor = wb_robot_get_device("motor_name");`), and by applying a motor command (`wb_motor_set_position(motor, 10);`):

> ```c
> #include <webots/robot.h>
>
> // Added a new include file
> #include <webots/motor.h>
>
> #define TIME_STEP 64
>
> int main(int argc, char **argv)
> {
>   wb_robot_init();
>
>   // get the motor devices
>   WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
>   WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
>   // set the target position of the motors
>   wb_motor_set_position(left_motor, 10.0);
>   wb_motor_set_position(right_motor, 10.0);
>
>   while (wb_robot_step(TIME_STEP) != -1);
>
>   wb_robot_cleanup();
>
>   return 0;
> }
> ```
Save the modified source code (`File / Save Text File`), and compile it (`Build / Build`).
Fix any compilation errors if necessary.
When Webots proposes to reload the world, choose `Yes`.

If everything is ok, your robot should move forwards.
The robot will move using it's maximum speed for a while and then stop once the wheels have rotated of 10 radians.

In the "controllers" directory of your project, a directory containing the *e-puck\_go\_forward* controller has been created.
The "e-puck\_go\_forward" directory contains an "e-puck\_go\_forward" binary file generated after the compilation of the controller.
Note that the controller directory name should match with the binary name.

### Extend the Controller to Speed Control

The wheels of differential wheels robots are often controlled in velocity and not in position like we did in the previous example.
In order to control the motors of the wheels in speed you need to set the target position to the infinity and the set the desired speed:

> ```c
> #include <webots/robot.h>
>
> // Added a new include file
> #include <webots/motor.h>
>
> #define TIME_STEP 64
>
> #define MAX_SPEED 6.28
>
> int main(int argc, char **argv)
> {
>   wb_robot_init();
>
>   // get a handler to the motors and set target position to infinity (speed control)
>   WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
>   WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
>   wb_motor_set_position(left_motor, INFINITY);
>   wb_motor_set_position(right_motor, INFINITY);
>
>   // set up the motor speeds at 10% of the MAX_SPEED.
>   wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
>   wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);
>
>   while (wb_robot_step(TIME_STEP) != -1) {
>   }
>
>   wb_robot_cleanup();
>
>   return 0;
> }
> ```

Try to change your previous controller by this one, and then recompile and reload the world.
The robot will now move (the wheels will rotate at a speed of 1 radian per second) and never stop.

### Conclusion

We hope you enjoyed creating your first simulation.
You have been able to set up your environment, to add a robot and to program it.
The important thing is that you learnt the fundamental concepts summarized below:

A Webots world is made up of nodes organized in a VRML97-like tree structure.
A world is saved in a ".wbt" file stored in a Webots project.
The project also contains the robot controllers which are the programs that define the robots' behavior.
Robot controllers can be written in C (or other languages).
C controllers have to be compiled before they can be executed.
Controllers are linked to robots via the `controller` fields of the robot nodes.
