## Programming

### How Can I Get the 3D Position of a Robot/Object?

There are different functions depending whether this information must be accessed in a normal controller, in a [Supervisor](../reference/supervisor.md) controller or in a physics plugin.
All the functions described below will return the 3D position in meters and expressed in the global (world) coordinate system.

Clearly, the position of a robot can also be approximated by using *odometry* or *SLAM* techniques.
This is usually more realistic because most robots don't have a [GPS](../reference/gps.md) and therefore have no mean of precisely determining their position.
You will find more info about *odometry* and *SLAM* techniques in `Cyberbotics' Robot Curriculum`.

#### Get Position in Controller Code:

To get the position of a robot in the robot's controller code: add a [GPS](../reference/gps.md) node to the robot, then use the `wb_robot_get_device`, `wb_gps_enable` and `wb_gps_get_values` functions.
Note that the [GPS](../reference/gps.md)'s resolution field must be 0 (the default), otherwise the results will be noisy.
You will find more info about the [GPS](../reference/gps.md) node and functions in the [Reference Manual](../reference/gps.md).
Note that the [GPS](../reference/gps.md) can also be placed on a robot's part (arm, foot, etc.) to get the world/global coordinates of that particular part.

#### Get Position in Supervisor Code:

1. To get the 3D position of any [Transform](../reference/transform.md) (or derived) node using the  [Supervisor API](../reference/supervisor.md): you can use the `wb_supervisor_node_get_position` function.
Please check this function's description in the [Reference Manual](../reference/supervisor.md).
2. To get the 3D position of any [Transform](../reference/transform.md) (or derived) node placed at the root of the Scene Tree (the nodes visible when the Scene Tree is completely collapsed), you can use the `wb_supervisor_field_get_sf_vec3f` function.
Here is an [example](supervisor-programming.md#tracking-the-position-of-robots).

A simulation example that shows both the [GPS](../reference/gps.md) and the [Supervisor](../reference/supervisor.md) APIs techniques is included in the Webots installation, you just need to open this world: "[WEBOTS\_HOME/projects/samples/devices/worlds/gps.wbt](https://github.com/cyberbotics/webots/tree/master/projects/samples/devices/worlds/gps.wbt)".

#### Get Position in Physics Plugin Code:

In the physics plugin you can use ODE's `dBodyGetPosition` function.
Note that this function returns the position of the center of mass of the body: this may be different from the center of the [Solid](../reference/solid.md).
Please find a description of ODE functions [here](http://ode.org/wiki/index.php?title=Manual).

### How Can I Get the Linear/Angular Speed/Velocity of a Robot/Object?

Webots provides several functions to get the 3D position of a robot or an object (see above): by taking the first derivative of the position you can determine the velocity.
There are also some functions (see below) that can be used to get the velocity directly:

#### Get Velocity in Controller Code:

To get the angular velocity of a robot (or robot part) in the robot's controller code: add a [Gyro](../reference/gyro.md) node to the robot (or robot part), then use the `wb_robot_get_device`, `wb_gyro_enable` and `wb_gyro_get_values` functions.
You will find more information about the [Gyro](../reference/gyro.md) node and functions in the [Reference Manual](../reference/gyro.md).

#### Get Velocity in Supervisor:

Using the `wb_supervisor_node_get_velocity` function it is possible to retrieve both the linear and angular velocity of any [Solid](../reference/solid.md) node.
You will find more information about this function in the [Reference Manual](../reference/supervisor.md).

#### Get Velocity in Physics Plugin Code:

In the physics plugin you can use the ODE's `dBodyGetLinearVel` and `dBodyAngularVel` functions.
These functions return the linear velocity in meters per second, respectively the angular velocity in radians per second.
Please find a description of ODE functions here: [here](http://ode.org/wiki/index.php?title=Manual).

### How Can I Reset My Robot?

Please see [this section](using-numerical-optimization-methods.md#resetting-the-robot).

### What Does This Mean: "Could not find controller {...} Loading void controller instead." ?

This message means that Webots could neither find an executable file (e.g. `.exe`), nor an interpreted language file (e.g. `.class`, `.py`, `.m`) to run as controller program for a robot.
In fact, Webots needs each controller file to be stored at specific location in order to be able to executed it.
The requested location is in the "controllers" subdirectory of the current Webots project directory, e.g. "my\_project".
Inside the "controllers" directory, each controller project must be stored in its own directory which must be named precisely like the `controller` field of the [Robot](../reference/robot.md).
Inside that directory, the executable/interpretable file must also be named after the `controller` field of the [Robot](../reference/robot.md) (plus a possible extension).
For example if the controller field of the robot looks like this, in the Scene Tree:

```
Robot {
  controller "my_controller"
}
```

Then the executable/interpretable file will be searched at the following paths:

```
my_project/controllers/my_controller/my_controller.exe (Windows only)
my_project/controllers/my_controller/my_controller (Linux/Mac only)
my_project/controllers/my_controller/my_controller.class
my_project/controllers/my_controller/my_controller.py
my_project/controllers/my_controller/my_controller.m
```

If Webots does not find any file at the above specified paths, then the error message in question is shown.
So this problem often happens when you:

- Have moved the project or source files to a location that does not correspond to the above description.
- Use an external build system, e.g. Visual Studio, that is not configured to generate the executable file at the right location.
- Have changed the [Robot](../reference/robot.md)'s controller field to a location where no executable/interpretable file can be found.
- Have "reloaded" the world after "cleaning" of the controller project.

### What Does This Mean: "Warning: invalid WbDeviceTag in API function call" ?

A `WbDeviceTag` is an abstract reference (or handle) used to identify a simulated device in Webots.
Any `WbDeviceTag` must be obtained from the `wb_robot_get_device` function.
Then, it is used to specify a device in various Webots function calls.
Webots issues this warning when the `WbDeviceTag` passed to a Webots function appears not to correspond to a known device.
This can happen mainly for three reasons:

1. The `WbDeviceTag` is 0 and thus invalid because it was not found by the `wb_robot_get_device` function call.
Indeed, the `wb_robot_get_device` function returns 0, if it cannot not find a device with the specified name in the robot.
Note that the name specified in the argument of the `wb_robot_get_device` function must correspond to the `name` field of the device, not to the VRML97 DEF name! 2.
Your controller code is mixing up two types of `WbDeviceTag`s, for example because it uses the `WbDeviceTag` of a [Camera](../reference/camera.md) in a `wb_distance_sensor_*` function.
Here is an example of what is wrong:

> ```c
> #include <webots/robot.h>
> #include <webots/camera.h>
> #include <webots/distance_sensor.h>
>
> #define TIME_STEP 32
>
> int main() {
>   wb_robot_init();
>   WbDeviceTag camera = wb_robot_get_device("camera");
>   wb_camera_enable(camera, TIME_STEP);
>   ...
>   double value = wb_distance_sensor_get_value(camera);  // WRONG!
>   ...
> }
> ```

3. The `WbDeviceTag` may also be invalid because it is used before initialization with the `wb_robot_get_device` function call, or because it is not initialized at all, or because it is corrupted by a programming error in the controller code.
Here is such an example:

> ```c
> #include <webots/robot.h>
> #include <webots/camera.h>
> #include <webots/distance_sensor.h>
>
> #define TIME_STEP 32
>
> int main() {
>   wb_robot_init();
>   WbDeviceTag distance_sensor, camera = wb_robot_get_device("camera");
>   wb_camera_enable(camera, TIME_STEP);
>   wb_distance_sensor_enable(distance_sensor, TIME_STEP);  // WRONG!
>   ...
> }
> ```

### Is It Possible to Apply a (User Specified) Force to a Robot?

Yes.
You need to use a *physics plugin* to apply user specified forces (or torques).
Then you can add the physics plugin with the menu item: `Wizards` > `New Physics Plugin`.
After having added the plugin you must compile it using Webots editor.
Then you must associate the plugin with your simulation world.
This can be done by editing the `WorldInfo.physics` field in the Scene Tree.
Then you must modify the plugin code such as to add the force.
Here is an example:

```c
#include <ode/ode.h>
#include <plugins/physics.h>

dBodyID body = NULL;

void webots_physics_init() {
  // find the body on which you want to apply a force
  body = dWebotsGetBodyFromDEF("MY_ROBOT");
  ...
}

void webots_physics_step() {
   ...
   dVector3 f;
   f[0] = ...
   f[1] = ...
   f[2] = ...
   ...
   // at every time step, add a force to the body
   dBodyAddForce(body, f[0], f[1], f[2]);
   ...
}
```

There is more info on the plugin functions in the [Reference Manual](../reference/physics-plugin.md).
Additional information about the ODE functions can be found [here](http://ode.org/wiki/index.php?title=Manual).
You may also want to study this example distributed with Webots:

```
WEBOTS_HOME/projects/samples/demos/worlds/salamander.wbt
```

In this example, the physics plugin adds user computed forces to the robot body in order to simulate Archimedes and hydrodynamic drag forces.

### How Can I Draw in the 3D Window?

There are different techniques depending on what you want to draw:

1. If you just want to add some 2d text, you can do this by using the function: `wb_supervisor_set_label`.
This will allow you to put 2d overlay text in front of the 3d simulation.
Please lookup for the [Supervisor API](../reference/supervisor.md) documentation.
2. If you want to add a small sub-window in front of the 3d graphics, you should consider using the [Display](../reference/display.md) node.
This will allow you to do 2d vector graphics and text.
This is also useful for example to display processed camera images.
Please lookup for the [Display](../reference/display.md) node documentation.
3. If you want add 3d graphics to the main window, this can be done by using the [Supervisor API](../reference/supervisor.md).
The [Supervisor API](../reference/supervisor.md) can be used to create new nodes - meaning that you can create an [IndexedFaceSet](../reference/indexedfaceset.md) or [IndexedLineSet](../reference/indexedlineset.md) and adjust vertex positions or indexing accordingly to create and update the shape you wish to draw in the 3D scene.

### What Does This Mean: "The time step used by controller {...} is not a multiple of WorldInfo.basicTimeStep!"?

Webots allows to specify the *control step* and the *simulation step* independently.
The control step is the argument passed to the `wb_robot_step` function, it specifies the duration of a step of control of the robot.
The *simulation step* is the value specified in `WorldInfo.basicTimeStep` field, it specifies the duration of a step of integration of the physics simulation, in other words: how often the objects motion must be recomputed.
The execution of a simulation step is an atomic operation: it cannot be interrupted.
Hence a sensor measurement or a motor actuation must take place between two simulation steps.
For that reason the control step specified with each `wb_robot_step` function call must be a multiple of the simulation step.
If it is not the case you get this error message.
So, for example if the `WorldInfo.basicTimeStep` is 16 (ms), then the control step argument passed to the `wb_robot_step` function can be 16, 32, 48, 64, 80, 128, 1024, etc.

### How Can I Detect Collisions?

Webots does automatically detect collisions and apply the contact forces whenever necessary.
The collision detection mechanism is based on the shapes specified in the `boundingObject`s.
Now if you want to programmatically detect collision, there are several methods:

- In controller code: you can detect collision by using [TouchSensor](../reference/touchsensor.md)s placed around your robot body or where the collision is expected.
You can use [TouchSensor](../reference/touchsensor.md)s of type "bumper" that return a boolean status 1 or 0, whether there is a collision or not.
In fact a "bumper" [TouchSensor](../reference/touchsensor.md) will return 1 when its `boundingObject` intersects another `boundingObject` and 0 otherwise.
- In supervisor code: you can detect collisions by tracking the position of robots using the `wb_supervisor_field_get_*` functions.
Here is a naive example assuming that the robots are cylindrical and moving in the xz-plane.

> ```c
> #define ROBOT_RADIUS ...
> ...
> int are_colliding(WbFieldRef trans1, WbFieldRef trans2) {
>   const double *p1 = wb_supervisor_field_get_sf_vec3f(trans1);
>   const double *p2 = wb_supervisor_field_get_sf_vec3f(trans2);
>   double dx = p2[0] - p1[0];
>   double dz = p2[2] - p1[2];
>   double dz = p2[2] - p1[2];
>   return sqrt(dx * dx + dz * dz) < 2.0 * ROBOT_RADIUS;
> }
>   ...
>   // do this once only, in the initialization
>   WbNodeRef robot1 = wb_supervisor_node_get_from_def("MY_ROBOT1");
>   WbNodeRef robot2 = wb_supervisor_node_get_from_def("MY_ROBOT2");
>   WbFieldRef trans1 = wb_supervisor_node_get_field(robot1, "translation");
>   WbFieldRef trans2 = wb_supervisor_node_get_field(robot2, "translation");
>   ...
>   // detect collision
>   if (are_colliding(trans1, trans2)) {
>     ...
>   }
> ```

- In the physics plugin: you can replace or extend Webots collision detection mechanism.
This is an advanced technique that requires knowledge of the [ODE (Open Dynamics Engine) API](http://ode.org/wiki/index.php?title=Manual).
Your collision detection mechanism must be implemented in the `webots_physics_collide` function.
This function is described in the [Physics Plugin](../reference/physics-plugin.md) chapter of the [Reference Manual](../reference/physics-plugin.md).

### Why Does My Camera Window Stay Black?

The content of the camera window will appear only after all the following steps have been completed:

1. The [Camera](../reference/camera.md)'s `name` field was specified.
2. The `WbDeviceTag` for the [Camera](../reference/camera.md) node was found using the `wb_robot_get_device` function called with the corresponding [Camera](../reference/camera.md)'s `name`.
3. The [Camera](../reference/camera.md) was enabled using the `wb_camera_enable` function and a refresh rate of `r` milliseconds.
4. One or several `wb_robot_step` function (or equivalent function) calls have been called covering a time span of at least `r` milliseconds.
5. The `wb_camera_get_image` function was called.
