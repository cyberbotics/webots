## Supervisor Programming

The programming examples provided here are in C, but same concepts apply to C++, Java, Python and MATLAB.

### Introduction

If the `supervisor` field of the [Robot](../reference/robot.md) node is set to `TRUE` the `wb_supervisor_*` functions are available in addition to the regular `wb_robot_*` functions.

A [Robot](../reference/robot.md) node whose `supervisor` field is set to `TRUE` can be used as any regular [Robot](../reference/robot.md) node, but in addition, the `wb_supervisor_*` functions can also be used to control the simulation process and modify the Scene Tree.
For example the [Supervisor AP](../reference/supervisor.md) can replace human actions such as measuring the distance travelled by a robot or moving it back to its initial position, etc.
The [Supervisor API](../reference/supervisor.md) can also be used to take a screen shot or a video of the simulation, restart or terminate the simulation, etc.
It can read or modify the value of every fields in the Scene Tree, e.g. read or change the position of robots, the color of objects, or switch on or off the light sources, and do many other useful things.

One important thing to keep in mind is that the [Supervisor API](../reference/supervisor.md) corresponds to functionalities that are usually not available on real robots; it rather corresponds to a human intervention on the experimental setup.
Hence, the `wb_robot_*` vs. `wb_supervisor_*` distinction is intentional and aims at reminding the user that [Supervisor API](../reference/supervisor.md) functions may not be easily transposed to real robots.

Now let's examine a few examples of [Supervisor](../reference/supervisor.md) code.

### Tracking the Position of Robots

The [Supervisor API](../reference/supervisor.md) is frequently used to record robots trajectories.
Of course, a robot can find its position using a [GPS](../reference/gps.md), but when it is necessary to keep track of several robots simultaneously and in a centralized way, it is much simpler to use the [Supervisor API](../reference/supervisor.md).

The following [Supervisor](../reference/supervisor.md) code shows how to keep track of a single robot, but this can easily be transposed to an arbitrary number of robots.
This example code finds a `WbNodeRef` that corresponds to the robot node and then a `WbFieldRef` that corresponds to the robot's `translation` field.
At each iteration it reads and prints the field's values.

```c
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>

int main() {
  wb_robot_init();

  // do this once only
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

  while (wb_robot_step(32) != -1) {
    // this is done repeatedly
    const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
    printf("MY_ROBOT is at position: %g %g %g\n", trans[0], trans[1], trans[2]);
  }

  wb_robot_cleanup();

  return 0;
}
```

Note that the [Supervisor API](../reference/supervisor.md) is defined in the `supervisor.h` header file which should be included in addition to the `robot.h` header file.
Otherwise a [Supervisor](../reference/supervisor.md) controller works like a regular [Robot](../reference/robot.md) controller and everything that was explained in the "Controller Programming" section does also apply to "Supervisor Programming".

As illustrated by the example, it is better to get the `WbNodeRef`s and `WbFieldRef`s only once, at the beginning of the simulation (keeping the invariants out of the loop).
The `wb_supervisor_node_get_from_def` function searches for an object named "MY\_ROBOT" in the Scene Tree.
Note that the name in question is the DEF name of the object, not the name field which is used to identify devices.
The function returns a `WbNodeRef` which is an opaque and unique reference to the corresponding Scene Tree node.
Then the call to `wb_supervisor_node_get_field` function finds a `WbFieldRef` in the specified node.
The "translation" field represents the robot's position in the global (world) coordinate system.

In the `while` loop, the `wb_supervisor_field_get_sf_vec3f` function call is used to read the latest values of the specified field.
Note that, unlike sensor or actuator functions, the `wb_supervisor_field_*` functions are executed immediately: their execution is not postponed to the next `wb_robot_step` function call.

### Setting the Position of Robots

Now let's examine a more sophisticated [Supervisor](../reference/supervisor.md) example.
In this example we seek to optimize the locomotion of a robot: it should walk as far as possible.
Suppose that the robot's locomotion depends on two parameters (a and b), hence we have a two-dimensional search space.

In the code, the evaluation of the a and b parameters is carried out in the `while` loop.
The `actuateMotors` function here is assumed to call the `wb_motor_set_postion` function for each motor involved in the locomotion.
After each evaluation the distance travelled by the robot is measured and logged.
Then the robot is moved (translation) back to its initial position (0, 0.5, 0) for the next evaluation.
To move the robot we need the `wb_supervisor_*` functions and hence the `supervisor` field of the [Robot](../reference/robot.md) node should be `TRUE`.

```c
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 32

int main() {
  wb_robot_init();

  // get handle to robot's translation field
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

  double a, b, t;
  for (a = 0.0; a < 5.0; a += 0.2) {
    for (b = 0.0; b < 10.0; b += 0.3) {
      // evaluate robot during 60 seconds (simulation time)
      for (t = 0.0; t < 60.0; t += TIME_STEP / 1000.0) {
        actuateMotors(a, b, t);
        if (wb_robot_step(TIME_STEP) != -1)
          goto my_exit;
      }

      // compute travelled distance
      const double *pos = wb_supervisor_field_get_sf_vec3f(trans_field);
      double dist = sqrt(pos[0] * pos[0] + pos[2] * pos[2]);
      printf("a=%g, b=%g -> dist=%g\n", a, b, dist);

      // reset robot position
      const double INITIAL[3] = { 0, 0.5, 0 };
      wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);
    }
  }

my_exit:
  wb_robot_cleanup();

  return 0;
}
```

As in the previous example, the `trans_field` variable is a `WbFieldRef` that identifies the `translation` field of the robot.
In this example the `trans_field` is used both to get (using the `wb_supervisor_field_get_sf_vec3f` function) and to set (using the `wb_supervisor_field_set_sf_vec3f` function) the field's value.

Please note that the program structure is composed of three nested `for` loops.
The two outer loops change the values of the a and b parameters.
The innermost loop makes the robot walk during 60 seconds.
One important point here is that the `wb_robot_step` function call is placed in the innermost loop.
This allows the motor positions to be updated at each iteration of the loop.
If the `wb_robot_step` function call was placed anywhere else, this would not work.
