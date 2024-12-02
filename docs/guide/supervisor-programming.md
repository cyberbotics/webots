## Supervisor Programming

The programming examples provided here are in C, but same concepts apply to C++, Java, Python and MATLAB.

### Introduction

If the `supervisor` field of the [Robot](../reference/robot.md) node is set to `TRUE` the `wb_supervisor_*` functions are available in addition to the regular `wb_robot_*` functions.

A [Robot](../reference/robot.md) node whose `supervisor` field is set to `TRUE` can be used as any regular [Robot](../reference/robot.md) node, but in addition, the `wb_supervisor_*` functions can also be used to control the simulation process and modify the Scene Tree.
For example the [Supervisor API](../reference/supervisor.md) can replace human actions such as measuring the distance travelled by a robot or moving it back to its initial position, etc.
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
At each iteration it reads and prints the field's `values`.

%tab-component "language"
%tab "C"
```c
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>

#define TIME_STEP 32

int main() {
  wb_robot_init();

  // do this once only
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");
  if (robot_node == NULL) {
    fprintf(stderr, "No DEF MY_ROBOT node found in the current world file\n");
    exit(1);
  }
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

  while (wb_robot_step(TIME_STEP) != -1) {
    // this is done repeatedly
    const double *values = wb_supervisor_field_get_sf_vec3f(trans_field);
    printf("MY_ROBOT is at position: %g %g %g\n", values[0], values[1], values[2]);
  }

  wb_robot_cleanup();
  return 0;
}
```
%tab-end

%tab "C++"
```cpp
#include <webots/Supervisor.hpp>

#define TIME_STEP 32

using namespace webots;

int main() {
  Supervisor *supervisor = new Supervisor();

  // do this once only
  Node *robot_node = supervisor->getFromDef("MY_ROBOT");
  if (robot_node == NULL) {
    std::cerr << "No DEF MY_ROBOT node found in the current world file" << std::endl;
    exit(1);
  }
  Field *trans_field = robot_node->getField("translation");

  while (supervisor->step(TIME_STEP) != -1) {
    // this is done repeatedly
    const double *values = trans_field->getSFVec3f();
    std::cout << "MY_ROBOT is at position: " << values[0] << ' '
              << values[1] << ' ' << values[2] << std::endl;
  }

  delete supervisor;
  return 0;
}
```
%tab-end

%tab "Python"
```python
from controller import Supervisor
import sys

TIME_STEP = 32

supervisor = Supervisor()

# do this once only
robot_node = supervisor.getFromDef("MY_ROBOT")
if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")

while supervisor.step(TIME_STEP) != -1:
    # this is done repeatedly
    values = trans_field.getSFVec3f()
    print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))
```
%tab-end

%tab "Java"
```java
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;

public class SupervisorController {

  public static void main(String[] args) {

    final int TIME_STEP = 32;

    final Supervisor supervisor = new Supervisor();

    // do this once only
    final Node robot_node = supervisor.getFromDef("MY_ROBOT");
    if (robot_node == null) {
      System.err.println("No DEF MY_ROBOT node found in the current world file");
      System.exit(1);
    }
    final Field trans_field = robot_node.getField("translation");

    while (supervisor.step(TIME_STEP) != -1) {
      // this is done repeatedly
      final double[] values = trans_field.getSFVec3f();
      System.out.format("MY_ROBOT is at position: %g %g %g\n",
                        values[0], values[1], values[2]);
    }
  }
}
```
%tab-end

%tab "MATLAB"
```MATLAB
function supervisor_controller

TIME_STEP = 32;
% do this once only
robot_node = wb_supervisor_node_get_from_def('MY_ROBOT');
if robot_node == 0
  wb_console_print('No DEF MY_ROBOT node found in the current world file', WB_STDERR);
  quit(1);
end
trans_field = wb_supervisor_node_get_field(robot_node, 'translation');

while wb_robot_step(TIME_STEP) ~= -1
  % this is done repeatedly
  values = wb_supervisor_field_get_sf_vec3f(trans_field);
  wb_console_print(sprintf('MY_ROBOT is at position: %g %g %g\n', values(1), values(2), values(3)), WB_STDOUT);
end
```
%tab-end
%end

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

In the code below, the evaluation of the a and b parameters is carried out in the `for` loop.
In the third `for` the comments suggest an assumed function which will call the `wb_motor_set_postion` function for each motor involved in the locomotion.
After each evaluation the distance travelled by the robot is measured and logged.
Then the robot is moved (translation) back to its initial position (0, 0.5, 0) for the next evaluation.
To move the robot we need the `wb_supervisor_*` functions and hence the `supervisor` field of the [Robot](../reference/robot.md) node should be `TRUE`.

%tab-component "language"
%tab "C"
```c
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 32

static int my_exit(void) {
  wb_robot_cleanup();
  return 0;
}

int main() {
  wb_robot_init();

  // get handle to robot's translation field
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

  int a, b;
  for (a = 0; a < 25; ++a) {
    for (b = 0; b < 33; ++b) {
      // evaluate robot during 60 seconds (simulation time)
      const double t = wb_robot_get_time();
      while (wb_robot_get_time() - t < 60.0) {

        // perform robot control according to a, b
        // (and possibly t) parameters.

        // controller termination
        if (wb_robot_step(TIME_STEP) == -1)
          return my_exit();
      }

      // compute travelled distance
      const double *values = wb_supervisor_field_get_sf_vec3f(trans_field);
      double dist = sqrt(values[0] * values[0] + values[2] * values[2]);
      printf("a=%d, b=%d -> dist=%g\n", a, b, dist);

      // reset robot position and physics
      const double INITIAL[3] = { 0, 0.5, 0 };
      wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);
      wb_supervisor_node_reset_physics(robot_node);
    }
  }
  // end of tests
  return my_exit();
}
```
%tab-end

%tab "C++"
```cpp
#include <cmath>
#include <webots/Supervisor.hpp>

#define TIME_STEP 32

using namespace webots;

static int cleanUp(Supervisor *supervisor) {
  delete supervisor;
  return 0;
}

int main() {
  Supervisor *supervisor = new Supervisor();

  // get handle to robot's translation field
  Node *robot_node = supervisor->getFromDef("MY_ROBOT");
  Field *trans_field = robot_node->getField("translation");

  int a, b;
  for (a = 0; a < 25; ++a) {
    for (b = 0; b < 33; ++b) {
      const double t = supervisor->getTime();
      // evaluate robot during 60 seconds (simulation time)
      while (supervisor->getTime() - t < 60.0) {

        // perform robot control according to a, b
        // (and possibly t) parameters.

        // controller termination
        if (supervisor->step(TIME_STEP) == -1)
          return cleanUp(supervisor);
      }

      // compute travelled distance
      const double *values = trans_field->getSFVec3f();
      double dist = sqrt(values[0] * values[0] + values[2] * values[2]);
      std::cout << "a=" << a << ", b=" << b << " -> dist=" << dist << std::endl;

      // reset robot position and physics
      const double INITIAL[3] = {0, 0.5, 0};
      trans_field->setSFVec3f(INITIAL);
      robot_node->resetPhysics();
    }
  }
  // end of tests
  return cleanUp(supervisor);
}
```
%tab-end

%tab "Python"
```python
from math import sqrt
from controller import Supervisor

TIME_STEP = 32

supervisor = Supervisor()

# get handle to robot's translation field
robot_node = supervisor.getFromDef("MY_ROBOT")
trans_field = robot_node.getField("translation")

for a in range(0, 25):
    for b in range(0, 33):
        # evaluate robot during 60 seconds (simulation time)
        t = supervisor.getTime()
        while supervisor.getTime() - t < 60:

            # perform robot control according to a, b
            # (and possibly t) parameters.

            # controller termination
            if supervisor.step(TIME_STEP) == -1:
                quit()

        # compute travelled distance
        values = trans_field.getSFVec3f()
        dist = sqrt(values[0] * values[0] + values[2] * values[2])
        print("a=%d, b=%d -> dist=%g" % (a, b, dist))

        # reset robot position and physics
        INITIAL = [0, 0.5, 0]
        trans_field.setSFVec3f(INITIAL)
        robot_node.resetPhysics()
```
%tab-end

%tab "Java"
```java
import java.lang.Math;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Supervisor;

public class SupervisorController {

  public static void main(String[] args) {

    final int TIME_STEP = 32;
    final Supervisor supervisor = new Supervisor();

    // get handle to robot's translation field
    final Node robot_node = supervisor.getFromDef("MY_ROBOT");
    final Field trans_field = robot_node.getField("translation");

    int a, b;
    for (a = 0; a < 25; ++a) {
      for (b = 0; b < 33; ++b) {
        // evaluate robot during 60 seconds (simulation time)
        final double t = supervisor.getTime();
        while (supervisor.getTime() - t < 60.0) {

          // perform robot control according to a, b
          // (and possibly t) parameters.

          // controller termination
          if (supervisor.step(TIME_STEP) == -1)
            return ;
        }

        // compute travelled distance
        double[] values = trans_field.getSFVec3f();
        double dist = Math.sqrt(values[0] * values[0] + values[2] * values[2]);
        System.out.format("a=%d, b=%d -> dist=%g\n", a, b, dist);

        // reset robot position and physics
        double INITIAL[] = {0.0, 0.5, 0.0};
        trans_field.setSFVec3f(INITIAL);
        robot_node.resetPhysics();
      }
    }
  }
}
```
%tab-end

%tab "MATLAB"
```MATLAB
function supervisor_controller

TIME_STEP = 32;

% get handle to robot's translation field
robot_node = wb_supervisor_node_get_from_def('MY_ROBOT');
trans_field = wb_supervisor_node_get_field(robot_node, 'translation');

for a = 0:25
  for b = 0:33
    % evaluate robot during 60 seconds (simulation time)
    t = wb_robot_get_time();
    while wb_robot_get_time() - t < 60
      % perform robot control according to a, b
      % (and possibly t) parameters.
      if wb_robot_step(TIME_STEP) == -1
        wb_robot_cleanup();
        quit(0);
      end
    end
	
    % compute travelled distance
    values = wb_supervisor_field_get_sf_vec3f(trans_field);
    dist = sqrt((values(1) * values(1)) + (values(3) * values(3)));
    wb_console_print(sprintf('a=%g, b=%g -> dist=%g\n', a, b, dist), WB_STDOUT);
	
    % reset robot position and physics
    INITIAL = [0, 0.5, 0];
    wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);
    wb_supervisor_node_reset_physics(robot_node);
  end
end
```
%tab-end
%end

As in the previous example, the `trans_field` variable is a `WbFieldRef` that identifies the `translation` field of the robot.
In this example the `trans_field` is used both to get (using the `wb_supervisor_field_get_sf_vec3f` function) and to set (using the `wb_supervisor_field_set_sf_vec3f` function) the field's value.

Please note that the program structure is composed of three nested `for` loops.
The two outer loops change the values of the a and b parameters.
The innermost loop makes the robot walk during 60 seconds.
One important point here is that the `wb_robot_step` function call is placed in the innermost loop.
This allows the motor positions to be updated at each iteration of the loop.
If the `wb_robot_step` function call was placed anywhere else, this would not work.
