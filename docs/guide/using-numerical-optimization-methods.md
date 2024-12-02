## Using Numerical Optimization Methods

### Choosing the Correct Supervisor Approach

There are several approaches to using optimization algorithms in Webots.
Most approaches rely on a [Supervisor](../reference/supervisor.md) controller.

A numerical optimization can usually be decomposed in two separate tasks:

1. Running the optimization algorithm: Systematical Search, Random Search, Genetic Algorithms (GA), Particle Swarm Optimization (PSO), Simulated Annealing, etc.
2. Running the robot behavior with a set of parameters specified by the optimization algorithm.

One of the important things that needs to be decided is whether the implementation of these two distinct tasks should go into the same controller or in two separate controllers.
Let's discuss both approaches:

#### Using a Single Controller

If your simulation needs to evaluate only one robot at a time, e.g. you are optimizing the locomotion gait of a humanoid or the behavior of a single robot, then it is possible to have both tasks implemented in the same controller; this results in a somewhat simpler code.
Here is a pseudo-code example for the systematical optimization of two parameters *a* and *b* using only one controller:

```c
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 5

int main() {
  wb_robot_init();
  double a, b, time;
  for (a = 0.5; a < 10.0; a += 0.1) {
    for (b = 0.1; b < 5.0; b += 0.5) {
      resetRobot();  // move robot to initial position

      // run robot simulation for 30 seconds
      for (time = 0.0; time < 30.0; time += TIME_STEP / 1000.0) {
        actuateMotors(a, b, time);
        if (wb_robot_step(TIME_STEP) != -1)
          goto my_exit;
      }

      // compute and print fitness
      double fitness = computeFitness();
      printf("with parameters: %g %g, fitness was: %g\n", a, b, fitness);
    }
  }

my_exit:
  wb_robot_cleanup();
  return 0;
}
```

In this example the robot runs for 30 simulated seconds and then the fitness is evaluated and the robot is moved back to it initial position.
Note that this controller needs to be executed in a [Robot](../reference/robot.md) node whose `supervisor` field is set to `TRUE` in order to access the `wb_supervisor_field_*` functions that are necessary to read and reset the robot's position.

#### Using Two Distinct Types of Controllers

If, on the contrary, your simulation requires the simultaneous execution of several robots, e.g. swarm robotics, it is advised to use two distinct types of controller: one for the optimization algorithm and one for the robot's behavior.
The optimization algorithm should go in a [Supervisor](../reference/supervisor.md) controller while the robots' behavior can go in a regular (non-Supervisor) controller.

Because these controllers will run in separate system processes, they will not be able to access each other's variables.
Though, they will have to communicate by some other means in order to specify the sets of parameters that need to be evaluated.
It is possible, and recommended, to use Webots [Emitters](../reference/emitter.md) and [Receivers](../reference/receiver.md) to exchange information between the [Supervisor](../reference/supervisor.md) controller and the other controllers.
For example, in a typical scenario, the [Supervisor](../reference/supervisor.md) controller will send evaluation parameters (e.g., genotype) to the robot controllers.
The robot controllers listen to their [Receivers](../reference/receiver.md), waiting for a new set of parameters.
Upon receipt, a robot controller starts executing the behavior specified by the set of parameters.
In this scenario, the [Supervisor](../reference/supervisor.md) controller needs an [Emitter](../reference/emitter.md) and each individual robot needs a [Receiver](../reference/receiver.md).

Depending on the algorithms needs, the fitness could be evaluated either in the [Supervisor](../reference/supervisor.md) controller or in the individual robot controllers.
In the case it is evaluated in the robot controller then the fitness result needs to be sent back to the [Supervisor](../reference/supervisor.md) controller.
This bidirectional type of communication requires the usage of additional [Emitters](../reference/emitter.md) and [Receivers](../reference/receiver.md).

### Resetting the Robot

When using optimization algorithm, you will probably need to reset the robot after or before each fitness evaluation.
There are several approaches to resetting the robot:

#### Using the wb\_supervisor\_field\_set\_* and wb\_supervisor\_simulation\_reset\_physics Functions

You can easily reset the position, orientation and physics of the robot using the `wb_supervisor_field_set...` and `wb_supervisor_simulation_reset_physics` functions, here is an example:

```c
// get handles to the robot's translation and rotation fields
WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");
WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");

// reset the robot
const double INITIAL_TRANS[3] = { 0, 0.5, 0 };
const double INITIAL_ROT[4] = { 0, 1, 0, 1.5708 };
wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL_TRANS);
wb_supervisor_field_set_sf_rotation(rot_field, INITIAL_ROT);
wb_supervisor_simulation_reset_physics();
```

The drawback with the above method is that it only resets the robot's main position and orientation.
This may be fine for some types of optimization, but insufficient for others.
Although it is possible to add more parameters to the set of data to be reset, it is sometimes difficult to reset everything.
Neither motor positions, nor the robot controller(s) are reset this way.
The motor positions should be reset using the `wb_motor_set_position` function and the robot controller should be reset by sending a message from the supervisor process to the robot controller process (using Webots [Emitter](../reference/emitter.md) / [Receiver](../reference/receiver.md) communication system).
The robot controller program should be able to handle such a message and reset its state accordingly.

#### Using the wb\_supervisor\_world\_reload Function

This function restarts the physics simulation and all controllers from the very beginning.
With this method, everything is reset, including the physics and the motor positions and the controllers.
But this function does also restart the controller that called the `wb_supervisor_world_reload` function, this is usually the controller that runs the optimization algorithm, and as a consequence the optimization state is lost.
Hence for using this technique, it is necessary to develop functions that can save and restore the complete state of the optimization algorithm.
The optimization state should be saved before calling the `wb_supervisor_world_reload` function and reloaded when the [Supervisor](../reference/supervisor.md) controller restarts.
Here is a pseudo-code example:

```c
#include <webots/robot.h>
#include <webots/supervisor.h>

void run_robot(const double params[]) {
  read_sensors(params);
  compute_behavior(params):
  actuate_motors(params);
}

void evaluate_next_robot() {
  const double *params = optimizer_get_next_parameters();
  ...
  // run robot for 30 seconds
  double time;
  for (time = 0.0; time < 30.0; time += TIME_STEP / 1000.0) {
    run_robot(params);
    if (wb_robot_step(TIME_STEP) == -1)
      break;
  }
  ...
  // compute and store fitness
  double fitness = compute_fitness();
  optimizer_set_fitness(fitness);
  ...
  // save complete optimization state to a file
  optimizer_save_state("my_state_file.txt");
  ...
  // start next evaluation
  wb_supervisor_world_reload();
  wb_robot_step(TIME_STEP);
  exit(0);
}

int main() {
  wb_robot_init();
  ...
  // reload complete optimization state
  optimizer_load_state("my_state_file.txt");
  ...
  if (optimizer_has_more_parameters())
    evaluate_next_robot();
  ...
  wb_robot_cleanup();
  return 0;
}
```

If this technique is used with Genetic Algorithms for example, then the `optimizer_save_state` function should save at least all the genotypes and fitness results of the current GA population.
If this technique is used with Particle Swarm Optimization, then the `optimizer_save_state` function should at least save the position, velocity and fitness of all particles currently in the swarm.

#### Using the wb\_supervisor\_world\_reset Function

Similarly to the [`wb_supervisor_world_reload`](../reference/supervisor.md#wb_supervisor_world_reload) function, this function resets the physics simulation.
However, it does not restart the controllers.
The advantage of this function is that it is possible to restart only the desired controllers using the [`wb_supervisor_node_restart_controller`](../reference/supervisor.md#wb_supervisor_node_restart_controller) function.
Typically, you will restart the [Robot](../reference/robot.md) controllers but not the [Supervisor](../reference/supervisor.md) one.
Thus, there is no need for the [Supervisor](../reference/supervisor.md) controller to save and restore the complete state of the optimization algorithm.

#### By Starting and Quitting Webots

Finally, the last method is to start and quit the Webots program for each parameter evaluation.
This may sound like an overhead, but in fact Webots startup time is usually very short compared to the time necessary to evaluate a controller, so this approach makes perfectly sense.

For example, Webots can be called from a shell script or from any type of program suitable for running the optimization algorithm.
Starting Webots each time does clearly reload the world completely, so each robot will start from the same initial state.
The drawback of this method is that the optimization algorithm has to be programmed outside of Webots.
This external program can be written in any programming language, e.g. shell script, C, PHP, perl, etc., provided that there is a way to call Webots and wait for its termination, e.g. like the C standard `system` function does.
On the contrary, the parameter evaluation must be implemented in a Webots controller.

With this approach, the optimization algorithm and the robot controller(s) run in separate system processes, but they must communicate with each other in order to exchange parameter sets and fitness results.
One simple way is to make them communicate by using text files.
For example, the optimization algorithm can write the genotypes values into a text file then call Webots.
When Webots starts, the robot controller reads the genotype file and carries out the parameter evaluation.
When the robot controller finishes the evaluation, it writes the fitness result into another text file and then it calls the `wb_supervisor_simulation_quit` function to terminate Webots.
Then the control flow returns to the optimization program that can read the resulting fitness, associate fitness with the current genotype and proceed with the next genotype.

Here is a possible (pseudo-code) implementation for the robot evaluation controller:

```c
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 10

double genotype[GENOME_SIZE];

int main() {
  wb_robot_init();
  ...
  genotype_read("genotype.txt", genotype);
  ...
  // run evaluation for 30 seconds
  for (double time = 0.0; time < 30.0; time += TIME_STEP / 1000.0) {
    read_sensors(genotype);
    actuate_motors(time, genotype);
    if (wb_robot_step(TIME_STEP) == -1)
      break;
  }
  ...
  double fitness = compute_fitness();
  fitness_save(fitness, "fitness.txt");
  ...
  wb_supervisor_simulation_quit();
  wb_robot_step(TIME_STEP);
  return 0;
}
```

You will find complete examples of simulations using optimization techniques in Webots distribution: look for the worlds called "advanced\_particle\_swarm\_optimization.wbt" and "advanced\_genetic\_algorithm.wbt" located in the "[WEBOTS\_HOME/projects/samples/curriculum/worlds]({{ url.github_tree }}/projects/samples/curriculum/worlds)" directory.
These examples are described in the *Advanced Programming Exercises* of [Cyberbotics' Robot Curriculum](http://en.wikibooks.org/wiki/Cyberbotics'_Robot_Curriculum).
