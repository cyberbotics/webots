## Modeling

### My Robot/Simulation Explodes or Behaves Weirdly, What Should I Do?

The explosion is usually caused by inappropriate values passed to the physics engine (ODE).
There are many things you can be try to improve the stability of the simulation (adapted from ODE's User Guide):

1. Reduce the value of `WorldInfo.basicTimeStep`.
This will also make the simulation slower, so a tradeoff has to be found.
Note that the value of the control step (`wb_robot_step(TIME_STEP)`) may have to be adapted to avoid warnings.
2. Reduce the value of the `JointParameters.springConstant` and `JointParameters.dampingConstant` fields or avoid using springs and dampers at all.
3. Avoid large mass ratios.
A `Joint` that connects a large and a small mass (`Physics.mass`) together will have a hard time to keep its error low.
For example, using a `Joint` to connect a hand and a hair may be unstable if the hand/hair mass ratio is large.
4. Increase the value of `WorldInfo.CFM`.
This will make the system numerically more robust and less susceptible to stability problems.
This will also make the system look more *spongy* so a tradeoff has to be found.
5. Avoid making robots (or other objects) move faster than reasonably for the time step (`WorldInfo.basicTimeStep`).
Since contact forces are computed and applied only at every time step, too fast moving bodies can penetrate each other in unrealistic ways.
6. Avoid building mechanical loops by using [Connector](../reference/connector.md) nodes.
The mechanical loops may cause constraints to fight each other and generate strange forces in the system that can swamp the normal forces.
For example, an affected body might fly around as though it has life on its own, with complete disregard for gravity.
7. Avoid thin objects.
The problem with thin objects is the following: since the simulation time increases step-by-step, if the time step is too large and a thin object is moving fast, it is possible that at step _t_, the thin object does not collide and at step _t+1_ it is already completely inside another object.
In that case, ODE (the physics engine of Webots) might not know on which side to push back the thin object and this might result in big instabilities.
Similarly, you can think of a bullet fired into a wall.
If the simulation step is too large, it is completely possible that the bullet goes through the wall without even noticing it.
At time _t_, it will be on one side of the wall and at time _t+1_, it will be already on the other side, so no step is producing a collision.

### How to Make Replicable/Deterministic Simulations?

In order for Webots simulation results to be reproducible, the following conditions must be fulfilled:

1. Each simulation must be executed with the same version of the Webots software, on the same machine.
Running the same simulation on different machines or different Webots versions may result in small numerical differences.
However, it is possible to run fully reproducible results on different machines if these machines have the same operating system (same version, same updates), same graphics card and same graphics driver.
It turns out that different OpenGL drivers or different graphics cards produce slightly different 3D rendering results.
So, if your simulation doesn't rely on OpenGL rendering (no camera, lidar or range-finder), it should be reproducible on machines with a different graphics configuration.
2. Each simulation must be restarted either by pushing the `Reload` button, or by using the `wb_supervisor_world_reload` function, or by restarting Webots.
The random seeds used by Webots internally are reset for each simulation restarted with one of the above methods.
3. The `synchronization` flag of every robot and supervisor must be TRUE.
Otherwise, the number of physics steps per control step may vary with the current CPU load and hence the robot's behavior may also vary.
4. The controllers (and physics plugin) code must also be deterministic.
In particular, that code must not use a pseudo random generator initialized with a non-deterministic seed such as the system time.
For example this is not suitable for replicable experiments: `srand(time(NULL))`.
Note that uninitialized variables may also be a source of undeterministc behavior.
5. Webots physics must run in single thread mode.
The number of threads used by the physics engine (ODE) can be changed either globally in the [preferences](preferences.md) or using the `WorldInfo.optimalThreadCount` field.
It should be set to 1.
6. The Webots random number generator should have a fixed seed.
The seed is defined in the `WorldInfo.randomSeed` field, it should be non-negative to avoid non-replicable time based seed.

If the six above conditions are met, Webots simulations become replicable.
This means that after the same number of steps two simulations will have exactly the same internal state.
Hence if both simulations are saved using the `Save as...` button, the resulting files will be identical.
This is true independently of the simulation mode used to execute the simulation: `Step`, `Real-Time`, `Run` or `Fast`.
This is also true whether or not sensor noise is used (see below).

### My Robot Sinks into the Ground, How to Remove This Effect?

#### My Robot Is Heavy and Sinks into the Ground Constantly

My robot weighs over hundreds of kilograms.

- Reduce the `basicTimeStep` field value of the [WorldInfo](../reference/worldinfo.md) node.
- Reduce the `softCFM` field of the [ContactProperties](../reference/contactproperties.md) node which defines the contact between the robot (typically the robot wheels or feet) and the ground.

#### My Robot Suddenly Sinks into the Ground (ElevationGrid) as It Moves from One Tile to Another

This is an ODE bug.
A workaround is to make the [ElevationGrid](../reference/elevationgrid.md) tiles larger than the size of the robot.

### How to Remove the Noise from the Simulation?

There are two sources of noise in Webots: the *sensor/actuator noise* and the *physics engine noise*.
The amount of sensor/actuator noise can be changed (or removed) by the user (see below).
The physics engine's noise cannot be changed because it is necessary for the realism of the simulation.
To completely remove the sensor/actuator noise the following field values must be set to 0:

1. `lookupTable` fields: the third column of each `lookupTable` field be set to 0.
2. [GPS](../reference/gps.md) nodes: the `accuracy` field must be set to 0.
3. [Camera](../reference/camera.md), [Lidar](../reference/lidar.md) and [RangeFinder](../reference/rangefinder.md) nodes: the `noise` field must be set to 0.
4. [Radar](../reference/radar.md) nodes: the `rangeNoise`, `speedNoise` and `angularNoise` fields must be set to 0.
5. [Receiver](../reference/receiver.md) nodes: the `signalStrengthNoise` and `directionNoise` fields must be set to 0.

### How Can I Create a Passive Joint?

First of all, any joint, passive or active, must be created by adding a `Joint`-derived node (depending on the constraint type requested) in the Scene Tree.
A `Joint` is passive if its device is null (or at least not a `Motor`-derived node).
Alternatively, it is also possible to make a `Motor` become passive during the simulation; this can be done like this:

```c
wb_motor_set_motor_force(motor, 0.0);
```

The effect is similar to turning off the power of a real motor.

### Is It Possible to Fix/Immobilize One Part of a Robot?

To immobilize one part of the robot, you need to fix the part to the static environment.
This must be done with a *physics plugin*.
You can add a physics plugin with the menu item: `Wizards / New Physics Plugin`.
In the plugin code, you must simply add an ODE *fixed joint* between the *dBodyID* of the robot part and the static environment.
This can be implemented like this:

```c
#include <ode/ode.h>
#include <plugins/physics.h>

void webots_physics_init() {

  // get body of the robot part
  dBodyID body = dWebotsGetBodyFromDEF("MY_ROBOT_PART");

  // get the matching world
  dWorldID world = dBodyGetWorld(body);

  // the joint group ID is 0 to allocate the joint normally
  dJointID joint = dJointCreateFixed(world, 0);

  // attach robot part to the static environment: 0
  dJointAttach(joint, body, 0);

  // fix now: remember current position and rotation
  dJointSetFixed(joint);
}

void webots_physics_step() {
  // nothing to do
}

void webots_physics_cleanup() {
  // nothing to do
}
```

You will find the description of Webots physics plugin API [here](../reference/physics-plugin.md).
You will find the description about the ODE functions on [this page](http://ode.org/wiki/index.php?title=Manual).

### Should I Specify the "mass" or the "density" in the Physics Nodes?

It is more accurate to specify the mass if it is known.
If you are modeling a real robot it is sometimes possible to find the mass values in the robot's specifications.
If you specify the densities, Webots will use the volume of each `boundingObject` multiplied by the density of the corresponding [Physics](../reference/physics.md) node to compute each mass.
This may be less accurate because `boundingObject`s are often rough approximations.

### How to Get a Realisitc and Efficient Rendering?

The quality of the rendering depends on the [Shapes](../reference/shape.md) resolution, on the setup of the [Materials](../reference/material.md) and on the setup of the [Lights](../reference/light.md).

The bigger the number of vertices is, the slower the simulation is (except obviously in `fast` mode).
A tradeoff has to be found between these two components.
To be efficient, [Shapes](../reference/shape.md) should have a reasonable resolution.
If a rule should be given, a [Shape](../reference/shape.md) shouldn't exceed 1000 vertices.
Exporting a [Shape](../reference/shape.md) from a CAD software generates often meshes having a huge resolution.
Reducing them to low poly meshes is recommended.

The rendering is also closely related to the `Materials`.
To set a [Material](../reference/material.md) without texture, set only its [Appearance](../reference/appearance.md) node.
Then you can play with the `diffuseColor` field to set its color (avoid to use pure colors, balancing the RGB components gives better results).
To set a [Material](../reference/material.md) with texture, set only its [ImageTexture](../reference/imagetexture.md) node.
Eventually, the `specularColor` field can be set to a gray value to set a reflection on the object.
The other fields (especially the `ambientIntensity` and the `emissiveColor` fields) shouldn't be modified except in specific situations.

The `color` field of the [ElevationGrid](../reference/elevationgrid.md) shouldn't be used for a realistic rendering because it is not affected by the ambient light with the same way as the other [Shapes](../reference/shape.md).

Here is a methodology to set up the lights:

1. Place the lights at the desired places.
Often, a single directional light pointing down is sufficient.
2. Set both their `ambientIntensity` and their `intensity` fields to 0.
3. Increase the `ambientIntensity` of the main light.
The result will be the appearance of the objects when they are in shadows.
4. Switch on the shadows if required.
The shadows are particularily costly, and are strongly related to the [Shapes](../reference/shape.md) resolution.
5. Increase the `intensity` of each lamp.
