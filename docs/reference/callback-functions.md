## Callback Functions

The plugin code must contain user-implemented functions that will be called by Webots during the simulation.
These user-implemented functions and their interfaces are described in this section.
The implementation of the `webots_physics_step` and `webots_physics_cleanup` functions is mandatory.
The implementation of the other callback functions is optional.

Since Webots 7.2.0, the ODE physics library used in Webots is multi-threaded.
This allows Webots to run some physics simulation much faster than before on multi-core CPUs.
However, it also implies that programming a physics plug-in is slightly more complicated as the `webots_physics_collide` callback function may be called from different threads.
Hence, it should be re-entrant and every call to an ODE API function modifying the current world (contacts, bodies, geoms) should be mutex protected within this callback function.
We recommend using POSIX mutexes as exemplified here:

```c
static pthread_mutex_t mutex;

void webots_physics_init() {
  pthread_mutex_init(&mutex, NULL);
  ...
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
...
    dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();
    pthread_mutex_lock(&mutex);
    dJointAttach(dJointCreateContact(world,
                                     contact_joint_group,
                                     &contact[i]),
                                     robot_body,
                                     NULL);
    pthread_mutex_unlock(&mutex);
...
}

void webots_physics_cleanup() {
  ...
  pthread_mutex_destroy(&mutex);
}
```

### "void webots\_physics\_init(dWorldID, dSpaceID, dJointGroupID)"

This function is called upon initialization of the world.
Its arguments are obsolete and should not be used.
This function is a good place to call the `dWebotsGetBodyFromDEF` and `dWebotsGetGeomFromDEF` functions (see below for details) to get pointers to the objects for which you want to control the physics.
Before calling this function, Webots sets the current directory to where the plugin's ".dll", ".so" or ".dylib" was found.
This is useful for reading config files or writing log files in this directory.

The obsolete arguments can be retrieved as follows:

```c
void webots_physics_init(dWorldID, dSpaceID, dJointGroupID) {
  // get body of the robot part
  dBodyID body = dWebotsGetBodyFromDEF("MY_ROBOT_PART");
  dGeomID geom = dWebotsGetGeomFromDEF("MY_ROBOT_PART");

  // get the matching world
  dWorldID world = dBodyGetWorld(body);

  // get the matching space
  dSpaceID space = dGeomGetSpace(geom);
}
```

This function is also the preferred place to initialize/reinitialize the random number generator (via the `srand` function).
Reinitializing the generator with a constant seed allows Webots to run reproducible (deterministic) simulations.
If you don't need deterministic behavior you should initialize the `srand` function with the current time: `srand(time(NULL))`.
Webots itself does not invoke the `srand` function; however, it uses the `rand` function, for example to add noise to sensor measurements.
In order to have reproducible simulations, it is also required that all controllers run in *synchronous* mode.
That means that the `synchronization` field of every [Robot](robot.md) must be set to TRUE.
Finally, note that ODE uses its own random number generator that you might also want to reinitialize separately via the `dRandSetSeed` function.

### "int webots\_physics\_collide(dGeomID, dGeomID)"

This function is called whenever a collision occurs between two geoms.
It may be called several times (or not at all) during a single simulation step, depending on the number of collisions.
Generally, you should test whether the two colliding geoms passed as arguments correspond to objects for which you want to control the collision.
If you don't wish to handle a particular collision you should return 0 to inform Webots that the default collision handling code must be used.

Otherwise you should use the ODE's `dCollide` function to find the contact points between the colliding objects and then you can create contact joints using the ODE's `dJointCreateContact` function.
Normally the contact joints should be created within the contact joint group given by the `dWebotsGetContactJointGroup` function.
Note that this contact joint group is automatically emptied after each simulation step, see [here](execution-scheme.md).
Then the contact joints should be attached to the corresponding bodies in order to prevent them from inter-penetrating.
Finally, the `webots_physics_collide` function should return either 1 or 2 to inform Webots that this collision was handled.
If the value 2 is returned, Webots will moreover notify graphically that a collision occurred by changing the color of the corresponding boundingObject Geometry in the 3D view.

Since Webots 7.2.0, a multi-threaded version of ODE is used.
Therefore, this function may be called from different threads.
You should ensure it is re-entrant and that every ODE function call modifying the ODE world is protected by mutexes as explained earlier.

### "void webots\_physics\_step()"

This function is called before every physics simulation step (call to the ODE `dWorldStep` function).
For example it can contain code to read the position and orientation of bodies or add forces and torques to bodies.

### "void webots\_physics\_step\_end()"

This function is called right after every physics simulation step (call to the ODE's `dWorldStep` function).
It can be used to read values out of `dJointFeedback` structures.
ODE's `dJointFeedback` structures are used to know how much torque and force is added by a specific joint to the joined bodies (see ODE User Guide for more information).
For example, if the plugin has registered `dJointFeedback` structures (using the ODE's `dJointSetFeedback` function), then the structures will be filled during the `dWorldStep` function call and the result can be read straight afterwards in the `webots_physics_step_end` function call.

### "void webots\_physics\_cleanup()"

This function is the counterpart to the `webots_physics_init` function.
It is called once, when the world is destroyed, and can be used to perform cleanup operations, such as closing files and freeing the objects that have been created in the plugin.
