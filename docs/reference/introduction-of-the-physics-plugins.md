## Introduction of the Physics Plugins

This chapter describes Webots capability to add a physics plugin to a simulation.
A physics plugin is a user-implemented shared library which is loaded by Webots at run-time, and which gives access to the low-level API of the [ODE](http://www.ode.org) physics engine.
A physics plugin can be used, for example, to gather information about the simulated bodies (position, orientation, linear or angular velocity, etc.), to add forces and torques, to add extra joints, e.g., "ball & socket" or "universal joints" to a simulation.
For example with a physics plugin it is possible to design an aerodynamics model for a flying robot, a hydrodynamics model for a swimming robot, etc.
Moreover, with a physics plugin you can implement your own collision detection system and define non-uniform friction parameters on some surfaces.
Note that physics plugins can be programmed only in C or C++.
