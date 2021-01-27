## Examples

Webots comes with several examples of physics plugins.
When opening an example, the code of the physics plugin should appear in Webots text editor.
If it does not appear automatically, then you can always use the menu: `Tools / Edit Physics Plugin`.

A simple example is the "[WEBOTS\_HOME/projects/samples/howto/physics/worlds/physics.wbt]({{ url.github_tree }}/projects/samples/howto/physics/worlds/physics.wbt)" world.
In this example, the plugin is used to add forces to make the robot fly, to communicate with the Webots model, to detect objects using a `Ray` object, to communicate with a `Robot` to visualize the `Ray` and to define a frictionless collision between the robot and the floor.

The "[WEBOTS\_HOME/projects/robots/epfl/lis/worlds/blimp.wbt]({{ url.github_tree }}/projects/robots/epfl/lis/worlds/blimp.wbt)" shows how to suppress gravity, and apply a thrust force (propeller) for a blimp model.
