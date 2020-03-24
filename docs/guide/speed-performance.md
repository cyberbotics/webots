## Speed/Performance

### Why Is Webots Slow on My Computer?

You should verify your graphics driver installation.
Please find instructions in [this section](verifying-your-graphics-driver-installation.md).

If you are using a laptop computer, please check the power options and make sure you are using the high performance power plan.

On Ubuntu (or other Linux) we also recommend to disable all visual effects.
Depending on the graphics hardware, there may be a huge performance drop of the rendering system (up to 10x) when the visual effects are on.
You can easily disable the operating system's visual effects using some tools like *Compiz Config Settings Manager* or *Unity Tweak Tool*.

### How Can I Change the Speed of the Simulation?

There are several ways to increase the simulation speed:

1. Use the `Run` button.
This button runs the simulation as fast as possible using all the available CPU power.
Otherwise, using the `Real-Time` running mode, Webots may not be using all the available CPU power in order to obtain a simulation speed that is close to the speed of the real world's phenomena.
2. Use the `Fast` button.
This button runs the simulation as fast as possible using all the available CPU power.
In this mode the simulation speed is increased further by leaving out the graphics rendering, hence the 3d window is black.
3. Increase the value of `WorldInfo.basicTimeStep`.
This field sets the granularity of the physics simulation.
With a higher `WorldInfo.basicTimeStep`, the simulation becomes faster but less accurate.
With a lower `WorldInfo.basicTimeStep`, the simulation becomes slower but more accurate.
There is an additional restriction: `WorldInfo.basicTimeStep` must be chosen such as to be an integer divisor of the *control step* which is the value passed as parameter to the `wb_robot_step` (or equivalent) function.
4. Decrease the value of `WorldInfo.FPS`.
This field represents the maximum rate at which the 3D display of the main windows is refreshed.
With a lower value, the simulation becomes faster but more flickering.
With a higher value, the simulation becomes slower but less flickering.
5. Try changing the value of `WorldInfo.optimalThreadCount`.
This field specifies how many threads are used to simulate the physics of the world.
Depending on the world you can get a better performance by reducing or increasing this value.
In general it is better to have a low number of threads for simple worlds and a bigger number of threads for complex worlds that include several robots physically independent from each other.
6. Disable unnecessary shadows.
Webots uses a lot of CPU/GPU power to compute how and where the objects shadows are cast.
But shadows are irrelevant for most simulation unless they should explicitly be seen by [Cameras](../reference/camera.md).
Unnecessary shadows can be disabled by unchecking the `castShadows` field of light nodes: [PointLight](../reference/pointlight.md), [SpotLight](../reference/spotlight.md), or [DirectionalLight](../reference/directionallight.md).
7. Simplify your simulation by removing unnecessary objects.
In particular, try to minimize the number of [Physics](../reference/physics.md) nodes.
Avoid using a [Solid](../reference/solid.md) nodes when a [Transform](../reference/transform.md) or a [Shape](../reference/shape.md) would do the trick.
8. Simplify the `boundingObject`s to increase the speed of the collision detection.
Replace complex primitives, like [Cylinder](../reference/cylinder.md), [IndexedFaceSet](../reference/indexedfaceset.md), [Mesh](../reference/mesh.md) and [ElevationGrid](../reference/elevationgrid.md) by simpler primitives, like [Sphere](../reference/sphere.md), [Capsule](../reference/capsule.md), [Box](../reference/box.md) and [Plane](../reference/plane.md).
Avoid using a composition of primitives (in a [Group](../reference/group.md) or a [Transform](../reference/transform.md)) when a single primitive would do the trick.
9. Set an empty string in the `controller` field for any [Robot](../reference/robot.md) nodes that don't need to be controlled, instead of using the `void` controller.
