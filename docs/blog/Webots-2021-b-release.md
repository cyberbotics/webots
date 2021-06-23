# Version R2021b Released

<p id="publish-data">By Stefania Pedrazzi - XX July 2021</p>

---

It's that time of year again! Today we're happy to announce the release of Webots R2021b!
And it's packed with some new features, improvements and, of course, bug fixes.

Here we are going to present some of the main new features, but for a comprehensive list of changes please refer to the [ChangeLog](../reference/changelog-r2021.md).


## Improved Support for Reinforcement Learning

We added a set of new functionalities to make it easier to run reinforcement learning experiments.
Other than reducing the Webots package size to speed-up installation on servers and virtual containers, some additional Supervisor API functions to better manage the reset of the simulation has been included.
In particular, it is now possible to save the state of a given node and all its descendant nodes and restore it later during the simulation run or export a node definition as string to duplicate the node a later point.
Then, it is now also possible to artificially and immediately set the position of active and passive joints without having to activate the motor.

An example showing the integration of OpenAI Gym with Webots is now available in our samples library.

---

## Reduce Package Size

For this release we worked hard at finding solutions to reduce the size of the Webots installation packages.
To this end, we removed textures, meshes and sounds from the package. These resources are now downloaded from the web only when needed and cached for subsequent use.
Additionally, we removed the offline documentation and ..

TODO (update with package size difference).

---

## New Web Rendering Engine

Since the introduction of the web interface for streaming simulations and playing recorded animations, we struggled finding optimal parameters for shadows and illuminations that will reproduce exactly the same rendering as in the Webots Desktop application.
Finally, we decided to port our custom 3D renderer WREN to WebAssembly and use it for the web simulation interface in order to minimize the differences with the Desktop version and make it straightforward to run high quality simulations and animations on the web.

During this work, we also took the time to refresh and improve the graphical user interface.

%figure "New Web Simulation Interface"
![Web Simulation Interface](images/web_simulation_interface.thumbnail.jpg)
%end

---

## Extended Motor and Joint Functionalities

The [Motor](../reference/motor.md) and [Joint](../reference/joint.md) nodes have been extended to provide more realistic simulations.

We enhanced the [Motor](../reference/motor.md) node to support coupled motors specifying the linked motor in the `name` field and setting their ratio using the new `multiplier` field.

Then, we added three new PROTO nodes. The [HingeJointWithBacklash](../guide/hinge-joint-with-backlash.md) and [Hinge2JointWithBacklash](../guide/hinge-2-joint-with-backlash.md) allow users to easily add a backlash effect in hinges. The [Gear](../guide/object-gear.md) model can be used to simulate a collision-based transmission.

TODO add figure

---

## New Robot Models

We've added many new robot models in this release.

### Modular Mobile Platforms

We just added three new models of modular mobile platform robots used for logistic and indoor transport.
All these models have predefined fields to mount additional devices on the platform and front/back lidars.

TODO show the three robot figures on a single line

#### [Robotnik Summit-XL Steel](../guide/summit-xl-steel.md)
%figure "Summit-XL Steel in Webots"
![Summit-XL Steel in Webots](images/summit_xl_steel.wbt.thumbnail.jpg)
%end

#### [Mobile Industrial Robots MiR100](../guide/mir100.md)
%figure "MiR100 in Webots"
![MiR100 in Webots](images/mir100.wbt.thumbnail.jpg)
%end

#### [REC Fabtino](../guide/fabtino.md)

%figure "Fabtino in Webots"
![Fabtino in Webots](images/fabtino.wbt.thumbnail.jpg)
%end

### Nyrio Ned

The Nyrio Ned is a 6-axis collaborative robot designed for teaching programming in an industrial environment.

![Nyrio Ned in Webots video](https://www.youtube.com/watch?v=diBAJY1WJPQ)

### NVIDIA JetBot

The NVIDIA [JetBot](../guide/jetbot.md) is an affordable two-wheeled robot based on the NVIDIA Jetson Nano board widely used in AI robotics applications.

%figure "JetBot in Webots"
![JetBot in Webots](images/jetbot.wbt.thumbnail.jpg)
%end

A standalone model of the NVIDIA [Jetson Nano](../guide/single-board-computers.md#nvidia-jetson-nano) board is also available.

---

## New Sensors

The [SICK S300](../guide/lidar-sensors.md#sick-s300) lidar model is now included in our sensors library.

%figure "SICK S300"
![SICK S300](images/sick_s300.thumbnail.png)
%end

---

## ROS 2 & Webots

TODO

---

## Extra Goodies

A new [Billboard](../reference/billboard.md) node has been introduced to display information in the main view regardless of the position and orientation of the viewpoint.

We added [JavaScript scripting support](../reference/javascript-procedural-proto.md) for procedural PROTO nodes in addition to LUA.

**Go and [download](https://cyberbotics.com/#download) Webots R2021b today, so you don't miss out on all these great new features!**
