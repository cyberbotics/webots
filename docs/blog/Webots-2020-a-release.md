# Version R2020a Released

<p id="publish-data">By Stefania Pedrazzi - 18th December 2019</p>

---

Here we are! It's already time for a new Webots release!

This new Webots version focuses on improving the performance of the scene load and the robustness of the simulation.
But as usual we also extended our asset library with new robots, objects, environments and appearances.
Moreover the [Supervisor API](../reference/supervisor.md) has been extended.

This article only lists some of the major changes.
Besides these, we are always fixing bugs and improving the quality and robustness of Webots.
Please refer to the changelog for a comprehensive list of all the changes, found [here](../reference/changelog-r2020.md).

---

## New Robot Models

### TIAGo Robots

Simulate the TIAGo robot from [PAL Robotics](http://pal-robotics.com/) in Webots.

%figure "Tiago Model in Webots"
![Tiago Model in Webots](images/tiago.jpg)
%end

Several models of the robot have been created for Webots:
  - [TIAGo Base](https://cyberbotics.com/doc/guide/tiago-base?version=R2020a)
  - [TIAGo Iron](https://cyberbotics.com/doc/guide/tiago-iron?version=R2020a)
  - [TIAGo Steel](https://cyberbotics.com/doc/guide/tiago-steel?version=R2020a)
  - [TIAGo Titanium](https://cyberbotics.com/doc/guide/tiago-titanium?version=R2020a)
  - [TIAGo++](https://cyberbotics.com/doc/guide/tiagopp?version=R2020a)

### TurtleBot3 Burger

Simulate the TurtleBot3 Burger robot from [Robotis](http://www.robotis.us) in Webots.

%figure "TurtleBot3 Burger Model in Webots"
![TurtleBot3 Burger in Webots](images/turtlebot3.jpg)
%end

The LDS-01 lidar sensor is now available too, just plug it on your favorite simulated robot.

### Robotino 3

Simulate the Robotino 3 from [Festo](https://www.festo-didactic.com/int-en/) in Webots.

%figure "Robotino 3 Model in Webots"
![Robotino 3](images/robotino3.jpg)
%end

This omnidirectional robot is highly configurable, you can therefore easily add some lidar, camera or gripper.

---

## ROS 2 & Webots

ROS 2 Eloquent has been [released](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/) approximately one month ago.
We understood the high interest for ROS 2 among the robotics community and we are therefore proud to announce that Webots is now compatible with the following versions of ROS 2:
  - [Crystal Clemmys](https://index.ros.org/doc/ros2/Releases/Release-Crystal-Clemmys)
  - [Dashing Diademata](https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata)
  - [Eloquent Elusor](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor)

%figure "Simulation of UR and ABB robots in Webots with ROS 2"
![ROS 2 in Webots](images/ros2_demo.thumbnail.gif)
%end

See the [webots\_ros2 package documentation](http://wiki.ros.org/webots_ros2) for the installation and usage instructions.
Or, if you can't wait to see the latest features and supported robots, go directly to the [webots\_ros2 Github repository](https://github.com/cyberbotics/webots_ros2).


---

## Extended Supervisor API

We are always collecting feedbacks form users, and in particular recently we received many requests about adding new functionalities to the [Supervisor API](../reference/supervisor.md).

So for this release we extended the [Supervisor API](../reference/supervisor.md) features to be able to:
  - Add a [force](../reference/supervisor.md#wb_supervisor_node_add_force) or a [torque](../reference/supervisor.md#wb_supervisor_node_add_torque) to a [Solid](../reference/solid.md) node.
  - Import and [remove](../reference/supervisor.md#wb_supervisor_field_remove_sf) nodes in SFNode fields.
  - [Reset](../reference/supervisor.md#wb_supervisor_simulation_reset) the simulation without restarting the robot controllers.

---

## New Appearances

To let you model more realistic robots and environments we expanded our `PBRAppearance` library with some new PROTO files.

| | | |
| :---: | :---: | :---: |
| ![Copper](images/appearances/Copper.thumbnail.png) | ![CorrugatedPlates](images/appearances/CorrugatedPlates.thumbnail.png) |  ![CorrugatedPvc](images/appearances/CorrugatedPvc.thumbnail.png) |
| ![DryMud](images/appearances/DryMud.thumbnail.png) | ![FormedConcrete](images/appearances/FormedConcrete.thumbnail.png) |  ![Pcb](images/appearances/Pcb.thumbnail.png) |
| ![RoughPolymer](images/appearances/RoughPolymer.thumbnail.png) | ![TireRubber](images/appearances/TireRubber.thumbnail.png) |  |

---

## Simplify the HDR Backgrounds

In last releases we did a great effort to improve the quality of the rendering in Webots, using PBR and HDR backgrounds.
Unfortunately former equirectangular HDR background images turned out to be very heavy and memory-intensive.
This is causing some issues with low to mid range graphics cards and in general they are significantly slowing down the world loading.
That's why we dropped the support of the equirectangular HDR images and the `Cubemap` node.
Instead we restored the standard `<cube_face>Url` VRML fields and added the new `<cube_face>IrradianceUrl` and `luminosity` fields of the [Background](../reference/background.md) node to specify light reflections and scale the light contribution on the [PBR appearances](../reference/pbrappearance.md).
So now we have a better performance with the same realistic rendering quality.

%figure "New structure of Background node"
![New Background Fields](images/background_new_fields.thumbnail.png)
%end

But do not worry about the compatibility of your custom HDR backgrounds!
We are now also providing some tools to help you with the update.
You can find them on the [Webots GitHub repository](https://github.com/cyberbotics/webots/tree/R2020a/scripts/image_tools), and in case you need some help with the conversion please do not hesitate to contact us on Discord.

### Updated TexturedBackground PROTO

The TexturedBackground PROTO has been updated accordingly to the new [Background](../reference/background.md) node definition and we added some new HDR textures.

Note that we deprecated some of the old default backgrounds that were not in HDR format.

---

## Viewpoint Follow Functionality

We all know that it could be tricky to make the viewpoint follow smoothly a robot to record fancy movies or just to run nice simulations.
To help you in this task, we replaced the `followOrientation` field of the [Viewpoint](../reference/viewpoint.md) node with the `followType` and added a new "Pan and Tilt Shot" option that will automatically move the viewpoint so that it always looks at the object center.

---

## Extra Goodies

We are very happy to communicate that we fixed the error accumulation issues occurring when reverting the scene using the physics reset function. Thank you to all the users that reported this problem!

Webots R2020a contains a new complete apartment environment: check it [here](../guide/samples-environments.md#complete_apartment-wbt).

On Linux, you can now also program your controllers with Python 3.8.

Finally, after one year of open-source, we removed the license system and turned also the old Webots versions prior to R2019a free.
Just note that old versions are no longer maintained.


**Go and [download](https://cyberbotics.com/#download) Webots R2020a today, so you don't miss out on all these great new features!**
