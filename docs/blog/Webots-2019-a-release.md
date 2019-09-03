# Version R2019a - Webots Goes Open Source

<p id="publish-data">By Tom Norton - 18th December 2018</p>

---

## New Beginnings

After 20 years as proprietary software, with the release of version R2019a Webots is now fully open-source, under the Apache 2.0 License!
We have made this change as we believe that making our software as accessible as possible is the best way for robotics research to advance.
As a result, we will no longer be selling licenses, as anyone can install and use Webots completely free of charge.
However, we will continue to support any users with active Premier service for the duration of said service.

We will be shifting our efforts towards offering bespoke consulting, training and support services to Webots users, to use our expertise to develop and enhance user simulations.
The source code for Webots is now publicly available on [GitHub](https://github.com/cyberbotics/webots), and we will welcome any and all contributions from users.
We are excited for this next chapter of Webots' life and we hope you will join us in our continued goal to make cutting-edge robotics research as accessible and practical as possible.

---

## Next-Gen Graphics

%figure "Left: Before Graphics Update, Right: After Graphics Update"
![new graphics](images/graphics_before_after.thumbnail.jpg)
%end

You may remember from the release of Webots R2018b that we implemented our own bespoke rendering engine, WREN (Webots Renderer).
Aside from the introduction of the PBR (Physically-Based Rendering) material system, the initial implementation goals of WREN were to implement all the required features from previous Webots versions, being as slim and fast as possible.

### High Dynamic Range + More

With this update, WREN jumps forward to the cutting edge.
WREN now features a full HDR (High Dynamic Range) rendering pipeline with adjustable exposure, allowing for much more realistic lighting and accurate color reproduction, especially with PBR.
Advanced Post-Processing has arrived, with support for Ambient Occlusion (using the GTAO technique by Jorge Jimenez) and Bloom for simulating camera overexposure, heightening the level of realism in your Webots simulations.

HDR brings a change to how lights work in Webots - lights no longer have intensities between 0 and 1, instead between 0 and infinity.
This now means that intensity is a physically-based quantity and is measured in Lumens.
The consequence of this is that neither constant or linear attenuation give very realistic results anymore for `PointLight` or `SpotLight` nodes, and thus inverse-square attenuation has been made the new default value for these lights' `attenuation` field.
So don't worry if your sims look a bit dark - you may just need to up your light(s)' intensities!
If you're really confused, have a look at our samples for some guidance.

### PBR: Everything

With this release, we have (with lots of effort) completely overhauled all objects, worlds, robots and vehicles.
Everything in Webots now uses the Physically-Based Rendering (PBR) system, leading to a universal visual upgrade and a greater degree of realism.
As a result, some important PROTO files have had their fields adjusted to accomodate this new system.
Please check the [ChangeLog](../reference/changelog-r2019.md) for a complete list of breaking changes.
In addition to this, we have added a library of ready-made PBR materials (as `PBRAppearance`-based PROTO files) already used across Webots that will help you make your next simulation look awesome.
These can be found in `$WEBOTS_HOME/projects/appearances`.

---

## New Robot Models

We've added a couple of new robot models in this release too.
These models were created from scratch using the PBR material system and so look great!

### ROBOTIS OP3

![youtube video](https://www.youtube.com/watch?v=MgykUcSfUFI)

Manufactured by [ROBOTIS](http://www.robotis.us/robotis-op3/), we've added the third-generation OP3 to the family of ROBOTIS models in Webots.

#### Some Features:
20 degrees of freedom (20 DYNAMIXEL XM430)

Several sensors including FULL HD Logitech C920

Support for official pre-recorded ROBOTIS motion playback.

### Tinkerbots

![youtube video](https://www.youtube.com/watch?v=QMbojDv5DH0)

We've continued to build support for modular robots by adding a set of models for the Tinkerbots modular robot kit. It's easy to design and build a model from the Webots GUI, with the kit's collection of passive connectors, active sensors and actuators.

---

## Extra Goodies

Webots now features full support for macOS 10.14 Mojave as well as Retina displays.

We've added a new `ConveyorBelt` PROTO for use in industrial simulations.

We've added a model of the Tesla model 3, compatible with our SUMO interface.

There's now an API function for the "Move Viewpoint to Object" action.

**Go and [download](https://cyberbotics.com/#download) Webots R2019a today, so you don't miss out on all these great new features!**
