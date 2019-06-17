# Version R2019b Released

<p id="publish-data">By Fabien Rohrer - 17th June 2019</p>

Today we're happy to announce the release of the all-new Webots R2019b, packed with some great new features.
We're going to talk about some of them here, but for a comprehensive list of changes please refer to the ChangeLog, found [here](https://www.cyberbotics.com/dvd/common/doc/webots/ChangeLog.html).

---

## Bunch of New Robot Models

We've added a couple of new robot models in this release.

### Universal Robots UR3e, UR5e and UR10e Arms and ROBOTIQ 3-Finger Gripper

The Universal Robots [UR3e](https://www.cyberbotics.com/doc/guide/ure), [UR5e](https://www.cyberbotics.com/doc/guide/ure) and [UR10e](https://www.cyberbotics.com/doc/guide/ure) are flexible collaborative robot arms with 6 degrees of freedom.
These arms can be equipped by the a [ROBOTIQ 3F Gripper is a 3-fingers adaptive robot gripper](https://www.cyberbotics.com/doc/guide/gripper-actuators#robotiq-3f-gripper).

![youtube video](https://www.youtube.com/watch?v=WIY9ebqSXUc)

### Clearpath Moose

The [Clearpath Robotics Moose robot](https://www.clearpathrobotics.com/moose-ugv/) is a large 8-wheeled all-terrain unmanned ground vehicle.
More information [here](https://www.cyberbotics.com/doc/guide/moose).

![youtube video](https://www.youtube.com/watch?v=joPAnZcOouc)

### ABB IRB 4600/40 Arm

The [ABB IRB 4600/40](https://www.cyberbotics.com/doc/guide/irb4600-40) is a 6 DOF indoor arm.

![youtube video](https://www.youtube.com/watch?v=Jq0-DkEwwj4)

### Telerob Telemax PRO

The [Telerob Telemax PRO robot](https://www.cyberbotics.com/doc/guide/telemax-pro) is a tracked robot equipped with a 7-axis manipulator.

![youtube video](https://www.youtube.com/watch?v=lUWMGk0i9Tc)

---

## New Post-processing Effects Available in Robot Cameras

In order to increase the realism of robot cameras, they can now be affected by post-processing effects already included in Webots such as Bloom and Ambient Occlusion.

---

## More Assets

- Added the following PROTO objects: 'IntermodalContainer', 'IntermodalOfficeContainer', 'Radiator', 'WallPlug', 'FireExtinguisher', 'CapScrew', 'ElectricalPlug', 'EyeScrew', 'ScrewHole', 'Washer', 'DoubleFluorescentLamp' and 'OfficeTelephone'.
- Added the following PROTO appearances: 'CorrodedMetal', 'CorrugatedMetal', 'DamascusSteel', 'DarkParquetry', 'ElectricConduit', 'FlexibleAluminiumDuct', 'GalvanizedMetal', 'HammeredCopper', 'OldPlywood', 'OsbWood', 'Plaster', 'RustyMetal', 'ScrewThread' and 'WireFence'.
- Added a 'PipeBoundingObject' and a 'TorusBoundingObject' PROTO nodes to simplify the creation of complex bounding objects.

---

## Blender to Webots exporter Add-on

![Blender Add-On](https://raw.githubusercontent.com/omichel/blender-webots-exporter/master/demo.gif)

https://github.com/omichel/blender-webots-exporter

---

## Improved the Webots Online 3D Viewer: 'webots.min.js'

- Switched from 'X3DOM' to 'three.js'.
- Used ES6, Babel and minifyjs to provide 'webots.min.js' as a single and minified file.
- Reduced the rendering difference between Webots and 'webots.min.js'.
- Added support for PBR appearances and HDR textured background.
- Improved loading time and rendering speed.
- Extended the Webots websocket server to an HTTP server serving the texture images too.

---

## Extra Goodies

- Simplified Installation on Windows: Webots can now be installed without administrator privileges.

- Deprecated the Python 2.7 API.

**Go and [download](https://cyberbotics.com/#download) Webots R2019b today, so you don't miss out on all these great new features!**
