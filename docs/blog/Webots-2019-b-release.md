# Version R2019b Released

<p id="publish-data">By Fabien Rohrer - 17th June 2019</p>

Today we're happy to announce the release of the all-new Webots 2019b, packed with some great new features.
We're going to talk about some of them here, but for a comprehensive list of changes please refer to the ChangeLog, found [here](https://www.cyberbotics.com/dvd/common/doc/webots/ChangeLog.html).

---

## Bunch of New Robot Models

### Universal Robots UR3e, UR5e and UR10e Arms and ROBOTIQ 3-Finger Gripper

https://www.cyberbotics.com/doc/guide/ure
https://www.cyberbotics.com/doc/guide/gripper-actuators#robotiq-3f-gripper

![youtube video](https://www.youtube.com/watch?v=WIY9ebqSXUc)

### Clearpath Moose

https://www.cyberbotics.com/doc/guide/moose

![youtube video](https://www.youtube.com/watch?v=joPAnZcOouc)

### ABB IRB 4600/40 Arm

https://www.cyberbotics.com/doc/guide/irb4600-40

![youtube video](https://www.youtube.com/watch?v=Jq0-DkEwwj4)

### Telerob Telemax PRO

https://www.cyberbotics.com/doc/guide/telemax-pro

![youtube video](https://www.youtube.com/watch?v=lUWMGk0i9Tc)

---

## Bloom and Ambient Occlusion are now also available as effects for color cameras

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
