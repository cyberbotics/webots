# Webots R2023 Change Log

## Webots R2023a Revision 1
Released on ??
  - Enhancements
    - Greatly improved the performance of the Python API `Camera.getImage` method ([#5610](https://github.com/cyberbotics/webots/pull/5610)).
    - Added default [Solid](solid.md).`recognitionColors` value for [animals](../guide/object-animals.md) and [Barn](../guide/object-buildings.md#barn) PROTO models ([#5606](https://github.com/cyberbotics/webots/pull/5606)).
    - Added `recognitionColors` field to [SolidBox](../guide/object-solids.md#solidbox) and Tractor PROTO models ([#5606](https://github.com/cyberbotics/webots/pull/5606)).
    - Improved the bounding objects of the Nao robot PROTO so that `selfCollision` can be activated without any issue ([#5622](https://github.com/cyberbotics/webots/pull/5622)).
  - Bug Fixes
    - Fixed Python API `Supervisor.setSimulationMode` which was failing ([#5603](https://github.com/cyberbotics/webots/pull/5603)).
    - Fixed crash resulting from requesting pose tracking of unsuitable nodes ([5620](https://github.com/cyberbotics/webots/pull/5620)).

## Webots R2023a
Released on November 29th, 2022.
  - New Features
    - Added support for any current and upcoming version of Python for robot controllers ([#5297](https://github.com/cyberbotics/webots/pull/5297)).
    - **Added new `projection` field to [Camera](camera.md), [RangeFinder](rangefinder.md), and [Lidar](lidar.md) nodes. This field replaces the deprecated `spherical` field ([#5266](https://github.com/cyberbotics/webots/pull/5266)).**
    - Added a non-interactive terminal for web streaming ([#5119](https://github.com/cyberbotics/webots/pull/5119)).
    - Added a customizable window for animations to display graphs or other user defined content ([#5265](https://github.com/cyberbotics/webots/pull/5265)).
  - Enhancements
    - Added the support of robot windows for MJPEG streaming ([#5272](https://github.com/cyberbotics/webots/pull/5272)).
    - Added additional checks for path validity in wizards and save world dialogs ([#5350](https://github.com/cyberbotics/webots/pull/5350)).
    - Added warning if external mesh used in [CadShape](cadshape.md) node is fully transparent ([#5359](https://github.com/cyberbotics/webots/pull/5359)).
    - Added warning if external mesh used in [Mesh](mesh.md) node has more than 100'000 vertices ([#5359](https://github.com/cyberbotics/webots/pull/5359)).
  - New Robots
    - Added a model of the [ROSbot](../guide/rosbot.md) robot from [Husarion](https://husarion.com/) and an obstacle avoidance demo ([#5168](https://github.com/cyberbotics/webots/pull/5168)).
  - New Devices and Objects
    - Added some static animals (cow, horse, deer, sheep, dog, fox, cat and rabbit) and a barn ([#4539](https://github.com/cyberbotics/webots/pull/4539)).
    - Added a model of the [Mpu-9250](../guide/imu-sensors.md#mpu-9250) IMU ([#5168](https://github.com/cyberbotics/webots/pull/5168)).
    - Added a model of the [RPLidarA2](../guide/lidar-sensors.md#slamtec-rplidar-a2) lidar ([#5168](https://github.com/cyberbotics/webots/pull/5168)).
    - Added a model of the [Astra](../guide/range-finder-sensors.md#orbbec-astra) RGB-D camera ([#5168](https://github.com/cyberbotics/webots/pull/5168)).
  - New Examples
    - Added a sample world showing how to compute the attitude of a robot from IMU sensors ([#5256](https://github.com/cyberbotics/webots/pull/5256)).
  - Bug Fixes
    - Fixed the OpenAI Gym example which was not converging after ENU/FLU conversion ([#5581](https://github.com/cyberbotics/webots/pull/5581)).
    - Fixed wrong warnings due to SFNode empty flag in the controller ([#5430](https://github.com/cyberbotics/webots/pull/5430)).
    - Fixed controller restart after crash ([#5284](https://github.com/cyberbotics/webots/pull/5284)).
    - Fixed the export of [Lidar](lidar.md)'s rotating head to X3D ([#5224](https://github.com/cyberbotics/webots/pull/5224)).
    - Fixed behavior of `WbLightSensor::computeLightMeasurement` when spotlight is rotated ([#5231](https://github.com/cyberbotics/webots/pull/5231)).
    - Fixed the reset of the viewpoint in animation when the follow is activated ([#5237](https://github.com/cyberbotics/webots/pull/5237)).
    - Fixed a recursion bug in web animation ([#5260](https://github.com/cyberbotics/webots/pull/5260)).
    - Fixed reset of [Lidar](lidar.md) memory mapped file in an extern controller restarted during the simulation run ([#5305](https://github.com/cyberbotics/webots/pull/5305)).
    - Fixed a crash when trying to connect a remote extern controllers while a world is loading ([#5310](https://github.com/cyberbotics/webots/pull/5310)).
    - Fixed close and resize buttons rendered in black in the [RangeFinder](rangefinder.md) overlay if underlying pixel value is `inf` ([#5337](https://github.com/cyberbotics/webots/pull/5337)).
    - Fixed error message about missing PROTO declaration when copying and pasting a PROTO node that was just added using the Add Node dialog ([#5341](https://github.com/cyberbotics/webots/pull/5341)).
    - Fixed the description of base nodes in the Add Node dialog ([#5346](https://github.com/cyberbotics/webots/pull/5346)).
    - Fixed robot window updates when an extern controller reconnects ([#5367](https://github.com/cyberbotics/webots/pull/5367)).
    - Fixed regeneration of PROTO nodes derived from a procedural PROTO ([#5413](https://github.com/cyberbotics/webots/pull/5413)).
    - Fixed processing concatenated messages in default robot window JS code ([#5442](https://github.com/cyberbotics/webots/pull/5442)).
    - Fixed missing check on required PROTO header ([#5453](https://github.com/cyberbotics/webots/pull/5453)).
    - Fixed a crash when streaming and deleting a subnode ([#5457](https://github.com/cyberbotics/webots/pull/5457)).
    - Fixed a crash when converting certain PROTOs to base node ([#5460](https://github.com/cyberbotics/webots/pull/5460)).
    - Fixed the item selection in the scene tree after a conversion to base node ([#5460](https://github.com/cyberbotics/webots/pull/5460)).
    - Fixed motor sound glitches when motor was stopped ([#5488](https://github.com/cyberbotics/webots/pull/5488)).
  - Cleanup
    - Removed deprecated `windowPosition`, `pixelSize`, `type`, `colorNoise` fields of [Camera](camera.md) node ([#5266](https://github.com/cyberbotics/webots/pull/5266)).
  - Dependency Updates
    - Upgraded to Qt6.4 on Windows ([#5301](https://github.com/cyberbotics/webots/pull/5301)).
