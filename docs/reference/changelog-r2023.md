# Webots R2023 Change Log

## Webots R2023a
Released on December, 12th, 2022.
  - New Features
    - Added support for any current and upcoming version of Python for robot controllers ([#5297](https://github.com/cyberbotics/webots/pull/5297)).
    - **Added new `projection` field to [Camera](camera.md), [RangeFinder](rangefinder.md), and [Lidar](lidar.md) nodes. This field replaces the deprecated `spherical` field ([#5266](https://github.com/cyberbotics/webots/pull/5266)).**
    - Added a non-interactive terminal for web streaming ([#5119](https://github.com/cyberbotics/webots/pull/5119)).
    - Added a customizable window for animations to display graphs or other user defined content ([5265](https://github.com/cyberbotics/webots/pull/5265)).
  - Enhancements
    - Add the support of robot windows for mjpeg streaming ([#5272](https://github.com/cyberbotics/webots/pull/5272)).
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
    - Fixed wrong warnings due to SFNode empty flag in the controller ([#5430](https://github.com/cyberbotics/webots/pull/5430)).
    - Fixed processing concatenated messages in default robot window JS code ([#5442](https://github.com/cyberbotics/webots/pull/5442)).
  - Cleanup
    - Removed deprecated `windowPosition`, `pixelSize`, `type`, `colorNoise` fields of [Camera](camera.md) node ([#5266](https://github.com/cyberbotics/webots/pull/5266)).
  - Dependency Updates
