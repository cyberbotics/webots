# Webots R2024 Change Log

## Webots R2024a
Released on December **th, 2023.
  - New Features
    - **Change the name of the web scene format from `X3D` to `W3D` ([#6280](https://github.com/cyberbotics/webots/pull/6280)).**
  - Enhancements
    - Improved the image range of the rotating [Lidar](lidar.md) ([#6324](https://github.com/cyberbotics/webots/pull/6324)).
  - Cleanup
    - Removed deprecated `windowPosition`, `pixelSize` fields of [Display](display.md) node ([#6327](https://github.com/cyberbotics/webots/pull/6327)).
  - Bug Fixes
    - Fixed error message on Windows when `libssl-3-x64.dll` was added to `PATH` ([#6553](https://github.com/cyberbotics/webots/pull/6553)).
    - Fixed length of arrays returned by `getPose()` in Java ([#6556](https://github.com/cyberbotics/webots/pull/6556)).
    - Fixed length of arrays returned by `CameraRecognitionObject.getColors()` in Java ([#6564](https://github.com/cyberbotics/webots/pull/6564))
    
