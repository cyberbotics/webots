# Webots R2024 Change Log

## Webots R2024a
Released on December **th, 2023.
  - New Features
    - **Change the name of the web scene format from `X3D` to `W3D` ([#6280](https://github.com/cyberbotics/webots/pull/6280)).**
    - Removed support for macOS 11 "Big Sur" and added support for macOS 14 "Sonoma" ([#6580](https://github.com/cyberbotics/webots/pull/6580)).
    - Added the `indirectFieldAccess` tag to allow the `fields` variable to be used in proto templates without referencing a specific field ([#6614](https://github.com/cyberbotics/webots/pull/6614)).
  - Enhancements
    - Improved the image range of the rotating [Lidar](lidar.md) ([#6324](https://github.com/cyberbotics/webots/pull/6324)).
  - Cleanup
    - Removed deprecated `windowPosition`, `pixelSize` fields of [Display](display.md) node ([#6327](https://github.com/cyberbotics/webots/pull/6327)).
  - Bug Fixes
    - Fixed error message on Windows when `libssl-3-x64.dll` was added to `PATH` ([#6553](https://github.com/cyberbotics/webots/pull/6553)).
    - Fixed length of arrays returned by `getPose()` in Java ([#6556](https://github.com/cyberbotics/webots/pull/6556)).
    - Fixed length of arrays returned by `CameraRecognitionObject.getColors()` in Java ([#6564](https://github.com/cyberbotics/webots/pull/6564)).
    - Fixed handling of device objects with the same name in the controller API ([#6579](https://github.com/cyberbotics/webots/pull/6579)).
    - Fixed crashes (with some graphics cards) caused by references to unused GLSL uniforms ([#6587](https://github.com/cyberbotics/webots/pull/6587)).
    - Fixed `Brake`s added to the second axis of a `Hinge2Joint` being applied to the non-transformed axis ([#6584](https://github.com/cyberbotics/webots/pull/6584)).
    - Fixed invalid absolute sound file path resulted in crash ([#6593](https://github.com/cyberbotics/webots/pull/6593))
    - Fixed Speaker relative sound file path not working with external controller ([#6605](https://github.com/cyberbotics/webots/pull/6605)).
    - Fixed the bug that when the language is Python, getTargets() cannot correctly obtain the multi-target data detected by the radar ([#6606](https://github.com/cyberbotics/webots/pull/6606))
    - Fixed incomplete loading while minimized under some windowing systems ([#6617](https://github.com/cyberbotics/webots/pull/6617)).
    - Fixed unitialized sliding friction when using asymmetric rolling friction ([#6618](https://github.com/cyberbotics/webots/pull/6618)).
    - Made `supervisor_export_image` work even if using the `--minimize --no-rendering` options ([#6622](https://github.com/cyberbotics/webots/pull/6622)).
    - Fixed CA certificates needed by `test_suite.py` missing in Windows development environment ([#6628](https://github.com/cyberbotics/webots/pull/6628)).
    - Fixed to use a version of Qt that is still supported ([#6623](https://github.com/cyberbotics/webots/pull/6623)).
    - Fixed connection errors when using long robot names in some environments ([#6635](https://github.com/cyberbotics/webots/pull/6635)).
    - Fixed crash on macos when closing Webots under some circumstances ([#6635](https://github.com/cyberbotics/webots/pull/6635)).
    - Fixed error on macos when when putting displays and cameras in separate windows ([#6635](https://github.com/cyberbotics/webots/pull/6635)).
    - Fixed crash when `wb_supervisor_field_get_name` was called with NULL ([#6647](https://github.com/cyberbotics/webots/pull/6647)).
    - Fixed bug where the wrong y coordinate was used for one corner of the box drawn to represent a box-plane collision ([#6677](https://github.com/cyberbotics/webots/pull/6677)).
    - Fixed bug where the rotation axis of the omni wheel rollers were incorrect ([6710](https://github.com/cyberbotics/webots/pull/6710)).
    - Fixed a bug where Webots would crash if a geometry was inserted into a `Shape` node used a bounding box ([#6691](https://github.com/cyberbotics/webots/pull/6691)).
    - Removed the old `wb_supervisor_field_import_sf_node` and `wb_supervisor_field_import_mf_node` functions from the list of editor autocomplete suggestions ([#6701](https://github.com/cyberbotics/webots/pull/6701)).
    - Fixed a bug preventing nodes from being inserted into unconnected proto fields ([#6735](https://github.com/cyberbotics/webots/pull/6735)).
    - Fixed crash when an invalid HDR image was set as a world background ([#6744](https://github.com/cyberbotics/webots/pull/6744)).
