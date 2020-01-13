# Webots R2020 Change Log

## Webots R2020a Revision 1
Released on January 14th, 2020.

  - New Features
    - Added a `ScratchedPaint` appearance.
  - Enhancements
    - Improved the SUMO interface to use the version of SUMO already installed (if any).
    - Improved the SUMO interface to be compatible with the latest version (1.4).
    - Improved the factory hall environment sample.
  - Bug fixes
    - Fixed the direction of the force applied with the [`wb_supervisor_node_add_force_with_offset()`](supervisor.md#wb_supervisor_node_add_force_with_offset) function.
    - Fix default controller of the [ABB IRB 4600/40](../guide/irb4600-40.md) robot.
    - Fix libController dependencies causing problems with extern controllers.
  - Enhancements
    - **The `wb_range_finder_save_image` function now supports the HDR format instead of TIFF.**
  - Cleanup
    - OpenCV is not distributed within Webots anymore.

## [Webots R2020a](../blog/Webots-2020-a-release.md)
Released on December 18th, 2019.

  - New Robots
    - Added several TIAGo robots from PAL Robotics: [TIAGo Base](../guide/tiago-base.md), [TIAGo Iron](../guide/tiago-iron.md), [TIAGo Steel](../guide/tiago-steel.md), [TIAGo Titanium](../guide/tiago-titanium.md) and [TIAGo++](../guide/tiagopp.md).
    - Added a model of the [Robotino 3](../guide/robotino3.md) from [Festo](https://www.festo-didactic.com/int-en/).
    - Added a model of the [TurtleBot3 Burger](../guide/turtlebot3-burger.md) from [Robotis](http://www.robotis.us/) and its [LDS-01](../guide/lidar-sensors.md#robotis-lds-01) lidar sensor.
  - New Samples
    - Added a `complete_apartment` world.
  - New Features
    - Improved the [Background](background.md) node:
      - Added the `Background.luminosity` field which specifies the light contribution of the [Background](background.md) node. Open this field in the `TexturedBackground` and the `TexturedBackgroundLight` PROTO nodes.
      - Dropped the support of the equirectangular projection in textures to improve loading time.
      - Dropped the `Cubemap` node to improve consistency.
      - Deprecated non-HDR backgrounds.
      - Restored the `Background.*Url` fields, and support only `JPEG` and `PNG` format there.
      - Introduced the `Background.*IrradianceUrl` fields to define an `HDR` irradiance map.
      - Added image tools to help with `HDR` format and equirectangular projections.
      - Added new HDR background: `entrance_hall`
    - Added several new appearances: `Copper`, `CorrugatedPlates`, `CorrugatedPvc`, `DryMud`, `FormedConcrete`,  `Pcb`, `RoughPolymer` and `TireRubber`.
    - Replaced the [Viewpoint](viewpoint.md) `followOrientation` field by a `followType` field for more flexibility.
    - Added new functions to add force and torque to [Solid](solid.md) nodes: `wb_supervisor_node_add_force`, `wb_supervisor_node_add_force_with_offset` and `wb_supervisor_node_add_torque`.
    - Added function to import and remove nodes from SFNode fields: `wb_supervisor_field_remove_sf`, `wb_supervisor_field_import_sf_node` and `wb_supervisor_field_import_sf_node_from_string`.
    - Improved the Webots online 3D viewer: `webots.min.js`
      - Improved support of the Webots rendering pipeline: supported the Bloom post-processing effect.
      - Added support for the `ImageTexture.filtering` field.
      - Improved the console log history. Added a button to clear the console.
    - Improved [robotbenchmark](https://robotbenchmark.net) worlds.
      - Improved overall graphics quality (using the PBR materials and the HDR backgrounds).
      - Improved `humanoid_sprint` benchmark metrics.
    - Added a [script to cleanup the Webots preferences](https://github.com/cyberbotics/webots/blob/master/scripts/preferences_cleaner/README.md).
    - Linux: Added support for Python 3.8.
  - Enhancements
    - **Updated argument type in the [`wb_robot_set_mode()`](robot.md#wb_robot_set_mode) function from `void *` to `const char *`.**
    - **Renamed the `physics` field of the `MultiSenseS21` and `SickLdMrs` PROTO nodes into `enablePhysics`.**
    - Improved the [Sick LD MRS](../guide/lidar-sensors.md#sick-ld-mrs) PROTO to support the following types: `400001`, `400102`, `400001S01`, `400102S01` and `800001S01`.
    - **Improved the [`wb_supervisor_simulation_reset`](supervisor.md#wb_supervisor_simulation_reset) to avoid restarting the controllers, if needed the controllers can be restarted with [`wb_supervisor_node_restart_controller`](supervisor.md#wb_supervisor_node_restart_controller).**
    - Set the [ABB IRB 4600/40](../guide/irb4600-40.md) root node to [Robot](robot.md) instead of [Solid](solid.md) to be able to insert it everywhere.
    - Webots now waits for extern controllers if the `Robot.synchronization` field is set to `TRUE`.
    - Device names are displayed in the scene tree next to the node name.
    - Split the Webots and controller libraries to avoid possible conflicts with external libraries.
    - Windows/Linux: Move the `Check for updates...` menu from `Tools` to `Help` for consistency with other applications.
    - E-puck2
      - Added the pi-puck extension.
      - Fixed [DistanceSensor](distancesensor.md) noise calibration.
      - Improved the meshes and appearances.
  - Bug fixes
    - Fixed the [`wb_supervisor_node_reset_physics()`](supervisor.md#wb_supervisor_node_reset_physics) and [`wb_supervisor_simulation_reset_physics()`](supervisor.md#wb_supervisor_simulation_reset_physics) functions when applied on dynamic objects (in motion and containing joints).
    - Fixed [Lidar](lidar.md) point cloud access in controllers (thanks to Alexander).
    - Fixed bugs in Python Display.imageNew() when passing an image array: rearranged image data from column-major order and memory leak (thanks to Inbae Jeong).
    - Fixed [Nao.selfCollision](../guide/nao.md) due to overlapping bounding objects in feet (thanks to Sheila).
    - Fixed [infra-red DistanceSensors](distancesensor.md) or [Pen](pen.md) versus [Plane](plane.md) collision detection.
    - Fixed determinism in camera rendering order.
    - Fixed missing `WB_NODE_MUSCLE` and `WB_NODE_PROPELLER` types in [`wb_node_get_name()`](supervisor.md#wb_supervisor_node_get_type) function.
    - Fixed missing `WB_NODE_NORMAL` node type in MATLAB API.
    - Fixed arguments of [`wb_supervisor_node_get_number_of_contacts_points()`](supervisor.md#wb_supervisor_node_get_number_of_contact_points) function in MATLAB API.
    - Fixed missing [`wb_robot_set_mode()`](robot.md#wb_robot_set_mode) function in MATLAB API.
    - Fixed `simulation_server.py` script to work with Python3.
    - Fixed `simulation_server.py` script overwriting the DISPLAY environment variable.
    - Fixed exporting first translation and rotation fields change during animation recording and simulation streaming.
    - Fixed displaying streaming server initialization errors in the Webots console.
    - Fixed errors sending messages containing single quote (') and backslash (\) to the robot windows.
  - Dependency Updates
    - Upgraded to Qt 5.13.1 on Windows and macOS.
  - Cleanup
    - Drop support of native Qt robot windows:
      - Dropped the documentation.
      - Dropped automatic link to Qt libraries from controller and plugins default Makefile.include.
      - Dropped the `samples/howto/gui_tracker.wbt` simulation.
      - Dropped the OSM import robot window.
