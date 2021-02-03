# Webots R2021 Change Log

## Webots R2021a Revision 1
Released on XX Xth, 2021.

  - Enhancements
    - Added a script to convert PROTO files to use [Mesh](mesh.md) nodes instead of [IndexedFaceSet](indexedfaceset.md) nodes to speed-up loading times, improve PROTO readability and maintenance ([#2668](https://github.com/cyberbotics/webots/pull/2668)).
    - Converted several PROTO files to use [Mesh](mesh.md) nodes ([#2668](https://github.com/cyberbotics/webots/pull/2668)).
    - Don't display warnings for recent Intel and AMD graphics cards ([#2623](https://github.com/cyberbotics/webots/pull/2623)).
    - Rework of car meshes to have more realistic rear lights for Mercedes Benz, Lincoln, Citroen, BMW and Range Rover models ([#2615](https://github.com/cyberbotics/webots/pull/2615)).
  - Bug fixes
    - Fixed [X3D export](https://www.cyberbotics.com/doc/guide/web-interface) of USE nodes linking to DEF nodes declared in PROTO exposed fields ([#2687](https://github.com/cyberbotics/webots/pull/2687)).
    - Fixed memory leak in [Display](display.md) image ([#2663](https://github.com/cyberbotics/webots/issues/2663)).
    - Fixed high DPI support on Windows and Linux ([#2631](https://github.com/cyberbotics/webots/pull/2631)).
    - Fixed painting with the [Pen](pen.md) device on the bottom face of a [Cylinder](cylinder.md) geometry ([#2629](https://github.com/cyberbotics/webots/pull/2629)).
    - Fixed Display external window not updated when the attached camera image changes ([#2589](https://github.com/cyberbotics/webots/pull/2589)).
    - Fixed reset of simulations including [BallJoint](balljoint.md) nodes like the [Stewart Platform](https://github.com/cyberbotics/webots/blob/master/projects/samples/demos/worlds/stewart_platform.wbt) ([#2593](https://github.com/cyberbotics/webots/pull/2593)).
    - Fixed in the interaction between [IndexedFaceSets](indexedfaceset.md) and distance sensor rays that resulted in the wrong contact point being considered for collision ([#2610](https://github.com/cyberbotics/webots/pull/2610)), affecting TexturedBoxes.
    - Fixed a strategy used to find a MATLAB executable in the `PATH` environment variable ([#2624](https://github.com/cyberbotics/webots/pull/2624)).
    - Fixed external force/torque logic such that the closest dynamic Solid ancestor is picked if the selected one lacks it ([#2635](https://github.com/cyberbotics/webots/pull/2635)).
    - Fixed the [robot window example](../guide/samples-howto.md#custom_robot_window-wbt) ([#2639](https://github.com/cyberbotics/webots/pull/2639)).
    - Fixed visual bug where the [Lidar](lidar.md) point cloud disappears when out-of-range points are present ([#2666](https://github.com/cyberbotics/webots/pull/2666)).
  - Cleanup
    - Changed structure of the [projects/samples/howto]({{ url.github_tree }}/projects/samples/howto) directory, so each demonstration is in a dedicated directory ([#2639](https://github.com/cyberbotics/webots/pull/2639)).

## Webots R2021a
Released on December 15th, 2020.

  - New Features
    - Added the `wb_supervisor_node_get_from_device` function to retrieve the node's handle of a device ([#2074](https://github.com/cyberbotics/webots/pull/2074)).
    - Devices added during simulation run are now available in the robot controller ([#2237](https://github.com/cyberbotics/webots/pull/2237)).
    - Added the `wb_connector_is_locked` function to returns the current *isLocked* state of a [Connector](connector.md) ([#2087](https://github.com/cyberbotics/webots/pull/2087)).
    - Added the possibility to use an EUN (East-Up-North) coordinate system in the `coordinateSystem` field of the [WorldInfo](worldinfo.md) node ([#2228](https://github.com/cyberbotics/webots/pull/2228)).
    - Added the possibility to disable rendering in the `realtime` mode ([#2286](https://github.com/cyberbotics/webots/pull/2286)).
    - **Added a new argument to the [Supervisor](supervisor.md) [`wb_supervisor_node_get_number_of_contact_points`](supervisor.md#wb_supervisor_node_get_number_of_contact_points) API function to retrieve the number of contact points including those of the desendant nodes ([#2228](https://github.com/cyberbotics/webots/pull/2228)).**
    - Added a new [`wb_supervisor_node_get_contact_point_node`](supervisor.md#wb_supervisor_node_get_contact_point_node) [Supervisor](supervisor.md) API function to get the node reference associated to a given contact point ([#2228](https://github.com/cyberbotics/webots/pull/2228)).
    - Added two new functions to the [Camera](camera.md) API called [`wb_camera_get_exposure`](camera.md#wb_camera_get_exposure) and [`wb_camera_set_exposure`](camera.md#wb_camera_set_exposure) to retrieve and change the [Camera](camera.md) exposure ([#2363](https://github.com/cyberbotics/webots/pull/2363)).
    - Added a new functionality in the [Recognition](recognition.md) node and [Camera](camera.md) API for generating segmented ground truth images ([#2199](https://github.com/cyberbotics/webots/pull/2199)).
    - Added options to enable and disable recognition and segmentation functionalities in the [Camera](camera.md) tab of the default robot window ([#2431](https://github.com/cyberbotics/webots/pull/2431)).
    - Added a new [`wb_inertial_unit_get_quaternion`](inertialunit.md#wb_inertial_unit_get_quaternion) [InertialUnit](inertialunit.md) API function to get the orientation measurement represented as a quaternion ([#2424](https://github.com/cyberbotics/webots/pull/2424)).
    - **The [InertialUnit](inertialunit.md)`.lookupTable` field was deprecated and the related `wb_inertial_unit_get_lookup_table_size` and `wb_inertial_unit_get_lookup_table` functions were removed ([#2424](https://github.com/cyberbotics/webots/pull/2424)).**
    - Added a `PlasticFruitBox` and a `MetalStorageBox` PROTO object ([#2427](https://github.com/cyberbotics/webots/pull/2427)).
    - Added a `data_type` parameter to a Python method [`getPointCloud`](lidar.md#wb_lidar_get_point_cloud) that retrieves a `bytearray` representation of points a few times faster than the previous version.
  - Enhancements
    - Added support for Python 3.9 ([#2318](https://github.com/cyberbotics/webots/pull/2079)).
    - Improved [Solid](solid.md).`recognitionColors` value for the [BiscuitBox](../guide/object-kitchen.md#biscuitbox) PROTO model ([#2401](https://github.com/cyberbotics/webots/pull/2401)).
    - Improved documentation of [Camera](camera.md) image format and how to show the image retrieved through the [Camera](camera.md) API in a [Display](display.md) device ([#2443](https://github.com/cyberbotics/webots/pull/2443)).
    - The [Lidar](lidar.md) and [RangeFinder](rangefinder.md) devices now return infinity when the depth is out of range ([#2509](https://github.com/cyberbotics/webots/pull/2509)).
  - Bug fixes
    - **Fixed reversed pixel bytes order for [Display](display.md) images loaded with [`wb_display_image_new`](display.md#wb_display_image_new)** ([#2452](https://github.com/cyberbotics/webots/pull/2452)).
    - **Fixed cube, compact and flat texture mappings of [TexturedParallelepiped](../guide/object-geometries.md#texturedparallelepiped) proto** ([#2364](https://github.com/cyberbotics/webots/pull/2364)).
    - macOS: Fixed ability to inverse the distance sensor condition in BotStudio with the e-puck robot ([#2391](https://github.com/cyberbotics/webots/pull/2391)).
    - Fixed crash in the Python [Display.imageNew](display.md#wb_display_image_new) function when passing the image data in string/bytes format ([#2443](https://github.com/cyberbotics/webots/pull/2443)).
    - Fixed crash in the [Supervisor](supervisor.md) API occurring when [setting](supervisor.md#wb_supervisor_field_set_mf_bool) an item of a multiple field just before [inserting](supervisor.md#wb_supervisor_field_insert_mf_bool) or [removing](supervisor.md#wb_supervisor_field_remove_mf) an item in the same field during the same controller step ([#2366](https://github.com/cyberbotics/webots/pull/2366)).
    - Fixed values returned by the [Supervisor](supervisor.md) [get field value](supervisor.md#wb_supervisor_field_get_sf_bool) functions after [setting](supervisor.md#wb_supervisor_field_set_sf_bool) the field value in the same controller step ([#2375](https://github.com/cyberbotics/webots/pull/2375)).
    - Fixed supervisor label color change which was not working if the text remained unchanged ([#2357](https://github.com/cyberbotics/webots/pull/2357)).
    - Fixed recording of movies which was broken when the [WorldInfo](worldinfo.md).`basicTimeStep` was greater than 40 ([#2268](https://github.com/cyberbotics/webots/pull/2268)).
    - Fixed recording of movie canceled on simulation reset if recording is started from a [Supervisor](supervisor.md#wb_supervisor_movie_start_recording) controller ([#2406](https://github.com/cyberbotics/webots/pull/2406)).
    - macOS: Fixed handling of <kbd>Ctrl</kbd> key from the [Keyboard](keyboard.md) API ([#2265](https://github.com/cyberbotics/webots/pull/2265)).
    - Fixed synchronization bug in the start up of extern controllers when the `WEBOTS_PID` environment variable was defined ([#2260](https://github.com/cyberbotics/webots/pull/2260)).
    - Fixed large amount of CPU resources consumed by Webots in run mode when waiting for extern controllers ([#2377](https://github.com/cyberbotics/webots/pull/2377)).
    - Fixed [Lidar](lidar.md) and [RangeFinder](rangefinder.md) memory leak when the robot-window is opened ([#2210](https://github.com/cyberbotics/webots/pull/2210)).
    - Fixed noise generation for [Camera](camera.md), [Lidar](lidar.md) and [RangeFinder](rangefinder.md) producing fixed patterns on some GPUs (like the NVIDIA GeForce RTX series)([#2215](https://github.com/cyberbotics/webots/pull/2215)).
    - Fixed re-initialization of external camera window if recognition is enabled ([#2196](https://github.com/cyberbotics/webots/pull/2196)).
    - Fixed precision of devices using rays ([Camera](camera.md), [DistanceSensor](distancesensor.md), [Radar](radar.md), and [Receiver](receiver.md)) located on articulated robots' parts ([#2266](https://github.com/cyberbotics/webots/pull/2266)).
    - Fixed position lags of articulated robots' parts in recorded [animations](../guide/web-animation.md) ([#2266](https://github.com/cyberbotics/webots/pull/2266)).
    - Fixed the `inverse_kinematics` controller ([#2211](https://github.com/cyberbotics/webots/pull/2211)).
    - Fixed exported URDF axis when the [Joint](joint.md) anchor is not equal to the [Solid](solid.md) endpoint translation ([#2212](https://github.com/cyberbotics/webots/pull/2212)).
    - Fixed disappearing [DistanceSensor](distancesensor.md) rays [optional rendering](https://cyberbotics.com/doc/guide/the-user-interface#view-menu) after simulation reset ([#2276](https://github.com/cyberbotics/webots/pull/2276)).
    - Fixed cylinder-to-cylinder collision detection in certain cases ([#2282](https://github.com/cyberbotics/webots/pull/2282)).
    - Windows: Fixed parsing of arguments with spaces when starting Webots from a DOS console ([#2330](https://github.com/cyberbotics/webots/pull/2330)).
    - Fixed [Battery](robot.md#wb_robot_battery_sensor_enable) sensor when the [Robot](robot.md) contains a [Propeller](propeller.md) device ([#1801](https://github.com/cyberbotics/webots/pull/1801)).
    - Fixed [Motor](motor.md) target position not updated when changing the `position` field of a [JointParameters](jointparameters.md) node ([#2326](https://github.com/cyberbotics/webots/pull/2326)).
    - Fixed [`wb_motor_get_target_position`](motor.md#wb_motor_get_target_position) function if the joint initial position is not 0 ([#2326](https://github.com/cyberbotics/webots/pull/2326)).
    - Fixed the disabled `Help...` item in the context menu for some nodes ([#2327](https://github.com/cyberbotics/webots/pull/2327)).
    - Added missing `supervisor` field in the `Tractor` and `TeslaModel3` PROTO nodes ([#2351](https://github.com/cyberbotics/webots/pull/2351)).
    - Fixed infra-red [DistanceSensor](distancesensor.md) returned value when pointing at a texture used several times in the world (thanks to [Justin-Fisher](https://github.com/Justin-Fisher)) ([#2378](https://github.com/cyberbotics/webots/pull/2378)).
    - Fixed performance regression (thanks to [Acwok](https://github.com/Acwok)) ([#2434](https://github.com/cyberbotics/webots/pull/2434)).
    - **Deprecated all the device getter methods in Python.** They are replaced by a new [`Robot.getDevice`](robot.md#wb_robot_get_device) method which is closer to the C API ([#2510](https://github.com/cyberbotics/webots/pull/2510)).
    - **Fixed reversed pixel bytes order for [Display](display.md) images loaded with [`wb_display_image_new`](display.md#wb_display_image_new)** ([#2452](https://github.com/cyberbotics/webots/pull/2452)).
    - Fixed the [webots\_physics\_collide(dGeomID, dGeomID)](callback-functions.md) callback function so that it gets called even if the [`boundingObject`](solid.md#solid-fields) is a [Group](group.md) node ([#2505](https://github.com/cyberbotics/webots/pull/2505)).
  - Cleanup
    - **Deleted `run` mode as the same behavior now can be achieved by using `fast` mode while keeping the rendering turned on ([#2286](https://github.com/cyberbotics/webots/pull/2286)).**
    - **Changed ROS message type published by the [GPS](gps.md) node when it is configured to work in `local` GPS coordinate system ([#2368](https://github.com/cyberbotics/webots/pull/2368))**.
    - Fixed updating the robot's device list after importing a device node at run time in a field descending from a joint ([#2482](https://github.com/cyberbotics/webots/pull/2482)).
  - Dependency Updates
    - Upgraded to Qt 5.15.1 on Windows ([#2312](https://github.com/cyberbotics/webots/pull/2312)).
    - **Stopped releasing Webots packages compatible with Ubuntu 16.04.** To run Webots on Ubuntu 16.04, please [compile from sources](https://github.com/cyberbotics/webots/wiki/Linux-installation) ([#2473](https://github.com/cyberbotics/webots/pull/2473)).
    - Updated the Thymio II Aseba controller to Aseba 1.3.3 ([#2518](https://github.com/cyberbotics/webots/pull/2518)).
