# Webots R2021 Change Log

## Webots R2021a
Released on December XXth, 2020.

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
    - Added a new [`wb_inertial_unit_get_quaternion`](inertialunit.md#wb_inertial_unit_get_quaternion) [InertialUnit](inertialunit.md) API function to get the orientation measurement represented as a quaternion ([#2424](https://github.com/cyberbotics/webots/pull/2424)).
    - **The [InertialUnit](inertialunit.md)`.lookupTable` field was deprecated and the related `wb_inertial_unit_get_lookup_table_size` and `wb_inertial_unit_get_lookup_table` functions were removed ([#2424](https://github.com/cyberbotics/webots/pull/2424)).**
    - Added a `PlasticFruitBox` and a `MetalStorageBox` PROTO object ([#2427](https://github.com/cyberbotics/webots/pull/2427)).
    - Added a `data_type` parameter to a Python method [`getPointCloud`](lidar.md#wb_lidar_get_point_cloud) that retrieves a `bytearray` representation of points a few times faster than the previous version.
    - **Deprecated all the device getter methods in Python.** They are replaced by a new [`Robot.getDevice`](robot.md#wb_robot_get_device) method which is closer to the C API ([#2510](https://github.com/cyberbotics/webots/pull/2510)).
  - Bug fixes
    - **Fixed reversed pixel bytes order for [Display](display.md) images loaded with [`wb_display_image_new`](display.md#wb_display_image_new)** ([#2452](https://github.com/cyberbotics/webots/pull/2452)).
    - Fixed the [webots\_physics\_collide(dGeomID, dGeomID)](callback-functions.md) callback function so that it gets called even if the [`boundingObject`](solid.md#solid-fields) is a [Group](group.md) node ([#2505](https://github.com/cyberbotics/webots/pull/2505)).
  - Cleanup
    - **Deleted `run` mode as the same behavior now can be achieved by using `fast` mode while keeping the rendering turned on ([#2286](https://github.com/cyberbotics/webots/pull/2286)).**
    - **Changed ROS message type published by the [GPS](gps.md) node when it is configured to work in `local` GPS coordinate system ([#2368](https://github.com/cyberbotics/webots/pull/2368))**.
    - Fixed updating the robot's device list after importing a device node at run time in a field descending from a joint ([#2482](https://github.com/cyberbotics/webots/pull/2482)).
