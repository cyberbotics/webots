# Webots R2022 Change Log

## Webots R2022b
Released on XX, XXth, 2022.

  - New Features:
    - Upgraded the simulation server to docker-compose to run Theia IDE on webots.cloud ([#4259](https://github.com/cyberbotics/webots/pull/4259))
    - Moved the robot windows to the web browser ([#4085](https://github.com/cyberbotics/webots/pull/4085)).
    - Added a share button to upload scenes and animations to [webots.cloud](https://webots.cloud) ([#3971](https://github.com/cyberbotics/webots/pull/3971)).
    - Added Wizard for the creation of PROTO files ([#4104](https://github.com/cyberbotics/webots/pull/4104)).
    - Added two new Robot API functions, `wb_robot_step_begin` and `wb_robot_step_end`, to optimize computer intensive controllers ([#4107](https://github.com/cyberbotics/webots/pull/4107)).
    - Changed the behavior of `wb_robot_wwi_receive_text` to iterate through the received messages buffer ([#4336](https://github.com/cyberbotics/webots/pull/4336)).
    - Added the ability to define multiple `Extra Project` paths through the Preferences menu, and an alternative method by setting the `WEBOTS_EXTRA_PROJECT_PATH` environment variable. ([#4364](https://github.com/cyberbotics/webots/pull/4364)). **Previously set Extra project paths should be re-set in the preferences menu**
    - Added support for [TrackWheel](trackwheel.md) and [Track](track.md) animation in WebotsJS ([#4394](https://github.com/cyberbotics/webots/pull/4394)).
  - New Objects:
    - Added some objects on the hospital theme: hospital bed, drip stand, medicine bottle, hand sanitizer, curtain, photo frame, flower pot, emergency exit sign and a fabric appearance ([#4166](https://github.com/cyberbotics/webots/pull/4166)).
    - Extended the CardboardBox to become a container and added a cardboard appearance ([#4359](https://github.com/cyberbotics/webots/pull/4359)).
  - Added [SCARA T6](../guide/scara-epson-t6.md) robot and a demo showing it sorting fruits in a food industry setting ([#4192](https://github.com/cyberbotics/webots/pull/4192).
  - Enhancements:
    - Add a python controller for the Mavic to show how to move the drone to specific coordinates and altitude ([#4293](https://github.com/cyberbotics/webots/pull/4293)).
    - Improved the structure of the **Nao** PROTO: the **version** field changed and the **color** field was replaced with a **customColor** field ([#4180](https://github.com/cyberbotics/webots/pull/4180)).
    - Allowed negative `scale` values in the [Transform](transform.md) node and added a `ccw` field in the [Mesh](mesh.md) node ([#4243](https://github.com/cyberbotics/webots/pull/4243)).
    - Added rendering of anchors in joints ([#4256](https://github.com/cyberbotics/webots/pull/4256)).
  - Dependency Updates
    - Upgraded to Qt6.2 on Windows, macOS and Linux ([#4189](https://github.com/cyberbotics/webots/pull/4189)).
    - Removed Qt WebKit, WebChannel and WebEngine dependencies ([#4137](https://github.com/cyberbotics/webots/pull/4137)).

## Webots R2022a Revision 1
Released on XX XX, 2022.
  - Enhancements
    - Added two new PBR appearances: ScuffedPlastic and WornBurlap ([#4174](https://github.com/cyberbotics/webots/pull/4174)).
    - Added a new HDR background: `music_hall` ([#4177](https://github.com/cyberbotics/webots/pull/4177)).
    - Replaced cubic background PNG images with more efficient JPG images ([#4182](https://github.com/cyberbotics/webots/pull/4182)).
    - Changed the way MATLAB is detected in the system using a new Webots preference ([#4233](https://github.com/cyberbotics/webots/pull/4233)).
    - Forbid the USE of [TrackWheel](trackwheel.md) to avoid wrong behavior ([#4257](https://github.com/cyberbotics/webots/pull/4257)).
  - Cleanup
    - Removed `wb_robot_get_type` API function as it no longer serves a purpose ([#4125](https://github.com/cyberbotics/webots/pull/4125)).
  - Bug fixes
    - Fixed bug in [`wb_supervisor_node_get_field_by_index`](supervisor.md#wb_supervisor_node_get_field_by_index) and [`wb_supervisor_node_get_proto_field_by_index`](supervisor.md#wb_supervisor_node_get_proto_field_by_index) API functions ([#4366](https://github.com/cyberbotics/webots/pull/4366)).
    - Fixed redirection of stdout/stderr to the terminal when no Webots console is open ([#4372](https://github.com/cyberbotics/webots/pull/4372)). 
    - Fixed the URDF exportation of [SolidReference](solidreference.md) nodes ([#4102](https://github.com/cyberbotics/webots/pull/4102)).
    - Fixed insufficient value sanity check in Solid's `postPhysicsStep` resulting in exploding [Track](track.md) ([#4133](https://github.com/cyberbotics/webots/pull/4133)).
    - Fixed `wb_supervisor_world_save` behavior when no argument is provided in non-C APIs ([#4140](https://github.com/cyberbotics/webots/pull/4140)).
    - Fixed the ROS `camera/recognition_objects` topic that was always returning an empty list of objects ([#4139](https://github.com/cyberbotics/webots/pull/4139)).
    - Fixed depths greater than `maxRange` to return `inf` for the [RangeFinder](rangefinder.md) device ([#4167](https://github.com/cyberbotics/webots/pull/4167)).
    - Converted missing sample world `gears.wbt` to ENU ([4201](https://github.com/cyberbotics/webots/pull/4201)).
    - Fixed texture of [Camera](camera.md) devices not being retrieved ([#4218](https://github.com/cyberbotics/webots/pull/4218)).
    - Fixed bug where changes in a DEF node did not propagate for PROTO ([#4245](https://github.com/cyberbotics/webots/pull/4245)).
    - Fixed incorrect update of [Mesh](mesh.md) node in a [Shape](shape.md) when the url is updated either manually or from a supervisor ([#4245](https://github.com/cyberbotics/webots/pull/4245)).
    - Fixed a bug that caused an object to sink into the ground after moving it with a supervisor ([#4070](https://github.com/cyberbotics/webots/pull/4070)).
    - Fixed bug where the [Skin](skin.md) node was invisible both to segmentation and `RangeFinder` devices ([#4281](https://github.com/cyberbotics/webots/pull/4281)).
    - Fixed measurements close to the near plane for the [RangeFinder](rangefinder.md) device ([#4309](https://github.com/cyberbotics/webots/pull/4309)).
    - Fixed bug where updating the url of a [Mesh](mesh.md) node resulted in multiple updated being issued ([#4325](https://github.com/cyberbotics/webots/pull/4325)).

## Webots R2022a
Released on December 21th, 2021.

  - New Features:
    - Released [Skin](skin.md) node ([#3566](https://github.com/cyberbotics/webots/pull/3566)).
    - Added support for rolling friction in [ContactProperties](contactproperties.md) ([#3771](https://github.com/cyberbotics/webots/pull/3771)).
    - Added the [ColladaShapes](../guide/object-shapes.md#colladashapes) PROTO that allows importing a Collada file on the fly ([#3956](https://github.com/cyberbotics/webots/pull/3956)).
    - **The entire library of robots, objects and worlds has been converted to the FLU/ENU coordinate system, and might require manual changes of your local files. Additional details are available [here](https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a).**
  - Enhancements
    - Improved performance on [Lidar](lidar.md) point cloud generation ([#3499](https://github.com/cyberbotics/webots/pull/3499)).
    - Added speed vector output to GPS ([#3742](https://github.com/cyberbotics/webots/pull/3742)).
    - Added `wbu_car_set_[right/left]_steering_angle` that allows to directly control the steering of the wheels ([#3933](https://github.com/cyberbotics/webots/pull/3933)).
    - Improved the user experience when using the object rotation around axis feature ([#3540](https://github.com/cyberbotics/webots/pull/3540)).
    - Increased the mouse wheel speed when zooming the 3D window ([#3565](https://github.com/cyberbotics/webots/pull/3565)).
    - Added warning when attempting to add a node to an already started simulation ([#3926](https://github.com/cyberbotics/webots/pull/3926)).
  - Bug fixes
    - Fixed memory leak due to incorrect cleaning of [ImageTexture](imagetexture.md) nodes ([#3830](https://github.com/cyberbotics/webots/pull/3830)).
    - Fixed various Python API functions crashing with Python 3.9 ([#3502](https://github.com/cyberbotics/webots/pull/3502)).
    - Fixed a crash caused when getting contact points of a PROTO ([#3522](https://github.com/cyberbotics/webots/pull/3522)).
    - Fixed a crash caused by acos function being called with an out-of-range value and leading to a stack overflow ([#3734](https://github.com/cyberbotics/webots/pull/3734)).
    - Fixed crash caused by the auto-regeneration of a [Robot](robot.md) node ([#3869](https://github.com/cyberbotics/webots/pull/3869)).
    - Fixed bug which made the points returned by `getPointCloud` python API inaccessible ([#3558](https://github.com/cyberbotics/webots/pull/3558)).
    - Fixed starting of Webots from the Windows CMD.exe console ([#3512](https://github.com/cyberbotics/webots/pull/3512)).
    - Fixed 'Convert to Base Node(s)' with textures defined by urls ([#3591](https://github.com/cyberbotics/webots/pull/3591)).
    - Fixed memory leak due to incorrect cleaning of shadow coords buffer ([#4038](https://github.com/cyberbotics/webots/pull/4038)).
    - Fixed the force direction applied by the [Track](track.md) node ([#3693](https://github.com/cyberbotics/webots/pull/3693)).
    - Fixed broken Lua gd on Windows ([#3769](https://github.com/cyberbotics/webots/pull/3769)).
    - Fixed [`wb_supervisor_node_set_visibility`](supervisor.md#wb_supervisor_node_set_visibility) applying visibility to parent and sibling nodes if not used with geometry or [Transform](transform.md) nodes ([#3543](https://github.com/cyberbotics/webots/pull/3543)).
    - Fixed updating the robot window after restarting an extern controller ([#3544](https://github.com/cyberbotics/webots/pull/3544)).
    - Fixed calculation of `front_speed_sum` in the Driver library so that both front wheels are considered in case of 4x4 cars ([#3546](https://github.com/cyberbotics/webots/pull/3546)).
    - Fixed Shift + Left Button drag event when the picked [Solid](solid.md) is child of a [Transform](transform.md) node and the horizontal plane is not clearly visible from view ([#3530](https://github.com/cyberbotics/webots/pull/3530)).
    - Fixed mass computation after inserting a [Physics](physics.md) node in case the [Solid.boundingObject](solid.md) was already defined ([#3240](https://github.com/cyberbotics/webots/pull/3240)).
    - Fixed pickable state for cone and cylinder ([#3644](https://github.com/cyberbotics/webots/pull/3644)).
    - Fixed mass calculation of Mesh nodes ([#3719](https://github.com/cyberbotics/webots/pull/3719)).
    - Fixed regression where the v3.3 (21 DoF) variant of the [Nao](../guide/nao.md) PROTO had no hands ([#3696](https://github.com/cyberbotics/webots/pull/3696)).
    - Fixed laser and infra-red distance sensors hitting fully transparent objects ([#3726](https://github.com/cyberbotics/webots/pull/3726)).
    - Fixed propagation of rotation change from supervisor ([#3752](https://github.com/cyberbotics/webots/pull/3752)).
    - Fixed incorrect update of the differential slip ratio in 4x4 vehicles ([#3770](https://github.com/cyberbotics/webots/pull/3770)).
    - Fixed `wb_keyboard_get_key()` to be MT-safe ([#3783](https://github.com/cyberbotics/webots/pull/3783)).
    - Display a warning in the console when the robot battery is empty ([#3783](https://github.com/cyberbotics/webots/pull/3783)).
    - Fixed incorrect node enumeration in Matlab API and missing `WB_MF_ROTATION` constant ([#3808](https://github.com/cyberbotics/webots/pull/3808)).
    - Fixed incorrect boundingSphere computation for ElevationGrid ([#3828](https://github.com/cyberbotics/webots/pull/3828)).
    - Fixed bug where deleting a node from [Supervisor](supervisor.md) did not refresh the scene tree ([#3867](https://github.com/cyberbotics/webots/pull/3867)).
    - Display a warning in case a file cannot be saved or a build process would fail because of insufficient write permissions ([#4046](https://github.com/cyberbotics/webots/pull/4046)).
  - Dependency Updates
    - **Stopped support for Ubuntu 16.04 ([#3480](https://github.com/cyberbotics/webots/pull/3480)).**
