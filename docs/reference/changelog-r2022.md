# Webots R2022 Change Log
## Webots R2022b
Released on XX, XXth, 2022.

  - New Features:
    - Added a share button to upload scene and animation to webots.cloud ([#3971](https://github.com/cyberbotics/webots/pull/3971)).
## Webots R2022a
Released on XX, XXth, 2022.

  - New Features:
    - Released [Skin](skin.md) node ([#3566](https://github.com/cyberbotics/webots/pull/3566)).
    - Added support for rolling friction in [ContactProperties](contactproperties.md) ([#3771](https://github.com/cyberbotics/webots/pull/3771)).
    - Added the [ColladaShapes](../guide/object-shapes.md#colladashapes) PROTO that allows importing a Collada file on the fly ([#3956](https://github.com/cyberbotics/webots/pull/3956)).
  - Enhancements
    - Improved performance on [Lidar](lidar.md) point cloud generation ([#3499](https://github.com/cyberbotics/webots/pull/3499)).
    - Improved the user experience when using the object rotation around axis feature ([#3540](https://github.com/cyberbotics/webots/pull/3540)).
    - Increased the mouse wheel speed when zooming the 3D window ([#3565](https://github.com/cyberbotics/webots/pull/3565)).
    - Added speed vector output to GPS ([#3742](https://github.com/cyberbotics/webots/pull/3742)).
    - Added warning when attempting to add a node to an already started simulation ([#3926](https://github.com/cyberbotics/webots/pull/3926)).
    - Added `wbu_car_set_[right/left]_steering_angle` that allows to directly control the steering of the wheels ([#3933](https://github.com/cyberbotics/webots/pull/3933)).
  - Bug fixes
    - Fixed the force direction applied by the [Track](track.md) node ([#3693](https://github.com/cyberbotics/webots/pull/3693)).
    - Fixed broken Lua gd on Windows ([#3769](https://github.com/cyberbotics/webots/pull/3769)).
    - Fixed [`wb_supervisor_node_set_visibility`](supervisor.md#wb_supervisor_node_set_visibility) applying visibility to parent and sibling nodes if not used with geometry or [Transform](transform.md) nodes ([#3543](https://github.com/cyberbotics/webots/pull/3543)).
    - Fixed updating the robot window after restarting an extern controller ([#3544](https://github.com/cyberbotics/webots/pull/3544)).
    - Fixed calculation of `front_speed_sum` in the Driver library so that both front wheels are considered in case of 4x4 cars ([#3546](https://github.com/cyberbotics/webots/pull/3546)).
    - Fixed Shift + Left Button drag event when the picked [Solid](solid.md) is child of a [Transform](transform.md) node and the horizontal plane is not clearly visible from view ([#3530](https://github.com/cyberbotics/webots/pull/3530)).
    - Fixed various Python API functions crashing with Python 3.9 ([#3502](https://github.com/cyberbotics/webots/pull/3502)).
    - Fixed mass computation after inserting a [Physics](physics.md) node in case the [Solid.boundingObject](solid.md) was already defined ([#3240](https://github.com/cyberbotics/webots/pull/3240)).
    - Fixed a crash caused when getting contact points of a PROTO ([#3522](https://github.com/cyberbotics/webots/pull/3522)).
    - Fixed starting of Webots from the Windows CMD.exe console ([#3512](https://github.com/cyberbotics/webots/pull/3512)).
    - Fixed bug which made the points returned by `getPointCloud` python API inaccessible ([#3558](https://github.com/cyberbotics/webots/pull/3558)).
    - Fixed 'Convert to Base Node(s)' with textures defined by urls ([#3591](https://github.com/cyberbotics/webots/pull/3591)).
    - Fixed pickable state for cone and cylinder ([#3644](https://github.com/cyberbotics/webots/pull/3644)).
    - Fixed mass calculation of Mesh nodes ([#3719](https://github.com/cyberbotics/webots/pull/3719)).
    - Fixed regression where the v3.3 (21 DoF) variant of the [Nao](../guide/nao.md) PROTO had no hands ([#3696](https://github.com/cyberbotics/webots/pull/3696)).
    - Fixed laser and infra-red distance sensors hitting fully transparent objects ([#3726](https://github.com/cyberbotics/webots/pull/3726)).
    - Fixed a crash caused by acos function being called with an out-of-range value and leading to a stack overflow ([#3734](https://github.com/cyberbotics/webots/pull/3734)).
    - Fixed propagation of rotation change from supervisor ([#3752](https://github.com/cyberbotics/webots/pull/3752)).
    - Fixed incorrect update of the differential slip ratio in 4x4 vehicles ([#3770](https://github.com/cyberbotics/webots/pull/3770)).
    - Fixed wb_keyboard_get_key() to be MT-safe ([#3783](https://github.com/cyberbotics/webots/pull/3783)).
    - Display a warning in the console when the robot battery is empty ([#3783](https://github.com/cyberbotics/webots/pull/3783)).
    - Fixed incorrect node enumeration in Matlab API and missing `WB_MF_ROTATION` constant ([#3808](https://github.com/cyberbotics/webots/pull/3808)).
    - Fixed incorrect boundingSphere computation for ElevationGrid ([#3828](https://github.com/cyberbotics/webots/pull/3828)).
    - Fixed memory leak due to incorrect cleaning of [ImageTexture](imagetexture.md) nodes ([#3830](https://github.com/cyberbotics/webots/pull/3830)).
    - Fixed bug where deleting a node from [Supervisor](supervisor.md) did not refresh the scene tree ([#3867](https://github.com/cyberbotics/webots/pull/3867)).
    - Fixed crash caused by the auto-regeneration of a [Robot](robot.md) node ([#3869](https://github.com/cyberbotics/webots/pull/3869)).
  - Dependency Updates
    - **Stopped support for Ubuntu 16.04 ([#3480](https://github.com/cyberbotics/webots/pull/3480)).**
