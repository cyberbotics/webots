# Webots 6 Changelog


## Webots 6.4.4
Released on April 4th, 2012 (revision 8839).

  - Added new Nao protos: `Nao_H21_V40`, `Nao_H25_V40`, `Nao_H21_V33` and `Nao_H25_V33`, matching the most recent documentation of Aldebaran
  - Added new `naoqisim` controller to interface between NaoQi (e.g. Choregraphe) and Webots
  - Removed deprecated `naoqi_for_webots` controller code which is now replaced by `naoqisim`
  - Removed all Webots 5 compatibility functions, e.g. function with names not starting with the `wb_` prefix
  - Removed deprecated mtn_*() functions (Aibo)
  - Set the default language to English (was previously set to the system language)
  - Fixed bug when making high resolution movies
  - Restored original direction of torque applied by `wb_servo_set_force()` for `rotational` Servo (thanks to Jesse)
  - Fixed a matlab specific bug regarding the function `wb_get_range_image()` which happens when a camera is used in `both` mode
  - Fixed a path problem arising when a world file is saved in a directory whose names begins with `webots` but which is not WEBOTS_HOME
  - Removed long-time deprecated non-object-oriented version of Java API and ControllerLauncher
  - Adjusted the speedUnit and wheelRadius values for the Khepera III PROTO (thanks to Felix)
  - Fixed: TouchSensor returned wrong values if anything was changed in the Scene Tree and the world was not reloaded
  - Improved the physics auto-disable feature by coloring the boundingObject of the disabled solids in blue
  - DARwIn-OP: calibrated the servo on the real robot (thanks to Vitor)
  - Don't display trial version dialog box if the license warnings are disabled in the preferences (thanks to Lucian)
  - Added missing file: `Webots/resources/ogre/textures/wrong_texture_size.png` which was causing crash when attempting to use a texture with a wrong resolution (thanks to Mattej)

## Webots 6.4.3
Released on January 10th, 2012 (revision 8315).

  - Fixed crash when IndexedFaceSet having wrong coordIndex was defined as boundingObject (thanks to Hunter)
  - Adjusted the horizontal field of view of the camera of the e-puck robot to 48 degrees (thanks to Nikola)
  - Upgraded to Python 2.7 (deprecating Python 2.6)
  - Mac OS X: added support for Lion (10.7) in addition to Snow Leopard (10.6), dropped support for Leopard (10.5)
  - Windows: fixed false positive detection of a Trojan by Kapersky anti-virus (thanks to Abdeslam)
  - Linux: fixed missing main menu bar of Webots with recent window managers (thanks to Jean-Marc)

## Webots 6.4.2
Released on December 9th, 2011 (revision 8033).

  - Fixed: when using force control (i.e. `wb_servo_set_force()`), the `maxForce` (as specified either in the world or using `wb_servo_set_motor_force()`) was only respected for positive forces/torques (thanks to Jesse)
  - Display a warning when a texture doesn't have a correct size: width and height should be a power of two (Thanks to Cristiano)
  - Added the visual notification of collisions by a change in the color of the bounding objects
  - Added the possibility to interactively apply a torque to a solid node by holding the both the Ctrl and Alt keys, left-clicking and dragging the mouse
  - Fixed: some edges of bounding meshes were not visible (Thanks to Taylor Murphy and Alex Morison)
  - Fixed: on Mac OS X and Linux, Webots could not save a movie in a folder which name or path contains a space (Thanks to Tobias)
  - Fixed: on Mac OS X, Webots could not execute controllers compiled with non-Universal x86\_64 architecture (thanks to Sébastien)
  - Fixed: `wb_supervisor_node_get_field()` now works also with PROTO fields
  - Use the default HTML or PDF browser under Linux
  - Fixed: when the Makefile was missing for the compilation of a physics plugin, Webots used to instal the wrong type of Makefile
  - Added the Hokuyo URG-04LX, URG-04LX-UG01 and UTM-30LX devices and a related example
  - Fixed bug related with the Display device which was interactive even if not used
  - Fixed bug related with the DifferentialWheels having children with physics: a hinge joint was created rather than a fixed joint.
  - Fixed e-puck crosscompilation on Mac OS X when starting Webots from the icon (thanks to Paul)
  - Fixed: crash when first the Solid is not found in a call to dWebotsGetGeomFromDEF() where the `.` character is used to delimit several Solid nodes
  - Fixed: Solid nodes with boundingObject containing a Group crashed the multiclustering system used in Robotstadium
  - Added an example of passive dynamic walker (thanks to Sasha)
  - Fixed: `wb_supervisor_node_get_from_def()` did not work with fields placed into PROTO nodes (thanks to Li Liu)
  - Fixed the Java functions allowing to get the grey channel of the cameras (thanks to Jorge)

## Webots 6.4.1
Released on September 1st, 2011 (revision 7471).

  - Added DARwIn-OP, youBot, Nao and Pioneer 3AT simulations to guided tour
  - Added models of Pioneer 3DX and Pioneer 3AT (running with SICK lidar and Microsoft Kinect)
  - Fixed networking problems with license server on slow Internet connections
  - Katana (IPR) robot: allowed the build of ipr clients from the Webots IDE
  - Allowed a bunch of features in the FREE version of Webots: world save, text save, controller cross-compilation and new project wizard
  - Improved cameras
    - Added the Microsoft Kinect device
    - Added a new camera type (called `both`) allowing to get simultaneously the color and the depth data
    - Improved the performance of the cameras in fast mode
    - Improved the performance of the noisy cameras
    - Added the possibility to add noise on a range-finder camera
    - Fixed bug with cameras in fast mode
    - Separated the gaussianNoise field into two fields (colorNoise and depthNoise). **Warning: gaussianNoise is deprecated from now**
  - Fixed crash when adding devices and reseting a controller from a Supervisor (thanks to Sven)
  - Fixed crash when modifying the scene tree after an `import VRML97` operation (thanks to Weiwei Wan)
  - e-puck: Fixed bug preventing to set speeds in remote-control mode (thanks to Christophe)
  - DARwIn-OP: proto of the robot and controller using the DARwIn-OP gait generator
  - KUKA youBot
    - Added a new wheels model based on non-uniform friction increasing drastically the simulation speed
    - Added a small inverse kinematics module
    - Added a prototype of the tower of hanoi contest

## Webots 6.4.0
Released on May 31th, 2011 (revision 7009).

  - Linux: removed dependency on libcurl
  - Windows: upgraded built-in gcc to version 4.5.2
  - Improved Cylinder-Cylinder and Cylinder-Capsule collisions by adding new contact points in case of resting contacts
  - Added a model of the Robotis DARwIn-OP robot
  - Added a model of the KUKA youBot robot
  - Improved the camera model of the e-puck: adjusted its translation and its near fields (thanks to Jose)
  - Fixed crash occurring when the number of rays of a DistanceSensor having an aperture of 0 was modified (thanks to Juan)
  - Modified how contact properties (coulombFriction, bounce, bounceVelocity and forceDependentSlip fields) must be specified
  - Added new Damping node that allow to damp the linear and angular velocity of any Solid object
  - Updated latest URBI version 2.7.1 (thanks to Mefyl)
  - Added a model of the Boomer 3050 tractor
  - Added a model of the Puma 560 robot arm (thanks to Andy)
  - Improved drawing of contact surfaces: now only the contour of each contact surface is drawn
  - Fixed wrong library name for libzzip-0 on Linux 32
  - Adapted Robotstadium simulation to the SPL 2011 rules (mainly changed from 3 to 4 Nao per team)
  - Simplified ROS configuration and facilitated the use the Webots C++ API in a C++ ROS node
  - Reduced the save time for world files containing large IndexedFaceSet nodes
  - Extended the xDimension * zDimension limits for ElevationGrid (thanks to Aaron)
  - Fixed crash when a Shape that contains an Extrusion or a Cone is DEF`d and then USE`d in a boundingObject (thanks to Aaron)
  - Fixed: rotations entered in the Scene Tree were not automatically normalized (thanks to Aaron)

## Webots 6.3.4
Released on February 25th, 2011 (revision 6445).

  - Reduced the load time for world files containing large IndexedFaceSet nodes
  - Fixed problems with relative paths for text editor files
  - Fixed typos and entirely proofread Webots User Guide (thanks to Luc)
  - Ratslife: Adapted the scenario in order to begin the third edition
  - Fixed the e-puck crosscompilation tool: uploading a new firmware was difficult when the current firmware was printing on stdout
  - Added a new Nao model (NaoV3H.proto) with hands and wrist motors: LWristYaw, LHand, RWristYaw and RHand
  - Adapted `naoqi_for_webots` project to support hands and wrist motors
  - Global license server setup available in the preferences (beta)

## Webots 6.3.3
Released on January 17th, 2011 (revision 6135).

  - Linux: fixed dependency on libcurl-gnutls (replaced with libcurl.so.4)
  - Fixed inappropriate warning occuring when reverting a new world (new.wbt)
  - Fixed crash when a MFString is focused in the Scene Tree and then `Delete` button is clicked
  - Fixed crash when numeric field is focused in the Scene Tree and then the `Revert`, `Load` or `New` button is clicked
  - Added a sample of passive rope (thanks to Jalila)
  - Improved the structure of the resources directory (improved the hierarchy scalability, and added the possibility to create default controllers)
  - Added a model of iRobot Create, and its virtual wall
  - Added a library of 40 interactive objects related with the apartment theme
  - Changed build system for `naoqi_for_webots` controller: a standard Webots Makefile is provided for Linux and a .vcproj is provided for Windows, the cmake files are no longer distributed
  - Upgraded to Ogre 1.7.2 rendering system (Linux and Windows only, Mac OS X version of Ogre 1.7.2 is apparently buggy, so we kept version 1.7.1)
  - Added the possibility to have proto fields of type SFNode, e.g. physics, boundingObject, geometry, material, appearance, color, etc.
  - Fixed bug with some non-English locales under Linux (and possibly other systems)

## Webots 6.3.2
Released on December 9th, 2010 (revision 5890).

  - Restored HTML built-in documentation
  - Fixed: in specifying contact properties, the Physics.bounce field was wrongly used instead of the Physics.bounceVelocity field (which was ignored)
  - Added a sample ROS (Robot Operating System) controller node in Webots / projects / languages / ros / (Linux and Mac OS X only)
  - Filter out of the console any ANSI escape sequence (produced by Matlab, ROS and others)
  - Added the possibility to automatically remove idle solids from the physics computation (see the WorldInfo::physicsDisable* fields)
  - Added the possibility to fix the root solid of a robot to the static environment (by removing its physics field)
  - Fixed crash when the user answers `Yes` to the question: `Adapt translation and rotation to preserve absolute position and orientation?` (thanks to Robin)
  - Fixed: imported/exported objects in the Scene Tree now use .wbo (instead of .wbt) as filename extension (thanks to Robin)
  - Fixed fatal error caused by missing name on dongle
  - Fixed `*** missing separator. Stop.` error when compiling physics plugins (Windows only)
  - Fixed race condition in the creation of temp files that occured in simulations containing several Matlab controllers
  - Merged `Insert after` and `New node` buttons into new `Add New` button
  - Fixed: when a DEF node was reset (reset to default) in the Scene Tree the first corresponding USE occurence did not reflect the change
  - Fixed startup crash on some Mac OS X 10.5 locale configurations (thanks to François)
  - Fixed (shared memory) stability issue that appears while repeatedly changing the Robot's controller (using `wb_supervisor_field_set_sf_string()`) while a Camera is in use (Windows only)
  - Fixed: changes made to the translation/rotation fields of objects using in the Scene Tree were ignored unless the .wbt was saved and then reverted (but this worked fine when the objects were moved using the mouse)

## Webots 6.3.1
Released on October 20th, 2010 (revision 5670).

  - Upgraded URBI for Webots to version 2.3
  - Fixed: 30 day trial version was disabled if license warning were disabled
  - Fixed crash occuring when deleting the physics node of a Solid wheel in a DifferentialWheel robot
  - Fixed: robot devices can now be deleted while the simulation and even if they are used (as a WbDeviceTag) by the controller (thanks to Gabor)
  - Added new `wb_supervisor_node_get_position()` and `wb_supervisor_node_get_orientation()` functions to easily find the current global position and orientation of a node
  - The function `wb_supervisor_node_get_name()` was renamed `wb_supervisor_node_get_type_name()`
  - Fixed: Servo axes may become incorrect when anything is modified in the Scene Tree while the simulation is running
  - Fixed problem with VRML97 import caused by special characters (including `-`) in DEF names
  - Made the delete operation more user friendly in the scene tree: after deleting a list item, the next list item is selected
  - Improved the behavior of the increase and decrease buttons for floating point values in the scene tree: the increment (or decrement) is computed from the precision (latest decimal) of the floating point value
  - Improved the behavior of the text field in the scene tree: the text is actually changed only after the user presses enter or leaves (i.e., unselects) the text field

## Webots 6.3.0
Released on September 14th, 2010 (revision 5549).

  - Deprecated the useless `wb_camera_move_window()` function
  - Fixed: red and blue components were inverted in the `color` argument of the `wb_pen_set_ink_color()` function
  - Fixed MATLAB R2010a compatibility issue: `Error using ==> feval [...] The input character is not valid in MATLAB statements or expressions.`
  - Unified the return values of Webots according to the POSIX standard (thanks to Alexandre)
  - Added a parameter to the `wb_supervisor_simulation_quit()` function allowing to define the exit status of Webots (thanks to Alexandre)
  - Ratslife: added a new supervisor generating random mazes

## Webots 6.3.beta1
Released on September 3rd, 2010 (revision 5507).

  - Fixed Matlab version of `wb_touch_sensor_get_values()` function (thanks to Sylvain)
  - Improved the Display device
    - Raised the `wb_display_image_delete()` function to the OO languages in order to facilitate the memory management of the internal images
    - Added a new pixel format to the `wb_display_image_new()` function
    - Fixed a memory leak related with the ImageRef class under Java (thanks to Aaron)
    - Fixed bug with the pixel format of the Display.imageNew() function under Java (thanks to Nikolaus)
  - Fixed bug with supervisor translation/rotation field set for some Solid nodes containing other Solid nodes (thanks to Thierry)
  - Fixed: the real-time simulation speed (WorldInfo.runRealTime=TRUE) could be inacurate by a few percents depending on the simulation
  - Added a `filtering` field to the ImageTexture node
  - Removed obsolete Physics.orientation field:
  - Fixed various errors in automatic calculation of inertia matrices (dMass)
  - Manualy specified inertia matrices (Physics.inertiaMatrix) must now contain only 6 values instead of 9, see Reference Manual for description
  - Added automatic calculation of inertia matrix for IndexedFaceSet nodes (in boundingObject), it is no longer required to specify the inertia matrix manually for these nodes
  - Upgraded to ODE-0.11.1:
    - Improved IndexedFaceSet (trimesh-trimesh) collision detection
    - Improved collision detection of Cylinder with the other primitives (more stable contact points)
    - Added cylinder-trimesh (Cylinder-IndexedFaceSet) collision detection
  - Added new possibility to simulate uneven terrain (a.k.a. height map, height field, elevation data):
    - Added possibility to specify the `height` values in the ElevationGrid node
    - Added possibility to use the ElevationGrid as a collision detection primitive (in boundingObjects)
    - Added `uneven_terrain.wbt` demo in Webots Guided Tour
  - Added new `Capsule` node, that can be used as graphical or collision detection primitive

  - Added new `Plane` node, that can be used as graphical or collision detection primitive, e.g. for the floor, a slope, infinite walls
  - Fixed bug preventing botstudio controllers to work and causing Webots to quit after complaining that the void controller is missing
  - Added `x86_64` architecture in libController.dylib: this allows to compile controllers in 64-bit and to inter-operate with 64-bit versions of Matlab, Java and Python (Mac)
  - Removed `ppc` architecture from libController.dylib and Webots executable (Mac). **Warning: support for PowerPC Macs was definitively dropped with this release! Do not install this version on a PowerPC Mac!**

## Webots 6.2.4
Released on June 17th, 2010 (revision 4967).

  - Fixed: Connector axes (green, blue and black lines) were drawn inconsistently (thanks to Mehdi)
  - Launch correct version of webots-bin (wrt glibc) on ubuntu 10.4
  - Fixed: some wrong constant values in include/controller/cpp/webots/Node.hpp and lib/matlab/WB_NODE_*.m (thanks to Thierry)
  - Fixed: `wb_supervisor_node_get_from_def()` fails for nodes that have previously been returned by `wb_supervisor_field_get_sf_node()` or `wb_supervisor_field_get_mf_node()` (thanks to Thierry)
  - Improved Viewpoint rotation: the rotation center is now the 3D coordinate of the mouse click rather than the center of the selected object
  - Upgraded to Ogre 1.7.1 rendering system
  - Fixed: webots was sometimes unable to start a controller due to a combination of spaces and conflicting files in the path of the controller's executable: CreateProcess() failed with error code 193, (Windows)
  - Added an example of robot with three omni-wheels: `projects/samples/howto/worlds/omni_wheels.wbt` (thanks to Mehdi)
  - Upgraded to wxWidgets 2.8.11
  - Added Urbi 2.0 engine and examples
  - Webots does no longer allow to modify any file located in its installation directory, instead it offers to automatically copy the project to a safe location
  - Renamed directory projects/packages to projects/languages
  - Moved directory `projects/default/*` into `resources/*`. **Warning: this version requires to change one line in all your controller and plugin Makefiles:**.


## Webots 6.2.3
Released on May 18th, 2010 (revision 4819).

  - Added three simple examples of four-wheeled vehicles: Ackerman steering, omni-wheels robot, etc.
  - Changed the default value of the `resolution` field of GPS nodes: it is now 0 instead of 0.001. This may remove a source of noise from some simulations
  - Fixed memory leak and optimized `wb_supervisor_node_get_field()` and `wb_supervisor_node_get_from_def()` in case of recurrent calls concerning the same field/node (thanks to Thierry)
  - Updated Robotstadium according to RoboCup SPL Rule Book 2010:

  - Fixed: Webots crashed when a Cylinder has no vertices: bottom FALSE, side FALSE, top FALSE
  - Fixed important memory leak when using `wb_supervisor_simulation_revert()` in Fast mode (thanks to Joel)
  - Fixed Webots crashes when using `wb_supervisor_field_get_sf_node()` on a field that contains a NULL node (thanks to Thierry)
  - Wrong value returned by `wb_supervisor_node_get_type()` and `wb_supervisor_node_get_name()` for some nodes (thanks to Thierry)
  - Fixed: `wb_servo_get_motor_force_feedback()` that was wrongly returning motor torque computed at body's CoM instead of joint axis
  - Added source code of `naoqi_for_webots` (formerly called `nao_in_webots`) controller used for connecting NaoQi clients to simulated Nao robot in Webots
  - In .protos files: the url field of ImageTexture nodes is now specified with respect to the .proto file's location instead of the .wbt file's location as it was before: **warning: this requires to move the textures files referenced in .proto files from the `worlds` to the `protos` directory.**

## Webots 6.2.2
Released on March 18th, 2010 (revision 4691).

  - Limited the number of lines in the console window on Mac OS X to avoid performances issues
  - URBI 1.5 for Webots was removed from this version
  - Fixed: problem for starting Matlab controllers on Mac OS X systems that defaults to a 64-bit architecture (added `-maci` option to matlab's command line)
  - Fixed: menu Wizard -> New Project Directory that failed because of missing Makefile
  - Extended from 255 to 65535 the maximal number of devices per robot (the definition of WbDeviceTag has changed)
  - Fixed the Lidar device, bound the resulted values if they are bigger than the max range (thanks to Nikolaus)
  - Fixed problem: a Connector fails to attach while locking `unilaterally` unless the peer Connector is also locked (thanks to Kostas)
  - Added support for the floor sensors of the e-puck in the crosscompilation mode
  - Improved the Display node:

  - Fixed startup problems with the linux 64 bits distributions having libc 2.10
  - Modified the type of the `pixelSize` field of both the Camera and Display nodes from SFInt32 to SFFloat
  - Improved the High Quality rendering: added another shaders especially for the grounds and for the boxes rendering
  - Improved the LED and Charger nodes: they can modify several materials and lights at the same time
  - Fixed problem with Linux Fedora 12 (and probably other recent Linux distributions) and controllers binary signature

## Webots 6.2.1
Released on January 21th, 2010 (revision 4551).

  - Fixed: crash on Windows when a texture doesn't exist (warnings improved)
  - Fixed various causes of crash when resizing the main window or subwindows to a very small size
  - Added `Restore Layout` menu
  - Fixed: bumper touch sensors were fired by distance sensor rays
  - Upgraded to Ogre 1.6.5
  - Added autonomous vehicle example
  - Added the SpotLight node
  - Fixed the Display device which wasn't displaying well on some hardware
  - Increased the limit on the polygones number of the IndexedFaceSet primitive to cast shadows
  - Fixed the Lidar device which returned wrong values arround the corners of the sub-cameras
  - Added a genetic algorithm example in the Advanced Programming Exercises of Cyberbotics' Robot Curriculum
  - Fixed: prototyped Servo nodes have .proto instead of .wbt translation/rotation values (thanks to Thierry)
  - Improved the conditions to display the center and the mass center of Solid nodes
  - Fixed the Pen device has been broken in Webots 6.2.beta3
  - Fixed crash on Windows with Microsoft Office IME 2007 Japanese keyboard driver
  - Mouse `zoom`, `pan` and `lift` motions are now scaled with WorldInfo.lineScale which is more convenient for large worlds
  - Fixed: problem that prevented to drag `enki` simulated objects with the mouse (thanks to Chrisantha)

## Webots 6.2.0
Released on December 22th, 2009 (revision 4468).

  - Fixed crash occurring when deleting a TextureCoordinate point from the scene tree
  - Fixed rare dependency problem with glibc-2.9 version on Linux 64
  - Fixed crash occurring while contact points are displayed during a physics simulation explosion

## Webots 6.2.beta3
Release on December 17th, 2009 (revision 4453).

  - Fixed MPEG-4 movie creation on Mac OS X 10.6
  - Fixed crash caused by physics simulation explosion
  - Fixed `wb_camera_set_fov()` function and added a new CameraZoom node
  - Adjusted the weight and radius of ball in robotstadium worlds to match the official RoboCup 2010 specifications
  - Fixed crash of `Make Animation`
  - Fixed annoying persistence of the 3d-view when Scene Tree, Text Editor or Console window is maximized (Mac)
  - Restored compatibility with the Mesa Project Software Rasterizer (Linux)
  - Added omnidirectional cameras based on a spherical projection over several subcameras rendering
  - Added an example of a Lidar device; a model of the Sick LMS291
  - Upgraded gcc compiler to version 4.4.0 under Windows

## Webots 6.2.beta2
Released on November 30th, 2009 (revision 4341).

  - Compiled with gcc-4.2 on Mac OS X
  - Added the dot (.) as scoping operator to dWebotsGetBodyFromDEF() and dWebotsGetGeomFromDEF() functions
  - Fixed: the Scene Tree `Transform` button that could produce unsupported node structures
  - Fixed several bugs in the new rendering engine:

  - Improved the e-puck model:


## Webots 6.2.beta1
Released on November 19th, 2009 (revision 4280).

  - Added compatibility with Windows 7: fixed printf() redirection in Webots console and freeze when using Build/Compile/Clean buttons
  - Removed unused API functions: `wb_supervisor_field_get_mf_rotation(),` `wb_supervisor_field_set_mf_rotation()`, `wb_supervisor_field_get_mf_bool()` and `wb_supervisor_field_set_mf_bool()`
  - Removed obsolete Gripper node (mainly used in old Khepera worlds)
  - Fixed texture path problem with controller rendered camera images
  - Removed the useless `display` field from the Camera node
  - Fixed crash and improved the behavior of the find/replace dialog with respect to multiple buffers
  - Migrated the Webots rendering system to OGRE 3D (Object-Oriented Graphics Rendering Engine):
    - Modified the rendering of the lights and of the materials in order to fit better the VRML97 specifications
      - Improved the lighting model; the ambient, diffuse and specular fields can behave differently
      - Improved the rendering of every worlds (mainly the worlds of the guided tour and of robotstadium)
      - Added support for texturing the Cone, the Cylinder and the Sphere primitives
      - Added a subdivision field to the Cone primitive
      - Improved the TextureTransform node (supported in every primitive able to have a texture)
    - Improved the rendering abilities:

    - Moved the far clipping plane to infinity
      - Removed the far field from the Viewpoint and the Camera nodes
      - Removed the `wb_camera_get_far()` function
    - Improved the range-finder cameras
      - Improved significantly the performances of the range-finder cameras
      - Added the maxRange field which allows to set up the maximum range of the range-finder
      - Added the `wb_camera_get_max_range()` function which allows to get the maxRange field
      - Replaced the `wb_range_image_get_value()` function (Deprecated) by the `wb_range_image_get_depth()` function which returns a linear scale (instead of logarithmic)
  - Added semantic validation of the node structure when loading/importing .wbt/.vrml files
  - Fixed problem with Webots random number generator: sensor noise (lookupTables) is now deterministic (until now only the physics noise was deterministic)
  - Fixed buffering of mouse move events for 3D navigation in the main window
  - Fixed Webots crash when project directories contain space characters (Mac+Linux)
  - Fixed compilation problems if Webots installation directory contains space characters (Mac+Linux)
  - Fixed: right after after having been inserted with the Scene Tree, USE nodes of geometrical primitives appear 10x smaller than their corresponding DEF nodes
  - Under Linux, fixed crash while trying to run a 64 bit controller with a 32 bit webots
  - Added `Open Recent` menu to find more easily recently opened .wbt files
  - Removed `author` and `constructor` fields from all Solid (and derived) nodes to simplify Scene Tree
  - Added a default values to every `name` field
  - Robotstadium: was completely relooked: lighting, shininess, textures, colors, etc. were changed (the physics was not modified)
  - Added new `force-3d` type of TouchSensor that computes and returns the 3d-vector of the force applied to the sensor
  - Fixed the handling of the CLASSPATH environment variable in order to match the Sun standard
  - Removed the black and white Camera type
  - Changed Webots code for sensor/actuator simulation to double-precision floating point numbers.
  - Replaced buggy --batch option with new --minimize option (all platforms)
  - Added command line options (--minimize, --mode, --version) to Windows and Mac platforms
  - Fixed wrong positions of `minStop` and `maxStop` of Servos (thanks to Jesse)
  - Made the `position` field of Servo nodes editable: this allows to specify initial positions/posture that differs from the zero positions/posture

## Webots 6.1.5
Released on June 17th, 2009 (revision 3425).

  - Fixed problems with Python 2.6 on Windows, Linux and Mac OS X
  - Restored Mac OS X 10.4 compatibility which was broken in previous version
  - Fixed Makefile.include: improved the management of the space characters for the CLASSPATH variable
  - Fixed ray-cylinder collision detection bug affecting distance sensors and cylinders (thanks to Yang)

## Webots 6.1.4
Released on June 5th, 2009 (revision 3388).

  - Fixed cross-compilation problem with the e-puck robot
  - Fixed crash on Mac OS X when both a robot window and a dialog box are open at the same time (thanks to Alina)
  - NaoV3R.proto: changed joint limits (minPosition/maxPosition fields) to Choregraphe values instead of incorrect `red` doc values
  - Robotstadium: resized soccer field from 6.8 x 4.4 m to 6 x 3 m and changed floor texture according to RoboCup SPL 2009 rules (thanks to Çetin)
  - Robotstadium: changed kick-off and penalty kick positions according to the latest RoboCup SPL 2009 rules
  - Robotstadium: added `nao_penalty.wbt` world in order to help training for penalties
  - Robotstadium: improved penalty kick rules in `nao_soccer_supervisor.c` in order to better match the SPL rules
  - Robotstadium: randomized the ball motion by moving the center of mass slightly off (0.5 mm)
  - Robotstadium: modified the ball physics: mass is now 26 grams and rolling friction and air resistance are simulated separately
  - Robotstadium: removed 5 minutes time limit for URBI controllers use in Robotstadium

## Webots 6.1.3
Released on May 25th, 2009 (revision 3352).

  - Improved the way to include multiple jar files; the CLASSPATH variable of the Makefiles supports now a list of files
  - Fixed: `wb_servo_set_force(0.0)` (or `Servo.setForce(0.0)`) mistakenly reenables the Servo P-controller (thanks to Shanaz)
  - Upgraded to Python 2.6
  - Matlab: fixed bad error message formatting and `Warning: Invalid escape sequence appears in format string.` (Windows only)
  - Restored URBI 1.5 compatibility
  - Robotstadium: Adapted URBI 1.5 Nao controller example for NaoV3R.proto
  - NaoV3R.proto: fixed small error in the mass center of the `Chest`: mass center moved 15.658mm backwards
  - NaoV3R.proto: slightly changed shape of foot tips: now there are two distinct (left/right) bumpers in each foot
  - Robotstadium: added complete C++ player example in `nao_soccer_player_blue`
  - Robotstadium: fixed error in `nao_soccer_supervisor.c` routine that identifies which robot touched the ball last (thanks to apryce)

## Webots 6.1.2
Released on April 16th, 2009 (revision 3202).

  - Added automatic dependencies management for physics and fast2d plug-ins
  - Fixed the standard output and error redirection for Matlab and Microsoft Visual Studio
  - Updated simplified and traditional Chinese translations
  - Follow object can now follow any Solid node (including children of other Solid nodes)
  - Fixed problem with
  - Upgraded mpeg2enc and jpeg2yuv to version 1.9.0 and fixed dependency on old libstc++.so.5 (Linux only)
  - Replaced `WB_SERVO_INFINITY` by `INFINITY` (see Servo documentation for details)
  - Set the camera image to const unsigned char * / const float *
  - Upgraded to swig 1.3.39
  - Upgraded to wxWidgets 2.8.10
  - Fixed missing keyboard events under Windows for arrow keys and other special keys
  - Fixed keyboard focus not shown by Webots text editor (colored frame missing)
  - Fixed Edit menu: Cut, Copy, Paste and Select All functions and corresponding Ctrl shortcuts now work between the text editor, the console and any other text widget (Linux/Mac)
  - Fixed Python`s and added MATLAB`s click-to-error behavior in Webots Console
  - Fixed problem with DistanceSensors when using the `Enki` plugin (Fast2D)
  - Added source code of the `Enki` plugin and added Linux 64 bit and Mac OS X Intel binaries to Webots distribution
  - Added `nao2_matlab.wbt` world that shows how to use the MATLAB language for programming the Nao and a soccer Supervisor
  - Added `nao_python.py` example of a Python controller for the Nao robot
  - Fixed problem that prevented MATLAB syntax errors to be displayed in Webots console
  - Added support for HTTP proxy in the Webots preferences (used for the 30 day trial license and dongle synchronization)

## Webots 6.1.1
Released on March 13th, 2009 (revision 2983).

  - Fixed: Webots freezes if a controller returns from main() and then the user hits `Revert` or `Quit` (all platforms)
  - Skip binary architecture test for scripts (for backwards compatibility with webotsnao-1.2.0-Linux package)
  - Added support for executing MATLAB controllers on the Linux 32 and 64 bit platforms
  - Fixed wrong implementation of shininess in Material
  - Warning concerning the C API:

## Webots 6.1.0
Released on February 28th, 2009 (revision 2904).

  - Added simulations used by the Cyberbotics's Robot Curriculum in the projects/samples/curriculum folder
  - Added a new Display device to simulate a screen on which the robot controller can draw any text or graphics
  - Fixed error messages sent to /var/log/messages under Linux with a USB dongle
  - Fixed resizing problem in 3D window when making a video with format `current resolution` (thanks to Jimmy)
  - Added support for writing controllers in the MATLAB language:
    - All the controller API functions (including the Supervisor functions) are supported
    - An example is provided in `webots/projects/packages/matlab`
    - This is still experimental and was only tested with Matlab 7.7 on Windows Vista and Mac OS X Leopard
  - Added new `webots_physics_step_end()` function to facilitate reading the values out of dJointFeedback structs
  - Added model of the NaoV3 `Robocup Edition` (NaoV3R.proto) which supports two cameras
  - Upgraded Robotstadium contest worlds and controller files to NaoV3R.proto
  - Improved Robotstadium supervisor controller

## Webots 6.0.1
Released on January 14th, 2009 (revision 2722).

  - Fixed abnormal quit at startup on Mac OS X when using a license server
  - Fixed bug with the security which was too restrictive for the DEMO and EDU Webots version
  - Improved the Webots Console limits
  - Fixed the blinking Webots Console under Windows
  - Compiled the Java controller library with Java 1.5 in order to support Java 1.5 and later
  - Fixed the display of the Java runtime exceptions
  - For Robotstadium: added possibility to move a robot or the ball to any 3d-position
  - Fixed bug with the display of the real Khepera configuration
  - Added the support of `Makefile.crosscompilation` filenames in order to build crosscompiler easier for the new robots
  - Fixed selection problem in Scene Tree: fields sometimes need to be clicked twice to be selected
  - Modified the Scene Tree behaviour such as to keep the current expand/collapse/selection state on `Revert`
  - Added a feature for comment/uncomment a selection in the Text Editor (Ctrl-M)
  - Fixed bug with the disabled build menu
  - Improved the error messages when Java or Python are not installed

## Webots 6.0.0
Released on December 15th, 2008 (revision 2632).

  - **Warning: URBI is disabled in this release, because of some compatibility issues.** It will shortly be available again. If you need URBI please continue using Webots 5.10.0.
  - Save windows layout when closing world file
  - Added a menu for registering a laptop computer (floating licenses only)
  - Fixed bug with loosing the camera-followed object after a revert
  - Removed URBI from the guided tour and Nao Robocup (as URBI doesn't work yet with Webots 6)
  - Changed behavior of `Save` (worlds) menu: saving after the simulation has run is now blocked
  - Fixed several glitches in Motion Editor
  - Synchronized Servo selection between 3D view and Motion Editor
  - Added `Clear Console`, `Edit Controller`, `Edit Physics Plugin` menus and reorganized some menus into a new `Robot` top menu
  - Added `New Physics Plugin` wizard for creating physics plugin more easily

## Webots 6.0.beta2
Released on November 21th, 2008 (revision 2421).

  - Fixed bug with new supervisor functions disabling `wb_robot_keyboard_get_key()` function
  - Fixed path separators used in .wbt file for specifying text files
  - Save backups of world files using numbered tilda-based suffixes
  - Prevent the opening of the same text file in two different tabs
  - Added the `wb_supervisor_field_import_mf_node` function which allows to import a node into an `MF_NODE` field from a supervisor
  - Fixed crash when the ComputerID doesn't match the ones in the license file
  - Fixed arrow key pressing not forwarded to controller program under Windows
  - Improved the `Welcome to Webots` window
  - Disable useless text editor buttons and menus when no text buffer is open
  - Check that binary type for controllers and plug-ins is correct (Linux 32/64 and Mac OS X Universal/ppc/i386)
  - Added a `Cancel` option to the dialog window popping up when a text file was not saved in the text editor
  - Fixed: Guided Tour sometimes gets hidden by main window and it freezes when another dialog opens (Mac)

## Webots 6.0.beta1
Released on November 14th, 2008 (revision 2318).

  - Important modifications C API:
    - The `wb_` prefix was added to every function names, e.g. `robot_get_device()` becomes `wb_robot_get_device()`, etc.
    - The `Wb` prefix was added to every type name, e.g. DeviceTag is renamed WbDeviceTag, NodeRef is renamed WbNodeRef, etc.
    - The `WB_` prefix was added to every constant, e.g. `SERVO_INFINITY` becomes `WB_SERVO_INFINITY`, etc.
    - All includes were changed, e.g. #include <device/robot.h> now becomes #include <webots/robot.h>
    - The functions `robot_live()`, `robot_run()` and `robot_die()` are deprecated, they are now replaced by `wb_robot_init()` and `wb_robot_cleanup()`
  - Important modifications of the JAVA API:
    - A new SWIG-generated and object-oriented Java API replaces the previous flat Java API in Controller.java
  - Important modifications of the C++ and Python API:
    - Modified the classes hierarchy
  - General modifications of the API for all languages:
    - The `robot_console_print()` method is deprecated, it can be replaced by the language specific print methods
    - All floating point parameters and return values were changed from type `float` to type `double`
    - Added a new collection of Supervisor functions for setting and getting any field value in the scene tree
    - Added a new collection of functions for playing motion files
    - Deprecated `custom_robot_move()`, `custom_robot_set_rel_force_and_torque()` and `custom_robot_set_abs_force_and_torque()` functions, the physics plugin should be used in replacement
    - The function emitter_send_packet() was renamed `wb_emitter_send()`
    - Deprecated `gps_get_matrix()` and `gps_euler()` functions, the `wb_supervisor_field_get_sf_rotation()` should be used as replacement
    - Definitely removed all previously deprecated functions: `servo_motor_off()`, `servo_get_feedback()`, `emitter_get_buffer()`, `receiver_get_buffer()`, `receiver_get_buffer_size()`
  - Fixed: View/Rendering/Bounding Objects menus now correctly switches to bounding objects rendering instead of wireframe
  - Fixed: bounding objects rendered at wrong location (but computed at correct location) when Physics.centerOfMass is non-null
  - Renamed the `CustomRobot` node into `Robot`: existing .wbt files will be automatically and silently updated when they are opened
  - Changes two rules for all `lookupTables` fields:
    - The second column of all `lookupTable` fields are now interpreted as double (instead of unsigned short int) values. Therefore all sensors using `lookupTable` fields (DistanceSensor, LightSensor, TouchSensor, etc.) do now also return doubles in the corresponding `...\_get_value()` functions
    - In addition, an emtpy lookupTable ([ ]), now returns non-interpolated sensor values
  - Rearranged the Webots windows into a single big window (using the wxWidgets AUI system)
  - Upgraded to wxWidgets 2.8.9
  - Added `stewart_platform.wbt` world that demonstrates the usage of linear Servos and physics plugin
  - Added dWebotsGetBodyFromDEF() function to the physics plugin API
  - Added `laser` type for DistanceSensor nodes (a red dot is painted at the point where the laser beam hits an obstacle)
  - Added new `Gyro` node for measuring the angular velocity about 3 orthogonal axes
  - Added new `Compass` node to simulate digital compasses
  - For Robotstadium contest:
    - Fixed left/right ultrasound sensor mismatch in Nao.proto
    - Changed field dimensions to 6.8 x 4.4 meters (RoboCup 2009)
    - Added a new and more didactic C controller example (`nao_soccer_player_red.c`)
    - Adapted Java player example (Cracoucass) to Webots6 new object-oriented Java API
    - Adapted `nao_soccer_supervisor.c` to Webots6 API
    - Added 2-axis Gyro to Nao.proto and reoriented Accelerometer's axes
  - e-puck:
    - Updated the e-puck firmware (improved the communication protocol)
    - Fixed the floor sensors (notably, in BotStudio)
    - Improved the BotStudio user interface
  - Added a MotionEditor that allows to interactively create motion sequences for articulated robots
  - Fixed a bug (on Windows) which crashed Webots when an e-puck was switched off during a remote-control session
  - Fixed batch mode under Linux
  - Fixed bug when trying to open in Webots under Linux a world saved from Webots under Windows
  - Fixed bug that prevented the transmission of Emitter/Receiver packets with small Emitter.baudRate values
  - The Camera node now defaults to Webots rendering (small magenta framed window inside the main 3D view)
  - The `wb_supervisor_import_node()` function now uses a path relative to the supervisor controller
