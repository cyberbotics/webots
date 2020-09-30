# Webots 5 Changelog


## Webots 5.10.0
Released on August 5th, 2008 (revision 1680).

  - Several improvements in BotStudio:
    - Added arrows indicating the direction of transitions
    - Improved the `COM port` listbox by adding a `Refresh COM port...` item (same improvement for the e-puck's robot window)
    - Fixed crahs with a transition with no condition
    - Improved the stop condition
    - Fixed a bug which occurred when a motor speed wasn't set
  - Added sound plugin source code to Webots distribution, and changed base sample type from `signed char` to `short`
  - For Robotstadium contest:
    - Robotstadium now has 3 robots per team instead of 4 (due to an official RoboCup rule change)
    - Matches have now two halves of 10 minutes each (change in `nao_soccer_supervisor.c`)
    - Added an automatic penalty kick shoot-out for tied games (change in `nao_soccer_supervisor.c`)
    - Fixed problem in all Robotstadium .wbt files that caused blue Nao robots not to receive RoboCupGameControlData
  - Improved the protocol between Webots and the license server: non-blocking in case of small network failures
  - Added the compatibility with the three e-puck's camera models
  - Open a dialog box offering a retry when the connection to the license server failed instead of quitting
  - Fixed bug with spaces in the path of the urbi.key file
  - Improved the text editor: Python's syntax highlighting, auto-completion is different according to the language
  - Added Python language support for Webots controllers
  - Added oriented-object C++ API for Webots controllers

## Webots 5.9.2
Released on June 19th, 2008 (revision 1532).

  - Store open text editor files in the world file and close them when closing the world
  - Fixed crash when starting the Nao Robocup contest from the startup window
  - Fixed error in collision detection when Solid node has Group boundingObject and no Physics node
  - Fixed systematical crash of --batch mode since version 5.9.0 (Linux)
  - Fixed another bug in `robot_get_time()` function for asynchronous controllers
  - Added `differential_wheels_set_encoders()` function for the e-puck in the remote-control mode (firmware updated)
  - For Robotstadium contest:
    - Added `kick-off shot cannot score a goal` rule enforced by simulation Supervisor
    - Added automatic `throw-in` by Supervisor when the ball leaves the field's limits (white lines)
    - Moved physical field limits (boundingObjects) 1 meter back
    - Changed menu item `Follow Robot` to `Follow Object`: now the view point can also follow passive objects like the ball

## Webots 5.9.1
Released on May 27th, 2008 (revision 1458).

  - Fixed bug with robot_get_time() and asynchronous controllers
  - Merged `nao*_urbi.wbt` with nao*.wbt, also `robotstadium_urbi.wbt` with robotstadium.wbt
  - Upgraded to URBI for Webots version 1.5.4 rev. 2e80a3e
  - Fixed problem with missing `-*-helvetica-medium-r-*-*-12-*-*-*-*-*-*-*` font under Linux
  - Fixed crash when adding CoordIndex values in a new IndexedFaceSet from the scene tree window
  - Fixed the cross-compilation of the e-puck controllers under Windows
  - Slightly changed the compilation system and menus of the text editor:
    - Added new `Compile` menu item to compile a single source file (instead of whole project)
    - Added `Compile`, `Clean`, `Cross-compile clean` and `Replace...` buttons in the tool bar of the text editor
    - Improved handling of projects with directories containing both C/C++ and Java source files
    - Improved: it's no longer needed to maintain a list of Java source files in the Makefile
    - Fixed: Webots does no longer build .d files (dependencies) for Java projects
    - Added new `Make JAR file` menu to create JAR files for submission to online contests
    - Fixed: missing .class files (for internal classes) in generated JAR files
  - Added prototypes of the complete Java controller API to Reference Manual
  - Added functions: `connector_disable_presence()`, `gripper_disable_resistivity()`, `gripper_disable_position()` and `servo_disable_motor_force_feedback()` for symmetry with existing `enable()` functions
  - Fixed assertion failure when `supervisor_field_get()` is called with parameter ms=0, for disabling a request
  - For Robotstadium contest:

  - Fixed crash when trying to open an unreadable world file

## Webots 5.9.0
Released on April 23rd, 2008 (revision 1360).

  - Fixed minor glitches in video recording caused by mouse-manipulations before the recording is started by pressing the `Run` button
  - Fixed: Webots freezes if a controller has returned from the main() function and then the user hits `Revert` or `Quit` (Windows only)
  - Added multi-threaded simulation engine for all `nao` and `robotstadium` worlds (important speedup on multi-core CPUs)
  - Added hard-coded drag force that slows down the ball's rolling motion in `nao` and `robotstadium` worlds
  - Fixed wrong hip structure, mass distribution, foot shape and eye LEDs of Nao to reflect the new model (Robocup edition)
  - Fixed bug with remote control of the motor wheels of the Khepera III
  - Fixed bug that sometimes caused Webots to delete dJointFeedback structures registered by the user in the physics plugin
  - Added possibility to select a Solid node within a robot with a mouse click on the 3d model
  - Changed viewpoint rotation center: it is now easier to take a close look at objects
  - Fixed problem that made the Guided Tour crash when quickly pushing the [Next] or [Previous] button
  - Added Java interface (in Controller.java) for 15 functions that were previously only accessible from C/C++
  - Fixed display of the BotStudio, e-puck and Khepera windows under Mac OS X
  - Fixed Rat's life moving camera
  - Fixed the bug causing that nodes could only be added at the top of a Group or Transform boundingObject
  - Fixed systematic crash when a geometrical node (Box, Cylinder, Sphere, etc) is added to a Group boundingObject and then mouse-clicked

## Webots 5.8.0
Released on March 7th, 2008 (revision 1231).

  - Fixed bug with libController.dll and libjpeg (Windows)
  - Updated to URBI 1.5.4
  - Fixed bug with firmware upload from BotStudio under Windows and Mac OS X
  - Deprecated `emitter_send()`, `emitter_get_buffer()` and `receiver_get_buffer()` functions
  - Fixed behavior of `Add a node` dialog: now it correctly adds USE references to the closest DEF node with the given name preceding it
  - Fixed bug in USE/DEF referencing in .wva files (e.g. causing wrong color in recorded Nao simulation)
  - Fixed several bugs that prevented the recording of .wva animations to work with worlds containing proto instances
  - Adapted `packet size` to allow the recording of .wva animations of more complex worlds
  - Modified the soccer field of the Nao Robocup simulation so that it better matches the real setup
  - Fixed a problem that sometimes caused invalid USE/DEF references after a sequence of copy/paste/delete of nodes
  - Fixed bug when IndexedFaceSet has a coordIndex with three values and no final -1
  - Fixed minor bug (sign of right encoder value) in e-puck firmware and released version 1.3.1
  - Fixed bug in the e-puck Bluetooth connexion
  - Fixed rare crash or hang under Linux x86-64 with multi-core processors
  - Fixed problems when `WEBOTS_HOME` indicates a directory different from where webots executable is invoked
  - Reworked `MPEG-1` and `MPEG-4` movie creation with quality parameter made available
  - Added `supervisor_start_movie()` and `supervisor_stop_movie()` functions
  - Linux and Mac OS X: made `MPEG-4` creation faster by using TGA images
  - Mac OS X: added `MPEG-4` movie capability
  - Added possibility to add prototypes (PROTO) with the Scene Tree (`Add a node` dialog)
  - Added Khepera III proto and several simple proto examples
  - Improved error reporting and general stability of VRML97 parser

## Webots 5.7.3
Released on January 29th, 2008 (revision 1038).

  - Fixed automatic CLASSPATH generation for Java controllers
  - Added initial dialog box to choose between guided tour and contest worlds
  - Fixed Nao model for Robocup setup
  - Fixed Makefile.include causing a syntax error under some Windows configurations

## Webots 5.7.2
Released on January 23rd, 2008 (revision 1011).

  - Added accelerometer in Rat's Life
  - Added linear camera device in Botstudio for e-puck
  - Added support for e-puck accelerometer
  - Added e-puck cross-compilation capability
  - Added `robot_get_basic_time_step()` function
  - Added `robot_get_synchronization()` function
  - Added `robot_get_project_path()` function
  - Fixed contest worlds sometimes opening with evaluation version message
  - Fixed e-puck / Botstudio crash on Windows
  - Fixed Hemisson / Botstudio file problem when saving a new graph
  - Fixed Nao Robocup simulations
  - Fixed URBI demos
  - Fixed Shrimp example

## Webots 5.7.1
Released on January 17th, 2008 (revision 972).

  - Fixed occasional crash when deleting a PointLight using the Scene Tree (Windows)
  - Fixed wrong signs and invalid coordinate system reference in values returned by Accelerometer nodes
  - Fixed missing `robot_get_mode()` method in Java under Windows causing ratslife.wbt to fail
  - Removed annoying error message `Can not enumerate files in directory ...` in Webots Evaluation Version (Windows)
  - Fixed bug with camera range-finder images rendered in the Webots main window
  - New Linux i386 and amd64 deb packages
  - Fixed crash in batch mode with some OpenGL drivers
  - Added possibility of viewing contact points (collision detection)

## Webots 5.7.0
Released on December 21th, 2007 (revision 893).

  - Added Aldebaran Robotics' Nao humanoid robot model with RoboCup field
  - Fixed batch mode crash because of the display of a license warning
  - Added a provision on the first law of Asimov in the license agreement
  - Added a model of the iCub robot (RobotCub Consortium)
  - Fixed bug with Webots camera rendering with high resolutions
  - Proofread the whole Reference Manual
  - Fixed `Paste` and `Paste after` buttons behavior (Scene Tree), in order to prevent the user from building unsupported node relationships
  - Fixed `camera_save_image()` for Java controllers
  - Fixed bug in the controller API with `camera_move_window()` and Webots rendered cameras
  - Fixed display of pixel RGB color display for cameras associated with Java controllers
  - Improved font scaling for supervisor labels
  - Added experimental Camera.antiAliasing and Camera.gaussianNoise fields
  - Compiled with wxWidgets-2.8.7
  - Set smooth rendering for lines
  - Fixed compilation of controllers and physics library under Windows Vista
  - Added snap effect on mouse motion for assembling Connectors precisely
  - Added possibility to rotate the selected object about any of its x, y, or z-axis using Shift key and right mouse button
  - Fixed optimization bug with Webots rendered camera images
  - Fixed bug with real e-puck camera feedback and Java controller
  - Fixed auto completion in text editor for `robot_console_print()` in Java
  - Added missing `robot_get_mode()` method to the Java API
  - Fixed GUI bug with camera windows initially touching the edge of the main view
  - Fixed camera window pixel color display for Java controlled robot camera
  - Added a gradual field for the Charger and the LED nodes
  - Added missing DLLs for URBI
  - Fixed typo in `accelerometer_get_values()` of the Java controller API
  - Added new `View` menu and `Orthographic` projection mode
  - Added `Bounding Objects` view mode
  - Adapted `Makefile.include` files to allow controller (and plugin) compilation on Mac OS X 10.5 (Leopard)
  - Fixed MPEG movie creation for Mac OS X 10.5 (Leopard)

## Webots 5.6.0
Released on November 12th, 2007 (revision 718).

  - Added an Accelerometer node
  - Added MPEG-4 movie encoding for Linux (requires mencoder)
  - Added prototype of sound simulation
  - Improved the quality of the displayed text labels
  - Display text labels on the top of camera windows
  - Fixed crash on Linux 64 with XEON dual dualcore
  - Fixed the refresh of the 3D display when the main window is maximized under Linux
  - Fixed two possible causes of crash when `supervisor_simulation_revert()` or `supervisor_simulation_quit()` are used in --batch mode
  - Fixed bug with filename while exporting a Webots animation
  - Added visible rotation/translation axes for Servo nodes
  - Added `Display device axes` and `Axis size` widgets in Preferences->Rendering
  - Fixed unwanted passive hinge behavior between two physics-enabled Solid (or derived) nodes when one is the parent of the other
  - Fixed possible crash with Webots rendering of cameras defined in a proto
  - Fixed bug with BotStudio firmware upgrade under Windows

## Webots 5.5.3
Released on October 11th, 2007 (revision 583).

  - Compiled with wxWidgets-2.8.6
  - Added Rat's Life contest files and allowed compilation for these instead of roboka

## Webots 5.5.2
Released on September 27th, 2007 (revision 557).

  - Updated the version of the URBI package to 1.04 and added Aibo ball tracking demo
  - Mac OS X version doesn't crash any more after license warnings
  - Updated the version of the URBI package to 1.04
  - Fixed bug with built-in camera rendering and loading a new world
  - Added a Simplified Chinese translation of Webots
  - Fixed display of license in the about box in case of a webots.key license file
  - Added Bioloids Robot Dog demo (thanks to Jean-Christophe and Tribotix)
  - Fixed missing double buffer rendering with some Linux nVidia driver
  - Fixed poor OpenGL rendering performance of the selected object

## Webots 5.5.1
Released on September 20th, 2007 (revision 530).

  - Fixed slow controller compilation in Webots built-in editor (Windows)
  - Fixed distance sensor names in e-puck.wbt
  - Removed obsolete Joint nodes from urbi sample worlds

## Webots 5.5.0
Released on September 14th, 2007 (revision 518).

  - **Warning about the Joint node and the `joint` field.** In this version, the Joint node and the `joint` fields were completely removed in order to simplify the design of new robot models. The usage of the Joint node was found to be confusing, because it was often unclear if a Joint node was required or not. This version of Webots makes the appropriate decision automatically. In addition, all the instances of Joint nodes were removed from all the .wtb example in Webots distribution. When this version of Webots loads a .wbt file that contain Joint nodes it issues a error message similar to this:
  - **Change in the Shrimp model.** The `shrimp.wbt` model was slightly modified as a consequence of the removal of the Joint node. In the previous `shrimp.wbt` model, an empty `joint` field in a Solid node used to indicate that this joint was going to be created in the physics plugin. Now the same thing is indicated by a Servo node which `type` field constains the string `none`.
  - Fixed sometimes not working `jump-to-error` behavior of compilation console (Mac OS X)
  - Fixed wrong collision detection when boundingObject had a `Group->Shape` structure and Physics.centerOfMass/rotation had non-default values
  - Fixed sometimes incomplete list of gcc error messages displayed in text editor (Windows)
  - Fixed problem when updating a lserv3 dongle from Windows
  - Added color-coded axis system for Connectors
  - Added rupture simulation for Connectors
  - Added `snap` mechanism to precisely adjust Connectors when they become `locked`
  - Fixed crash when deleting spine or crossSection items of an Extrusion
  - Fixed crash when saving a world for which a robot followed by the camera was deleted
  - Webots rendered cameras now work properly with Java controllers
  - Fixed crash when Color item of Background.skyColor is deleted
  - Fixed problem when `WEBOTS_HOME` points to a non-existent directory
  - Fixed bug with the setting of the java command in the preferences
  - Added new `Add a node` dialog: now showing node descriptions and icons
  - Fixed crash within IndexedLineSet when values are added to `coordIndex` field while `coord` field is NULL
  - Fixed copy/paste of PROTO instances in the Scene Tree
  - Can now upload a new e-puck firmware even if no previous firmware is present on the robot
  - Fixed bug preventing a robot controller to read the real image from the e-puck camera in remote control mode
  - Proofread whole User Guide (thanks to Nathan)
  - Fixed dongle bug under Mac OS X with PowerPC processor
  - Added a bounding object to the floor in new.wbt
  - Fixed crash when copying/pasting an object between two different worlds
  - Fixed crash when creating MPEG movies on Mac OS X
  - Removed obsolete Joint node and `joint` fields
  - Fixed crash under Windows when deleting the items of a list field in the scene tree window (thanks to Tony)

## Webots 5.4.0
Released on July 18th, 2007 (revision 369).

  - **Warning about WorldInfo.basicTimeStep.** In this version, Webots checks that the controller step (the value passed to `robot_step()` or returned by the controller's run() function) is a multiple of the basicTimeStep. Simulations that do not meet this requirement will issue warning messages.
  - **Warning about `servo_set_force()` function.** In this version, the existing `servo_set_force()` function was renamed `servo_set_motor_force()`. There is now a new `servo_set_force()` function with a different purpose, please refer to the Reference Manual for more info. We recommend to replace any previous call to `servo_set_force()` by a call to `servo_set_motor_force()` in your controller code.
  - Webots can now run on the same machine as the lserv3
  - Added Robot.selfCollision field to enable intra-robot collision detection
  - Fixed behavior of `supervisor_set_label()` which was taking into account only the first call for the same id, now takes into account only the last call (thanks to Benoît)
  - Fixed defective use of the `controller` field as PROTO field
  - Propose to adapt translation and rotation when pasting a node at the parent's level in the scene tree window to preserve absolute position and orientation
  - Fixed crash when a non-existing world file name is specified on the command line (--batch mode only)
  - Fixed problem with EPFL license server setting
  - Added new functions for motor torque/force feedback: `servo_enable_motor_force_feedback()` and `servo_get_motor_force_feedback()`
  - Added new `servo_set_force()` function in order to allow simplified force control of servos
  - Deprecated `servo_motor_off()` function (use servo_set_motor_force(servo, 0) instead)
  - Removed obsolete Servo fields: `customForce`, `forceAndTorque` and `animation`
  - Removed obsolete Servo functions: `servo_set_abs_force_and_torque()`, `servo_set_rel_force_and_torque()`, `servo_run_animation()`, `servo_get_animation_number()` and `servo_get_animation_range()`
  - Fixed Makefile.openr in the ers-210 controller sample, so that it works with the new webots-openr package (with support for the Aibo camera)
  - Uses the MAC address of the Ethernet card on Mac OS X for the ComputerID
  - Fixed rounding problem in scheduler which sometimes caused `physics_step()` to be called too often (and eventually causes a drift in `robot_get_time()` with respect to the simulation time)
  - Added e-puck program uploader in the Tools menu

## Webots 5.3.1
Released on June 22nd, 2007 for Mac OS X only (revision 283).

  - Fixed hang of demo version on Mac OS X
  - Fixed bug in e-puck firmware and released version 1.1.7
  - Added msgfmt under Windows and Mac for locale translation compilation

## Webots 5.3.0
Released on June 21st, 2007 (revision 268).

  - Display a star (`*`) in the title bar of the main window when the world was modified
  - Fixed bug preventing the display of the Khepera window and displaying UTF-8 warning messages in the console
  - Fixed crash when `controllerArgs` exceeds 5 arguments (Java)
  - Fixed crash on exit on Mac OS X
  - Fixed general floating point bug on foreign locales
  - Added support for dongle-based license
  - Added Polish locale (thanks to Michal)
  - Added support for real Aibo camera (thanks to Mjof)

## Webots 5.2.0
Released on June 6th, 2007 (revision 170).

  - Added support for locales (see resources/locale/readme.txt to add your own locale)
  - Added French locale
  - Upgraded to wxWidgets 2.8.4
  - Enabled `supervisor_field_set()` to change the position and orientation of the Viewpoint node
  - Fixed camera flipped image when using Camera.windowPosition field
  - Fixed selection unstability while the simulation is running
  - Fixed incorrect DEF name when PROTO nodes are DEF'd
  - Fixed possible crash when LightSensors are copy/pasted or used inside PROTO nodes
  - Fixed problem with labels displayed by `soccer_supervisor` under Linux

## Webots 5.1.14
Released on May 21st, 2007 (revision 99).

  - **Warning about LightSensors** The lookupTable of LightSensor sometimes returned incorrect values in previous versions of Webots. This problem was fixed with this new release. As a consequence the `light_sensor_get_value()` function might in some situation return values different than before, therefore simulations tuned for specific return values will need to be slightly adapted to work with this version.
  - Fixed problem for loading physics plugins on Mac OS X
  - Reworked LightSensor model (see Reference Manual for more info):


## Webots 5.1.13
Released on May 9th, 2007 (revision 69).

  - **Warning about TouchSensors** If TouchSensors were used, existing robot models developed for previous releases of Webots may have to be slightly modified in order to work with this new version of Webots. The new `force` TouchSensors are unidirectional while the old ones were omnidirectional. The new `force` sensors measure the force along their z-axis, while the other axes are ignored. The modification is very simple: the TouchSensors must be reoriented such that their positive z-axes point from the body outwards, in the direction where the force need to be measured (where the collision is expected to take place). Please also note that the values returned by `bumper` sensors may also differ from those of previous Webots version because the new version returns fixed 0 or 1 while the previous version used to return the min or max values of the lookup table.
  - Added Camera.windowPosition and Camera.pixelSize fields for camera rendering in the main Webots window
  - Fixed controller compilation under Windows Vista
  - Fixed Webots document icon under Windows
  - Fixed some communication bugs with the e-puck robot
  - Fixed crash when trying to set an IndexedFaceSet plane bounding object inside a Transform node
  - Fixed bug when using several cameras of different resolution on the same controller
  - Fixed frozen sliders bug in BotStudio graphical programming interface
  - Added KHR2-HV humanoid robot simulation from Laurent Lessieux
  - Fixed and improved model of TouchSensors
  - Fixed `Wizard / New Project Directory...` menu failing to create directory on Linux
  - Fixed VRML97 export which was exporting too many useless fields
  - Fixed robot camera width and height inversion on Mac OS X
  - Fixed display bug when using backslash f in `robot_console_printf()` to clear the console
  - Fixed instability of `supervisor_robot_set_controller()` function

## Webots 5.1.12
Released on February 8th, 2007 (revision 1502).

  - Added cross-compiler for the e-puck robot
  - Fixed crash on File/Open or File/Revert after changing a robot controller
  - Fixed default directory on File/Open under Linux
  - Added spherical camera sample code in projects/samples/howto/
  - Now skip properly extra spaces in the the controllerArgs list
  - Fixed discrepancy between the value returned by the `robot_get_time()` function and the actual time displayed in the main window
  - Fixed creation of Cone objects with the New Node button of the Scene Tree
  - Display a warning message when attempting to use a new supervisor process with Webots EDU or STD
  - Fixed bug with key codes in `robot_keyboard_get_key()` (now use constants)
  - Check MAC address for eth0, eth1, eth2 under Linux
  - Fixed bug with differential wheels encoders and physics simulation
  - Fixed crash when a coordIndex is smaller than -1 (IndexedFaceSet)
  - Fixed instability of Emitter/Receiver on some Linux platforms
  - Fixed a rare rounding problem with some small time steps (below 1 ms)
  - Fixed bad default directory for `File/Open...` and `File/Save As...` menus
  - Added DifferentialWheels.maxForce field
  - Fixed sporadic crash when calling `robot_console_printf()`
  - Fixed bug with light sensors and non white light sources
  - Fixed hang on Mac OS X / Intel while compiling a controller
  - Fixed crash when `supervisor_field_set()` is applied to a not found node

## Webots 5.1.11
Released on December 22st, 2006 (revision 1429).

  - Made EDU and STD licenses platform independent
  - Added BotStudio graphical programming for the e-puck robot with remote control capability
  - Fixed crash after compilation from the IDE on Mac OS X
  - Upgraded to wxWidgets 2.8.0
  - Fixed floating rounding problem causing erratic physics simulation when the time step was smaller than 1 (thanks to Lorenzo)
  - Fixed performance problem when moving editor's slider on Mac OS X
  - Fixed crash with illegal instruction on Athlon processors under Linux
  - Fixed very slow camera windows rendering on Mac OS X
  - Added Surveyor SVR-1 robot model and programming interface
  - Fixed movie creation on Mac OS X Pentium (new universal binary for mpeg2enc)
  - Fixed viewpoint instabitily when Viewpoint.rotation is 1 0 0 6.28
  - System.out and System.err in Java controllers are now redirected to the Webots log window to facilitate debug

## Webots 5.1.10
Released on November 1st, 2006 (revision 1297).

  - Added `target_coordinate` supervisor for the IPR robot to display the coordinates of a target point (red ball in `ipr_cube.wbt`)
  - Fixed non-working jump-to-error functionality in compilation console (Mac OS X)
  - Reworked menus to make them more consistent on multiple platforms, especially Mac OS X
  - Fixed bug in Bluetooth binary communication with the e-puck under Linux
  - Made the `ipr_serial` controller synchronous to avoid hangs due to the scheduler of the operating system
  - Fixed dependency of libController on glib-2.6 under Linux, now works with glib-2.4 as well
  - `camera_save_image()` now saves a black and white image if the camera is black and white
  - Optimized movie creation on Mac OS X / Intel (compiled mjpegtools as Universal Binary)
  - Changed Emitter/Receiver internal code to packet-based communication and added new API functions: `emitter_send_packet()`, `emitter_set_range()`, `emitter_get_range()`, `receiver_get_queue_length()`, `receiver_next_packet()` and `receiver_get_data_size()`
  - Fixed memory leak on Revert (button) and `supervisor_simulation_revert()` function
  - Added Cylinder.subdivision field to tune the tessellation of Cylinder
  - Added a guided tour in the Help menu
  - Fixed simulation hang occurring on Linux under special circumstances

## Webots 5.1.9
Released on September 14th, 2006 (revision 1187).

  - Fixed inconsistency in the GUI when using the --mode command line option
  - Changes of controller in the scene tree window take effect immediately
  - Fixed `supervisor_import_node()`
  - Fixed `supervisor_set_controller()` under Linux and Mac OS X
  - Added mjpegtools in the Linux version of Webots
  - Fixed hang in the termination of controllers under Linux and Mac OS X
  - Fixed bug with non-convex extrusions corrupting OpenGL tesselator state
  - Many improvements and bug fixes in the provided examples
  - Fixed crash with `supervisor_simulation_physics_reset()`
  - Fixed bug with Physics.centerOfMass and boundingObject
  - Fixed reset of robots' position and orientation in soccer.wbt demo
  - Don't save a crashing world file as the default file in the preferences (Windows)
  - Fixed crash when clicking in the main window during the AVI dialog (Windows only)
  - Fixed e-puck window behavior when e-puck robot has no camera
  - Fixed search path for textures in libController
  - Fixed missing support for creaseAngle in Extrusion node in libController
  - Fixed 2 bugs in the creation and playback of .wva files (File/Make Animation)
  - Added new API functions: `receiver_get_signal_strength()`, `receiver_get_emitter_direction()`, `receiver_get_channel()` and `receiver_set_channel()`
  - Added obstacle detection and aperture fields for `infra-red` Emitter/Receiver
  - Added support for compilation of physics plugin from the Webots IDE
  - Reorganized the user guide
  - Improved the wizards
  - Rearranged sample files (worlds, controllers, physics plugins, etc.) in the projects folder
  - Linux MAC address reading not tied anymore to `eth0`
  - Set default values for Emitter.baudrate, Receiver.baudRate and Emitter.range to -1
  - `robot_console_printf()` can now clear the console with the `\f` character
  - Added confirmation dialog before saving a world
  - Remove `*.exe_webots`, `*.dll_webots`, `*.o`, `*.d` and `webots.log` after uninstallation on Windows
  - Added display of light sensor range
  - Fixed and improved `robot_keyboard_get_key()`, now takes into account shift, control and alt modifiers
  - Added auto-completion feature in the text editor
  - Syntax highlight is now language (C/C++/Java) dependent in the text editor
  - Added Shrimp demo (thanks to Simon)
  - Fixed bug with two-color ElevationGrid and robot camera
  - Export node in the scene tree window now automatically adds the .wbt extension if missing
  - Fixed `supervisor_export_image()`: now supports .jpeg extension and save in the supervisor controller directory
  - Allow to select and edit physics libraries from the scene tree window
  - Added `webots_physics_predraw()` function in the physics library API
  - Added e-puck chapter in the user guide
  - Added yamor.wbt world demonstrating the Connector node
  - Fixed bug with cameras update introduced in 5.1.8
  - Infra-red distance sensors now use red level of bounding objects rather than actual object
  - Nodes are now sorted alphabetically in the scene tree dialog for creating new nodes

## Webots 5.1.8
Released on July, 19, 2006 (revision 691).

  - Added new Connector nodes for modular robotics (thanks to Yvan)
  - Added a custom Java command in the preferences, allowing to pass extra Java options to the JVM or to choose to use java.exe instead of javaw.exe
  - Added `camera_move_window()` function
  - Fixed the camera viewpoint jump when selecting Robot View menu item
  - Now ElevationGrids with only two colors draw a chess board with these two colors (thanks to Simon)
  - Fixed problem with ElevationGrid not appearing in the scene tree window on creation of a new node (thanks to Simon)
  - The dialog offering to create a user directory now displays only once after a new installation of Webots
  - User directory is set automatically upon opening a world (thanks to Yvan)
  - Fixed bug with `robot_console_printf` (thanks to Yvan)
  - Fixed display of Box node when appearance is NULL (thanks to Simon)
  - Added support for the IPR (Katana) robot arm from Neuronics (thanks to Simon)
  - Bluetooth remote control for the e-puck robot on Mac OS X
  - Automatically finds serial ports under Mac OS X
  - Native support for Intel-based Macs (universal binary built)
  - Added camera support for the e-puck robot
  - Fixed bug on Windows when several cameras are set to display FALSE (thanks to Richard)
  - Fixed Visual Studio project for sample physics library (thanks to Sung-Hee)
  - Fixed Aibo remote control and Aibo chapter in user guide (thanks to Mjof)
  - Fixed viewpoint jumping to nan position after mouse move when Viewpoint.orientation angle was 0 (thanks to Simon)
  - Added `camera_get_near()` and `camera_get_far()` to the Java API (thanks to Richard)

## Webots 5.1.7
Released on May, 24th, 2006 (revision 506).

  - Add a startup option to choose the startup mode (stop, run or fast)
  - Fixed bug with keyboard reading and modifier keys
  - Added new roboka worlds and a supervisor allowing to set initial position
  - Moving objects while physics simulation is running doesn't break joints any more (thanks to Yvan)
  - Don't save position of iconified windows in world files
  - Uninstall removes the Webots registry entry under Windows
  - Fixed possible hang when displaying Servo warning messages (thanks to Simon)
  - Fixed cross-compilation problem with `khepera_gripper.c` (thanks to Sylvain)
  - Fixed another bug with gripper device (thanks to Fernando)
  - Improved the `flying_robot.wbt` and sample physics library (thanks to Harun)
  - Added `dWebotsGetTime()` in the physics library API (thanks to Jonas)
  - Fixed a couple of bugs in the physics library management (thanks to Harun)
  - Fixed revert in case of several instances of Webots running in parallel (thanks to Sergei)
  - Added `robot_get_time()` function in the Controller API
  - Fixed buffer overflow in Webots / controllers communication (thanks to Dennis)
  - Fixed bug with physics shared library under Windows

## Webots 5.1.6
Released on May, 10th, 2006 (revision 405).

  - Fixed Mac OS X problem with physics shared libraries (thanks to Antoine)
  - Fixed bug with Extrusion and creaseAngle affecting koala.wbt
  - Off PointLight are now displayed in black
  - Added reinitialization of the Webots random number generator and the ODE joints velocity and FMax in the `supervisor_simulation_physics_reset()` function
  - Fixed rare memory leak with IndexedLineSet

## Webots 5.1.5
Released on May, 9th, 2006 (revision 388).

  - Fixed initialisation of random seed, so that Webots simulations are now deterministic (thanks to Yvan)
  - Fixed koala.wbt which doesn't use any more non-convex Extrusion causing problems in some cases (thanks to Ankanadh)
  - Attempted to fix a bug with the gripper device (thanks to Fernando and Ricardo)
  - Fixed roboka.wbt allowing recompilation of Java controllers (thanks to Dominic)
  - Added dWebotsConsolePrintf() in the physics library API (thanks to Harun)
  - Automatic mouse control change when gravity is along the z-axis (thanks to David)
  - Fixed bug with light sensors and batch mode (thanks to Nikolaus and Yvan)
  - Updated the `blimp_lis` model and physics, now includes keyboard control (thanks to Antoine)
  - Fixed problem with cylinder getting down in non-physics gripper simulations (thanks to Ricardo)
  - Replaced `khepera_line` example with e-`puck_line` (thanks to Jean-Christophe)
  - Fixed broken K213 feedback (thanks to Sylvain)
  - Fixed rare crash when a DEF name was not found (thanks to Simon)
  - Fixed bug with DistanceSensor inside a non-moving robot or supervisor
  - Upgraded to Java 1.5.0 on all platforms
  - Fixed shadow bug for Extrusion showing up in `khepera_gripper.wbt`
  - Fixed occasional hang at startup under Linux (thanks to Paul)
  - Fixed crash when setting a texture on the ground (thanks to Simon)
  - Fixed rare bug under Windows with e-puck and light sensors (thanks to Jean-Christophe)
  - Fixed Makefile.include so that all class files are compressed into the jar file (thanks to Michał)
  - Linked libController with glib-1.2 to avoid undef symbols (thanks to Fabrizio)
  - Fixed libpng version problem (thanks to Heng and Paul)
  - Fixed bug with chargers and robots/supervisor without battery (thanks to Danny)

## Webots 5.1.4
Released on March, 28th, 2006 (revision 248).

  - **Warning:** This release is not fully upward compatible with Webots 5.1.2. The change is very minor though: if your world files have Servo nodes with an acceleration field different from -1, you should multiply it by 10^6 or set it to -1 (for infinity).
  - Built with wxWidgets 2.6.3
  - Display white wireframe only for bounding objects of Solid nodes
  - Fixed all Visual Studio project files (tested with Visual C++ 6.0)
  - Added Visual C++ sample project for pendulum.c sample controller
  - Fixed compilation problem with `SERVO_INFINITY` under Visual Studio
  - Fixed wrong webots document icon under Windows
  - Fixed warnings with roboka.wbt world
  - Added Roboka1 empty controller
  - Fixed bug with hoap2 worlds under Windows (thanks to Yvan)
  - Attempted to fix setgid flags on directories under Linux (thanks to Alessandro)
  - Replaced references to cyberboticspc1.epfl.ch by references to www.cyberbotics.com

## Webots 5.1.3
Released on March 21st, 2006 (revision 205).

  - Added controllerArgs field to robot nodes to pass arguments to a controller program
  - Fixed bug with `supervisor_field_set()` for a rotation angle in fast2D mode (thanks to Patrícia)
  - Added Aibo ERS7M3 target pattern for recharge station (thanks to Austin)
  - Fixed bug with Servo.acceleration which was expressed in rad/(micro s)² instead of rad/s² (thanks to Gerhard)
  - Changed the way C/C++ controllers are executed under Windows (`simple.exe` is renamed to `simple.exe_webots` before it is executed)
  - Fixed Roboka0.java so that it now works on Mac OS X as well
  - Added `supervisor_export_image()` and `supervisor_import_node()` in the Java API
  - Fixed bug causing crash of `aibo_ers7.wbt` under Mac OS X after 20 seconds of simulation
  - Fixed bug causing crash in with Charger node (thanks to Joakim)
  - Improved text editor with multiple buffers, printing, find/replace, etc. (thanks to Yvan)
  - New `gps_euler()` function contributed by Hannes Winlekmann

## Webots 5.1.2
Released on March 7th, 2006 (revision 138).

  - Fixed collision detection bug with cylinder object in `khepera_gripper.wbt` (thanks to Ricardo)
  - Added `SUPERVISOR_FIELD_` constants in the Java API (thanks to Harm)
  - Fixed Mac OS X version failing at launch
  - Fixed Memory leak causing physics simulation to behave strangely

## Webots 5.1.1
Released on March 2nd, 2006 (revision 117).

  - Attempted to improve the highest quality of MPEG movies generated under Linux and Mac OS X
  - Fixed LED update in robot camera views
  - Fixed bug with `robot_console_printf()` under Windows (thanks to Stevie)
  - Fixed bug with Mac OS X and multiple license webots.key file (thanks to Fernando)
  - Fixed bug with SFString parsing/saving and backslash characters (thanks to Matthew)
  - Fixed problems with accented chars in text editor under some Linux distributions (thanks to Yvan)
  - Linux movie script can now use other terminal applications than xterm
  - Added a new demo with linear servos: gantry.wbt (thanks to Yvan)
  - Fixed bug with shadows affecting the e-puck.wbt world
  - Improved the `aibo_ers7.wbt` model (thanks to Ricardo)
  - Implemented remote control support for the e-puck (via bluetooth)
  - Improved the e-puck model and robot window
  - Save robot windows position, size and visibility in .wbt files
  - Fixed bug with USE/DEF and DistanceSensor (thanks to Jim and Yvan)
  - Added a Preferences option to display the camera frustums
  - Fixed viewpoint screw up under Windows after Open or Import VRML97
  - Fixed plugins/physics/physics.c so that it works with C++ as well (thanks to Antoine)

## Webots 5.1.0
Released on December 23th, 2005.

  - **Warning:** This release is not fully upward compatible with Webots 5.0.10. The change is very minor though: if your world files have Servo nodes with a rotation angle different from 0, you should set it to 0 and add a Servo.position field which contains the value of this angle. Also, the physics and fast2d directories moved into a plugins directory.
  - Moved physics and fast2d directories into a plugins directory
  - Display camera frustum when a camera is selected (thanks to Sven)
  - No longer rewrite default preferences after installing a new version
  - Selecting of a Solid node in the scene tree window highlights this Solid node in the main window (thanks to Jimmy)
  - Added linear servos and improved rotational servos (thanks to Yvan)
  - Fixed memory leak in the log window (thanks to Yvan)
  - Fixed warning message when failing to load a physics library (thanks to Jonas)
  - Added a `webots_physics_draw()` function example in sample.c (thanks to Jonas)
  - Added Makefile.openr in ers210 controller (thanks to Tony)
  - Removed broken and useless MPEG-2 encoding
  - Fixed the Webots document icons under Windows (thanks to Matthew)
  - Fixed rotation vector switch in physics simulations
  - Fixed crash with empty ImageTexture and robot camera (thanks to Matthew)
  - Added Enki fast2d plugin for Mac OS X (thanks to Simon)
  - Fixed DirectionalLight rendering in libController (thanks to Yvan and Paolo)

## Webots 5.0.10
Released on November 18th, 2005.

  - Fixed Help/Register menu not working under Mac OS X (thanks to Valérie)
  - Fixed bug with distance sensors in fast2d mode (thanks to Yvan)
  - Fixed bug with ghost robots (passing through each other)
  - Added a Visual C++ project for the sample.dll physics library
  - Display a warning when using an unsupported boundingObject in fast2d mode
  - Fixed bug introduced in Webots 5.0.9 disabling collision detection between robots (thanks to Simon)
  - Documented the `robot_get_name()` function (thanks to Simon)
  - Display a warning in log window when trying to run a newly compiled controller with evaluation version
  - Added e-puck robot in e-puck.wbt (thanks to Francesco, Michael and Jim)
  - Fixed screenshot save problem under Linux (thanks to Jimmy)
  - Added a hidden inkEvaporation feature (thanks to Simon)
  - Added fast2d.h include file (thanks to Simon)

## Webots 5.0.9
Released on November 8th, 2005.

  - Fixed `camera_save_image()` with JPEG format under Windows (thanks to Edgar)
  - Fixed hang under Windows after loading or reverting
  - Fixed MTN path in the Aibo control panel
  - Fixed bug with DistanceSensor and infra-red factor
  - Changed shape used by Pen device from square to disk (thanks to Simon)
  - Fixed bug with DEF/USE DistanceSensor (thanks to Jim and Yvan)
  - Disable insertion of a device node inside a Transform or Group node in the scene tree window
  - Display a warning when a Solid or a device node is found as a children of a Group or Transform node

## Webots 5.0.8
Released on October, 27th, 2005.

  - Created a multiplatform binary server for Aibos ERS7/ERS210 (thanks to Ricardo)
  - Aibo ERS7 can now play MTN file with cross-compilation (thanks to Ricardo)
  - Fixed bug with `pen_write()` and Pen.write (thanks to Simon)
  - Fixed bugs with servo feedback (thanks to Yvan)
  - Fixed bug causing crash with Charger node
  - Fixed `soccer_supervisor` to handle rotation axis inversion (thanks to Thomas)
  - Fixed mur.bsg BotStudio/Hemisson program (thanks to Samir)
  - Fixed the prototype for the `mtn_play` function which returns void
  - When no extension is provided to Save as, add the .wbt extension (thanks to Hugo)
  - Reworked the Aibo window
  - Fixed a couple of problems with DEF names in the scene tree window
  - Added some explanations in the user guide on how to connect a Webots controller with the Matlab C interface (thanks to Mark)
  - Fixed ground sensors in `botstudio_*.wbt` and `hemisson_cross_compilation.wbt`
  - Removed accented chars from botstudio graphs
  - Fixed sensor name in rover.wbt
  - Fixed the two following problems in the scene tree window (thanks to Yvan):
  - Display a warning on the console when a device name is not found (probably misspelled) by a controller program (thanks to Jonas)
  - Added installation instructions for URBI in the user guide
  - Fixed compilation issues from the built-in editor in Windows (thanks to Stephen)
  - Added fast2d/enki shared libs in the Linux and Windows versions (thanks to Jim)
  - Distributing URBI for Webots as an optional Linux tarball (thanks to Matthieu)
  - Fixed rare Windows crash due to multi-threading sync (thanks to Sven)
  - Fixed `robot_keyboard` support from the Java API (thanks to Matei)
  - Fixed reference to `botstudio_pen.wbt` in the reference manual (thanks to Simon)
  - Fixed bug with different time steps in controllers and world (thanks to Simon)
  - Fixed a few glitches for the make output in the text editor console
  - Made `botsutdio_*.wbt` world files run real time (thanks to Samir)

## Webots 5.0.7
Released on 30-September-05.

  - Fixed flickering in Khepera window under Windows
  - Added support for URBI script language (thanks to Anthony and Jean-Christophe)
  - Fixed bug with DEF names selection in the scene tree window (thanks to Yvan)
  - Allowed infinite coulombFriction in Physics (by setting it to -1)
  - Changed default value from 0 to 1 for coulombFriction in Physics (thanks to Yvan)
  - Fixed problem with demo supervisors and Webots STD (thanks to Malachy)
  - Display warning if a solid has no bounding object and no mass defined (thanks to Yvan)
  - Updated user guide on `robot_run` function usage
  - Improved documentation on encoders in reference manual (thanks to Jim)
  - Upgraded GUI to wxWidgets 2.6.2
  - Mac OS: fixed text strings bug in the scene tree window (thanks to Simon)
  - Fixed bug with supervisor tracking and fast2d mode (thanks to Nikolaus and Yvan)
  - Added inverted double pendulum example: pendulum.wbt
  - Added DV (PAL/NTSC) resolutions for movie creation
  - Fixed `robot_console_print` in Java API under Windows (thanks to Jose)
  - Don't open a DOS console any more for Java controllers
  - Fixed bug with camera update and creation of new objects
  - Fixed warning in libController for WorldInfo fields
  - Fixed collision bug with Supervisor or CustomRobot without physics
  - Fixed copy/paste of the Camera node in the scene tree window
  - Fixed controller change bug in the scene tree window (thanks to Lewis)
  - Fixed possible memory leak with textured IndexedFaceSet (thanks to Yvan)
  - Fixed robot.h include for for compilation with Visual C++ (thanks to Anna)
  - Display a warning if servo velocity and force exceeds limits (thanks to Auke)
  - New Fast2D chapter in user guide (thanks to Yvan)
  - Fixed a few memory leaks (thanks to Yvan)
  - Change to the local physics library directory before calling `webots_physics_init` (thanks to Fabrice)
  - Fixed the documentation of the `robot_run` function (thanks to Auke)
  - Fixed documentation on communication between controllers and physics shared library (thanks to Fabrice)
  - Fixed compilation problem with Visual C++ (thanks to Anna and Leonard)
  - Fixed wrong display of month in the about box (thanks to Anthony)

## Webots 5.0.6
Released on 14-July-05.

  - Fixed display of distance sensor rays (thanks to Yvan)
  - Fixed wrong text control behavior in the scene tree window (thanks to Yvan)
  - Set the PDF and HTML helpers as background processes under Linux (thanks to Yvan)
  - Added documentation for the TouchSensor node (thanks to José)
  - Fixed force sensor lookupTable in `hoap2_sumo.wbt` example world

## Webots 5.0.5
Released on 13-July-05.

  - Fixed bug with `robot_console_printf` (thanks to Hugo)
  - Fixed opening of text editor window under Windows
  - Fixed calibration of Khepera gripper for remote control and cross-compilation
  - Bug fixes with distance sensor node and multiple rays (thanks to Yvan)
  - Mac and Linux: show MPEG movies after creation
  - The TouchSensor now takes into account the lookupTable for force measurements
  - Added PDF versions of the user guide and reference manual in the Help menu
  - Linux: HTML and PDF helpers are now based on /etc/mailcap (thanks to Christopher)
  - Added editor font selection in the general preferences (thanks to Christopher)
  - Fixed binocular.wbt sample of Mac OS X
  - Fixed automatic path to `WEBOTS_HOME`
  - Moved basicTimeStep, displayRefresh and runRealTime from Preferences to WorldInfo
  - Improved file open from finder in Mac OS X
  - Added mjpegtools on Mac OS X distribution for MPEG movie creation
  - Wizard now create also a fast2d subdirectory in the user directory
  - Display a warning when about to save a world where physics has run
  - Fixed multiple rays distance sensor bugs (thanks to Annelyse and Yvan)
  - Fixed bug in the text editor when using several files (thanks to Yvan)
  - Added soujourner.wbt example (thanks to Nicolas)
  - Removed dependency on libtiff for linux (thanks to Julien)
  - Fixed encoders with DifferentialWheels with physics
  - Added cable.wbt sample world (thanks to Lledó)
  - Fixed compilation under Windows (thanks to Alexander)
  - Fixed controllers list in the scene tree (thanks to Alexander)
  - Fixed cancel button in the first dialog of the controller wizard
  - Added `robot_console_printf()` function
  - Fixed problem with `supervisor_node_was_found()` in the Java API
  - Fixed build of new physics shared libs on Mac OS X
  - Fixed automatic dependencies build for physics shared libraries

## Webots 5.0.4
Released on 16-June-05.

  - Added an introduction in the API chapter of the reference manual about Java programming and other issues (thanks to José and Paul)
  - Batch mode without display working under Linux (thanks to Jim)
  - Added `servo_set_control_p()` function (thanks to Gabriel)
  - Fixed build of automatic dependencies in the Makefile.include (thanks to Julien)
  - Fixed mass distribution for Aibo models
  - Fixed bug with boundingObject and cameras
  - Fixed font to modern in Mac OS X text editor
  - Updated the Webots 5 icons
  - Skip VMware MAC addresses (thanks to Patrick)
  - Uses wxWindows 2.6.1 (with GTK 2 under Linux)
  - Added `supervisor_node_was_found()` in the Java API (thanks to Paul)
  - Fixed TCP/IP live mode for Webview
  - Fixed TouchSensor bug (thanks to Mark)
  - Many IDE improvements (thanks to Yvan)

  - The virtual time is now stored as a double in ms (lasts longer)
  - The basic simulation step is a floating point value (thanks to Jonas)
  - Fixed robot synchronization bug
  - Optimized time step and robot step scheduling
  - Updated sample physics library, `flying_robot` world and controller
  - The physics library can now communicate with controllers (see sample)
  - Emitter.range may be infinite when set to -1
  - Emitter.baudRate may be infinite when set to -1
  - DistanceSensor now may use several ray castings (thanks to Yvan)
  - Added the 2D simulation mode as an external plugin (thanks to Yvan)

## Webots 5.0.3
Released on 15-April-05.

  - Added preliminary force sensors in `hoap2_sumo.wbt` (Thanks to Rawichote)
  - Fixed controller crash at exit with disabled camera (thanks to Annelyse)
  - New images in the control window of the Aibo ERS-7 (thanks to Ricardo)

## Webots 5.0.2
Released on 3-April-05.

  - Fixed the compilation from the built-in IDE (thanks to Yvan)
  - Added `khepera_line` world and controller (thanks to Jean-Christophe)
  - Fixed hours counting which was reset after 1193 hours (thanks to Jim)
  - Simplified `khepera_gripper.c` source code (thanks to Aleksandar)
  - Added Fujitsu Hoap-2 model: `hoap2.wbt` (thanks to Pascal)
  - Fixed CPU usage in STOP mode for Linux and Windows (thanks to Stefan)
  - Added Physics.centerOfMass and Physics.orientation fields (thanks to Pascal)
  - Fixed small selection bug
  - Limited allowed files to be saved by competitors for the Roboka contest (log.txt and memory.dat)
  - Fixed rare bug with repeated calls to `supervisor_node_get_from_def` (thanks to Christian)
  - Fixed problem with the license check of the EPFL license server (thanks to Julien)
  - Fixed `camera_range_image_get_value()` macro with non default near/far parameters (thanks to Yizhen)
  - Added `camera_get_near()` and `camera_get_far()` functions
  - Fixed bug with range-finders occurring on buggy ATI Linux drivers
  - Added a warning when trying to import wrong VRML97 files (thanks to Amanda and Ludovic)
  - Fixed horizontal field of view in Camera and Viewpoint (thanks to Stefan)

## Version 5.0.1
Released on 15-March-05.

  - Fixed saved windows sizes under Linux
  - Fixed DirectionalLight orientation in sample worlds
  - Fixed speed problem on Mac OS X
  - Fixed jump to error under Windows in source code editor (thanks to Yvan)
  - Hopefully fixed hang bug (occurring rarely under Linux)
  - Fixed EPFL license check under Windows
  - Fixed Computer ID check under Windows
  - Opens the first controller of the scene tree when popping up the editor window
  - Added a star (`*`) to mark a modified file in the title bar of the editor window
  - Fixed text editor opening from the window menu
  - Fixed windows titles
  - Changed the window layout of the default.wbt world
  - Added the Webots icon to the text editor window
  - Fixed registration dialog (add trial option on www/registration/webots)
  - Fixed evaluation message in about box (add www/registration/webots/trial)

## Version 5.0.0
Released on 31-Jan-05.

  - Added Camera.near and Camera.far fields (thanks to Yizhen)
  - Fast mode now overrides real time preference (thanks to Jonas)
  - Fixed crash with basic step < 1 (thanks to Jonas)
  - Fixed Viewpoint.near/far warning in libController (thanks to Yizhen)
  - Added a compilation button in the text editor window (thanks to Yvan)
  - Added simple shadows (thanks to Yvan)
  - Improved VRML97 import for Pro E: ignore PROTO and convert Anchor to Group (thanks to Francesco)
  - Several rectangle ground textures made up of two triangles each can be seen by IR distance sensors, see modified rover.wbt (thanks to Rainer Paine)
  - Pen device can now write on several ground textures
  - Use servo sliders in the default robot window
  - Fixed movie sizes under Linux and increased proposed movie sizes
  - Fixed bug with IndexedFaceSet and distance sensors (thanks to Sergei)
  - Fixed collision of two differential wheel robots without physics (thanks to Nikolaus)
  - Added LED control in the khepera.c sample (thanks to Simon)
  - Fixed sphere lighting (thanks to Sergei)
  - Fixed AVI dialog crash (thanks to Nicolas)
  - Fixed wrong small movie size
  - Fixed popup of generic robot window (thanks to Pascal)
  - Documented the Gripper node in the reference manual (thanks to Jaeho)
  - Fixed `servo_motor_off` (thanks to Ludovic and Mehdi)
  - Fixed default synchronization of Supervisor to TRUE
  - Fixed light sensors seeing lights switched off (thanks to Nikolaus)
  - Fixed user guide references to the Preferences menu (thanks to Ricardo)
  - Fixed the distance sensor response when moving the robot with the mouse (thanks to Murray)
