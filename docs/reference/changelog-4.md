# Webots 4 Changelog


## Version 4.0.26
Released on 16-Nov-04.

  - Display rotation axis for Joint nodes (thanks to Auke)
  - Added Physics.orientation field (thanks to Pascal)
  - Added support for Aibo ERS-7 (thanks to Sergei)
  - Fixed color bug with some Extrusion nodes (thanks to Pascal)
  - Fixed color bug with scaled objects (thanks to Nikolaus and Annelyse)
  - Fixed `receiver_get_x/y/z` yet-undocumented functions (thanks to Jim)
  - Fixed bug on UNIX with spaces in directory names (thanks to Chris)
  - Improved VRML97 import feature (thanks to many people who sent me VRML97 samples)
  - Added near and far fields to the Viewpoint node (thanks to Régis)
  - Fixed buffer overflow problem in executable test under Windows
  - Fixed path to upload the Aibo binary files (thanks to Lukas)
  - Fixed bug when a directory called webots was in the unix PATH
  - Removed bboxCenter and bboxSize from Group and inherited nodes
  - Try to open also webots.key.txt and license.srv.txt (thanks to Chris)
  - Added physics shared library support on Mac OS X (thanks to Daniel)
  - Fixed bug on Windows with `-SERVO_INFINITY` (thanks to Nicolas)
  - Fixed bug on Windows when quitting Webots while iconified (thanks to Nicolas)
  - Fixed bug with multiple network cards on Mac OS X (thanks to Daniel)
  - Fixed crash when the license server is not available (thanks to Alessandro)
  - Added support for COM5,COM6,COM7,COM8 serial ports (thanks to Nitin)
  - Swapped default file name and user path in the preferences (thanks to Christopher)
  - No need to copy the new.wbt file in the user world directory (thanks to Sergei)
  - Fixed simulation hang due to robot window display (thanks to Lukas)
  - Fixed physics/Makefile.include to use local ODE include files (thanks to Lucien)
  - Fixed MTN playback with variable number of interpolation frames (thanks to Sergei)
  - Better error messages while parsing world files (thanks to Sergei)
  - Fixed servo limits handling (thanks to Sergei)
  - Uses jar files if any in the controllers' directory (thanks to José)

## Version 4.0.25
Released on 3-Sept-04.

  - Fixed `missing ode.dll` bug with Windows (thanks to Robert)
  - Fixed `could not unregister class` bug with Windows (thanks to Robert)
  - Fixed revert crash under Windows (thanks to Robert)
  - Fixed occasional revert freeze under Linux
  - Fixed bug with real Khepera and Emitter device (thanks to Nitin)
  - Fixed bug with dWebotsGetGeomFromDEF and empty children list (thanks to Alessandro)
  - Don't display empty robot windows
  - Reviewed the Webots User Guide (thanks to Sergei)
  - Added standard video sizes for movie creation
  - Fixed text fields not always updating the scene tree and scene
  - Reverted to GTK+1.2 (Linux): better stability and working mouse wheel
  - Removed endless popup windows in case of some SEGV crash
  - Fixed crash with wrong gripper definition (thanks to Alex)
  - The speedometer now display 3 digits (could show x800!)

## Version 4.0.24
Released on 2-July-04.

  - Added a simple text editor for controllers
  - Upgraded Webots GUI to GTK+2 (Linux)
  - Added random seed reset in `supervisor_simulation_physics_reset` (thanks to Yvan)
  - Fixed crash with empty Coordinate node in a boundingObject (thanks to Dominik)
  - Scan all MAC addresses under Windows (thanks to Chris)
  - Fixed ODE stack overflow with large physics-based models
  - Fixed creation of shapes in 2D modes (thanks to Yizhen)

## Version 4.0.23
Released on 14-June-04.

  - Added support for cylinders in 2D mode (thanks to Yizhen)
  - Added a translation field to the Joint node (thanks to Yvan)
  - Fixed the insertion of a Solid node at the root level
  - Changed the WorldInfo gravity field to a 3D vector (thanks to Yariv)
  - Added `mtn_*` functions for servo motion playback (thanks to Lukas)
  - Fixed MPEG save directory (thanks to Lucien and Nikolaus)
  - Fixed bug in dWebotsGetGeomFromDEF() (thanks to Yariv)
  - Introduced `robot_run` function to replace `robot_step` (thanks to Lukas)
  - Added several MPEG-1 compression options (Linux and Mac OS X)
  - Moved the Preferences... item from the File menu to the Edit menu
  - Added an empty city-like world: town.wbt (misses boundingObjects)
  - Fixed the expiration date of the demo version (thanks to Bothari)

## Version 4.0.22
Released on 13-May-04.

  - Fixed judo.wbt world (robot hand and head bounding boxes)
  - Added trimesh-based rough terrain example: `aibo_ers210_rough.wbt`
  - Added triangle mesh collision detection (Thanks to Lluis)
  - Fixed missing Solid node in scene tree window top node creation
  - Fixed crash when the WorldInfo node is missing
  - New world with 6 Aibos in a RoboCup soccer field: `aibo_ers210_soccer.wbt`
  - Fixed bug with physics less servo (causing `khepera_gripper.wbt` to fail)
  - Fixed display bug with cylinder caps in libController and webview
  - Fixed controller hang caused by `receiver_get_z` (thanks to Jim)
  - Fixed `robot_keyboard_*` functions under Windows
  - Improved Aibo ERS-210 interface: LED, touch sensors, etc. (thanks to Lukas)
  - The camera window is not any more a topmost window under Windows

## Version 4.0.21
Released on 20-April-04.

  - Added `robot_keyboard_` functions to read the keyboard from a controller (see controllers/koala/koala.c)
  - Introduced servo acceleration for the position controller (thanks to Lukas)
  - Fixed bug with servo control when minPosition=0 (thanks to Anders, Dominic, Xavier and Marton)
  - Fixed Coordinate creation in boundingObject in the scene tree window (thanks to Mohammed)
  - Added `camera_save_image()` function that support both PNG and JPEG formats (thanks to Naveen)
  - Documented all the VRML97 nodes used in Webots in the reference manual (available from the ? button of the scene tree window)
  - Fixed Khepera + Gripper + Camera problem with disappearing red stick
  - Fixed Linux crash occurring when the `LD_LIBRARAY_PATH` environment variable was bigger than 256 characters (thanks to Alexandre)
  - Fixed cross-compilation problem with the khepera controller (thanks to Basel)
  - Disabled the insertion of a Solid, Device or Robot node at a wrong position in the scene tree window
  - Fixed mouse wheel zoom under Windows
  - BotStudio doesn't hang any more when the Hemisson robot is unplugged
  - Fixed default world to `botstudio_line.wbt` for Hemisson licenses
  - Improved Aibo ERS-210 model, remote control of real robot (thanks to Lukas)
  - Fixed bug with endless popup message window (thanks to Pascal)
  - Fixed lowercase issue with ComputerID (thanks to Nicolas)
  - Added `webots_physics_draw()` function for custom 3D drawing from the physics library (thank to Jonas)
  - Create better quality MPEG-2 movies under Linux and MacOS X by using mjpegtools (thanks to Auke and Alcherio)
  - Added the `SERVO_INFINITY` position value to make a servo turn endless (thanks to Lucien)
  - Fixed bugs with servo limits and position sign (thanks to Lukas)
  - Fixed complex boundingObject construction from the scene tree window (thanks to Lucien)
  - Allow the user to insert IndexedFaceSet nodes as boundingObject in the scene tree window, should be limited to rectangles though (thanks to Lucien)
  - Added a check box in BotStudio to enable or disable the RBx I/O (thanks to Rémi)
  - Fixed conflict of log() function with Visual C++ (thanks to Jaeho)
  - Windows uninstall clean up the registry (thanks to Yann-Aël)
  - Fixed HemiOS version check in BotStudio (thanks to Rémi)
  - Fixed angle problem in `judo_reset_supervisor` for `JUDOKA_1` (thanks to Mike)

## Version 4.0.20
Released on 5-March-04.

  - Fixed crash caused by the transformation of a device node to a Group node in the scene tree window
  - Added the `gps_euler` function in the C and Java API, this is useful to simulate a compass and two inclinometers (thanks to Mike Jost)
  - Added ODE include files to compile ODE shared library (thanks to Dave)
  - Included the latest Hemisson firmware in controllers/botstudio
  - Added RB0 input and RB6/RB7 outputs to BotStudio (thanks to Rémi)
  - Added boebot.wbt demo (thanks to Sébastien)
  - Added a preliminary model of Aibo ERS-7
  - Added crawling, walking and swimming salamander.wbt demo (thanks to Jérôme)
  - Fixed slow down bug with selection of IndexedFaceSet nodes
  - Fixed the name for physics libraries which was locked to sample.dll (thanks to Alexis)
  - Fixed Khepera gripper presence sensor with a real gripper (thanks to Fernando)
  - Added the capability to talk with a real Khepera from a robot controller using the Khepera protocol (thanks to José)
  - Fixed crash with `khepera_gripper.wbt` and real Khepera robots (thanks to Fernando)
  - Implemented a multi-thread API in C with mutexes
  - All Webots API functions are now MT-safe for use with multi-threaded controllers
  - Fixed the choice of geometry primitives in the scene tree window (thanks to Sébastien)
  - Disabled OpenGL vertical sync on Windows for a much faster 3D rendering: I get a 4x performance increase on my laptop (thanks to Jean-Christophe)
  - Fixed crash due to ODE stack overflow when handling complex models with physics
  - Fixed issue in physics shared library with dGeomGetBody on a non-placeable geom (thanks to Jérôme)
  - BotStudio 2.0.7: fixed quit without saving french message and sensors measurement display
  - Fixed bug with dWebotsGetGeomFromDEF() and nested structures with servos (thanks to Jérôme)

## Version 4.0.19
Released on 23-January-04.

  - Changed judo.wbt world to have a more realistic mass, set joint limits and added a tilt servo in the neck
  - Added the capability to add custom physics with a shared library using ODE (Windows and Linux only)
  - Added documentation on how to make accelerated movies
  - Added ODE world parameters to the WorldInfo node (CFM and ERP fields)
  - Added `supervisor_simulation_physics_reset()` to stop the inertia in the world (thanks to Stéphane)
  - Fixed bug with distance sensors and multiple robot in 2D mode (thanks to Nikolaus)
  - Fixed a memory leak with textured object removal and camera controller
  - Added documentation in the user guide on how to transfer to your own robot
  - Fixed bug on some remote X display configuration (thanks to Nikolaus)
  - Added documentation on the floating license server in the user guide
  - Fixed bug with IR distance sensors and textured IndexedFaceSet nodes
  - Fixed the ground sensor values for the simulated Hemisson (thanks to Neil)
  - Fixed left and right Hemisson sensors in `botstudio_line.wbt` and `botstudio_maze.wbt` (thanks to Neil)

## Version 4.0.18
Released on 16-December-03.

  - Fixed bug in toolbar causing random crashes at startup
  - Creates higher quality MPEG-1 movies (thanks to Auke)
  - First beta version of a fast 2D simulation mode for simple setups
  - Upgraded cplusplus.dev Dev-C++ project sample to Dev-C++ 4.9.8.5
  - Added beta 2D simulation mode for faster simulations on very simple worlds (see `khepera_fast2d.wbt` example)
  - Windows uninstaller erases the Webots preferences for the current user
  - Added gravity field in the WorldInfo node
  - Fixed crash when adding a texCoordIndex in an IndexedFaceSet node (thanks to Sébastien)
  - Fixed crash when specifying a too big color index for a LED (thanks to Frans)
  - Improved the chapter 5 of the user guide on how to setup a Visual C++ project for a Webots controller
  - Reinforced socket error checking to avoid Judo contest server crashes

## Version 4.0.17
Released on 28-November-03.

  - Added the capability to plug a distance sensor on a finger of a gripper, see `khepera_gripper.wbt` (thanks to Ming)
  - Added a section in the controllers chapter of the user guide on setting up a development environment
  - Fixed Mac OS X run and stop buttons (thanks to Marc and Simon)
  - Wait longer (1 second) before killing a controller process (thanks to Mike)
  - Removed useless include files for the C API (thanks to Grace)
  - Fixed menu shortcuts on Mac OS X, Linux and Windows (thanks to Simon)
  - Fixed bug on Mac OS X 10.3 (thanks to Chris)
  - Fixed wrong drawing bug with the pen device (thanks to Rémi)
  - The Mac OS X version is now distributed as a disk image file (.dmg)
  - Fixed corrupted `judo_supervisor` controller
  - Fixed harmless bug in libController box rendering
  - Changed LED colors in judo.wbt (thanks to Frans)
  - Double-clicking on world files now works on Mac OS X
  - Added icons for world files on Mac OS X
  - Fixed `supervisor_fied_*` methods in the Java API (thanks to José)
  - Fixed crash of the webview server when the IP address provided is NULL
  - Display Preferences/About/Quit menu items in the Mac OS X menu and Webots menus
  - Fixed selection bug when the XYZ axes are displayed (thanks to Frans)

## Version 4.0.16
Released on 14-November-03.

  - Fixed wrong VRML97 compliance with texture mapping on IndexedFaceSet
  - Added supervisor functions to the Java API (thanks to José)
  - Improved interactive rotation with the mouse (thanks to Simon)
  - Fixed x y z character display on R G B axis for Linux and Mac OS X
  - Fixed `servo_get_position` return value which was always positive (thanks to many people who reported this bug)
  - Fixed problems with bad USE list in the scene tree window
  - Allowed PoinLight to be embedded as a children of a Solid node (thanks to Simon)
  - Fixed license server problem with licenses limited in time (thanks to Simon)
  - Improved cross-compilation Makefiles for the Khepera robots
  - Added support for real Khepera LED (thanks to Darren)
  - Webots Hemisson now works only in real time mode (thanks to Rémi)
  - Fixed camera reading bug under Mac OS X (thanks to Andrew)
  - Fixed collision detection bug with CustomRobot without physics (thanks to José)

## Version 4.0.15
Released on 07-November-03.

  - Fixed speedUnit and axleLength for the Hemisson robot (thanks to Jean-Philippe)
  - Installer sets up AUTOEXEC.BAT appropriately under Windows 98 and 95
  - Fixed buffer overflow bug in `emitter_send()` with a large buffer (thanks to Stéphane)
  - Added X-red Y-green and Z-blue axis system displaying the orientation of the world
  - Added physics modelling and improved the kiki tutorial in the user guide
  - Fixed a bug in the scene tree window enabling IndexedFaceSet node to be used in a boundingObject
  - Fixed the new button in the Webots Hemisson distribution (thanks to Rémi)
  - Added documentation in the user guide for the edit menu

## Version 4.0.14
Released on 21-October-03.

  - Restored the stay on top behavior for camera windows (thanks to Antoine)
  - Fixed main display hang (thanks to Jérôme)
  - Added LEDs to the Khepera robots
  - Improved the tcpip example (thanks to Darren)
  - Fixed BotStudio saving graph with multiple condition transitions
  - Added `khepera_gripper_camera` example world and controller
  - Fixed gripper + camera bug (thanks to Manuel)
  - Fixed `robot_get_device` for devices that are children of Solid nodes (thanks to Ming)

## Version 4.0.13
Released on 17-October-03.

  - Added `servo_get_feedback` function (thanks to Jérôme)
  - Added set score reading in Judoka0.java (thanks to Frans)
  - Robot view now also follows robots in 3D, useful for flying, or swimming robots (thanks to Jérôme)
  - Fixed crash when attempting to transform a geometry node (thanks to Jérôme)
  - Changed the title of the Webots window and the BotStudio window
  - Fixed a few bugs in BotStudio
  - Fixed bad MAC addresses on Windows
  - Fixed the display of the license message under Windows 98/ME (thanks to Frans)

## Version 4.0.12
Released on 7-October-03.

  - Fixed touch sensors in judo.wbt (thanks to Naveed)
  - Fixed the floor in judo.wbt so that it is not slippery like ice any more
  - Changed logo in the about box
  - Set 8 colors for each LED in judo.wbt, useful for debug (thanks to Frans)
  - The Judo reset supervisor now accepts X Z alpha position/orientation values to reset the robots (thanks to Frans)
  - Removed the useless webots.key file in the downloadable demo
  - Controllers quit after Webots crashes (thanks to Naveed)
  - Added MyEclecticJudoka example in the Judoka0 controller (thanks to Cosmin)
  - Added display field in the Camera node to disable camera window popup
  - Robot windows are destroyed properly on load / reload (thanks to Frans)
  - Fixed transparency problem while supervisor labels are displayed (thanks to Jérôme)
  - Added `servo_set_rel_force_and_torque` and `servo_set_abs_force_and_torque` functions (thanks to Jérôme)
  - Added `gps_euler` beta method in Java API to retrieve inclination and compass
  - Fixed bug with camera data in the Mac OS X / Java interface
  - Added batch mode to disable window display (webots --batch world.wbt)
  - Fixed texture images on Windows for the blimp example (thanks to Jean-Christophe)
  - Improved user guide and reference manual (thanks to Frans)
  - Added a Visual C++ project for the khepera controller, fixed braiten and tcpip Visual C++ projects (thanks to Pontsho)
  - More robust when launching worlds files not in the default world directory (thanks to Rémi)

## Version 4.0.11
Released on 6-October-03.

  - Improved AVI movie export under Windows (thanks to Frans)
  - Fixed crash when a DifferentialWheels robot misses a wheel name like `left wheel` or `right wheel` (Thanks to Andrew)
  - Fixed some typo in the documentation (thanks to Frans)
  - Added `blimp_asl2` flying robot example (thanks to Jean-Christope)
  - Fixed flipped texture images in controller's camera (thanks to Jean-Christophe)
  - Added LED support in the Java API (thanks to Frans)
  - Save the position and size of the BotStudio window (Windows only)
  - Added an icon to the BotStudio window
  - Save the window size and position in the preferences
  - Added a GPS device to the judo robots
  - Added a `judo_reset_supervisor` allowing complex learning algorithms to be implemented in the judo robot contest
  - Fix bug with the colors of label displayed from the supervisor
  - Fixed the Simulation -> Show Robot Window menu item
  - Removed dependency on libpng under Linux to avoid libpng version problems (thanks to Sue)
  - Fixed bug with the movement of custom robots and servos (thanks to Jean-Christophe)
  - Fixed bug occurring when the default file name is not found in the user directory (thanks to Vaibhav)
  - Fixed Windows .wbt icons (thanks to Alessandro)
  - Fixed distance sensor measurement error occurring after `supervisor_field_set` (thanks to Mathieu)
  - Added `supervisor_simulation_revert(void)`
  - Fixed a crash occurring when changing the controller of a robot while the simulation is running
  - Added `supervisor_robot_set_controller(NodeRef robot, const char *controller)`

## Version 4.0.10
Released on 22-August-03.

  - Stripped down webots and controllers binaries to get smaller executables (thanks to Alessandro)
  - Added touch sensors in the feet of the robots in judo.wbt
  - Fixed return value for `servo_get_position` (thanks to Mugjug)
  - Fixed a few floating license issues (thanks to Naveed)
  - Fixed Z-buffer bug for cameras on Mac OS X (thanks to Julien)
  - Added distance sensor names in the judo.wbt world (thanks to Philippe)
  - Added `custom_robot` functions to the Java API (thanks to José)
  - Webots display a clear error message when refusing to launch on some Windows 2K/XP machines (thanks to Cosmin)
  - Added lib/RCXController.jar for Lego Mindstorms cross-compilation (thanks to Carle)
  - Fixed missing texture for kiki.wbt sample world (thanks to Martin)
  - Fixed display bug when opening a new graph with a node selected
  - Fixed memory leak in BotStudio
  - Fixed crash with empty condition (thanks to Alexandre)
  - Fixed timer sensor condition display in BotStudio (thanks to Neil)

## Version 4.0.9
Released on 30-July-03.

  - Added a camera and a distance sensor to each Judo robot in judo.wbt
  - Added `hemisson_botstudio`.wbt as the default world for the Hemisson distribution
  - Added ground texture to the judo.wbt world
  - BotStudio now works on Mac OS X, including serial communication with the Hemisson robot
  - Fixed bug with BotStudio and COM1/COM2 ports (thanks to Jim)
  - Made BotStudio window resizable (thanks to Rémi)
  - Fixed change of robot controller in the scene tree window on Windows (thanks to Fabien)
  - Fixed MAC address problem with non permanent Internet connection on Windows (thanks to Philippe)
  - Fixed Sphere subdivision problem with controller camera
  - Added missing texture for hemisson.wbt (thanks to Rémi)
  - Fixed bug with distance sensors and collision detections (thanks to Nikolaus)
  - Added `servo_get_position` and `servo_motor_off` to the Java API (thanks to Philippe)

## Version 4.0.8
Released on 25-July-03.

  - Added judo.wbt world and Judoka controllers for the contest
  - Added hemisson_maze.wbt world with BotStudio programming
  - Added Hemisson cross-compilation system
  - Fixed `custom_robot_set_abs_force_and_torque`

## Version 4.0.7
Released on 23-July-03.

  - Added inertiaMatrix field to the Physics node (thanks to Jean-Christophe)
  - Fixed bug in scene tree window for inserting Extrusion / IndexedFaceSet nodes in a boundingObject or as a normal shape (thanks to Fabien)
  - Fixed bug with distance sensors and textured objects introduced in version 4.0.6 (thanks to Tiphaine)
  - Fixed a reference to an inexisting world file in the reference manual (thanks to Tiphaine)

## Version 4.0.6
Released on 08-July-03.

  - Fixed bug with IR distance sensors and complex objects made up of several Transform nodes (thanks to Nikolaus)
  - Fixed socket cleanup on Windows in the tcpip controller example (Thanks to Jean-Christophe)

  - Fixed collision problems with physics-less differential wheels robots (thanks to Tiphaine)
  - Fixed the editing of the rotation angle value in the scene tree window (thanks to Nikolaus)

## Version 4.0.5
Released on 03-July-03.

  - Changed the splash screen to the about box showing up only on the first run
  - Extended the deadline of the default evaluation license file
  - Added documentation on the LED node in the reference manual

## Version 4.0.4
Released on 30-June-03.

  - Added a section in the user guide explaining how to cite Webots in a publication
  - Added the client.c file in the tcpip controller example as well as Microsoft Visual C++ project files for Windows
  - Fixed bug when a boundingObject was containing a Group with several Transform nodes (thanks to Nikolaus)
  - Added braiten controller example (Thanks to Nikolaus)
  - Uses the CLASSPATH global environment variable when executing Java controllers (Thanks to Matthew)
  - Fixed bug with Pen device causing crash when loading a new world
  - Added a controller selector in the scene tree window (thanks to Nikolaus)
  - Added a note in the Solid documentation saying that the scale field should always be set to 1 1 1 to avoid problems with bounding objects (thanks to Nikolaus)
  - Disabled scale editing for Solid nodes and bounding objects in the scene tree window (thanks to Nikolaus)
  - Added range finder support in Camera node, see `range_finder.wbt` example (thanks to Alcherio)

## Version 4.0.3
Released on 20-June-03.

  - Added documentation in the user guide on the `Make Movie` and `Screenshot` menu items (thanks to Mathieu)
  - Added `custom_robot_set_abs_force_and_torque` (thanks to Stéphane)
  - Changed field names in the Physics node (staticFriction became coulombFriction and kineticFriction was changed to forceDependentSlip) to match ODE definitions (thanks to Alexis)
  - Fixed the position of X Y Z alpha in the scene tree window on Windows
  - Fixed the display of MF fields in the scene tree window
  - Fixed return values of the light sensors for the simulated Khepera robots (thanks to Jean-Christophe)

## Version 4.0.2
Released on 12-June-03.

  - Fixed bug with `custom_robot_set_relative_force_and_torque` (thanks to Alexis)

## Version 4.0.1
Released on 10-June-03.

  - Fixed rotation angle overflow showing up in very long simulation runs (thanks to Makoto Sato)
  - Added humanoid robot example
  - Inverted the mouse wheel rotation to match 3D navigation standards (thanks to Julien)
  - Display coordinates names and units in the scene tree window (thanks to Julien)
  - Added spin buttons to change numerical values in the scene tree window
  - Changed `custom_robot_add_rel_force_and_torque()` function to `custom_robot_set_rel_force_and_torque()` to fix an important issue regarding basic simulation time step (thanks to Alexis)
  - The scene tree editor prevents to insert a non legal geometry primitive in a boundingObject (thanks to Jean-Christophe)
  - In worlds with Physics nodes, added density -1 where mass was specified to avoid confusion (thanks to Tiphaine)
  - Fixed the generation of USE list when creating a new node in the scene tree window (thanks to Stéphane)
  - Fixed `flying_robot` example so that the robot actually flies
  - Fixed bug with MFNode display in the scene tree window
  - Added a help button in the scene tree window to display node documentation (thanks to Jean-Christophe)
  - Added the `servo_motor_off` function to switch off the motor of a servo (thanks to Eric)
  - Fixed possible random position bugs with CustomRobot node
  - Improved and updated the documentation, especially for the Physics node (thanks to Alexis)

## Version 4.0.0
Released on 22-May-03.

  - Fixed jump around 2*pi while rotating a solid in a physics simulation
  - Fixed `servo_enable_position()` causing Webots to crash
  - Fixed arm and grip position measurement for real Khepera gripper
  - Added kiki.wbt and kiki_camera.wbt described in the Webots user guide tutorial
  - Removed the double splash window in evaluation version
  - Fixed alife*.wbt worlds to work in evaluation mode

## Version 4.0.beta5
Released on 16-May-03.

  - Added a robot window for controlling Servo nodes (double-click on a robot to get it pop up)
  - Added many examples from Webots 3
  - Fixed update of real gripper arm and grip positions on the Khepera window
  - Fixed match with real Khepera gripper aperture
  - Added `hemisson` and `hemisson_pen` examples with support for Hemisson cross-compilation (see controllers/hemisson/hemisson.c)
  - Added pipe relay example: `khepera_pipe.wbt` running pipe controller (Linux and Mac OS X only)
  - Fixed presence and resistivity computation with simulated Gripper

## Version 4.0.beta4
Released on 08-May-03.

  - Added support for remote control of a real Khepera Gripper
  - Added controlP field in Servo node to control proportional PID parameter
  - Added density field in Physics node
  - Added `khepera_gripper.wbt` example
  - Improved the user guide
  - Fixed a few object selection issues
  - Fixed support for Khepera cross-compilation
  - Fixed bug in incremental encoders for differential wheels robots (the encoder value was multiplied by the speedUnit value)
  - Fixed the new button and menu item
  - Fixed Khepera serial support for Mac OS X
  - Fixed images in Windows HTML documentation
  - Improved user guide

## Version 4.0.beta3
Released on 15-April-03.

  - Fixed controller cameras on Mac OS X
  - Added `khepera_k213.wbt` example
  - Added `servo_enable_position`, `servo_get_position` and `servo_disable_position`
  - Added `custom_robot_add_rel_force_and_torque` function
  - Added Aibo ERS-210 model in aibo_ers210.wbt
  - Click on an already selected object doesn't reselect it
  - Display recursively the boundingObject of all children of a Solid
  - Added biped.wbt example
  - Implemented proportional control for servos
  - Reworked reference manual / user guide
  - Fixed hand `with custom_robot` interface
  - Fixed Preferences window size on Mac OS X
  - Fixed Khepera drawing on Mac OS X
  - Fixed color button bug in scene tree window on Mac OS X
  - Added a tutorial chapter on LEGO Mindstorms RCX with Webots
  - Working cross-compilation for the LEGO Minstorms RCX (Rover example)
  - Added the Java Rover example (run on Linux, Mac OS X and Windows
  - Changed license file security system (rely on computer ID)
  - Changed header of saved wbt files to `#VRML_SIM V4.0 utf8`
  - Improved about box and license information system
  - Fixed mouse capture and window focus problems on Windows
  - Added an incomplete chapter on LEGO Mindstorms RCX simulation in Webots
  - Added a paragraph in the Khepera chapter of the user guide
  - Cleanup texture labels (in soccer.wbt) when loading a new world
  - Fixed crash occurring when creating a new world
  - Fixed crash with displaying some USE/DEF nodes
  - Improved real Khepera interface, use simulation stop button to stop the real robot, step mode working with real robot
  - Fixed bug with copy/paste of a node producing USE (null) in the scene tree
  - Fixed hang with revert and some crashes when quitting
  - Added `robot_get_mode()` function to determine whether a controller program controls a simulated robot, a real robot with natively compiled controller or a remote controlled real robot
  - Fixed material modifier effect on decreasing lookupTable for DistanceSensor
  - Fixed bug with the remote control of a real Khepera robot
  - Fixed real time mode
  - Windows controllers compilation works with msys/mingw, DOS/mingw, SciTE/mingw and cygwin
  - Windows controllers compilation allows to specify a `DOS_CONSOLE` variable in the Makefile to display a DOS console with when running the controller (useful if you use printf), see Makefile.included for instructions

## Version 4.0.beta2
Released on 15-March-03.

  - Fixed the Solid lock field to avoid moving locked Solid nodes
  - Locked the soccer field in soccer.wbt
  - Fixed bug with selection of robots after running soccer.wbt
  - Fixed bug with disappearing textures with Mesa
  - Fixed the step and fast buttons
  - Improved multi-threaded controller polling

## Version 4.0.beta1
Released on 17-February-03.

  - Added a soccer example: soccer.wbt
  - Added the ODE physics engine
  - Use wxWindows (native GUI on Linux, Windows and Mac OS X)
  - Fixed differential wheels encoder bug with negative values
  - Improved real Khepera support (now integrated into Webots)
  - Fixed ground texture layout
  - Displays OpenGL information
  - Moved speedometer into the preferences
  - Zoom performed by pressing both left and right mouse buttons
  - Added online help for 3D navigation and moving objects
