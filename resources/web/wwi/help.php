<?php
header("Access-Control-Allow-Origin: *");
require 'useragent.php';
$touchInterface = isMobileDevice();
$showRun = isset($_GET["run"]) && $_GET["run"] == "true";
$uiImageName = "help_web_interface.png";
if ($showRun)
  $uiImageName = "help_web_interface_with_run.png"
?>
<h2>Web Interface</h2>
 <img src="images/documentation/<?=$uiImageName?>" width="500" alt="">
 <h3>Toolbar</h3>
  <ul>
   <li><img src="images/quit.png" width="20" alt=""> <b>Quit</b>: quit the simulation and return to the upper page.</li>
   <li><img src="images/info.png" width="20" alt=""> <b>Info</b>: open the <a href="#help_info_window">window</a> containing introductory information about the simulation.</li>
   <li><img src="images/reset.png" width="20" alt=""> <b>Reset</b>: save the changes applied to the controller sources and reset the simulation.</li>
   <li><img src="images/step.png" width="20" alt=""> <b>Step</b>: execute one simulation step.</li>
   <li><img src="images/real_time.png" width="20" alt=""> <b>Play</b>: run the simulation in real time.</li>
<?php if ($showRun) { ?>
   <li><img src="images/run.png" width="20" alt=""> <b>Run</b> (optional): run the simulation as fast as possible.</li>
<?php } ?>
   <li><img src="images/pause.png" width="20" alt=""> <b>Pause</b>: pause the simulation.</li>
   <li><img src="images/console.png" width="20" alt=""> <b>Console</b>: open the <a href="#help_console">console window</a> where the simulation messages are printed.</li>
   <li><img src="images/help.png" width="20" alt=""> <b>Help</b>: open this help window containing the web interface documentation and the Webots reference manual.</li>
<?php
  if (!isIOS()) {
    echo "<li><img src=\"images/fullscreen.png\" width='20' alt=''> <b>Fullscreen</b>: enter full screen mode.</li>";
    echo "<li><img src=\"images/exit_fullscreen.png\" width='20' alt=''> <b>Exit fullscreen</b>: exit full screen mode.</li>";
    echo "</ul>";
  } else {
    echo "</ul>";
    echo "<i>Note: On iOS devices the <img src=\"images/fullscreen.png\" width='20' alt=''> fullscreen option is not available due to system limitations. ";
    echo "But from Safari browser it is possible to add the web page to the Home Screen and then execute it as a standalone and fullscreen app. ";
    echo "This action is available from the pop-up menu appearing when tapping on the Share button in the Safari toolbar.</i>";
  }
?>
  </ul>
 <h3>Simulation time</h3>
  <img src="images/documentation/help_simulation_time.png" width="100" alt="">
  <p>The current simulation time is indicated on the first line of the simulation time element.
  </p>
  <p>Additionally, for each simulation a time-out value is defined determining how long the simulation will run automatically.
     When the time-out period elapsed, the simulation will stop and the user has to restart it manually clicking on the <em>Play</em> button.
     The time at which the simulation will stop is indicated on the second line of the simulation time element.
     The next stop time is updated when clicking on the <em>Pause</em> or <em>Step</em> button.
  </p>
 <h3>World selection</h3>
  <img src="images/documentation/help_world_selection.png" width="300" alt="">
  <p>If the loaded project "worlds" directory contains more than one world file, a drop-down list is displayed containing all the world file names.
     Changing the selected world name will load the corresponding world.
  </p>
 <h3 id="help_info_window">Info window</h3>
  <img src="images/documentation/help_info_window.png" width="500" alt="">
  <p>The info window contains some introductory information about a simulation and can be displayed by clicking on the tool bar <em>Info</em> button.
     It may contains several tabs.</p>
 <h3 id="help_console">Console</h3>
  <img src="images/documentation/help_console_window.png" width="500" alt="">
  <p>The console window displays all the simulation information, warning and error messages.
     These messages could come from the simulation core or from the robot controller.
     In fact, it also displays all the messages printed to the standard output (<em>stdout</em>) and standard error <em>stderr</em> from the robot controller,
     i.e. for example using the <em>print</em> statement in Python.
     The console can be open by clicking on the tool bar <em>Console</em> button.
     Keep an eye on the console if the simulation does not work as expected!
  </p>
 <h3>Controller editor</h3>
  <img src="images/documentation/help_editor_window.png" width="500" alt="">
  <p>This is an editor window where it is possible to modify the robot controller programs in order to possibly improve the behavior of the robot.
     It can be open from the robot <a href="#help_edit_controller">context menu</a>, popped up when clicking with the right mouse button on the robot in the 3D scene,
     and it will automatically contains all the editable source files.
  </p>
  <img src="images/documentation/help_editor_menu.png" width="140" alt="">
  <p>In the top right corner of the editor there is a menu containing the following items:
   <ul>
    <li><b>Save</b>: save the modifications applied to the file in the selected tab.</li>
    <li><b>Save All</b>: save the modifications applied to all the files open in the editor.</li>
    <li><b>Reset</b>: reset the modifications applied to the file in the selected tab and restore the original version.</li>
    <li><b>Reset All</b>: reset the modifications applied to all the files open in the editor and restore the original versions.</li>
   </ul>
  </p>
<?php
  if (!$touchInterface) {
    if (isMac())
      $shortcut = 'Cmd-S';
    else
      $shortcut = 'Ctrl-S';
    echo "<p>Note that the Save keyboard shortcut (" . $shortcut . ") only works if the focus is on the editor text area and the text input caret blinks.
     Otherwise, the browser page will be saved instead of the file changes.</p>";
  }
?>
 <h2>Interaction with objects and robots</h2>
  <p>An object context menu is displayed when
<?php
  if ($touchInterface)
    echo "pressing and holding";
  else
    echo "clicking with the right mouse button";
?>
     on an object or robot in the 3D scene. The objects context menu provides the following actions:
   <ul>
    <li><b>Follow/Unfollow</b>: make the camera follow the selected object or revert it to a static position.</li>
    <li>Zoom (not available yet): zoom and center the camera to the selected object.</li>
    <li>Delete (not available): delete the selected object from the scene.</li>
    <li>Properties (not available yet): show the properties of the selected object.</li>
    <li><b>Help</b>: open the documentation page of the selected object.</li>
   </ul>
  </p>
  <img src="images/documentation/help_robot_window.png" width="500" alt="">
  <p>Additional actions are available in robots' context menu:
   <ul>
    <li id="help_edit_controller"><b>Edit controller</b>: open the robot controller program sources in the editor window.</li>
    <li><b>Robot window</b>: open the window containing robot's devices information.<br><small> Note that the robot window is not available for all the robots.</small></li>
   </ul>
  </p>
 <h2>Interaction with the 3D scene</h2>
  <h3>Objects selection</h3>
   <p>It is possible to select an object or a robot in the 3D scene by
<?php
  if ($touchInterface)
    echo "by tapping on it.";
  else
    echo "clicking on it using the left mouse button.";
?>
      Once selected, a white outline representing the physical shape of the object is displayed.
      The physical shape could differ from the graphical shape of the object, as it usually consists of simple geometries,
      and it is used by the physics engine to compute collisions and forces.
   </p>
  <h3>Navigation in 3D</h3>
     <h4>Rotate viewpoint</h4>
     <p>To rotate the camera around the <em>x</em> and <em>y</em> axis, you have to
<?php
  if ($touchInterface)
    echo "touch the screen with two fingers and move them:";
  else
    echo "set the mouse pointer in the 3D scene, press the left mouse button, and drag the mouse:";
?>
     <ul>
      <li>If you clicked on an object, the rotation will be centered around the picked point on this object.</li>
      <li>Otherwise, the rotation will be centered around the position of the camera.</li>
     </ul>
    </p>
   <h4>Translate viewpoint</h4>
    <p>To translate the camera in the <em>x</em> and <em>y</em> directions, you have to
<?php
  if ($touchInterface)
    echo "touch the screen with one finger and move it.";
  else
    echo "set the mouse pointer in the 3D scene, press the right mouse button, and drag them mouse.";
?>
    </p>
    <p>The translation speed is proportional to the distance from the viewpoint to the 3D point on which you clicked.
       If you click on the background, the translation speed is proportional to the distance to origin of the world, e.g., (0, 0, 0).</p>
<?php
  if (!$touchInterface) {
    echo "<h4>Zoom</h4>";
    echo "<p>To zoom in or out the 3D scene, set the mouse pointer in the 3D scene and use the wheel of the mouse.</p>";
  }
?>
   <h4>Zoom/Tilt</h4>
   <p>
<?php
  if ($touchInterface) {
    echo "To zoom the camera, you have to touch the screen with two fingers and pinch or stretch them. ";
    echo "Additionally, moving only one of the fingers horizontally the camera will rotate around its <em>z</em> axis (tilt movement).";
  } else {
    echo "To zoom and tilt the camera, you have to set the mouse pointer in the 3D scene and press both left and right ";
    echo "mouse buttons (or the middle button):";
    echo "<ul>";
    echo "<li>If you drag the mouse vertically, the camera will zoom in or out.</li>";
    echo "<li>If you drag the mouse horizontally, the camera will rotate around its <em>z</em> axis (tilt movement).</li>";
    echo "</ul>";
  }
?>
    </p>
    <p>The zoom speed is proportional to the distance from the viewpoint to the 3D point on which you clicked
<?php
  if (!$touchInterface)
    echo "(or on which the mouse pointer was located when you used the mouse wheel)";
?>
      . If you click on the background
<?php
  if (!$touchInterface)
    echo "(or have the mouse pointer over the background while using the mouse wheel)";
?>
      , the translation speed is proportional to the distance to origin of the world, e.g., (0, 0, 0).</p>
