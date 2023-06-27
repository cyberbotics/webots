## General Bugs

### Collision Detection

Although collision detection generally works well , `Cylinder-Cylinder`, `Cylinder-Capsule`, `IndexedFaceSet-IndexedFaceSet` and `IndexedFaceSet-Cylinder` collision detection may occasionaly yield wrong contact points.
Sometimes the contact points may be slightly off the shape, therefore causing unrealistic reaction forces to be applied to the objects.
Other times there are too few contact points, therefore causing vibration or instabilities.

### Intel Graphics Cards

Webots should run on any fairly recent computer equipped with a NVIDIA or AMD graphics card and up-to-date graphics drivers.
Webots is not guaranteed to work with Intel graphics cards: it may crash or exhibit display bugs.
Upgrading to the latest versions of the Intel graphics driver may help resolve such problems (without any guarantee).
Graphics drivers from Intel may be obtained from the [Intel download center website](http://downloadcenter.intel.com).
Linux graphics drivers from Intel may be obtained from the [Intel Linux Graphics website](http://intellinuxgraphics.org).

### NAOqi Support is Discontinued: NAQqi-based Controllers may not Work Properly

The makers of the NAO humanoid robot, SoftBank Robotics, no longer maintain the NAOqi interface library that is used to connect Webots to NAOqi and Choregraphe.
As a consequence Cyberbotics cannot ensure that the NAOqi-based Webots controller will work with the various versions of NAOqi and Choregraphe released by SoftBank Robotics.
The `naoqisim` controller is provided as-is, without any guarantee it will work properly.
A known bug is that the robot window cannot not be displayed properly when using the `naoqisim` controller on Windows.
If you experience any problem with NAOqi or Choregraphe in Webots, please report it to SoftBank Robotics directly.

### MATLAB and Remote Control Plugins

MATLAB controllers produce errors when loading a remote control plugin.
This prevent users from performing remote control of real robots from MATLAB.
This problem arises with both the e-puck and darwin-op remote control plugins.

### Orientation Dependent Friction

Although the friction model of ODE is very accurate, the true friction cone is approximated by a linearized version which can introduce some orientation specific artifacts.
It is for example possible that an object slips more easily on another object in some direction than in some other, even if the friction coefficients are set to be symmetric.
However, in most of the cases it is possible to get rid of these effects by tuning correctly the friction parameters.

### Remote Desktop

Webots is not guaranteed to work through a remote desktop application.
This is because Webots has strong ties with the local graphics card for on-screen and off-screen OpenGL rendering.
Unfortunately, several remote desktop applications do not support this very well.

### Virtualization

Because it highly relies on OpenGL, Webots may not work properly in virtualized environments (such as VMWare or VirtualBox) which often lack good OpenGL support.
Hence, Webots may exhibit some display bugs, run very slowly or crash in such environments.

In the case of VirtualBox, it is known that enabling 3D acceleration in `Settings > Display > Acceleration > Enable 3D Acceleration` will cause Webots to crash (due to the old version of OpenGL provided by VirtualBox).
Therefore, please disable the 3D acceleration in your VirtualBox to use Webots.
