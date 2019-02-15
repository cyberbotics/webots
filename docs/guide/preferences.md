## Preferences

The Webots preferences can be modified by a dialog box which can be open from the `Webots / Preferences` menu item on Mac, and from the `Tools / Preferences` menu item on the other operating systems.

The dialog box is separated into tabs.
Each of the following subsection corresponds to one of these tabs.

### General

The **General** tab contains various preferences about the application.

- The **Language** option allows you to choose the language of Webots user interface (restart needed).
- The **Startup mode** allows you to choose the state of the simulation when Webots is started (pause, realtime, run, fast; see the `Simulation` menu).
- The **Editor font** defines the font to be used in Webots text editor and in the Console.
It is recommended to select a fixed width font for better source code display.
The default value of this preference is "Consolas,10" on Windows, "Courier,14" on Mac and "Monospace" on linux.
- The **Number of threads** defines how many threads can be created by Webots at maximum.
The recommended value matches with the number of logical cores of the computer processor.
It may be interesting to reduce this value in some specific cases, for example when another process requires intensively other cores.
For now this value affects only the physical engine speed, and the controller compilation speed.
Note that this is the maximum number of threads allowed, but the actual number of threads used is the one defined in the `optimalThreadCount` field of the [WorldInfo](../reference/worldinfo.md) node.
- The **Python command** defines which Python command is invoked by Webots when starting a Python controller.
The default value is `python`.
It should work on most systems assuming that `python` is installed and available from the command line.
On some systems, it may be useful to set it to `python3.7` for example if you want to launch the controllers with this specific version of Python.
Bear in mind that this value may be overridden by the content of a `runtime.ini` file of a Python controller that may redefine a specific Python command to launch that controller.
- The **Warnings: Display save warning only for scene tree edit** checkbox prevents Webots from displaying any warning dialog window when you quit, reload or load a new world after the current world was modified by either changing the viewpoint, dragging, rotating, applying a force or torque to an object, or modifying the world from a controller.
It will however still display a warning if the world was modified from the scene tree.
- The **Telemetry: Send technical data to Webots developpers** checkbox allows Webots to send anonymous technical data to Webots developpers in order to help improving the software.
A complete description of all the data sent is available [here](telemetry.md).
- The **Update policy: Check for Webots updates on startup** checkbox allows Webots to check if a new version is available for download at every startup.
If available, a dialog window will inform you about it.

### OpenGL

The **OpenGL** tab contains preferences about setting the 3D rendering abilities.
The default parameters of these settings may vary from one computer to another depending on the system's hardware & OpenGL capabilities.

- The **Main 3D view anti-aliasing** option allows you to enable Anti-Aliasing, specifically [SMAA 1x](http://www.iryoku.com/smaa/) on the 3D scene in Webots.
This option can lead to marginally reduced performance, but it improves graphical fidelity somewhat.

> **Note** This option does not apply to any [Camera](../reference/camera.md) rendering, this is managed by the `Disable camera anti-aliasing` setting in the same tab of the preferences dialog.

- The **Ambient Occlusion** option allows you to enable [GTAO](http://iryoku.com/downloads/Practical-Realtime-Strategies-for-Accurate-Indirect-Occlusion.pdf), a modern form of Screen-Space Ambient Occlusion on the 3D view.
This option enables a much higher level of realism in the scene, but at a non-negligible performance cost.
To mitigate this, it is set to "medium" quality by default.
Ultra quality gives the best results, but is the most performance-heavy.

- The **Texture Quality** option allows you to reduce the resolution of all textures in the scene in order to conserve GPU memory. If set to `high` the maximum available (i.e. original) resolution will be used. If set to `medium`, the resolution of images bigger or equal to 1024 is divided by 2 (width and height taken into account independently). If set to `low`, the resolution of images bigger or equal to 512 is divided by 4.

- The **Disable shadows** option allows you to disable completely the shadows in the 3D view and in the [Camera](../reference/camera.md) rendering, whatever the values of the *Light.castShadows* fields.

Globally speaking, performance can be improved by disabling this feature, but on the other hand the rendering is more difficult to understand, and less pretty.

- The **Disable camera anti-aliasing** option allows you to bypass all the *Camera.antialiasing* fields and to disable this feature.
We observed that some hardware doesn't support the OpenGL feature about anti-aliasing when rendering into a texture (RTT).

### Network

The **Network** tab allows you to manually configure a HTTP proxy that Webots will use to access its license server over the Internet.

- The **Proxy type** check box allows you to enable or disable the SOCK v5 proxy protocol.

- The **Proxy hostname** field allows you to set the hostname of the proxy server.

- The **Proxy port** field allows you to set the port used by the proxy server.

- The **Proxy username** field is optional. It allows you to specify a username sent to the proxy server.

- The **Proxy password** field is optional as well and allows you to specify the user password sent to the proxy server.

After changing the proxy configuration, it is recommended to restart Webots to ensure the changes are properly taken into account.
If you clear the **Proxy hostname** field, Webots will try to retrieve the default system proxy on the next launch.
