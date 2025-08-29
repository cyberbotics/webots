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
- The **Extra project path** defines the paths to user folders similar to the `WEBOTS_HOME/projects` folder.
These user folders should contain projects resources that can be used in the current project (such as PROTO nodes, controllers, textures, etc.).
Multiple user folders can be added by separating them with `:` (or `;` on Windows).
They may contain multiple sub-folders, each one associated to one sub-project (which should respect the [Standard File Hierarchy of a Project](the-standard-file-hierarchy-of-a-project.md)).
Alternatively, the environment variable `WEBOTS_EXTRA_PROJECT_PATH` can be used.
If the `Extra project path` parameter and the `WEBOTS_EXTRA_PROJECT_PATH` environment variable are set, then both will be considered, with the priority given to the paths set in the `Preferences` setting.
Duplicated PROTO nodes will be disabled in the `Add node` menu.
These paths have the priority over the other search paths.
- The **Warnings: Display save warning only for scene tree edit** checkbox prevents Webots from displaying any warning dialog window when you quit, reload or load a new world after the current world was modified by either changing the viewpoint, dragging, rotating, applying a force or torque to an object, or modifying the world from a controller.
It will however still display a warning if the world was modified from the scene tree.
- The **Thumbnail: Capture thumbnail on world save or share** checkbox allows a 768px by 432px (16:9) screenshot of the [3D Window](the-3d-window.md) to be taken when the world is saved or exported. All optional renderings, overlays and selections are hidden for the thumbnail. It can be viewed locally and is used during the loading process of the Webots [Web Interface](web-interface.md).
- The **Telemetry: Send technical data to Webots developpers** checkbox allows Webots to send anonymous technical data to Webots developpers in order to help improving the software.
A complete description of all the data sent is available [here](telemetry.md).
- The **Update policy: Check for Webots updates on startup** checkbox allows Webots to check if a new version is available for download at every startup.
If available, a dialog window will inform you about it.

### OpenGL

The **OpenGL** tab contains preferences about setting the 3D rendering abilities.
The default parameters of these settings may vary from one computer to another depending on the system's hardware & OpenGL capabilities.

- The **Ambient Occlusion** option allows you to enable [GTAO](http://iryoku.com/downloads/Practical-Realtime-Strategies-for-Accurate-Indirect-Occlusion.pdf), a modern form of Screen-Space Ambient Occlusion on the 3D view.
This option enables a much higher level of realism in the scene, but at a non-negligible performance cost.
To mitigate this, it is set to "medium" quality by default.
Ultra quality gives the best results, but is the most performance-heavy.

- The **Texture Quality** option allows you to reduce the resolution of all textures in the scene in order to conserve GPU memory. If set to `high` the maximum available (i.e. original) resolution will be used. If set to `medium`, the resolution of images bigger or equal to 1024 is divided by 2 (width and height taken into account independently). If set to `low`, the resolution of images bigger or equal to 512 is divided by 4.

- The **Max Texture Filtering** option allows you to reduce the maximum filtering level of all textures in the scene in order to reduce GPU usage.

- The **Disable shadows** option allows you to disable completely the shadows in the 3D view and in the [Camera](../reference/camera.md) rendering, whatever the values of the *Light.castShadows* fields.

Globally speaking, performance can be improved by disabling this feature, but on the other hand the rendering is more difficult to understand, and less pretty.

- The **Disable anti-aliasing** option allows you to disable the anti-aliasing in the 3D view and in the [Camera](../reference/camera.md) rendering.
The anti-aliasing algorithm used by Webots is [SMAA 1x](http://www.iryoku.com/smaa/).
We observed that some old graphics hardware doesn't support the OpenGL feature about anti-aliasing.
In such a case, it is better to disable anti-aliasing.
Otherwise, disabling anti-aliasing can lead to marginally increase performance at the cost of degrading graphical fidelity.

### Network

The **Network** tab contains preferences about configuring a HTTP proxy and managing the assets cache.

#### Proxy

The **Proxy** section allows you to manually configure a HTTP proxy that Webots will use to access its license server over the Internet.

- The **Type** check box allows you to enable or disable the SOCK v5 proxy protocol.

- The **Hostname** field allows you to set the hostname of the proxy server.

- The **Port** field allows you to set the port used by the proxy server.

- The **Username** field is optional. It allows you to specify a username sent to the proxy server.

- The **Password** field is optional as well and allows you to specify the user password sent to the proxy server.

After changing the proxy configuration, it is recommended to restart Webots to ensure the changes are properly taken into account.
If you clear the **Hostname** field, Webots will try to retrieve the default system proxy on the next launch.

#### Web Services

The **Web Services** section contains preferences about the URL of the simulation upload service and the web browser used by the HTML robot windows.

- The **simulation upload service** field allows you to set your own simulation upload server URL. The default value is `https://beta.webots.cloud`.
- The **robot window default web browser** field allows you to set the browser in which the robot window will be opened. For example, `firefox`, `google-chrome`, etc. The default value is an empty value, which corresponds to the system default web browser.
- The check box allows you to open the robot window always in a new web browser window instead of a new tab (only available if you set the **robot window default web browser** field).

#### Disk Cache

The **Disk Cache** section allows you to set the maximum size of the cache used by Webots to store the assets (textures, meshes and sounds) downloaded from the Internet and to clear the cache content.
If you change this value and the new cache size is smaller than the currently used cache size, then the cache is automatically cleaned.

The default location of the cache is the following:

%tab-component "os"

%tab "Windows"

`C:/Users/<USER>/AppData/Local/Cyberbotics/Webots/cache`

%tab-end

%tab "Linux"

`~/.cache/Cyberbotics/Webots`

%tab-end

%tab "macOS"

`~/Library/Caches/Cyberbotics/Webots", "/Library/Caches/Cyberbotics/Webots`

%tab-end

%end
