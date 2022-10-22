## The Standard File Hierarchy of a Project

Some rules have to be followed in order to create a project which can be used by Webots.
This section describes the file hierarchy of a simple project.

### The Root Directory of a Project

The root directory of a project contains at least a directory called "worlds" containing a single world file.
But several other directories are often required:

- "controllers": this directory contains the controllers available in each world files of the current project.
The link between the world files and this directory is done through the *controller* field of the [Robot](../reference/robot.md) node.
More information about this directory in the following subsections.
- "protos": this directory contains the PROTO files available for all the world files of the current project.
- "plugins": this directory contains the plugins available in the current project.
The link between the world files and this directory is done through the *physics* field of the [WordInfo](../reference/worldinfo.md) node.
- "worlds": this directory contains the world files, the project files (see below) and the textures (typically in a subdirectory called "textures").

> **Note**: Note that the directories can be created by using the [**File / New / New Project Directory...**](the-user-interface.md) menu item, described in [this chapter](getting-started-with-webots.md).

### The Project and Thumbnail Files

Each world file has a corresponding project file and thumbnail file.
These files are hidden.
The project files contain information about the GUI (such as the perspective).
If the world file is named "myWorldFile.wbt", its project file is named ".myWorldFile.wbproj" and its thumbnail ".myWorldFile.jpg".
These files are written by Webots when a world is correctly saved.
Removing the ".myWorldFile.wbproj" allows you to retrieve the default perspective.
The 768px by 432px ".myWorldFile.jpg" thumbnail is used for loading when viewing a simulation, animation or a scene on the web. If it is not captured or deleted, a default thumbnail is used.

### The "controllers" Directory

This directory contains the controllers.
Each controller is defined in a directory.
A controller is referenced by the name of the directory.
Here is an example of the controllers directory having one simple controller written in C which can be edited and executed.

```
controllers/
controllers/simple_controller/
controllers/simple_controller/Makefile
controllers/simple_controller/simple_controller.c
controllers/simple_controller/simple_controller[.exe]
```

> **Note**: The main executable name must be identical to the directory name.

<!-- -->

> **Note**: You can create all the files needed by a new controller using the [**File / New / New Robot Controller...**](the-user-interface.md) menu item, described in [this chapter](getting-started-with-webots.md).
