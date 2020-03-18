## The Scene Tree

As seen in the previous section, to access the Scene Tree Window you can either choose `Scene Tree` in the `Tools` menu, or press the `Show Scene Tree` button in the main toolbar.
The scene tree contains the information that describes a simulated world, including robots and environment, and its graphical representation.
The scene tree of Webots is structured like a VRML97 file.
It is composed of a list of nodes, each containing fields.
Fields can contain values (text strings, numerical values) or other nodes.

This section describes the user interface of the Scene Tree, and gives an overview of the VRML97 nodes and Webots nodes.

%figure "Scene Tree Window"

![scene_tree1.png](images/scene_tree1.thumbnail.jpg)

%end

The scene tree possesses a context menu which contains a number of useful actions, depending on the selection, including but not limited to: cut, copy and paste operations, resetting fields to their default values, moving the [Viewpoint](../reference/viewpoint.md) to an object, setting the [Viewpoint](../reference/viewpoint.md) to follow an object, or opening the Documentation viewer to view the documentation for the selected node.

Additionally, if the current selection is a [Robot](../reference/robot.md) node (or descendant, or `PROTO` instance based on [Robot](../reference/robot.md)) it is possible to open the corresponding robot window or open the robot's controller in the Text Editor.

### Field Editor

Nodes can be expanded with a double-click.
When a field is selected, its value can be edited at the bottom of the Scene Tree.
Double-clicking or pressing the `Enter` key on a field selects the first editable item of the field editor panel.
Keyboard focus can be returned to the Scene Tree by tabbing through all of the items in the field editor panel.
For text fields, changes are applied by pressing the `Enter` key.
This is the same for numeric fields, but the up and down arrow keys can also be used to adjust values up and down, with changes immediately applied.
For checkboxes, values are changed using the `Space` bar.
Applied changes are immediately reflected in the 3D window.
The following buttons are available in the field editor section:

%figure "Webots node editor"

![field_editor.png](images/field_editor.png)

%end

- **Show resize handles**: Displays the handles for resizing and scaling the selected node from the 3D Window.
This option is only displayed for Geometry nodes and nodes derived from [Transform](../reference/transform.md) node.
In case of procedural PROTO nodes, it is only available if the fields involved in the resizing or scaling are not used in template statements.

    > **Note**:
We recommend to use the Scene Tree to write Webots world files. However, because
the nodes and fields are stored in a human readable form, it is also possible to
edit world files with a regular text editor. Some search and replace operations
may actually be easier that way. Please refer to Webots [Reference Manual](../reference/webots-world-files.md) for
more info on the available nodes and the world file format.
