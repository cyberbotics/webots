## Tutorial 8: The Supervisor (40 Minutes)

This tutorial explains how to add and use a [Supervisor](../reference/supervisor.md).

A [Supervisor](../reference/supervisor.md) is a special type of [Robot](..reference/robot.md) which has additional powers.
In fact, any [Robot](..reference/robot.md) can be turned into a supervisor by setting the corresponding field called `supervisor` to true.
A [Supervisor](../reference/supervisor.md) can modify the environment by adding or removing nodes to the scene, it can change their properties by modifying the values of their fields in a programmatic way, allowing for instance to move or setup a robot a certain way and, last but not least, thanks to its unlimited access, it can be used to gather measurements about the state of the simulation as well as its evolution.

This tutorial will explore how to achieve these tasks using a [Supervisor](../reference/supervisor.md).

### Setting up the Environment and Adding a Supervisor

> **Hands-on #1**: Create the environment and add a Supervisor.
Create a new project from the `Wizards` menu by selecting the `New Project Directory...` menu item and follow the instructions:
1. Name the project directory `my_supervisor` instead of the proposed `my_project`.
2. Name the world file `my_supervisor.wbt` instead of the proposed `empty.wbt`.
3. Click all the tick boxes, including the "Add a rectangle arena" which is not ticked by default.
4. In order to have more space, enlarge the arena by setting the size to 10x10 meters by changing the `floorSize` field.
5. Add a [BB-8](bb8.md) robot to the scene, to do this click the `Add` button ![](images/add-button.png =26x26) and navigate to: `PROTO nodes (Webots projects) / robots / sphero / bb8`.
6. For the purpose of this tutorial, remove the default controller of BB-8 by clicking the `controller` field, then the `Select` button, and picking `void` from the list.
7. Add a simple [Robot](..reference/robot.md) node to the scene, this will become our Supervisor.
The [Robot](..reference/robot.md) node can be found in the `base nodes` category when clicking the `Add` button.
To better keep track of it, change the `name` field of this node to `supervisor`.
8. Despite the name, the node is still currently just a [Robot](../reference/robot.md), to turn this robot into a [Supervisor](../reference/supervisor.md) requires to set its `supervisor` field to "TRUE".
9. Much like a normal robot, the behavior of a supervisor is defined by a controller.
Add a controller using the `Wizards` menu and select `New Robot Controller..`, selecting the programming language you prefer.
For this tutorial, Python is the choice, but the code will be provided for all other options.
Set `supervisor_controller` as the name of the controller and click finish.
10. Expand once more the [Robot](..reference/robot.md) node, press the `controller` field and click the `select` button in order to give this node the controller we just created, namely pick `supervisor_controller` from the list.
11. Save the world.

If you followed these steps, your environment should look like this:

%figure "Resulting environment."

![tutorial_ball.png](images/tutorial_8_environment.png)

%end

### Moving Objects using a Supervisor

So far our [Supervisor](../reference/supervisor.md) is quite dull, because according to the controller that was attributed to it, the default behavior does nothing.

In this section, we will program the supervisor to move the BB-8 robot to a different location.
It should be noted that to achieve this we are effectively cheating, rather than instructing the BB-8 to move to a new location we will transport it there.
In other words the movement will ignore all the physics, but herein lies the power of a [Supervisor](../reference/supervisor.md), as it can bend the rules however it likes.

To move the BB-8 to a new location is quite straightforward, if you wished to do so without the help of a supervisor you would simply change its `translation` field to the desired value, say `0 0 2.5`.
The [Supervisor](../reference/supervisor.md) does it much in the same way.

As you might have noticed, the default controller we created using the `Wizard` is setup for a classic robot, not a supervisor.
In order to access the powers of a supervisor requires therefore some slight changes to the controller.
To begin with, replace the contents of the default controller with the following code, depending on the language you have picked and save.
In Python the changes made consist in replacing `from controller import Robot` with `from controller import Supervisor`, in order to have access to the new functionalities provided by the supervisor node.
Similarly, instead of creating a robot instance using `robot = Robot()`, we create a supervisor one instead by doing `robot = Supervisor()`.

>**Note:** It is important to remember that a supervisor is nothing more than a robot with special powers, which implies that whatever a robot can do, so can the supervisor.
This means that you do not need a `Robot` instance if you have a supervisor one.
For example the infinite loop that determines the pace of the controller (namely: `while robot.step(TIME_STEP) != -1`) does not need to be changed, as the supervisor can do the same.


%tab-component "language"

%tab "C"
```c
#include <webots/supervisor.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();

  // CODE FOR HANDS-ON 2

  int i = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    // CODE FOR HANDS-ON 3

    i++;
  }

  wb_robot_cleanup();

  return 0;
}
```
%tab-end

%tab "C++"
```cpp
#include <webots/Supervisor.hpp>

#define TIME_STEP 32

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {

  // create supervisor instance
  Robot *robot = new Supervisor();

  // CODE FOR HANDS-ON 2

  int i = 0;
  while (robot->step(TIME_STEP) != -1) {
    // CODE FOR HANDS-ON 3

    i++;
  }

  delete robot;

  return 0;
}
```
%tab-end

%tab "Python"
```python
from controller import Supervisor

TIME_STEP = 32

# create supervisor instance
robot = Supervisor()

# CODE FOR HANDS-ON 2

i = 0
while robot.step(TIME_STEP) != -1:
  # CODE FOR HANDS-ON 3

  i += 1
```
%tab-end

%tab "Java"
```java
import com.cyberbotics.webots.controller.Supervisor;

public class MySupervisor {

 public static void main(String[] args) {

    int TIME_STEP = 32;

    Supervisor robot = new Supervisor();

    int i = 0;
    while (robot.step(TIME_STEP) != -1) {

      i++;
    }
 }
}
```
%tab-end

%tab "MATLAB"
```MATLAB
TIME_STEP = 32;

% CODE FOR HANDS-ON 2

i = 0;
while wb_robot_step(TIME_STEP) ~= -1
  % CODE FOR HANDS-ON 3

  i = i + 1;
end
```
%tab-end

%end

Save the controller, and if necessary compile it using the `Build` button.
Now everything is ready to begin programming the supervisor so that it moves BB-8 to a new location.

> **Hands-on #2**: Move BB-8 using the Supervisor.
1. In principle the world could be very complex, so it is necessary to have a way of uniquely identifying our BB-8 among the other objects.
To do so we can use the DEF mechanism explored in [tutorial 2](tutorial-2-modification-of-the-environment.md).
Click the BB-8 node in the scene tree and give it a `DEF` name "BB-8", then save the world.
2. Behind the scenes, in Webots each node is uniquely identifiable by a node reference.
To retrieve this reference, the supervisor API method [getFromDef](../reference/supervisor?tab-language=python#wb_supervisor_node_get_from_def) can be used.
In placeholder 1, let's retrieve the node reference of BB-8.

%tab-component "language"

%tab "C"
```c
WbNodeRef bb8_node = wb_supervisor_node_get_from_def("BB-8");
```
%tab-end

%tab "C++"
```cpp
Node *bb8Node = robot.getFromDef("BB-8");
```
%tab-end

%tab "Python"
```python
bb8_node = robot.getFromDef('BB-8')
```
%tab-end

%tab "Java"
```java
Node bb8Node = robot.getFromDef("BB-8");
```
%tab-end

%tab "MATLAB"
```MATLAB
bb8_node = wb_supervisor_node_get_from_def('BB-8');
```
%tab-end

%end

3. Now that we have access to the node, we need to get access to its `translation` field, specifically to a reference to this field as we just did for the node.
To do so, the [getField](../reference/supervisor?tab-language=python#wb_supervisor_node_get_field) method can be used.

%tab-component "language"

%tab "C"
```c
WbFieldRef translation_field = wb_supervisor_node_get_field(bb8_node, "translation");
```
%tab-end

%tab "C++"
```cpp
Field *translationField = bb8Node.getField("translation");
```
%tab-end

%tab "Python"
```python
translation_field = bb8Node.getField('translation')
```
%tab-end

%tab "Java"
```java
Field translationField = bb8Node.getField("translation");
```
%tab-end

%tab "MATLAB"
```MATLAB
translation_field = wb_supervisor_node_get_field(bb8_node, 'translation');
```
%tab-end

%end

4. Finally, now that a reference to the translation field is available, all that remains to do is to set it to a different value.
Once again, the supevisor API has all the necessary tools to do so.
The `translation` field is of type `SFVec3`, which just means it is a three dimensional vector.
So the value can be set by using the [setSFVec3f](../reference/supervisor?tab-language=python#wb_supervisor_field_set_sf_vec3f) method.

%tab-component "language"

%tab "C"
```c
const double new_value[3] = {0, 0, 2.5};
wb_supervisor_field_set_sf_vec3f(translation_field, new_value);
```
%tab-end

%tab "C++"
```cpp
const double newValue[3] = {0, 0, 2.5};
translationField.setSFVec3f(newValue);
```
%tab-end

%tab "Python"
```python
new_value = [0, 0, 2.5]
translation_field.setSFVec3f(new_value)
```
%tab-end

%tab "Java"
```java
double newValue[3] = {0, 0, 2.5};
translationField.setSFVec3f(newValue);
```
%tab-end

%tab "MATLAB"
```MATLAB
new_value = [0, 0, 2.5]
wb_supervisor_field_set_sf_vec3f(translation_field, new_value);
```
%tab-end

%end

5. That is all there is to it, if you save, compile and the controller and run `one-step` of the simulation you will see that BB-8 is transported to a new location instantly.

This is a simple example, but the principle remains the same no matter which field you wish to change.
You can for instance increase the size of an object, perhaps change its color, change the light conditions, or reset its position if it goes out of bounds, the options are limitless.
It is just a matter of getting a node reference, from which a field reference can be obtained by name, and set its value by using the function that is appropriate for the type you are trying to change.

### Spawning and Removing Nodes

Supervisors can also be used to populate the environment, allowing to dynamically setup the scene.
This section focuses on how nodes can be added and removed, specifically we will remove [BB-8](bb8.md) from this world, and replace it with a different robot, namely [Nao](nao.md).

> **Hands-on #3**: Removing and adding nodes.
1. In the previous section, we already saw how to retrieve the node reference of an object.
A node can be removed from the scene tree by using the [remove](../reference/supervisor?tab-language=python#wb_supervisor_node_remove) method.
The `if` condition is not necessary, it simply adds a 10 step delay before the removal to make it more apparent.

%tab-component "language"

%tab "C"
```c
if (i == 10)
  wb_supervisor_node_remove(bb8_node);
```
%tab-end

%tab "C++"
```cpp
if (i == 10)
  bb8Node.remove();
```
%tab-end

%tab "Python"
```python
if i == 10:
  bb8_node.remove()
```
%tab-end

%tab "Java"
```java
bb8Node.remove();
```
%tab-end

%tab "MATLAB"
```MATLAB
wb_supervisor_node_remove(bb8_node);
```
%tab-end

%end

2. After 10 time steps, [BB-8](bb8.md) will be removed from the scene.
Now, let's instead add the [Nao](nao.md) robot after 20 timesteps.
In order to add a node, we must know where we wish to spawn it in the scene tree.
Should it be added at the top level of the scene tree? Should it inserted as a field of a another node?
These questions will change how the node will be inserted and which supervisor function needs to be used, but the constant factor among them is that we need a reference to this position.
In this context, the [Nao](nao.md) robot will be added at the last position in the scene tree, where BB-8 used to appear.
Although not apparent, the scene tree is in fact a [Group](../reference/group.md) node, and each of the objects in the scene tree like `WorldInfo`, `Viewpoint`, `TexturedBackground` and so forth are nothing more than nodes defined as its children.
We refer to this [Group](../reference/group.md) node containing everything as the `root` node.
In order to insert the [Nao](nao.md) robot, the reference we require is actually a reference to the `children` field of the `root` node.
In the spot marked by `CODE FOR HANDS-ON 2`, the following code allows to get this reference.

%tab-component "language"

%tab "C"
```c
WbNodeRef root_node = wb_supervisor_node_get_root();
WbFieldRef children_field = wb_supervisor_node_get_field(root_node, "children");
```
%tab-end

%tab "C++"
```cpp
Node *rootNode = robot.getRoot()
Field *childrenField = rootNode.getField('children')
```
%tab-end

%tab "Python"
```python
root_node = robot.getRoot()
children_field = rootNode.getField('children')
```
%tab-end

%tab "Java"
```java
Node rootNode = robot.getRoot()
Field childrenField = rootNode.getField('children')
```
%tab-end

%tab "MATLAB"
```MATLAB
root_node = wb_supervisor_node_get_root()
children_field = wb_supervisor_node_get_field(root_node, 'children')
```
%tab-end

%end

3. The spawning of a node can be done in two ways.
The first is to describe what you wish to insert, in other words, to spawn it from a string of text.
This method relies in the supervisor functions [importMFNodeFromString](../reference/supervisor?tab-language=python#wb_supervisor_field_import_mf_node_from_string) or [importSFNodeFromString](../reference/supervisor?tab-language=python#wb_supervisor_field_import_mf_node_from_string).
The "MFNode" and "SFNode" components in the name of these functions specify what is the type of the node where the objects is inserted *into*.
"MFNode" stands for multi-field node whereas "SFNode" stands for single-field node.
As previously mentioned, the [Nao](nao.md) should be added to the `children` field of the `root` node, and as you might guess, this `children` field is of type multi-field, namely `MFNode`.
This method is suitable for simple objects, for very complex ones like a custom made robot where the description could be hundreds or thousands of lines long, this method is not ideal.
The second method of inserting a node solves this issue, in fact this long description can be written to a text file with extension `.wbo` and then imported it into Webots by just providing the name of the file to the function [importMFNode](../reference/supervisor?tab-language=python#wb_supervisor_field_import_mf_node) or [importSFNode](../reference/supervisor?tab-language=python#wb_supervisor_field_import_sf_node).

Although the [Nao](nao.md) robot is rather complex, this robot already exists in the Webots library so it can be inserted simply by referring to its name.
Let's add it from string after 20 time steps, add the following snippet in `CODE FOR HANDS-ON 3`:

%tab-component "language"

%tab "C"
```c
if (i == 20)
  wb_supervisor_field_import_mf_node_from_string(children_field, "Nao { }");
```
%tab-end

%tab "C++"
```cpp
if (i == 20)
  childrenField.importMFNodeFromString("Nao { }");
```
%tab-end

%tab "Python"
```python
if i == 20:
  children_field.importMFNodeFromString('Nao { }')
```
%tab-end

%tab "Java"
```java
if (i == 20)
  childrenField.importMFNodeFromString(-1, "Nao { }");
```
%tab-end

%tab "MATLAB"
```MATLAB
wb_supervisor_field_import_mf_node_from_string(children_field, -1, 'Nao { }')
```
%tab-end

%end

As explained here in the function definition [importMFNodeFromString](../reference/supervisor?tab-language=python#wb_supervisor_field_import_mf_node_from_string), the "-1" specifies in which position we wish to insert the node, in this case, to insert it at the last position.

`"Nao { }"` is a string that describes what we wish to spawn.
The way the object is described is by using the VRML97 format, this is the format used in the world files as well.
After 20 timesteps, the [Nao](nao.md) robot will spawn in the middle of the scene.

>**Trick:** If you are not familiar with VRML97, an easy trick to define these strings is to let Webots do it for you.
You can manually create the object (using the Webots interface), and then save the world.
When saving, Webots will translate what you built into a string of text.
If you open the world file with a text editor, you can simply copy the description of the object.

4. Let's assume we wanted the [Nao](nao.md) to be spawned in the position BB-8 used to be, we certainly could move it there following the procedure of hands-on 2, but that would not be smart.
In fact, we can simply specify the translation field directly in the string!
Replace the string `"Nao { }"` with `"Nao { translation 0 0 2.5 }"` and it will spawn exactly at that location.
It does not stop there, in the same fashion we could define its `controller` parameter, or the `cameraWidth` or any other of its parameters in the same fashion.

