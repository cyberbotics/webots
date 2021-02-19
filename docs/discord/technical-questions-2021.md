# Technical-Questions 2021

This is an archive of the `technical-questions` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m) for year 2021.

## January

##### RoboCoder 01/01/2021 00:01:23
I tried 3.9 and 3.7, same result both times.

##### KajalGada 01/01/2021 00:39:04
`@RoboCoder` I assume you are using Windows. What did you set your Python command in Webots preferences?

##### RoboCoder 01/01/2021 01:32:28
I tried a few things, but right now its set to "python"

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/01/2021 11:08:41
`@RoboCoder` What PATH variables did you set exactly? You should add something like this:

C:\Users\USERNAME\AppData\Local\Programs\Python\Python38

C:\Users\USERNAME\AppData\Local\Programs\Python\Python38\Scripts


Unless you are on Linux or Mac, then it looks differently.

##### RoboCoder 01/01/2021 16:51:01
Should both of those be set in one PATH variable, or in two separate ones?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/01/2021 17:08:12
just add them beneath each other. Same variable

##### RoboCoder 01/01/2021 18:03:14
Okay. Does the variable name matter?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/01/2021 18:03:34
you are on windows right?


You have to change the "Path" variable [https://docs.alfresco.com/4.2/tasks/fot-addpath.html](https://docs.alfresco.com/4.2/tasks/fot-addpath.html)

##### RoboCoder 01/01/2021 18:08:16
Yep, on windows. I'll try that and get back to you.


Just wondering, does the version matter? I'm on 3.7 right now.


Okay, it's working now! I restarted the computer and it solved my issue. Thank you!

##### Yang 01/02/2021 11:50:31
Hi folks, I'm new to webots, just tried out a few examples on my PC, feels great! I'm wondering whether webots accepts urdf format files and mesh files (.stl etc.)?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/02/2021 12:37:40
`@Yang` you can directly import 3d meshes through the webots menu. In order to convert URDF to webot's PROTO format, use this:

[https://github.com/cyberbotics/urdf2webots](https://github.com/cyberbotics/urdf2webots)


I made a more detailed tutorial on how to do that:

[https://github.com/cyberbotics/urdf2webots/blob/master/docs/tutorial.md](https://github.com/cyberbotics/urdf2webots/blob/master/docs/tutorial.md)

##### Yang 01/02/2021 12:54:10
thank you Simon, I will check that out!

##### Nicolas Y 01/03/2021 14:30:49
Hi guys I'm also very new to Webots and I'm working on creating the 'Yamor Snake' but I can't figure out how to use 'Connectors' to get the pieces to stick together does anyone know how to get it done?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/03/2021 16:23:54
`@Nicolas Y` the hierarchy should look something like this:

Robot > children  > Hingejoint > endPoint Solid > children > Hingejoint > endPoint Solid > children > Hingejoint  ...etc

##### pnaraltnsk 01/03/2021 16:25:38
Hi, I am using webots for my graduation project and I am working on the Nao robot. I am trying to add pen node to my robot to mark the robot's walking path but for some reason, the pen node doesn't write. I added pen node to my robot's leftFootSlot and exactly like in pen.wbt example. Could you please help me out with this situation? Or do you know any other ways to mark the robot's walking path?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/03/2021 16:29:13
`@pnaraltnsk` Hi, make sure to follow the documentation: [https://cyberbotics.com/doc/reference/pen?tab-language=python](https://cyberbotics.com/doc/reference/pen?tab-language=python)

Make sure you activated the pen and that the parameters work for you. Also keep this in mind:

In order to be paintable, an object should be made up of a Solid node containing a Shape with a valid Geometry and an ImageTexture. The painted layer is applied over the texture without modifying it.

##### Nicolas Y 01/03/2021 16:30:16
`@Simon Steinmann` Thanks! I'll try it now 🙂

##### PowerYdRa 01/04/2021 05:21:22
hi guys, I am new in webots

I build this arena, my question is how to detect robot outside the arena? the idea is I want to make auto-referee for this arena, so which one outside first will be lose
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/795522239646662656/unknown.png)
%end

##### Stefania Pedrazzi [Cyberbotics] 01/04/2021 06:59:01
A solid could have multiple contact point, so the `index` specifies which contact point info you want. The `index` should be included between 0 and the result of `getNumberOfContactPoints()`:

[https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_node\_get\_contact\_point](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_node_get_contact_point)

Yes, the contact point can be returned only for a `Solid` (or `Robot`) node that doesn't have a direct parent `Solid` node, otherwise you will get NaN values as explained in the documentation.


The `time0to100` value is used to generate the object and compute the maximum acceleration value. So, yes, you can assume that the maximum acceleration is set only in the initialization. Given that the car is a procedural PROTO node, even if you should change the `time0to100` value, the whole robot node would be recreated and the controller will be restarted.


Hi, for your problem the best option would probably be to add a Supervisor node in your world and write a supervisor controller that monitors the status and the positions of the robots. Given that you know the size of the arena, you can get the robot position and check if it is outside:

[https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_node\_get\_position](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_node_get_position)

##### PowerYdRa 01/04/2021 07:33:10
sorry to reply my own question, while waiting the answer from the other, I do some test and the answer is **Supervisor**

just add robot node and enable Supervisor in properties and make some program to get the position of the robot
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/795555408290775070/unknown.png)
%end


thanks for reply, just test and its very easy to get robot position, many thanks for your reply

##### Stefania Pedrazzi [Cyberbotics] 01/04/2021 07:37:07
You're welcome!

##### PowerYdRa 01/04/2021 07:38:02
the next is, how to make robot falling if outside from the arena

any idea how to build proper arena for sumo competition?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/795556632985600010/unknown.png)
%end

##### Stefania Pedrazzi [Cyberbotics] 01/04/2021 07:40:02
Does your robot have Physics enabled? If yes, and if the floor `boundingObject` of  your arena corresponds to the graphical round area, then robot should automatically fall once outside.

##### PowerYdRa 01/04/2021 07:40:24
wait, check it first


this robot is e-puck, so default physics is enable, I think the problem is how I build the arena because its square floor with plane bounding, so I need to change bounding yes?

##### Stefania Pedrazzi [Cyberbotics] 01/04/2021 07:44:05
Yes, a plane in `boundingObject` has infinite size. You could use a Cylinder or a Box instead

##### PowerYdRa 01/04/2021 07:44:32
ok thanks, will try it and will give report here


wow great, thanks
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/795560824860311603/unknown.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 08:24:47
`@PowerYdRa`  A very simple way to check for "falling off" is to check the height of the robots. If the robot is below the arena surface, it loses. That's like 3 lines of code 🙂

##### PowerYdRa 01/04/2021 08:58:07
yes, thats the idea, but how to show some notification in the arena (not in console) ? any suggestion?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 09:07:55
`from controller import Supervisor



\# disqualify height

y\_min = 0



supervisor = Supervisor()

timestep = int(supervisor.getBasicTimeStep())

robot\_1 = supervisor.getFromDef('Robot\_1')

robot\_2 = supervisor.getFromDef('Robot\_2')



root = supervisor.getRoot()

rootChildren = root.getField('children')

nNodes = rootChildren.getCount()

robotList = []

for i in range(nNodes):

    node = rootChildren.getMFNode(i)

    if node.getBaseTypeName() == 'Robot' and node.getDef() != 'referee':

        robotList.append(node)

nRobots = len(robotList)    

isActive = [1] *  nRobots

print(str(nRobots) + " Robots found")





while supervisor.step(timestep) != -1:

    for i in range(nRobots):

        pos = robotList[i].getPosition()

        if pos[1] < y\_min:

            isActive[i] = 0

        if sum(isActive) <= 1:

            print('Fight over!')

            `


`@PowerYdRa` this is a little example of a more general approach of the referee. This way it does not matter how many robots you add and what their DEF is. The only thing that matters is, that the supervisor Robot has the DEF "referee"

##### PowerYdRa 01/04/2021 09:08:52
wow nice, thanks for the code, will try it

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 09:09:11
let me know if you have questions

##### PowerYdRa 01/04/2021 09:09:24
okay, thanks for your help

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 09:09:29
robot\_1 = supervisor.getFromDef('Robot\_1')

robot\_2 = supervisor.getFromDef('Robot\_2')



wups, these two lines are not needed

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/04/2021 09:12:27
You may use supervisor labels to display notifications in the 3D view: [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_set\_label](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_set_label)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 09:13:06
ahhh that's what I was looking for :p

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/04/2021 09:13:20
Or a Display device (which may be a child of your supervisor Robot): [https://www.cyberbotics.com/doc/reference/display](https://www.cyberbotics.com/doc/reference/display)

##### PowerYdRa 01/04/2021 09:14:55
noted, thanks for your reply

will try it after fix the arena view, its look weird only show white 😆

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 09:38:20
`@PowerYdRa` added the display
> **Attachment**: [sumo\_referee.py](https://cdn.discordapp.com/attachments/565154703139405824/795586909321560084/sumo_referee.py)


even changes the color from green to red, when a robot is out 😄

##### Master.L 01/04/2021 09:47:58
Hi I have a question. I want to rotate the coordinates of the anchor. How can you do it?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 09:48:26
What do you mean by anchor?


you mean in a hingejoint?

##### Master.L 01/04/2021 09:49:34
I want to rotate the anchor in the joint parameters of the hinge joint.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 09:50:02
it is relative to the parent


so you need to change the parent orientation, or add a transform node inbetween

##### Master.L 01/04/2021 09:50:53
I want to add a joint at a specific location. But I can't hold the axis of rotation in the direction I want.


Where should I add the transform node?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 09:52:26
The easiest solution would be to position the hingejoint with a parent transform node.


Or you position the location with the anchor, and change the axis value. but this will be less intuitive, as the axis wont be along either the x, y or z axis any more

##### Master.L 01/04/2021 10:34:14
Thank you `@Simon Steinmann`. Thanks to that, the problem was solved.

##### PowerYdRa 01/04/2021 10:53:21
auto referee for sumo competition
> **Attachment**: [sumo\_robot.mp4](https://cdn.discordapp.com/attachments/565154703139405824/795605787815968768/sumo_robot.mp4)


forget to give auto stop simulation, 😀

##### yash 01/04/2021 11:02:42
> A solid could have multiple contact point, so the `index` specifies which contact point info you want. The `index` should be included between 0 and the result of `getNumberOfContactPoints()`:

> [https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_node\_get\_contact\_point](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_node_get_contact_point)

> Yes, the contact point can be returned only for a `Solid` (or `Robot`) node that doesn't have a direct parent `Solid` node, otherwise you will get NaN values as explained in the documentation.

@Stefania Pedrazzi#5404  understood , but for example a two finger gripper to grasp an object , so I will have multiple contact points, and I get them by specifying the index , then how can I identify through this index that which solids is in contact !?

##### Stefania Pedrazzi [Cyberbotics] 01/04/2021 12:25:03
You cannot get information about the other solid which is in contact with the current node from the Supervisor API. If you need this advanced information, then you should implement a physics plugin and retrieve this information from the `webots_physics_collide` callback function:

[https://www.cyberbotics.com/doc/reference/physics-plugin?tab-language=python](https://www.cyberbotics.com/doc/reference/physics-plugin?tab-language=python)

However, if you just need to check if point belongs to the contact between the two finger, then there are maybe other simpler solutions, for example to compare the returned contact points of finger 1 and finger 2.

##### yash 01/04/2021 12:48:17
Alright understood , just one more thing , we have to specify a node to use the function  -

def getNumberofContactpoints() , how to do that?

##### Stefania Pedrazzi [Cyberbotics] 01/04/2021 12:53:05
There are many different ways to retrieve a node, here is the link to the documentation:

[https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_node\_get\_from\_def](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_node_get_from_def) (and all the other functions in this section)

[https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_field\_get\_mf\_node](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_get_mf_node)

[https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_field\_get\_mf\_node](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_get_mf_node)

You can find an example in the sample simulation:

[https://www.cyberbotics.com/doc/guide/samples-devices#supervisor-wbt](https://www.cyberbotics.com/doc/guide/samples-devices#supervisor-wbt)

##### PowerYdRa 01/04/2021 13:45:59
can webots save the world with lock mode? I mean object in the world can't be replace, add or remove until we insert some password or passcode?

##### yash 01/04/2021 13:46:05
> There are many different ways to retrieve a node, here is the link to the documentation:

> [https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_node\_get\_from\_def](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_node_get_from_def) (and all the other functions in this section)

> [https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_field\_get\_mf\_node](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_get_mf_node)

> [https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_field\_get\_mf\_node](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_get_mf_node)

> You can find an example in the sample simulation:

> [https://www.cyberbotics.com/doc/guide/samples-devices#supervisor-wbt](https://www.cyberbotics.com/doc/guide/samples-devices#supervisor-wbt)

`@Stefania Pedrazzi`  thanks a lot !

##### PowerYdRa 01/04/2021 13:50:34
this is because, I want to share the world to student, so they cant change everything except coding the robot only

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/04/2021 14:02:48
`@PowerYdRa` Something like that is not possible from Webots. It should be handled by your OS. For example, you can set the world file permissions to read-only mode, so only the administrator can edit it.

##### PowerYdRa 01/04/2021 14:04:29
if I setup server like in this tutorial [https://cyberbotics.com/doc/guide/web-simulation?tab-language=python](https://cyberbotics.com/doc/guide/web-simulation?tab-language=python)

can other people do some programming for robot?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/04/2021 14:05:48
Yes, in that case it would be similar to:

[https://robotbenchmark.net/](https://robotbenchmark.net/)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 14:15:39
`@PowerYdRa` you can have them export their robot as a .wbo file and give you their controller as well


you can then load them into your world

##### yash 01/04/2021 14:41:58
I have set  SUPERVISOR field to TRUE, so to use  .getDevice() functions for sensors and motors ,  will robot.getDevice() be applicable ?

##### Nicolas Y 01/04/2021 15:23:14
`@Simon Steinmann` Hi Simon, thanks for you're help earlier but I can't seem to figure out how to 'bound' them so that all eight 'Yamors' move as one bigger whole

##### Stefania Pedrazzi [Cyberbotics] 01/04/2021 15:32:42
Yes

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 15:32:52
You have to put a "Robot" node as your base node `@Nicolas Y`

##### Nicolas Y 01/04/2021 15:33:01
I did

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 15:33:13
and add motors in the device section of the hingejoints


what exactly is your problem?

##### Nicolas Y 01/04/2021 15:35:06
I'm not sure how to get the components to stick
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/795676689534812170/unknown.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 15:35:37
Post your world file here


one thing I can see from the screenshot is, that you have not added JointParameters


double click on them and add a Parameter node


The individual Solids also need a physics node and a bounding Geometry

##### Nicolas Y 01/04/2021 15:40:17
This is my world file
> **Attachment**: [YamorSnake1.wbt](https://cdn.discordapp.com/attachments/565154703139405824/795677996596985856/YamorSnake1.wbt)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 15:41:45
ohhh


you added all the segments as children of the robot


you have to put each next joint into the children node of the Solid before

##### Nicolas Y 01/04/2021 15:42:32
ohhh


I'll try that now

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 15:43:01
you can add boundingObject and physics by simply doubleclicking it


ohh wait, all the Yamor are individual robots?

##### Nicolas Y 01/04/2021 15:43:34
yes

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 15:43:55
are they supposed to be individually controlled?

##### Nicolas Y 01/04/2021 15:44:09
no I want them to work as one

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 15:44:09
Because I'm not sure about a chain of robot nodes

##### Nicolas Y 01/04/2021 15:44:37
oh?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 15:45:43
Perhaps the devs know the best solution for that. I'm not sure about Robots containing other Robots

##### Nicolas Y 01/04/2021 15:47:03
oh


that complicates things

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 15:48:34
`@Olivier Michel` How does a hierarchy of robots work? Do the child robots act as devices if they don't have a controller assigned?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/04/2021 15:49:23
No, a robot can access only the devices it contains. If a robot contains another robot, the devices of the child robot are not available to the parent robot.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/04/2021 15:50:36
`@Nicolas Y` I think you are supposed to add one Yamor, then doubleclick on 'extensionSlot' and add another

##### Nicolas Y 01/04/2021 15:51:22
`@Olivier Michel` in which case how can one replicate this?
%figure
![yamor.png](https://cdn.discordapp.com/attachments/565154703139405824/795680785843748954/yamor.png)
%end

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/04/2021 15:57:41
In this example (included in Webots), each module is an independent robot (there is no hierarchy).


Robots attach to each other with a Connector node.

##### Nicolas Y 01/04/2021 15:59:55
`@Olivier Michel` okay thanks I'll try that

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/04/2021 16:17:39
I would recommend you to start from the yamor demo and modify it bit-by-bit to ensure you are not breaking anything.

##### Nicolas Y 01/04/2021 16:33:35
`@Olivier Michel` sorry but despite checking the website I can't seem to figure out how to use the 'Connector' node to Combine the two 'Yamor' nodes

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/04/2021 16:34:45
I would recommend you to open the world file in a text  editor.


Yes, in the scene tree of Webots, but it may be easier to edit from a text editor (but this is a question of personal preferences).

##### pnaraltnsk 01/04/2021 20:28:39
Hi, I followed the documentation and checked everything else. Everything looks fine and the pen should have been working but it didn't. I am still struggling to find what's wrong.

##### TerryWr1st 01/05/2021 00:19:20
Hi all, new to this channel. I'm developing my thesis in Webots, which consists of a number of epucks at the start of the simulation. During runtime, I then need to spawn additional epucks and delete the old epucks which have served their purpose in the simulation. Is this possible in Webots? And is it achieved through a supervisor function, or some other method?

##### DDaniel 01/05/2021 00:47:52
Yes you can do it using the supervisor, you can add and remove the nodes of the robots from the scene tree: [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_field\_import\_mf\_node\_from\_string](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_field_import_mf_node_from_string)

##### TerryWr1st 01/05/2021 00:50:29
Ah ok, brilliant, thank you. Will take a look.

##### pk1jk1 01/05/2021 03:35:29
Has anyone successfully implemented RRT for a robot to get to a waypoint?

##### Alessia Valle 01/05/2021 08:54:35
Hi, I'm working on my Master Thesis and I need your help if possible. I have imported my four-wheeled robot CAD model into webots and I need to interface it with ROS (not ROS2) to implement SLAM algorithm. The robot is equipped with a Lidar sensor. I have gone through webots and ROS documentation but I am still stuck.

I have seen that to use slam\_gmapping, my mobile robot needs to provide odometry data and I need to publish two different transforms, base\_laser->base\_link and base\_link->odom. Since I am novice to ROS and I have no experience in interfacing it with webots, here are my questions:

- how to make my robot publish its odometry? my idea is to use Position Sensors, but I don't know how to turn the values from the position sensors into the standard nav\_msgs/Odometry used in ROS. I think I am missing some steps. 

- since I need to publish transforms of odom, base\_link and base\_laser to use with gmapping, do I need to create the URDF of my robot? 

Thank you in advance

##### Master.L 01/05/2021 09:11:28
Excuse me, I have a question.

I am implementing a delta robot model in webots.

There is an upper leg that moves by a hinge joint and a lower leg that is connected to the upper leg by a ball joint.

Since the hinge joint is moved after starting the simulation, the ball joint continues to rotate and the position value increases. Therefore, the part connected to the ball joint continues to rotate. How can I fix it?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/05/2021 09:15:59
`@Alessia Valle` a Position sensor measures the angle of a joint / motor. What you need is the Pose

[http://docs.ros.org/en/melodic/api/nav\_msgs/html/msg/Odometry.html](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)


this is the nav\_msgs definition

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/05/2021 09:16:39
Hello `@Alessia Valle`,

- For odometry, see the example here: [https://github.com/cyberbotics/webots\_ros2/blob/9682e06932929ea3bfab5a6a826f6fe67471aef6/webots\_ros2\_core/webots\_ros2\_core/webots\_differential\_drive\_node.py#L128-L197](https://github.com/cyberbotics/webots_ros2/blob/9682e06932929ea3bfab5a6a826f6fe67471aef6/webots_ros2_core/webots_ros2_core/webots_differential_drive_node.py#L128-L197)

(it is for ROS2 but there is almost no difference for ROS)

- You don't have create URDF to publish transforms. If your model is simple you can publish them manually, see an example for a physical robot:

[https://github.com/cyberbotics/epuck\_ros2/blob/master/epuck\_ros2\_driver/src/driver.cpp#L167-L232](https://github.com/cyberbotics/epuck_ros2/blob/master/epuck_ros2_driver/src/driver.cpp#L167-L232)

(If you need URDF you can use [https://cyberbotics.com/doc/reference/robot#wb\_robot\_get\_urdf](https://cyberbotics.com/doc/reference/robot#wb_robot_get_urdf))

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/05/2021 09:21:38
is the ball joint passive? If so. you have to add friction


so expand the balljoint and add a "JointParameters" node


there you can edit those values

##### yash 01/05/2021 09:34:26
I am getting some weird results when I play the simulation.
> **Attachment**: [test01.mp4](https://cdn.discordapp.com/attachments/565154703139405824/795948316768075806/test01.mp4)


what could be the issue ?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/05/2021 09:36:45
can you click on your robot and show a screenshot (with the white lines of the bounding box)


My first guess would be, that you used the visual mesh data as collision. And your shapes are not convex


Also it seems like it might be starting partially in the floor

##### yash 01/05/2021 09:39:27
this the bounding box
%figure
![Screenshot_from_2021-01-05_15-08-34.png](https://cdn.discordapp.com/attachments/565154703139405824/795949578347085854/Screenshot_from_2021-01-05_15-08-34.png)
%end


how could I rectify the problem ?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/05/2021 09:40:13
oh the grey metal part is not part of the robot?


and it looks like therre is a horizontal part, which has no visual

##### yash 01/05/2021 09:40:55
no it is not the part of the robot

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/05/2021 09:41:11
but the robot is attached to it?


In general it is much better (for performance AND accuracy) to replace the boundingObject with simple geometries, such as boxes, spheres, cylinders and capsules

##### yash 01/05/2021 09:42:08
yes I have positioned the robot accordingly, but it is not attached to it.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/05/2021 09:45:33
well it is hard to say. What do you expect to happen?

##### Master.L 01/05/2021 09:45:42
Thanks for solving it!!

##### yash 01/05/2021 09:47:12
the 2 link manipulator should rotate about it's axis provided. With the metal plate at it's position only.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/05/2021 09:48:39
you have to make its axis static then


remove the physics node of the Robot


it still seems weird to me, that you have 3 segments in your bounding object


there is something weird there

##### yash 01/05/2021 09:50:19
oh okay, alright let me look into it, thanks !

##### nelsondmmg 01/05/2021 21:53:51
Could you cite a reference for the differential ratio calculated for the rear wheel of the vehicle? I'm assuming that a similar correction need to be made before applying the acceleration to the wheels of the vehicle, but I cannot find any reference that explains how this differential ratio is calculated. Thanks.

##### PowerYdRa 01/06/2021 00:22:35
hi guys, I get a problem,

just install numpy via pip in terminal, but when call in webots, its give error No Module named 'numpy', how to define library / module installed to webots?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/796171825926504448/unknown.png)
%end

##### DDaniel 01/06/2021 00:41:20
Make sure in tools>preferences the python command is also python3

##### TerryWr1st 01/06/2021 00:49:02
Can Webots compile in x86, or only x64? I went through the visual studio setup both manually,  and with the wizard. The wizard sets the configuration properties as Release/x64, but this step isn't mentioned in the manual set up guide: [https://cyberbotics.com/doc/guide/using-your-ide](https://cyberbotics.com/doc/guide/using-your-ide)


Also, why does the wizard default to a Release configuration instead of debug?

##### Diego Rojas 01/06/2021 05:50:10
What would be the best way to simulate a ur5 robot on a 7th axis linear track using ROS2 and moveit2? I would like to control both the linear motion of the 7th axis track and the robot arm with moveit2. Using the follow\_joint\_trajectory action server, I was able to control the ur5 robot with ros2 control and moveit2, but now I want to attach the robot to a linear track. I have done this in the past in gazebo by creating a prismatic joint and using the follow\_joint\_trajectory controller to control it with ros\_control and moveit. Is this possible in webots also? How can I utilize the existing follow\_joint\_trajectory action server that is created in webots\_ros2 and create another follow\_joint\_trajectoy action server for a linear/prismatic joint with webots\_ros2?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/06/2021 07:12:43
Webots can compile controllers in either x64 or x86. However, by default it will build x64 binaries. If you want to build a x86 controller binary, you should be sure to link it with `WEBOTS_HOME\msys64\mingw32\bin\Controller.dll` instead of `WEBOTS_HOME\msys64\mingw64\bin\Controller.dll`.

##### yash 01/06/2021 07:38:12
I am working on a certain application that involves grasping a human foot. I imported the human foot CAD using .VRML format. And used the DEF geometry indexfaceset in the shape node to define the bounding object of the human foot. But the bounding object seems to be complex which is resulting in weird physics behavior.  How to address this issue such that the human foot remains stable ?
> **Attachment**: [2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/796281451237736478/2.mp4)


Because when I replaced the foot with a simple cylindrical solid it worked fine since the bounding object of the cylinder is not complex.

##### Stefania Pedrazzi [Cyberbotics] 01/06/2021 07:48:08
The used formula comes from standard steering dynamics. Depending if the front or rear  wheels drive the car (or all four wheels) you need to compute the ratio for the left and right wheel because the do not turn using the same angle.

Here is the reference to a chapter about steering dynamics: [https://link.springer.com/chapter/10.1007/978-1-4614-8544-5\_7](https://link.springer.com/chapter/10.1007/978-1-4614-8544-5_7) but you can probably find other references on the web.

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/06/2021 07:53:37
You should create a compound bounding object for the foot that is not a mesh, but a combination of simple primitives (spheres, capsules, cylinders and boxes). See this tutorial: [https://cyberbotics.com/doc/guide/tutorial-5-compound-solid-and-physics-attributes](https://cyberbotics.com/doc/guide/tutorial-5-compound-solid-and-physics-attributes) for an example of creating a compound bounding object.

##### yash 01/06/2021 08:01:33
alright ! but this wiil solve the issue right irrespective of the solid geometry ?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/06/2021 08:02:22
It should (as long as you don't include any mesh in the bounding object).

##### yash 01/06/2021 08:02:48
understood, thanks a lot !

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/06/2021 08:48:56
Hello `@Diego Rojas`, I have never tried adding 7th axis linear track, but here are a few ideas. `TrajectoryFollower` can control any motor present in the robot. Therefore, you can add the linear track (built with `SliderJoint` which behaves in the same way as the prismatic joint) in the Webots robot model.



Alternatively, you can use `ConveyorBelt` but it is more complicated. You need to enable a `Supervisor` field of your robot and extend `trajectory_follower` to control `speed` field of `ConveyorBelt`. This will also require some basic integration to control the position of the belt as `ConveyorBelt` allows only a speed control.

##### Alessia Valle 01/06/2021 09:08:34
Hi `@Darko Lukić`, thank you again for the suggestions.  I also found on the ROS wiki website the diff\_drive\_controller  [http://wiki.ros.org/diff\_drive\_controller](http://wiki.ros.org/diff_drive_controller) but I don't know if it is useful for my purposes and if it can be used with Webots. Any idea? 

Moreover, I have another doubt about transforms and URDF. Actually I need the static transforms between the wheels, the base\_link and base\_laser to set up navigation. But how can I publish transforms without the URDF file? Do these frames take the name of the corresponding nodes on Webots?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/06/2021 09:20:51
You can use `diff_drive_controller` as well. Your objective is to publish `Odometry` topic and transforms that describe translation and orientation between `base_link` and `odom` frames. You can use `diff_drive_controller` or write one by yourself.



You don't need URDF to publish transforms. In general, `robot_state_publisher` uses URDF to publish `TransformStamped` ([http://docs.ros.org/en/melodic/api/geometry\_msgs/html/msg/TransformStamped.html](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TransformStamped.html)) messages. However, you can publish those messages by yourself.

##### pk1jk1 01/06/2021 18:32:51
Hi! Trying to setup my environment for ros2 on my mac and am getting a file not found error, am I missing something? [https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/#environment-setup](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/#environment-setup)


This is what my directory looks like:
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/796446454096068608/unknown.png)
%end


Hi! Were you able to solve this problem? Am running into the same issue

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/06/2021 18:53:25
`@pk1jk1` 

[https://index.ros.org/doc/ros2/Installation/Foxy/macOS-Install-Binary/](https://index.ros.org/doc/ros2/Installation/Foxy/macOS-Install-Binary/)

You should follow a tutorial for macOS

##### pk1jk1 01/06/2021 19:00:00
~~I was following the tutorial, ran `tar xf ~/Downloads/ros2-release-distro-date-macos-amd64.tar.bz2` and am unsure why the next step of sourcing the ros2 file `. ~/ros2_foxy/ros2-osx/setup.bash` is trying to look for `local_setup.bash` in the directory above since the extract only puts everything into the `ros2-osx` folder as shown above~~



Figured it out needed to run `. ~/ros2_foxy/ros2-osx/setup.zsh`

##### Master.L 01/07/2021 03:02:30
Hi! Is there an inverse kinematics library for running delta-robots on webots?

##### Luiz Felipe 01/07/2021 08:33:45
Hello, I am trying to run a web simulation using a session and simulation server inside a docker container as explained in: [https://cyberbotics.com/doc/guide/web-simulation?tab-host=localhost](https://cyberbotics.com/doc/guide/web-simulation?tab-host=localhost)


I created the docker image, ran the docker container and inside the /usr/local/webots/resources/web/server i ran the './server.sh'



%figure
![Screenshot_from_2021-01-07_17-36-56.png](https://cdn.discordapp.com/attachments/565154703139405824/796658964321533992/Screenshot_from_2021-01-07_17-36-56.png)
%end


It seems to start successfully... but now I am having trouble testing if that works


As the documentation I am trying 'ws://localhost:1999/session?url=webots://github.com/cyberbotics/webots/branch/develop/projects/languages/python/worlds/example.wbt' in Connect to using the streaming viewer but i get streaming server error 1006

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/07/2021 08:43:56
`@Master.L` [https://gist.github.com/hugs/4231272](https://gist.github.com/hugs/4231272) this looks like what you're looking for


[https://www.marginallyclever.com/other/samples/fk-ik-test.html](https://www.marginallyclever.com/other/samples/fk-ik-test.html) or this could help

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/07/2021 08:46:17
Did you try to check the monitor page http://localhost:1999/monitor to check the status of your server?

##### Luiz Felipe 01/07/2021 09:35:08
Oh... If I check it, it appears the load:



%figure
![monitor.png](https://cdn.discordapp.com/attachments/565154703139405824/796673297160601600/monitor.png)
%end


But if i click the localhost/2000 I get 'This site can't be reached'... Hm...

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/07/2021 09:38:04
It should be http://localhost:2000 instead.

##### Luiz Felipe 01/07/2021 09:39:42

%figure
![monitor-2.png](https://cdn.discordapp.com/attachments/565154703139405824/796674417806802994/monitor-2.png)
%end


Yes.. It works... it seems that I am doing something wrong in the Connect to part....

##### YCSU 01/07/2021 09:51:21
Hi, is there a way to move all the motors of the robot arm at the same time? For example, I would like to move the tool center point (tcp) of a ur5e in a straight line from x=0 m to x=5 m. Right now I call the  \_setPosition\_ function for each motor in a sequential order with the inverse kinematics result, and judging from the movement, the  tcp is not moving in a straight line. Especially at the start of the movement, the arm jitters. I suspect it's caused by the asynchronous movement of the motors.

##### Luiz Felipe 01/07/2021 09:57:00
The session server and simulation servers seems to be up and rnning... Is that correct to put 'ws://localhost:1999/session?url=webots://github.com/cyberbotics/webots/branch/develop/projects/languages/python/worlds/example.wbt' i the 'Connect to' part to test the web simulation?
%figure
![Screenshot_from_2021-01-07_18-55-34.png](https://cdn.discordapp.com/attachments/565154703139405824/796678771264192522/Screenshot_from_2021-01-07_18-55-34.png)
%end

##### Alessia Valle 01/07/2021 11:56:31
Hello everyone! My tutor wrote to me and he has changed his mind about my thesis. He asked me to implement SLAM algorithm on Webots without any integration with ROS. Do you have some resources/documentation about it? Is there any existing project in which SLAM has been implemented?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/07/2021 12:14:22
The only examples about SLAM and Webots include ROS and ROS2:

[http://wiki.ros.org/webots\_ros/Tutorials/Sample%20Simulations](http://wiki.ros.org/webots_ros/Tutorials/Sample%20Simulations)

[https://github.com/cyberbotics/webots\_ros2/wiki/SLAM-with-TurtleBot3](https://github.com/cyberbotics/webots_ros2/wiki/SLAM-with-TurtleBot3)



If you need a background in this topic you can check:

[https://pythonrobotics.readthedocs.io/en/latest/modules/slam.html](https://pythonrobotics.readthedocs.io/en/latest/modules/slam.html)

[https://github.com/kanster/awesome-slam](https://github.com/kanster/awesome-slam)

[https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN\_](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_)



I am not sure whether there is a good implementation of SLAM that you can easily integrate into your project without ROS. `cartographer` is a good implementation, but I am not sure how hard is to use it without ROS:

[https://github.com/cartographer-project/cartographer](https://github.com/cartographer-project/cartographer)

##### yash 01/07/2021 12:25:58
I want to use the supervisor function as well as the Robot node , therefore should I write ——

from controller import Supervisor, Robot



And should define as -

robot = Robot()

supervisor = Supervisor() 

 

Please help me with this !?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/07/2021 12:27:32
If you set the motor positions in the single step they all move at the same time. Your problem is therefore something else. For example, the motor in shoulder moves the endpoint faster than the motor in the wrist even though they rotate at the same speed (the motor in the shoulder is farther away from the endpoint than the motor in the wrist). One solution would be to find a lot of IK solutions along a straight line and apply the new IK solution in each step.


No. `robot = Supervisor()` is enough. It will give you methods from `Robot` and `Supervisor` nodes.

##### Alessia Valle 01/07/2021 12:34:06
Thank you. So do you think that I will have to write the SLAM algorithm from scratch on my own?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/07/2021 12:35:39
Yes, but make sure the port is correct.

##### yash 01/07/2021 12:37:14
> No. `robot = Supervisor()` is enough. It will give you methods from `Robot` and `Supervisor` nodes.

`@Darko Lukić`  thank you !

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/07/2021 12:38:28
I don't know :/ Depends on your project.

##### Alessia Valle 01/07/2021 12:45:00
What do you mean by "it depends on your project?"😅

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/07/2021 12:47:48
I guess if you have a good understanding of SLAM than you can implement it, but if you want a robust solution it is better to use something like `cartographer`.

##### Alessia Valle 01/07/2021 12:58:44
Ok, I will go through the documents you have shared! Thank you for your availability! 😀

##### josiehughes 01/07/2021 13:09:59
Hi - I'm wondering about the current e-puc hardware and compatability with Webots. Is the Epuc-2 compataible with Webots?

##### Luiz Felipe 01/07/2021 14:07:09
I get streaming server error... will check the debug flags of the session and simulation servers...

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/07/2021 15:54:45
Hello `@josiehughes`, It is compatible

##### YCSU 01/07/2021 16:01:07
Got it. Thanks!

##### Ragemor 01/07/2021 17:43:11
Is there any Webots project about multi robot coordination using leader follower approach? I want to implement leader follower algorithm in my project but i cant understand how can i write code about that. Which nodes and api functions(or sensors) should i use? Do you have docs about leader follower and multi robots in webots?

##### adamadam333 01/07/2021 20:15:16
Hi, I am trying to rotate an epuck2 robot by 90 degrees, I tried using time but I couldn't measure exactly 90 degrees, is there a way to do it effectively not using time and angular speed?

##### adiagr 01/07/2021 21:36:03
The most effective way to turn an epuck2 is by using time and angular speed itself. You may want to have a look at the proto file and take into account the thickness of the wheels among the other things to get a more accurate value of axle length.

##### ArjunSadananda 01/07/2021 21:41:23
`@adamadam333` If you prefer, you could use encoders (and write a closed loop controller) instead of doing it by calculating the time (open loop controller).

##### Luiz Felipe 01/08/2021 06:08:10
In line 46 in [https://github.com/cyberbotics/webots/blob/master/resources/web/streaming\_viewer/index.html#L46](https://github.com/cyberbotics/webots/blob/master/resources/web/streaming_viewer/index.html#L46) of the new 2021a version the line it is not supposed to be:  `  <script src="[https://www.cyberbotics.com/wwi/R2021a/webots.min.js](https://www.cyberbotics.com/wwi/R2021a/webots.min.js)"></script>` ?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/08/2021 08:16:27
`@Luiz Felipe` There is `webots.min.js` distributed with Webots packages (`resources/web/wwi/webots.min.js`), but you can also use the one available on Cyberbotics servers.

##### Luiz Felipe 01/08/2021 08:21:10
Yes `@Darko Lukić` , I changed to use the one available on Cyberbotics servers because I am running the servers in a docker container. But the streaming server page seems a big buggy for the 2021a version in my computer... I am still having trouble following the Web Simulation documentation also, the session server and simulation server are fine, but I am not seem able to figure out how to open a simulation or send any information to the simulation server...


I just thought it is better for the default to be the one available on the Cyberbotics servers.

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/08/2021 08:28:01
It should be the same anyhow.

##### yash 01/08/2021 08:33:19
Can I put touch sensor in children of solid node ? Since when the touch sensor collides with another solid, I want the contact point of the collision?

##### Luiz Felipe 01/08/2021 08:39:46
Yups... The streaming viewer however does not open... The 2020b version works fine...
%figure
![Screenshot_from_2021-01-08_17-37-04.png](https://cdn.discordapp.com/attachments/565154703139405824/797021720057741312/Screenshot_from_2021-01-08_17-37-04.png)
%end

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/08/2021 08:43:19
Yes, you can. However to get the position of the contact point of the collision, you should use the supervisor API, not the touch sensor API.


I am sorry, but currently deploying a streaming server is not very easy, has some bugs and also is not very well documented. We are currently working on improving this, but it will take some time. Meanwhile, I am afraid, you will have to debug this kind of problem by yourself.

##### yash 01/08/2021 08:48:56
understood, thanks!

##### Luiz Felipe 01/08/2021 08:51:44
ok... no problem hehe

##### Master.L 01/08/2021 09:21:18
Excuse me, how can i get hinge joint Velocity?


Should I calculate by differentiating the position value?.

Can I get it through the sensor?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/08/2021 09:25:36
add PositionSensors


and yes, you have to calculate it by the change


vel = (pos\_new - pos\_old) / (timestep * 1000)


[https://cyberbotics.com/doc/reference/positionsensor?tab-language=python](https://cyberbotics.com/doc/reference/positionsensor?tab-language=python)


timestep = getBasicTimeStep()

so in ms, that's why you have to multiply by 1000

##### Master.L 01/08/2021 09:55:38
`@Simon Steinmann` Thank you. Succeeded in receiving the position value!!.

But I control the webots through ros. Is there a way to get time from ros topic or service?

##### josiehughes 01/08/2021 09:58:12
Thanks!

##### Master.L 01/08/2021 10:08:13
`@Simon Steinmann` Thank you. I found.👍

##### Ragemor 01/08/2021 10:14:59
Excuse me is there any Webots project about multi robot coordination using leader follower approach? I want to implement leader follower algorithm in my project but i cant understand how can i write code about that. Which nodes and api functions(or sensors) should i use? Do you have docs about leader follower and multi robots in webots?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/08/2021 10:19:44
No, we don't have any documentation about leader follower and I am not aware of any project about it.


Maybe this one can help? [http://www.diegoantognini.com/projects/dis/](http://www.diegoantognini.com/projects/dis/)

##### Ragemor 01/08/2021 14:00:31
Thank you. If anyone have an idea or doc about leader follower can share with me?

##### adamadam333 01/08/2021 14:23:25
could someone give me a hand with rotating an EPUCK2 robot by 90 degrees? Tried absolutely everything and nothing worked, I am a complete beginner so don't know much


and there are no resources online for it


would be very helpful


I could pay for a call teaching me on how to do it

##### Maheed 01/08/2021 14:40:49
Hi everyone, Im a webots beginner at the moment. I have 8 yamor modules connected together to look like a snake and I really need some help to program some controllers in C to make it do something such as move towards an a certain colour or avoid obstacles. Can anyone help me with this?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/08/2021 14:44:57
are you dead set on c?

##### Maheed 01/08/2021 14:46:30
Yep

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/08/2021 14:47:47
Then I wont be able to help much. But my approach would be to create basic functionality as "move forward" "turn left" "turn right" etc. And then combine those into algorithms for obstacle avoidance etc.

##### Maheed 01/08/2021 15:27:37
Okay thanks


Is there anyone here good with C who can help me with this please?

##### linexc 01/08/2021 23:58:08
Hello everyone, I have a problem regarding Webots and NAO robot. I am trying to write a ROS server and client in order to make the NAO in Webots perform some gestures. But when I tried to simulate it, the error showed always like this. But I really don't know why the network address in already in use. If anybody can help me, I would be very grateful.



%figure
![error.png](https://cdn.discordapp.com/attachments/565154703139405824/797253462727327784/error.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/09/2021 08:26:26
`@linexc` looks like a network problem. I have no experience with this exact thing, but usually you would use localhost


which should be 127.0.0.1

##### linexc 01/09/2021 10:40:19
thank you. I will try it now.

##### Chernayaten 01/09/2021 19:36:59
I am messing around with the pen.wbt example and I found that the floorSize and the floorTileSize have to match (for example 1 1 for both or 4 4) . Can someone explain why?


Also if anyone can help me figuring out how to make the robot(still using the one in pen.wbt) turn 90 degree angles..atm I am using setRotate(getTargetPosition() +/- Rotate) where Rotate = 2*math.pi +0.4. Not sure why 0.4 works, but it seems ok in general, except if I use a big time\_step and sometimes when I left the simulation run a while.



I feel like I should be able to use 1.0/3.0, 4.0/3.0 and stuff like that to get different angles, but I can't make it work

##### DrakerDG 01/10/2021 00:37:29
Hello everyone! Playing with lights and lighting in Webots.  I have placed three line follower robots using the same controller, I still don't understand how it works.  Can someone tell me how it works?

[https://youtu.be/jMzKwcVs\_08](https://youtu.be/jMzKwcVs_08)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/10/2021 12:37:11
`@DrakerDG` Hey, what part exactly don't you understand?

##### DrakerDG 01/10/2021 12:41:40
`@Simon Steinmann` hello! I don't understand how by duplicating or tripling the same robot (copy & paste) they can use the same controller separately.  What I mean is that I just copied and pasted them without changing anything and it worked.  Each robot can follow the line without affecting the other.  In the end I only changed the color of the chassis and nothing else.  I see that it works but I don't understand how


I understood that each robot should have its own controller, but apparently it is not entirely necessary

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/10/2021 12:50:15
Ah okay, I understand the confusion. Each robot runs its own instance of that controller. In a separate process


does that make sense?

##### DrakerDG 01/10/2021 12:53:45
`@Simon Steinmann` yes, thank you 😁👍🏼


`@Simon Steinmann` like as running the same application several times


`@Simon Steinmann` If it were different robots if each one is configured with their controller, right?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/10/2021 12:56:16
yes


Usually a Controller is specific to one Robot, but you can have many instances of that Robot


you can take a look at your task-manager while running it. You should see the main Webots process, and then at least one per running controller.

##### DrakerDG 01/10/2021 12:58:23
`@Simon Steinmann` For example robots that play soccer use the same controller all, so

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/10/2021 12:58:30
I mostly use Python and often need similar functionality. So I have sub-modules or scripts, which I dont change and can use in different controllers by simply importing them

##### DrakerDG 01/10/2021 12:59:09
Ok

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/10/2021 13:00:01
If you have one type of robot player, then yes, it would make sense to use the same controller. Or at least a shared base-version for stuff like movement, sensors etc.


The roles for behavior, if they should differ, could be put on top of that


What Programming language do you use?

##### DrakerDG 01/10/2021 13:01:05
I will continue playing and I will try to make modules as you indicate.  It's very interesting


I am currently programming in C language, but I am learning in a course to start writing in Python


`@Simon Steinmann` My object is to use Webots to test robot designs, observe their operation, mechanical behavior, improve it and then physically build the robot

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/10/2021 13:14:31
Python is definitely easier and quicker imho

##### DrakerDG 01/10/2021 13:17:40
`@Simon Steinmann`  definitely yes, the syntax is very friendly.  In the little that I have advanced I really like how simple it is


`@Simon Steinmann` Thank you very much for guiding me

##### AdityaDutt 01/10/2021 17:14:58
I opened up a new world in webots, and the console is flooded with errors that say [javascript] ReferenceError: Can't find variable: webots (undefined:1)



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/797876153640812614/unknown.png)
%end


would anyone know what the issue is? Thanks

##### adiagr 01/10/2021 17:30:24
The error for sure pertains to the robot window's javascript file. I'm also experiencing the same error with Webots 2021 version. Went through the changelog but couldn't find anything pertaining to webots.js

##### AdityaDutt 01/10/2021 17:30:50
should I stick with 2020 instead?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/10/2021 17:37:41
The robot window doesn't rely on webots.js. Could you tell me which world are you trying to open?

##### AdityaDutt 01/10/2021 17:39:01
I am using the erebus challenge world. [https://github.com/Shadow149/Erebus](https://github.com/Shadow149/Erebus). Under erebus-aura>game>worlds>GeneratedWorld.wbt


well i went back to 2020a rev 1 and the error is no longer there

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/10/2021 21:00:41
`@AdityaDutt` , `@adiagr`  I have just checked and `MainSupervisorWindows.html` is not valid HTML file, so `webots` cannot be injected. The file content should be something like:

```html
<html>
<head>
    <script src="MainSupervisorWindow.js"></script>
</head>
<body>
...
</body>
</html>
```

##### adiagr 01/10/2021 21:03:17
Hey, in a robot window plugin, should the js file be explicitly included using the script tag?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/10/2021 22:03:29
Yes


You can check the e-puck example:

[https://github.com/cyberbotics/webots/blob/f6ebe9cf8ce64c30cf453655376a815deeb28744/projects/robots/gctronic/e-puck/plugins/robot\_windows/e-puck/e-puck.html#L76](https://github.com/cyberbotics/webots/blob/f6ebe9cf8ce64c30cf453655376a815deeb28744/projects/robots/gctronic/e-puck/plugins/robot_windows/e-puck/e-puck.html#L76)

##### Diego Rojas 01/10/2021 23:30:11
What are the steps of designing a model for webots simulation? What is the process of designing something in soldworks and making a PROTO from the mesh? This is likely a simple question, but I cannot seem to find a direct answer or demo.

##### DrakerDG 01/10/2021 23:40:04
`@Diego Rojas` check this play list: [https://youtube.com/playlist?list=PLbEU0vp\_OQkUwANRMUOM00SXybYQ4TXNF](https://youtube.com/playlist?list=PLbEU0vp_OQkUwANRMUOM00SXybYQ4TXNF)

##### Diego Rojas 01/11/2021 00:13:32
`@DrakerDG` Thanks!

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 09:15:52
Take a look at this first: [https://www.cyberbotics.com/doc/guide/tutorial-7-your-first-proto](https://www.cyberbotics.com/doc/guide/tutorial-7-your-first-proto)

In general, the simplest way in my opinion, would be to build up your model in webots, then export the robot as a .wbo


then you just need to add the .proto header (as described in the link)

##### JSK 01/11/2021 10:07:52
Hello Team Webots


i have a question related to webots


just visited the official website and then github and now i am here


Hi `@Darko Lukić`  how are you? remember me?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/11/2021 10:11:47
I am sorry, I cannot recognize your username 🙁

##### JSK 01/11/2021 10:12:22
i am Jamal Shams Khanzada, couple of days ago i had asked you a question on github and you helped me


and i am thankful to you


i am here for another problem to be solved.


can you still help me to solve it?


actually when i fetched gyro data recently, i started to make controller but it needs numpy

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/11/2021 10:14:09
I recall now! I am glad I was able to help.


Feel free to ask a question and I, or somebody from the team, or the community, will probably be able to help you

##### JSK 01/11/2021 10:18:34
so when i imported numpy it says "ModuleNotFoundError: No module named 'numpy'"


although i have installed python latest version


you can see here
%figure
![Screenshot_from_2021-01-11_15-20-43.png](https://cdn.discordapp.com/attachments/565154703139405824/798134541603831808/Screenshot_from_2021-01-11_15-20-43.png)
%end


this is the error in webots terminal
%figure
![Screenshot_from_2021-01-11_15-22-51.png](https://cdn.discordapp.com/attachments/565154703139405824/798134959138930698/Screenshot_from_2021-01-11_15-22-51.png)
%end

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/11/2021 10:23:46
You installed `numpy` for Python2, not Python3


You need to `pip3 install numpy`

##### JSK 01/11/2021 10:24:31
ok i am installing


but how can i know that `numpy` is for python2 or python 3?


installed it


rebooting is necessary or not?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/11/2021 10:26:11
No, just restart the controller


In general (not necessary), if you install it with `pip3` it is for Python3, if you install it with `pip` it is for Python2.

##### JSK 01/11/2021 10:27:33
ok


i restaretd the controller but the problem is still there

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/11/2021 10:30:08
Do you get an error if you call?

```py
python3 -c 'import numpy'
```

##### JSK 01/11/2021 10:30:28
let me check it

##### Stefania Pedrazzi [Cyberbotics] 01/11/2021 10:30:57
`@JSK` did you maybe install Webots from the snap?

##### JSK 01/11/2021 10:31:31
`@Stefania Pedrazzi`  yes i installed it from snap

##### Stefania Pedrazzi [Cyberbotics] 01/11/2021 10:32:02
Ok, then you should use extern controller:

[https://www.cyberbotics.com/doc/guide/installation-procedure#extern-controllers](https://www.cyberbotics.com/doc/guide/installation-procedure#extern-controllers)

##### JSK 01/11/2021 10:32:52
negative... i got no error
%figure
![pip3.png](https://cdn.discordapp.com/attachments/565154703139405824/798137348495245312/pip3.png)
%end

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/11/2021 10:34:13
If you installed Webots as `snap` it will not work. As `@Stefania Pedrazzi` suggested you should use external controller or install Webots as a Debian package.

##### JSK 01/11/2021 10:35:26
i checked that site. i have been reading it for two days but i was actually avoiding external controller. but now i think i have to switch here on external controller


i think i should go for Debian package


isn't it necessary that Debian should be installed on latest version of Ubuntu (20.04) i am using (18.04)


i have started downloading Debian Version.


and until it is downloaded i think i should reread the documentation `@Stefania Pedrazzi` mentioned me

##### Stefania Pedrazzi [Cyberbotics] 01/11/2021 10:42:32
There is a debian package version for Ubuntu 18.04: [https://github.com/cyberbotics/webots/releases/download/R2021a/webots-R2021a-x86-64\_ubuntu-18.04.tar.bz2](https://github.com/cyberbotics/webots/releases/download/R2021a/webots-R2021a-x86-64_ubuntu-18.04.tar.bz2)

##### JSK 01/11/2021 10:43:55
Thank you . I am so thankful to you. 🙂

##### Stefania Pedrazzi [Cyberbotics] 01/11/2021 10:44:16
You're welcome

##### JSK 01/11/2021 10:44:20
aaaa one last question please


after the file is downloaded, how to install it? i mean should i extract the `tar.bz2` extension file or something else?

##### Stefania Pedrazzi [Cyberbotics] 01/11/2021 10:45:58
Oops.. I linked to the tarball instead of the debian package

##### JSK 01/11/2021 10:46:00
please don't mind i am a novice

##### Stefania Pedrazzi [Cyberbotics] 01/11/2021 10:46:42
The tarball just needs to be extracted and then you can launch it from the directory where you extracted it


Here is the link to the debian package: [https://github.com/cyberbotics/webots/releases/download/R2021a/webots\_2021a\_amd64.deb](https://github.com/cyberbotics/webots/releases/download/R2021a/webots_2021a_amd64.deb)

Double clicking on it should open the Software Application window to install it

##### JSK 01/11/2021 10:47:34
alright that sounds good

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 10:48:29
`@JSK` You may also have to set environment variables, especially when using conda

##### JSK 01/11/2021 10:49:42
yeah i read about environment variables in the documentation that `@Stefania Pedrazzi`  mentioned

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 10:51:02
you can launch webots from a terminal. it will have the variables of that terminal

##### JSK 01/11/2021 10:51:44
to be honest i tried to set the variables but when i started to search `usr/local/webots` i was not able to find that and hence i came on this platform for help. i am trying this for 3 days

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 10:52:11
you can edit your .bashrc in your home folder to add variables

##### JSK 01/11/2021 10:53:05
what does "edit" mean.


?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 10:53:17
open it in a text editor and add the variables


[https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=linux&tab-language=python](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=linux&tab-language=python)


here is more detail

##### JSK 01/11/2021 10:54:51
alright

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 10:57:26
you can add this at the end of your .bashrc

`export WEBOTS\_HOME="/home/simon/webots"

export LD\_LIBRARY\_PATH="${LD\_LIBRARY\_PATH}:${WEBOTS\_HOME}/lib/controller"

export PYTHONPATH="${PYTHONPATH}:${WEBOTS\_HOME}/lib/controller/python38"

export PYTHONIOENCODING="UTF-8"`

keep in mind to change the first line to the correct Path of your installation, and change the python version to the correct one


`python3 --version`  to get your python3 version

##### JSK 01/11/2021 11:00:44
thanks let me do it

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 11:00:45
the .bashrc file gets sourced every time you open a new terminal. So you have to open a new terminal for it to take effect. Then you can launch webots from the terminal, and it will have those variables

##### JSK 01/11/2021 11:03:18
can you tell me how to access .bashrc file? i cant find it

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 11:03:28
it's hidden


ctrl + h to see hidden files, or `gedit ~/.bashrc` in the terminal

##### JSK 01/11/2021 11:04:05
found it


updated .bashrc as you mentioned `@Simon Steinmann` .

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 11:07:37
now you can open a new terminal and launch webots from there. the variables should be set


Instead of setting those variables in the .bashrc, you could set them in `sudo gedit /etc/profile`. That should set them system wide

##### JSK 01/11/2021 11:08:47
error still exists

##### Alessia Valle 01/11/2021 11:09:48
Hello everyone! I would like to write a controller that allows my robot to keep the lane. What sensors do you suggest me to use?

##### MartinG 01/11/2021 11:10:33
Use a camera sensor and look into canny edge detection and hough line transform


I don't know your programming language preference, but I strongly recommend you use python since OpenCV makes it a much easier task than it seems.

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/11/2021 11:11:27
Something like this?

[https://github.com/lukicdarkoo/webots-example-lane-follower](https://github.com/lukicdarkoo/webots-example-lane-follower)


It is not robust, but it is very simple

##### Alessia Valle 01/11/2021 11:16:30
Yes, but I have never used OpenCV


Are there any references on Cyberbotics website?

##### MartinG 01/11/2021 11:19:03
I used a really solid (imo) guide that has some hand holding but also allows for you to figure the problems out by yourself, you can find it here [https://towardsdatascience.com/deeppicar-part-1-102e03c83f2c](https://towardsdatascience.com/deeppicar-part-1-102e03c83f2c)


Parts 3 and 4 are what you'll probably find the most useful.


And it does put up a paywall after 3 visits to the site I believe, but a bit of incognito browsing never hurt anyone.


I have a question of my own, using reset simulation crashes the whole program. The world is quite complex, however even leaving it to run for a long while after hitting reset doesn't seem to help.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 11:23:12
Perhaps try removing things from the world one by one and hit reset after. Try to find out what is causing the crash

##### Alessia Valle 01/11/2021 11:25:21
Thank you!

##### MartinG 01/11/2021 11:26:05
I'm glad to help, I'll be around if you have some more specific questions.


Yeah, that makes sense. Thanks.


Anyone with some deeper knowledge of arctan2 and trigonometry in general want to help me out a bit?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 14:16:06
`@MartinG` whaz do you want to k ow?


I got a pretty good handle on trigonometry and linear algebra, especially in the context of robotics and positions / orientations

##### MartinG 01/11/2021 14:20:37
Awesome.


So I'm trying to do a GPS guided navigation for the Tesla in Webots.


I do this by taking 5 points along the path, then converting them into a lot of points using a bspline path program.


The problem is figuring out how to calculate the angle the car needs to turn using arctan2

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 14:24:26

%figure
![220px-Atan2definition.png](https://cdn.discordapp.com/attachments/565154703139405824/798195623940718622/220px-Atan2definition.png)
%end

##### MartinG 01/11/2021 14:24:42
I've seen that graphic many times ahahah.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 15:09:39
btw, this link can be very useful for checking calculations etc.

[https://www.andre-gaschler.com/rotationconverter/](https://www.andre-gaschler.com/rotationconverter/)

##### AdityaDutt 01/11/2021 16:42:55
Hi again! I am trying to create a robot class with the motors, position sensors, and other stuff included in it. I am using this code:

```C++
class myRobot {
  public:
    Robot *robot;
    Motor *left;
    Motor *right;
    PositionSensor *LEnc;
    PositionSensor *REnc;
    int timeStep;
};
```

It says that 'Robot' does not specify a type, and same with 'Motor' and 'PositionSensor'. I suspect this is because something in Robot.hpp is not public.


I'm sort of a noob with C++ and OOP, so I'm not really sure what the issue is

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/11/2021 16:44:57
[https://www.cyberbotics.com/doc/guide/controller-programming?tab-language=c++](https://www.cyberbotics.com/doc/guide/controller-programming?tab-language=c++)


this should give a bit of guidance


and perhaps this

[https://www.cyberbotics.com/doc/guide/cpp-java-python?tab-language=c++](https://www.cyberbotics.com/doc/guide/cpp-java-python?tab-language=c++)

##### AdityaDutt 01/11/2021 16:47:08
thanks!

##### Chernayaten 01/11/2021 21:40:04
Would you perhaps be able to help me with the following? I want my robot to make a 90 degree clockwise turn which. I use setPosition to increment/decrement and thus move/turn the robot. I use a position sensor to properly stagger my commands so they don't overlap.



With my subpar trigonometry skills I figured that to turn the robot 90 degrees I have to inc/dec the position by (math.pi * robot\_radius)/ (2*wheel\_radius) . My simulations say I am completely wrong. Can you help me out? I can show how I reached the formula if you want

##### AdityaDutt 01/11/2021 21:53:28
funny I was doing the same thing today. This is the formula I landed on, and it works:

```
theta(in radians) = 
(angle(in degrees) * robot_radius * pi)/(360 * wheel_radius)
```


so if your robot radius(something you sort of have to play with to get accurate results) is 32.5, and wheel radius is 9.5, then your setPosition should be(in radians) (90 * 32.5 * PI)/(360*9.5)


pretty sure you forgot to multiply your angle and it should be 360*wheel\_rad

##### Chernayaten 01/11/2021 22:01:55
What do you mean it is something that I have to play around to get accurate results? Why?


Your formula translates to my\_formula / 2 since 90/360 = 1/4. Unfortunately the turn my robot does is still wrong

##### AdityaDutt 01/11/2021 22:14:02
it's not actually supposed to be the robot radius, it's meant to be the distance between the centers of the wheels, which is usually smaller than the robot radius given on the webots website


what does it do then?

##### Chernayaten 01/11/2021 22:15:37
Ahh.. yeah, I agree. The center of my wheels is right on the edge of the robot, in which case the distance should be the robot radius

##### AdityaDutt 01/11/2021 22:16:01
if it is extremely far off then there's something wrong with the formula


but if instead of 90 degrees it goes 80 or 100 degrees, then just mess with the robot radius


basically what my formula does is the following(90 degrees as an example)

1. 90/360 =1/4 shows that you are going 1/4 of the circumference.

2. calculate circumference through robot\_diameter*pi, and multiply by 1/4(or whatever fraction you got from step 1). This gives you the distance that each wheel has to go in terms of cm.


then the radians that each wheel has to turn is distance/wheelRadius


putting all that together gives the final formula

##### Chernayaten 01/11/2021 22:22:44
1/4 gives me a ~43.6 turning angle, 1/2 gives me a ~45.2 angle and 1/1 (so a full circle supposedly) gives me ~62 angle

##### AdityaDutt 01/11/2021 22:23:01
ooh


yeah


that's not right


this is what I have:

##### Chernayaten 01/11/2021 22:23:29
I do agree with your reasoning on how it works though

##### AdityaDutt 01/11/2021 22:23:34
```
(angle*WHEELBASE*PI)/(360*WHEELRAD)
```


WHEELBASE is the robot diameter


try that


and angle is in degrees

##### Chernayaten 01/11/2021 22:26:14
This is what I am trying. btw my result was in radians, I just converted it to angle so that it makes more sense. At least for me, angles feel far more natural when it comes to visualizing them

##### AdityaDutt 01/11/2021 22:26:15
*wheelbase is robot \_diameter\_, not radius


in case that was a matter of confusion


ok what's your wheelbase(\_diameter\_) and wheel radius

##### Chernayaten 01/11/2021 22:30:20
I'm pretty sure my wheelbase is the same as my robot diameter which is 0.9 and my wheel radius is 0.025.


My wheels are located on 0.045 to the left and the right of my robot, wheelbase is calculated as the distance from the center of the wheels which makes that 0.09


I'm using the robot from the pen.wbt world in the Devices Tour


I'll try playing around with the values to see if I can make it work. Thanks for taking the time, at least I know that my logic is not too far off

##### Luiz Felipe 01/12/2021 07:11:18
Just reporting. I think I found a bug (not sure), but receiving the message from the simulation server, when starting the Stream class another bar '/' is inserted in line 6 and the link becomes 'localhost:2001//mjpeg' ([https://github.com/cyberbotics/webots/blob/f6ebe9cf8ce64c30cf453655376a815deeb28744/resources/web/wwi/stream.js#L6](https://github.com/cyberbotics/webots/blob/f6ebe9cf8ce64c30cf453655376a815deeb28744/resources/web/wwi/stream.js#L6)) . For the x3d option the problem does not happen weirdly.


I am solving that by changing the message sent from the simulation server. I will check if it will also work with 'x3d' option or if using this solution only solves the 'mjpeg' but starts the error when using 'x3d'.

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/12/2021 07:19:17
Thank you for this contribution! Once you have implemented and tested it, please open a new PR to submit your change, so that it will be integrated into Webots).

##### yash 01/12/2021 07:39:03
How can I extract the position moved by a manipulator link ? I have used a hinge joint along with position sensor .

So when a manipulator link hits a solid it stops, at this moment I want to know the position of the link .?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/12/2021 07:51:50
Can't you use the position sensor for that?

##### yash 01/12/2021 08:11:01
> Can't you use the position sensor for that?

`@Olivier Michel`  ye it’s done ! Was thinking in a wrong direction... thanks !

##### Luiz Felipe 01/12/2021 08:37:38
If I git clone the webots repository the dependencies are not downloaded right? How can I download also with the dependencies? (I cloned but the /wwi/dependencies folder does not exist)


I guess I should use --recurse-submodules... will test it!

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/12/2021 08:45:02
Yes.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/12/2021 08:45:23
Use the build from source instructions on github


They give you a step for step process, including all dependencies

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/12/2021 08:46:04
Please follow strictly the instructions at [https://github.com/cyberbotics/webots/wiki/Linux-installation](https://github.com/cyberbotics/webots/wiki/Linux-installation) (assuming you are on Linux)

##### Luiz Felipe 01/12/2021 08:49:05
Thanks 🙂 doing that now

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/12/2021 08:51:44
`@AdityaDutt` `@Chernayaten`  [http://www.hmc.edu/lair/ARW/ARW-Lecture01-Odometry.pdf](http://www.hmc.edu/lair/ARW/ARW-Lecture01-Odometry.pdf) Take a look at this lecture presentation. Especially slide 23


delta-Theta is the angle change of the robot (you want 90° (pi/2)

delta-s  is the distance traveled by the right / left wheel
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/798475224294096907/unknown.png)
%end


2L is the distance between wheels. (as a note here, yesterday I found more accurate results using the outside edge to outside edge distance distance of the BoundingObject of the wheels, not the wheel center)


So solving for the difference in angles/position of your wheels, the formula should be like this:

`delta_pos = delta_theta * wheelBase / wheelRadius`


Now you can put in any theta angle in radians and get the delta\_pos


delta\_pos is how much more you have to turn one wheel over the other


I recommend staying with radians all the times. The only exception maybe being when printing a message. Then just multiply an angle by 180/pi  to get degrees

##### Luiz Felipe 01/12/2021 09:22:18
I followed the page, re-build webots, also installed the optional dependencies, but still nothing on the wwi/dependencies folder... should I download the dependencies in that folder manually?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/12/2021 09:23:19
what are you trying to achieve? Webots built successfully?

##### Luiz Felipe 01/12/2021 09:24:25
yups.. webots built successfully, i can open v2021a


i am trying to check a possible bug in the multimedia\_client.js file

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/12/2021 09:24:56
okay... and what is the issue?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/12/2021 09:27:01
`@Luiz Felipe` Can you check whether you have node/npm installed, `npm --version`. If there is a missing dependency `make` skips compiling the corresponding part. Node should be included in the optional dependencies, but just to make sure whether it is properly installed

##### Luiz Felipe 01/12/2021 09:28:28
Probably that is the reason `@Darko Lukić` (npm is installed correctly but probably make skipped that when i ran the command)


does make clear works to re-build one more time?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/12/2021 09:29:20
Yes, `make clean` works

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/12/2021 09:29:39
`make clean`

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/12/2021 09:29:59
Correct, sorry

##### Luiz Felipe 01/12/2021 09:30:15
Thanks =), I will re-build

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/12/2021 09:30:18
Or even more powerful `make cleanse` (will remove the dependencies).

##### Luiz Felipe 01/12/2021 09:31:38
For the web part, is there a list of bugs or features that need help? (I dont know much of javascript but I feel that part is really breakthrough work in  webots)


Now I got the dependencies `@Darko Lukić` . Thank you very much. I got an error during make regarding the npm but all the dependencies seems to be in the /dependencies folder.

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/12/2021 09:44:24
What is the error?

##### Luiz Felipe 01/12/2021 09:47:20

%figure
![Screenshot_from_2021-01-12_18-40-36.png](https://cdn.discordapp.com/attachments/565154703139405824/798488275685605396/Screenshot_from_2021-01-12_18-40-36.png)
%end


but it works

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/12/2021 09:51:21
Maybe to clean the npm cache `npm cache clean --force` and repeat the compilation. To clean the project you can also use `git clean -dfx`.

##### Luiz Felipe 01/12/2021 09:56:25
Ok, tomorrow I will try to compile again. I guess the bug is in this line: [https://github.com/cyberbotics/webots/blob/f6ebe9cf8ce64c30cf453655376a815deeb28744/resources/web/wwi/multimedia\_client.js#L77](https://github.com/cyberbotics/webots/blob/f6ebe9cf8ce64c30cf453655376a815deeb28744/resources/web/wwi/multimedia_client.js#L77) ... I will try to find a solution to send a PR for that


(giving the right url it works for the mjpeg also ;))

##### Apashe 01/12/2021 10:49:21
Hello everyone! recently started working with webots and I wanted to know how to get the current coordinates of the robot and point it to some point so that it gets there (I'm currently working with a robot "iRobot's Create")

how can this be done and is there an example?

now I want the robot to be able to drive from one current coordinate to another (without obstacles) on Rectangle Arena

##### Chernayaten 01/12/2021 10:50:09
GPS can get you current coordinates


Assuming you know the direction of your robot you can easily calculate the distance and the angle to your target

##### Apashe 01/12/2021 11:01:14
there are examples of how to use GPS

##### Chernayaten 01/12/2021 11:03:10
Yes, there's a gps.wbt file under the devices section.



GPS is simple to use, you can also check out its section oncthe webots site to see all the methods in your preferred language

##### owongcho 01/12/2021 16:39:37
hello! what type of simulation Webots generates? For example, is the type of simulation an emulation, discrete event simulation, or something else?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/12/2021 16:41:49
I would say emulation. The time is simulated and there is a fixed time step.

##### owongcho 01/12/2021 17:43:03
I see. thank you `@Olivier Michel`

##### Chernayaten 01/12/2021 19:14:50
Thanks for the information. I haven't looked at the lecture yet, but I will do so soon. I actually came to the same result as you, the outside distance seems to work better for some reason.



I also suspect that the initial anchor positions from the given example are wrong, unless I am mistaken. From Tutorial 6 on the 4 wheeled robot, the anchor field of the HingeJointParameters node should have the same values as the Solid translation one, but the robot of the pen.wbt world is missing the x values.



I'll review your information and hopefully get a good result. Thanks again (:

##### AdityaDutt 01/13/2021 00:49:06
I have the pioneer 3dx robot in between two walls. What would be a good way to ensure that the robot is straight/aligned with the wall? I tried checking two side sensors and making sure they return very similar values, or adjusting until they do, but the distance sensors are very noisy and not consistent much.



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/798715662129823805/unknown.png)
%end


How do I make sure the robot is straight?

##### TerryWr1st 01/13/2021 01:37:07
Have you specified a noise value in the sensor lookup table? Otherwise they should be accurate. 



Also, when your robot is in the above orientation, the distance sensor rays will be sent out on an angle (not perpendicular to walls as desired), which will increase/decrease your sensor values. So you would need to know your robot orientation for this approach as well

##### AdityaDutt 01/13/2021 01:37:32
wdym specify noise value?


I can pick how little/much noise I want?


and oh yeah I didn't think about the angle thing thanks

##### TerryWr1st 01/13/2021 01:41:31
[https://cyberbotics.com/doc/reference/distancesensor](https://cyberbotics.com/doc/reference/distancesensor)



Scroll down to the lookup table section

##### AdityaDutt 01/13/2021 01:42:37
wait so I can set my own noise stuff?


in code?


But if some competition is using webots that i'm participating in, what stops me from just setting 0 noise


how do I change the lookuptable values

##### TerryWr1st 01/13/2021 02:06:16
If the competition hasn't mentioned a restriction on sensor accuracy then dont set a noise. 



From the documentation, first column is sensor input, second column is what you want the input to represent, third column is the noise value. 



You can change these values in the scene tree (expand the fields of your distance sensor), or open the wbt file with a text editor

##### AdityaDutt 01/13/2021 03:05:56
got it thanks

##### Wasabi Fan 01/13/2021 04:52:17
I have a small projectile (on the order of 1cm diameter in the real world) that I'd like to simulate at moderate-to-fast speeds (tens of m/s) and detect contact with a particular square surface on a target robot. This is currently using a `TouchSensor`. Unfortunately, even with a projectile a few times larger and speeds a fraction of real-world, the simulation time step is too large to consistently (or, at higher speeds, essentially at all) have a time slice in which the projectile is actually intersecting the target. In other words, the collision is never detected. This is true even if I up the simulation speed to thousands of Hz, which isn't practical anyway. As far as I can gather, TouchSensor collision detection is purely based on intersection after a discrete time step.



One solution which seems technically possible but perhaps not practical is some form of intra-step interpolation (e.g., a ray intersection check which approximates my projectile as a point -- my understanding is that ODE supports this in some form).  What are others' thoughts here? And is there actually a practical means of implementation for the above in my Webots sim, with or (ideally) without a physics plugin? Alternately, this seems to be approaching the scale at which doing a purely mathematical hit-test without ever simulating the projectile may make more sense -- however, I \_do\_  need to simulate the time-of-flight, and I'm not sure what API would be sensible for computing those collisions, if any.

##### Luiz Felipe 01/13/2021 04:56:55
`@Wasabi Fan` , does decreasing the time step does not work even implementing a collision detector using a physics plugin following: [https://cyberbotics.com/doc/reference/introduction-of-the-physics-plugins](https://cyberbotics.com/doc/reference/introduction-of-the-physics-plugins) ?

##### Wasabi Fan 01/13/2021 05:01:28
I tried that once with a "normal" time-step (~60Hz) and it did not seem to change the results. Setting up the toolchain to compile physics plugins is a bit of extra work (which I preferred to not introduce into my general setup process), which is why I haven't done a lot of experimenting with it, but it's not out of the question. However, it's not clear to me what that sample plugin is intended to do -- is there an expectation that the joint-attachment collision handling is better in a scenario like this?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/13/2021 07:11:38
A possible solution (without a physics plugin) would be to turn the bullet into a Robot with the supervisor field set to TRUE. Then, add a distance sensor to the bullet looking backwards. At each time step, you should get the current position of the bullet and compute the vector between the current position and the previous position. You will use this vector to change the orientation of the distance sensor so that it looks exactly at the previous position and make a sensor measurement in the same time step. If the distance measured is smaller than the distance between the current and previous position, then a collision occurred and you should set the position of the bullet at the place of collision (which can easily be computed along the distance sensor ray).

##### Wasabi Fan 01/13/2021 07:16:36
Oh wow, that's very inventive and \_sounds\_ workable. I think there are some details I'd need to work out but I shall try that in the next few days and report back if I run into issues. Thanks!

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/13/2021 07:17:28
I would truly appreciate your feedback on this.

##### Moha 01/13/2021 08:38:00
hi guys

when I run external controller for nao\_demo\_python using pycharm after mentioned configuration in related link, I see according errors in pycharm, anybody knows the solution ?



C:\Users\MoHa\AppData\Local\Programs\Python\Python39\python.exe C:/Users/MoHa/Desktop/0/nao/controllers/nao\_demo\_python/nao\_demo\_python.py

Traceback (most recent call last):

  File "C:\Users\MoHa\Desktop\0\nao\controllers\nao\_demo\_python\nao\_demo\_python.py", line 18, in <module>

    from controller import Robot, Keyboard, Motion

  File "C:\Users\MoHa\AppData\Local\Programs\Webots\lib\controller\python39\controller.py", line 11, in <module>

    webots\_home = os.environ['WEBOTS\_HOME']

  File "C:\Users\MoHa\AppData\Local\Programs\Python\Python39\lib\os.py", line 679, in \_\_getitem\_\_

    raise KeyError(key) from None

KeyError: 'WEBOTS\_HOME'



Process finished with exit code 1

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/13/2021 08:40:21
[https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-language=python&tab-os=linux](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-language=python&tab-os=linux)


you have to set your environment variablres

##### Moha 01/13/2021 08:41:58
in pycharm or windows environment variables ?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/13/2021 08:42:39
usually windows



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/798834582328967208/unknown.png)
%end


like this


also add the Path variables

##### Moha 01/13/2021 08:44:18
👍

##### yash 01/13/2021 11:00:50
Hi !  I want to use this supervisor function -  def addForce(self, force, relative). Therefore what exactly should I specify in the relative argument .?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/13/2021 11:23:34
You're right, the documentation is not 100% clear here. When you look at the C or C++ code, you see the type is BOOL


so I'd assume setting it to True = force relative to Solid, False = force relative to world


`@Olivier Michel` We should update the documentation

##### yash 01/13/2021 11:26:15
okay , let me try that.

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/13/2021 11:28:03
It seems clear to me: [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_add\_force](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_add_force)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/13/2021 11:28:31
when looking at the Python code, it is not clear that it is boolean

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/13/2021 11:28:45
Feel free to suggest some change here: [https://github.com/cyberbotics/webots/edit/released/docs/reference/supervisor.md](https://github.com/cyberbotics/webots/edit/released/docs/reference/supervisor.md)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/13/2021 11:28:56
already at it 😉


[https://github.com/cyberbotics/webots/pull/2646](https://github.com/cyberbotics/webots/pull/2646)

##### Chernayaten 01/13/2021 11:34:43
From Tutorial 6 on the 4 wheeled robot, the anchor field of the HingeJointParameters node should have the same values as the Solid translation one. Is this correct?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/13/2021 11:37:19
Reviewed.


Yes.

##### Chernayaten 01/13/2021 11:41:13
There's a discrepancy in the pen.wbt device example

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/13/2021 11:43:27
What is the problem?

##### Chernayaten 01/13/2021 11:44:30
The anchor field of the HingeJointParameters node is 0 0.025 0 whereas the Solid node (for the wheel) is -0.045 0.025 0 (and 0.045 for the other wheel)


This is in the Pen device example

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/13/2021 11:49:28
This is correct, but is it not essential as the anchor may be anywhere on the rotation axis. However, you are right it is better to have it as close as possible to the rotating part (the wheel in this case) for stability reasons. May I let you propose a patch here: [https://github.com/cyberbotics/webots/edit/master/projects/samples/devices/worlds/pen.wbt](https://github.com/cyberbotics/webots/edit/master/projects/samples/devices/worlds/pen.wbt) ?

##### Chernayaten 01/13/2021 11:53:48
Done

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/13/2021 11:55:40
Thank you. I approved it. You should be able to merge it once the CI tests are complete.

##### Chernayaten 01/13/2021 13:00:44
" Only those with write access to this repository can merge pull requests. "

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/13/2021 13:06:56
OK, I merged it. Thank you.


I sent you a GitHub invitation to become "Committer" on this repo with write access.

##### AngelAyala 01/14/2021 12:53:23
Hi, I'm currently using a Reinforcement Learning algorithm with webots, and therefore it ia necessary to reset the simulation to its initiql state. A strange thing happens sometimes whe I'm testing the algorithm but webots start to use a huge memory amount reaching 5GB and it increases when the simulation is reset. I'm currently running three robots nodes and one is the supervisor as the agent interface, anyone else had been experiencing this or maybe you have any idea why it is happening?


On robot includes uses two display nodeeO


The other one is controlling the Mavic drone

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/14/2021 12:57:13
Are you sure that Webots uses all that memory and not the controllers?


It would be great to isolate the issue and report it:

[https://github.com/cyberbotics/webots/issues/new?assignees=&labels=&template=bug\_report.md](https://github.com/cyberbotics/webots/issues/new?assignees=&labels=&template=bug_report.md)

##### AngelAyala 01/14/2021 12:58:49
In the system monitor it appears assigned to the webots - bin process


Any controller is executed isolated from main process right?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/14/2021 12:59:18
Yes

##### AngelAyala 01/14/2021 13:01:13
Ok I will poste there

##### Kirlin 01/14/2021 19:44:08
Hi, can someone help me about boundingObject Box ? I'm using a Free 3D Butter Stick model in a simulation (that will be part of a robotics immersion week in college), but the mesh has probably some internal conflicts, so if I use it as the boundingObject Box the Solid will actually tilt out of the screen.

My solution is use a box shape as the bounding box, since I dont need the object to be precisely cut. Anyway, when I put the Box as the boundingObject Box, it wont go to the center of the object, and, no matter how I change the dimensions of the Box, it wont fit the object as wished. So, here is my question: Can I move the "center" (in relation to the object)  of a boundingObject Box ?


here is the pic of the uncentered boundingObject Box
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/799363474299420732/unknown.png)
%end


PS: I'm on Ubuntu

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/14/2021 19:55:04
You can add the `Box` node in a `Transform` node.

##### Kirlin 01/14/2021 19:56:45
Can you describe more how I do this ? I've never used a Transform node

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/14/2021 19:59:54
It should be `boundingObject > Transform > children > Box`



[https://cyberbotics.com/doc/reference/transform](https://cyberbotics.com/doc/reference/transform)

##### Kirlin 01/14/2021 20:06:42
Wow, this is so useful, helped me pretty out, thanks so much!!

##### Wasabi Fan 01/15/2021 02:32:14
I'm taking a look at this in more depth, and now realize one part of my desired functionality is that I know \_what\_ the projectile hit. There is a particular surface on the robot (at the moment, I'm enclosing it in a TouchSensor boundary) which is being aimed at, and other parts of the robot (or elements of the surroundings) \_aren't\_ something I want to detect. In fact, at these speeds, there's some risk of going \_through\_ the robot and seeing the other target on the back of that robot.



I think this could be solved if there is a way to know what object the distance sensor is seeing -- is this a possibility? One option we see is to add a `Recognition` on a camera, with small FOV, pointing in the same direction as the distance sensor, and use that to disambiguate. I think it \_should\_ work, minus approximation error for the bounding boxes and any perf hit. Is there a more sensible way?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/15/2021 08:19:14
Yes, this should work but it looks a bit overkill. Instead of this, you could add `TouchSensor` nodes in the walls and when you move back the bullet into the wall, this touch sensor would be activated so that you know which wall was hit.

##### MartinG 01/15/2021 08:37:13
`@Wasabi Fan` I'd recommend taking a more mathematical approach to this, it would execute quite a bit faster than using all kinds of sensors and what not. If you can get the current position of both the projectile and your robot and you know the size of both of them, it's as easy as a couple if conditions to check for a collision.


There's a ton of information on collision detection online, however if you have any specific issue, I'd be glad to help.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 08:58:34
Perhaps try running it without the displays. That is a lot of memory. Sounds to me like a lot of visual data is cumulatively buffered. You could also share your project (you can pm me too) and I'll have a look.


Oh and I have some experience with RL and Webots too, just fyi 🙂

##### yash 01/15/2021 09:02:55
this happens when I use the function---  addTorque()
%figure
![Screenshot_from_2021-01-15_14-31-59.png](https://cdn.discordapp.com/attachments/565154703139405824/799564262326272000/Screenshot_from_2021-01-15_14-31-59.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 09:08:49
In the Webots source code there is CCD functionality (Continuous collision detection). This would probably solve this problem. `@Olivier Michel` Is it possible to use that collision detection method? Either through a plugin or custom compile?

##### MartinG 01/15/2021 09:13:26
Where can I find it? I'd like to look through it.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 09:14:13
[https://github.com/cyberbotics/webots/tree/master/src/ode](https://github.com/cyberbotics/webots/tree/master/src/ode)


libccd is the library


[https://github.com/danfis/libccd](https://github.com/danfis/libccd)

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/15/2021 09:14:35
This might be possible but would certainly require a significant development effort.

##### MartinG 01/15/2021 09:14:54
Thanks.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 09:17:42
I suspected as much. I hope something along these lines is somewhere on the roadmap. This is one of the few areas, where Webots is behind its competition: The low level control of physic engine settings and choice of different collision detection methods.


But I understand that it is not the highest priority and a lot of work

##### DrakerDG 01/15/2021 09:27:45
Hey, I tried to install webots on ubuntu but it tells me that it can't download the .deb file, apparently it can't find it. Does anyone know how to solve this problem?
%figure
![Screenshot_from_2021-01-15_03-23-15.png](https://cdn.discordapp.com/attachments/565154703139405824/799570511645376522/Screenshot_from_2021-01-15_03-23-15.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 09:30:02
what command did you use?


did you follow these steps? [https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt)

##### DrakerDG 01/15/2021 09:32:55
Yes:



wget -qO- [https://cyberbotics.com/Cyberbotics.asc](https://cyberbotics.com/Cyberbotics.asc) | sudo apt-key add -

sudo apt-add-repository 'deb [https://cyberbotics.com/debian/](https://cyberbotics.com/debian/) binary-amd64/'

sudo apt-get update

sudo apt-get install webots

##### mjc87 01/15/2021 09:34:28
`@DrakerDG` I noticed that there is a mistake in the install script it should be releases/download/R2021a/webots\_2021a\_amd64.deb I think

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 09:34:48
maybe download it manually. The version just changed to 2021. **[https://github.com/cyberbotics/webots/releases/tag/R2021a](https://github.com/cyberbotics/webots/releases/tag/R2021a)**


`@Olivier Michel` I think you should take a look and make sure the script is correct

##### DrakerDG 01/15/2021 09:35:43
Ok, I will try

##### MartinG 01/15/2021 09:36:28
What's the exact command you entered to download it? Can you copy it from your console?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 09:36:42
he did

##### MartinG 01/15/2021 09:36:53
Oh, sorry I didn't see.

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/15/2021 09:48:37
Which script are you referring to?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 09:52:04
I dont know how apt works, but It looks for the wrong file. Read the last 4 lines of his console screenshot

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/15/2021 09:53:27
Yes, I saw that, but there is no script about it.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 09:53:57
what could cause this issue?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/15/2021 09:54:10
I don't know. I am investigating...

##### mjc87 01/15/2021 09:54:45
Yes, thats what I was referring, I also have very little knowledge of apt

##### yash 01/15/2021 09:55:35
I am facing an issue where my controller is crashing when I use the supervisor function - addTorque()
%figure
![Screenshot_from_2021-01-15_14-31-59.png](https://cdn.discordapp.com/attachments/565154703139405824/799577517596868608/Screenshot_from_2021-01-15_14-31-59.png)
%end

##### MartinG 01/15/2021 09:56:06
I think seeing your code and world would be helpful to get a better idea of what might be wrong.

##### yash 01/15/2021 09:56:38
sure



> **Attachment**: [force\_control.py](https://cdn.discordapp.com/attachments/565154703139405824/799578002249089053/force_control.py)


this is the controller code

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/15/2021 09:58:21
Are you on Ubuntu 20.04?

##### MartinG 01/15/2021 10:00:58
I think that the problem is in your last line of code, f=Node.addForce(hf,force,relative=False)

##### yash 01/15/2021 10:01:15
yes but how to rectify it

##### MartinG 01/15/2021 10:01:22
You're calling addForce from the Node object instead of on the node that you want it to be executed on.


actually no


try changing f to hf


I saw that you defined the foot as hf before the while loop


That is most likely your problem.

##### yash 01/15/2021 10:04:06
so should I define hf=robot.getFromDef("human\_foot")   in the while loop ?

##### MartinG 01/15/2021 10:04:31
No, in your last line just put an h before f


hf=Node.addForce(hf,force,relative=False)


I’m not sure that will work, but f doesn’t exist prior to that so it’s a likely cause.

##### yash 01/15/2021 10:05:46
its the same issue still, didnt work

##### MartinG 01/15/2021 10:06:05
Okay, give me five minutes.


I'll message you privately to not crowd the chat here.

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/15/2021 10:38:38
I believe the problem is now fixed (it was due to a misconfiguration of our debian server). Can you try again and report and success or failure?

##### DrakerDG 01/15/2021 10:46:47
`@Olivier Michel` Ok, at the moment you are downloading the binary file

##### mjc87 01/15/2021 10:46:57
`@Simon Steinmann` I've seen you've done some domain randomisation in the past, is it possible to introduce randomness to world descriptions so that blocks appear in a different place each time a simulation is started?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/15/2021 10:56:24
Great, let me know if that finally works.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 10:57:59
`@mjc87` [https://drive.google.com/file/d/1Z2332YFU1Um1NCwx3PoTcOE2jdu6QUxP/view?usp=sharing](https://drive.google.com/file/d/1Z2332YFU1Um1NCwx3PoTcOE2jdu6QUxP/view?usp=sharing) I did this a while back. Domain Randomization to create cv training dataset

##### mjc87 01/15/2021 10:59:55
Thanks, please could you give me a pointer how you implemented it? I'm trying to generate a student robot challenge where some 'targets' are randomly placed

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 11:00:40
let me see if I can find that project


`@mjc87` [https://drive.google.com/file/d/1vp4ZzyA44JpcGbI5NEUsCNZ7eRFrYd4g/view?usp=sharing](https://drive.google.com/file/d/1vp4ZzyA44JpcGbI5NEUsCNZ7eRFrYd4g/view?usp=sharing) take a look at this


the large size is due to the textures

##### DrakerDG 01/15/2021 12:22:32
It is works 😆 thank you)



%figure
![Screenshot_from_2021-01-15_05-53-32.png](https://cdn.discordapp.com/attachments/565154703139405824/799614996047986708/Screenshot_from_2021-01-15_05-53-32.png)
%end



%figure
![Screenshot_from_2021-01-15_05-55-28.png](https://cdn.discordapp.com/attachments/565154703139405824/799615195914960906/Screenshot_from_2021-01-15_05-55-28.png)
%end

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/15/2021 13:03:56
Nice! Thank you for the feedback.

##### R\_ 01/15/2021 13:57:14
[https://stackoverflow.com/questions/64875476/reinforcement-learning-webots-simulator](https://stackoverflow.com/questions/64875476/reinforcement-learning-webots-simulator)

##### valeria 01/15/2021 14:49:49
Hello, can anyone help me out? This is my very first time using webots and I'm using VS to write my Java code, however when I saved the file some "import errors" appeared. I have installed JDK and I use macOS then I don't understand why I have this. I also tried to clean the workspace but didn't work. Does anyone if I need to install something else in order to make it work?
%figure
![Screenshot_2021-01-15_at_15.48.25.png](https://cdn.discordapp.com/attachments/565154703139405824/799651562208755773/Screenshot_2021-01-15_at_15.48.25.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/15/2021 18:27:02
`@valeria` [https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=macos&tab-language=java](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=macos&tab-language=java) This may help. You have to set up some environment variables when using extern controllers (your own IDE like VC)

##### devindu 01/16/2021 15:58:04
I am trying to create a keyboard controller to my robot but it says INFO: my\_controller3: Starting controller: python.exe -u my\_controller3.py

WARNING: my\_controller3: The process crashed some time after starting successfully.

WARNING: 'my\_controller3' controller crashed.
%figure
![Cnt1.JPG](https://cdn.discordapp.com/attachments/565154703139405824/800031127715708938/Cnt1.JPG)
%end



%figure
![cnt2.JPG](https://cdn.discordapp.com/attachments/565154703139405824/800031139582574592/cnt2.JPG)
%end


Can anyone help me

##### Apashe 01/16/2021 17:59:25
Hello, What is the diameter of the wheels at iRobot's Create??

##### Chernayaten 01/16/2021 18:56:12
" Note [Python]: In C++, Python and Java the keyboard functions are in a dedicated class called Keyboard. In order to get the Keyboard instance, you should call the getKeyboard function of the Robot class. "


This is what I read about the Keyboard class, whereas in your code you do kb = Keyboard()


I believe it is 0.062

##### Apashe 01/17/2021 14:04:46
in general, where can I see all the data on the robot iRobot's Create as on the E-puck robot?

Can I have a link?

##### Chernayaten 01/17/2021 15:22:10
Robots are located under \Webots\projects\robots . From there you can navigate to the robot you can and find the protos folder (if there is one) and find the proto file you want. So for example the Create robot proto file for me is located under "C:\Program Files\Webots\projects\robots\irobot\create\protos" and the e-puck one under " C:\Program Files\Webots\projects\robots\gctronic\e-puck\protos" 



You can read some more on the PROTO mechanism here: [https://cyberbotics.com/doc/reference/proto](https://cyberbotics.com/doc/reference/proto)


`@Apashe`

##### Tahir [Moderator] 01/18/2021 11:00:32
Hello,

How can we access individual value inside the SFVEC3F in proto files?


For example:

position IS frontRightLeg.x

Or

position IS frontRightLeg[0]


These both gave errors

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/18/2021 11:02:58
You cannot based on the `IS` keyword. Instead, you should rely on Lua scripting to extract the individual components of a `SFVec3f`.

##### Tahir [Moderator] 01/18/2021 11:04:34
OK than instead 

position IS frontRightLegHip


Where frontRightLegHip is SFFLOAT


This should work


?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/18/2021 11:08:55
Yes, see an example of this page: [https://cyberbotics.com/doc/reference/procedural-proto-nodes](https://cyberbotics.com/doc/reference/procedural-proto-nodes)


In particular `fields.stepSize.value.x` to extract X component from `SFVec2f stepSize`.

##### Tahir [Moderator] 01/18/2021 11:19:30
OK thanks alot

##### ahforoughi 01/18/2021 11:58:50
Hi guys, How can we use openai gym in this platform? i want to simulate and train a robotic arm

##### MartinG 01/18/2021 11:59:48
I'll go ahead and guess that the Supervisor functionality is going to be your best friend for that.

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/18/2021 13:04:02
You may find the following project relevant:

[https://github.com/aidudezzz/deepbots](https://github.com/aidudezzz/deepbots)

##### John520 01/18/2021 17:11:34
Hi guys, would it be possible to create a crop field, for example a canola field, and a tractor can go inside the field to harvest? Thank you very much for your help!

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/18/2021 17:13:46
Probably depends on how visually accurate you want it. The simplest method I can imagine, is using a texture of the field, and then paining with the Pen node on top of that picture, where the tractor has harvested


[https://cyberbotics.com/doc/reference/pen](https://cyberbotics.com/doc/reference/pen)


that would probably have the least performance impact

##### John520 01/18/2021 17:26:36
Thank you very much for your help, `@Simon Steinmann` ! Except for creating a field of canola, I would also like to use a Lidar sensor to measure the height of the canola so that the harvesting header in front of the tractor can adjust its height accordingly for better harvesting. Is it possible to do so? Thanks a lot.


In other words, could the canola field be created with different heights?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/18/2021 17:31:01
[https://cyberbotics.com/doc/reference/elevationgrid?tab-language=python](https://cyberbotics.com/doc/reference/elevationgrid?tab-language=python)


perhaps this


I dont know how you would harvest it though

##### John520 01/18/2021 17:40:51
Thank you, `@Simon Steinmann`. Can the tractor go inside the field created by the elevationGrid? Can a lidar sensor detect the field?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/18/2021 18:45:07
it should be possible. You can disable collision and only have the visual Grid

##### John520 01/18/2021 20:56:29
Thank you `@Simon Steinmann`. Sounds great! I am going to try it out. Would it be possible to model a simple header (like a rectangular shape) and attach it to the tractor?

##### Homeralf 01/18/2021 23:56:27
Hi guys,



I would like to thank the team for this great simulator, the API and documentation are so great. 

I'm a hobbyist trying to develop a muscle-based robot and I've encountered a couple of issues.



I want to carry a simulation from a single python process.

As far as I know, each robot's external controller must be contained in its own process, so I spawn multiple processes from the main process (with the corresponding environment variables).

The actual problem comes when sharing data among processes. The objects I try to share can't be pickled (serialized) because they contain SwigPyObjects. I can solve this but is cumbersome and not idiomatic.



So I have a couple of questions I hope you can help me with:

Is there a way to run controllers without spawning additional processes?

I guess the serialization problem can't be solved because of Swig, however, I think an update on the python API is on the roadmap. It is possible this may solve this issue?





Thanks in advance!

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 08:19:41
`@Homeralf` Taking a step back, why do you want it to run from the same process? What data do you want to share?


In general [https://docs.python.org/3/library/multiprocessing.html](https://docs.python.org/3/library/multiprocessing.html) can work really well for parallel processing and data sharing. Then there is middlewares such as ROS. Webots itself has the Transmitter and Receiver node, which you can use to send data between Robots

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/19/2021 08:45:08
Hello `@Homeralf`. I am glad you like Webots. As you mentioned, the objects cannot be serialized because of SWIG. If you saw PR with objective to move away from SWIG in favor of Python native bindings then I believe that will not solve your problem. Maybe we can do something like this:

[https://stackoverflow.com/questions/9310053/how-to-make-my-swig-extension-module-work-with-pickle](https://stackoverflow.com/questions/9310053/how-to-make-my-swig-extension-module-work-with-pickle)


You cannot run the controllers in the same process

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 09:29:02
You also might want to take a look at pyro5. Could be exactly what you need

##### Homeralf 01/19/2021 09:42:57
Hello `@Simon Steinmann`



I want to run it from the same process because among other things I want to automate multiple simulations based on evolutionary algorithms.

The controllers must receive input from higher abstractions like the activations of the muscles, where the robot should be headed, the target speed of a solid, etc.



Yes, I was using the BaseManager from multiprocessing to share the data. However, it wouldn't work because the objects can't be serialized.

I would like to keep the project as lean as possible and I think using ROS may be a little bit overkill for now but I will consider it if things get complicated (I may be wrong).

Transceiver and receiver nodes seem like a good solution among robots.

Didn't know about pyro5, looks promising! I will give it a try!



Thanks for your time Simon, really appreciated it.


Hi `@Darko Lukić`,



Yes, that was the PR I saw.

Nice, looks exactly like the solution. I will try to implement it as soon as I get back from work.



Thanks for answering the questions, you've helped me a lot.

##### gaitt 01/19/2021 10:20:48
Hi guys, 

I'm building a tutorial for our software. The tutorial use Webots and ROS2. So far, so good. 

Basically, I have a scene with a UR5e robot and some objects. We execute some trajectories with the followJointTrajectory action.

I would like now to go further and to do so, I need to know the position of some Webots objects in ROS2. For example, the robot move a bottle and I would like to get the bottle position to be published in ROS2 through a geometry\_msgs::Pose. 

Any idea how I can achieve this using C++/ROS2 API?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/19/2021 10:29:20
`@gaitt` You can use Supervisor:

[https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_position](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_position)



Do you use the `webots_ros2` package or you created your own interface?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 10:31:16
If you want positions of objects relative to others (your robot for example), then you can use this script I made. Or you can at least take inspiration from it and create it in c++
> **Attachment**: [get\_relative\_position.py](https://cdn.discordapp.com/attachments/565154703139405824/801036050230673468/get_relative_position.py)

##### gaitt 01/19/2021 10:31:26
Yes I already use the  `webots_ros2` package.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 10:32:11
The supervisor only gives you absolute (world) coordinates. For inverse kinematics you usually need it realtive to the robot base


If you end up creating something like this in c++, it would be nice if you could share

##### gaitt 01/19/2021 10:38:43
Thanks `@Simon Steinmann`, well absolute position is what I need.


I was thinking there was an easier solution that writing a new ROS2 node.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 10:40:46
You need a supervisor controller (can be your ur5e). It can then get and publish the positions of anything in the simulation

##### yash 01/19/2021 11:39:11
Hi ! I am using wb\_motor\_get\_torque\_feedback(),  to get torque of motors for a 2R manipulator. But the function returns me the same value of torque for all the motors ? What could be the issue ?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 11:47:41
can you post your code or controller file?

##### yash 01/19/2021 11:48:01
alright



> **Attachment**: [force\_control.py](https://cdn.discordapp.com/attachments/565154703139405824/801055822753955870/force_control.py)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 11:53:24
line 104-108


you use the same motor


that might be it 😉

##### yash 01/19/2021 11:58:30
shit yes ! thanks a lot , very silly thing.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 11:58:50
it happens :p

##### yash 01/19/2021 11:58:57
I would like to plot these values , how can it be done ?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 11:59:24
double click on the robot, it should be plotted there


the robot window

##### yash 01/19/2021 12:06:07
No, but these are values that give torque feedback and doesn’t take any sensor into Accout


account*

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 12:06:56
then you have to either make a custom robot window, or plot it yourself with a python library


or use external IDEs

##### yash 01/19/2021 12:08:01
understood, I am using matplotlib, but I plot them with respect to time right i.e robot.step(timestep)?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 12:08:51
time = robot.getTime()

##### yash 01/19/2021 12:09:06
oh okay

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 12:09:17
that is the simulation time you see in Webots


which is increased by the timestep amount every step

##### yash 01/19/2021 12:11:29
ok, thank you very much

##### Ishi\_Senpai 01/19/2021 14:59:32
hello is there any tutorials on how to creat a snake like robot in webots as i know you have the yamor but i need to make an orrigonal one but its my first time using the software]

##### gaitt 01/19/2021 15:07:29
Can I retrieve a Supervisor from an already running instance of webots?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/19/2021 15:23:29
A supervisor is used like the robot node. So you have to start a controller of a robot, which has supervisor = true checkmark in the scene tree


And instead of robot =Robot() you do robot = Supervisor()

##### gaitt 01/19/2021 15:37:11
Ok I managed to get it! It look likes, if webots is running I can't get the handle. I need first to create the supervisor and then launch webots to get the handle properly. Now let see how to parse the rootNode ...

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/20/2021 08:49:55
`@gaitt` A supervisor needs to be initialized. For that a robot controller needs to be launched. Usually all robots controllers are launched, when the simulation starts. Through the use of a supervisor controller, you can start and stop other controllers. But I dont think you can start your first supervisor controller later on in the simulation.


You should also always have this open in a tab: [https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python)

You'll have to reference it constantly. Luckily it is very detailed. Also remember, that the `Supervisor() `also has all the functionality of `Robot()`. So you dont have to change any of your existing code, should you chose to turn your ur5e into a supervisor. All motor / sensor controls work the same

##### Majanao 01/20/2021 11:31:45
Hi all, I would like to balance the camera of the DJI Mavic. Is it possible to rotate it through a python script or do I have to add two motors to the drone model?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/20/2021 11:33:53
Sure, this is possible. You will have to edit the drone model (PROTO file) to add this capability.


In fact, you don't need to edit the drone model. Instead you should create a pan tilt camera proto and add it in the `cameraSlot` of the Mavic2Pro model so that it is used instead of the default `Camera` node.

##### Majanao 01/20/2021 11:53:47
Thanks a lot. I'll try that.

##### gaitt 01/20/2021 17:15:06
`@Simon Steinmann` Ok I see. Thanks a lot. 

Another question: can we add other sensors to a predefined robot. I use the predefined UR5e and so the controller is external. 

My goal would be to add a fixed rangefinder to the UR5e base and access it from my ROS2 node to process the depth image.


BTW, is there somewhere a piece of code to transform a rangefinder depth image to 3d points in world coordinate frame?

##### Apashe 01/20/2021 17:28:43
Hello, what is the distance between the wheels and the radius of the wheels themselves for the robot E-puck?

I did self.wheel\_radius = 0.021 and self.distance\_between\_weels = 0.052

but I don't have the full turn angle converge (I do it through the values PositionSensor)

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/20/2021 17:31:46
`@Apashe` This may help:

[https://github.com/cyberbotics/webots\_ros2/wiki/Tutorial-E-puck-for-ROS2-Beginners#differential-drive-calibration](https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-E-puck-for-ROS2-Beginners#differential-drive-calibration)


Yes, you can add any node to `toolSlot`


```py
def depth2pointcloud(depth_image, k_matrix):
    inv_fx = 1.0 / k_matrix[0, 0]
    inv_fy = 1.0 / k_matrix[1, 1]
    ox = k_matrix[0, 2]
    oy = k_matrix[1, 2]
    image_height, image_width = depth_image.shape
    points = np.zeros((image_width * image_height, 3), dtype=np.float32)
    counter = 0
    for y in range(image_height):
        for x in range(image_width):
            dist = depth_image[y, x]
            # This follows the coordinate system of the gripper
            points[counter, 1] = -np.float32((x - ox) * dist * inv_fx)
            points[counter, 2] = -np.float32((y - oy) * dist * inv_fy)
            points[counter, 0] = np.float32(dist)
            counter += 1
    return points[:counter].astype(np.float32)


def get_calibration_matrix_from_range_finder(range_finder):
    image_width = range_finder.getWidth()
    image_height = range_finder.getHeight()
    focal_length = 0.5 * image_width * (1 / math.tan(0.5 * range_finder.getFov()))
    k_matrix = np.array([
        [focal_length, 0, image_width / 2],
        [0, focal_length, image_height / 2],
        [0, 0, 0]
    ])
    return k_matrix


def get_point_cloud_from_range_finder(range_finder):
    depth_image = np.asarray(range_finder.getRangeImage(), dtype=np.float32)
    depth_image = depth_image.reshape((-1, range_finder.getWidth()))


    k_matrix = get_calibration_matrix_from_range_finder(range_finder)
    return depth2pointcloud(depth_image, k_matrix)
```

This snippet creates a point cloud in the range finder's reference frame.

##### gaitt 01/20/2021 17:38:51
Thanks a lot, much appreciated.

##### Apashe 01/20/2021 17:39:00
I don't use ros

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/20/2021 17:39:55
I know, but the principle is the same. You rotate the robot and move it along the line to calibrate it

##### Apashe 01/20/2021 17:40:19
I used the standard example for PositionSensor from Webots and [https://www.youtube.com/watch?v=WSjTWcTojHg&t=536s](https://www.youtube.com/watch?v=WSjTWcTojHg&t=536s)

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/20/2021 17:41:01
`@Apashe` 

```py
DEFAULT_WHEEL_RADIUS = 0.02
DEFAULT_WHEEL_DISTANCE = 0.05685
```

You can use these values

##### gaitt 01/20/2021 17:42:35
I did try `toolSlot` but as I expected it moves when the robot moves, I would like my rangefinder to be in a fixed frame

##### Apashe 01/20/2021 17:42:36
Thank you I'll go try it

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/20/2021 17:43:33
I see, in that case you can add it in directly in the world

##### gaitt 01/20/2021 17:53:08
I'm not sure how, could you elaborate? Device node must belong to a robot node, right?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/20/2021 19:13:51
Yes, you would have two robot and two controllers, one of them just publishing the point cloud


Alternately, you can add the range finder to the UR PROTO directly

##### gaitt 01/21/2021 08:23:49
Ok thanks, I will try that!

##### MartinG 01/21/2021 10:22:45
Anyone have some tips on getting started with Lidars in Webots?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/21/2021 10:46:21
Read the API documentation and just try them out :)

##### Tej 01/21/2021 14:55:02
good afternoon fellow robots

##### Apashe 01/21/2021 17:54:39
sending the robot coordinates from the supervisor to E-puck robot, but the data is not coming in

message = struct.pack('>7f', robot\_coords[0][0], robot\_coords[0][1], robot\_coords[0][2], robot\_coords[1][0], robot\_coords[1][1], robot\_coords[1][2], robot\_coords[1][3])

    emitter.send(message)



what could be the problem?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/801872405399601182/unknown.png)
%end

##### fowzan 01/21/2021 17:59:09
Hi guys


Is there any simulation links available for Spot mini ?


`@Darko Lukić`


`@Olivier Michel`


I would love to simulate spot mini on my terminal , could you share links if available

##### Apashe 01/21/2021 18:26:40
I realized that the error in

self.receiver = self.getDevice('receiver')

if convert root to to base node and add receiver (with your hands and delete the old one)



everything works

why so?


the robot also has receiver, why doesn't it work?


how to connect to receiver without convert root to base node

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/21/2021 19:06:06
[https://cyberbotics.com/doc/guide/spot](https://cyberbotics.com/doc/guide/spot)

##### fowzan 01/22/2021 08:42:53
Thank you


Is there any simulation for fleet management systems `@Darko Lukić`

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/22/2021 08:45:02
Something like this:

[http://www.diegoantognini.com/projects/dis/](http://www.diegoantognini.com/projects/dis/)

##### Apashe 01/22/2021 08:45:11
Help me, please

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/22/2021 08:47:12
I don't understand the issue. Can you send us a minimal example of the project so we can reproduce the problem?

##### Apashe 01/22/2021 08:52:58

> **Attachment**: [my\_odometry.7z](https://cdn.discordapp.com/attachments/565154703139405824/802098472752513044/my_odometry.7z)

##### fowzan 01/22/2021 08:54:23
What are the commands to move spot mini in webots ? I’m unable to navigate it
%figure
![image0.jpg](https://cdn.discordapp.com/attachments/565154703139405824/802098830870839366/image0.jpg)
%end


`@Darko Lukić`


Kindly share the commands

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/22/2021 08:59:08
Your emitter and receiver don't use the same channel. Either change the `receiver_channel` value in the e-puck node, or `Emitter > channel` in the supervisor node.


[https://cyberbotics.com/doc/reference/nodes-and-api-functions](https://cyberbotics.com/doc/reference/nodes-and-api-functions)



But please go through the tutorials first:

[https://cyberbotics.com/doc/guide/tutorials](https://cyberbotics.com/doc/guide/tutorials)



Here is an example for Spot:

[https://github.com/cyberbotics/webots/blob/master/projects/robots/boston\_dynamics/spot/controllers/spot\_moving\_demo/spot\_moving\_demo.c](https://github.com/cyberbotics/webots/blob/master/projects/robots/boston_dynamics/spot/controllers/spot_moving_demo/spot_moving_demo.c)

##### Apashe 01/22/2021 09:07:10
thanks

##### fowzan 01/22/2021 09:08:08
Where should I change this ? Is it in the script


I tried but apart from raising the legs it is not navigating `@Darko Lukić`


Kindly help me out

##### MartinG 01/22/2021 09:49:48
getRangeImage and getLayerRangeImage both return the same amount of elements, the documentation says that getLayerRangeImage should only return the values of the requested layer.

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/22/2021 10:02:11
`@MartinG` Can you confirm that `getRangeImage` is not equal to 1

##### MartinG 01/22/2021 10:06:37
I can now, just checked.

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/22/2021 10:18:49
It indeed seems like a bug (I can reproduce it). Could please report the issue:

[https://github.com/cyberbotics/webots/issues/new?assignees=&labels=&template=bug\_report.md](https://github.com/cyberbotics/webots/issues/new?assignees=&labels=&template=bug_report.md)

##### MartinG 01/22/2021 10:19:11
Will do. Thanks.

##### gaitt 01/22/2021 10:36:59
Hey Darko, so I have tried the first approach. I added a 2nd robot with only the kinect in it. The robot controller is set to extern, supervisor is true for all robot. But when I launch the "webots\_robotic\_arm\_node", it seems to me that the joint state publisher get stuck! My custom ROS2 node seems to behave correctly and is loaded as controller for the 2nd robot. Can we have multiple supervisor, 1 per robot?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/22/2021 10:40:21
`@gaitt` Check this example on how to have more than 1 robot in a simulation:

[https://github.com/cyberbotics/webots\_ros2/blob/master/webots\_ros2\_demos/launch/armed\_robots.launch.py#L31-L45](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_demos/launch/armed_robots.launch.py#L31-L45)



The ROS2 interface will not handle two robots by itself as it cannot know how to assign controllers.

##### JSK 01/22/2021 10:49:48
hi fellows


how does the saveExperimentData() works in webots controller terminating?


It says  "NameError: name 'saveExperimentData' is not defined"

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/22/2021 10:53:29
The `saveExperimentData` function is a placeholder function, you should implement it by yourself, it is just an example.

##### JSK 01/22/2021 11:00:23
ok

##### ssFuhrer 01/22/2021 22:44:43
Hey guys, I would like to ask a question



I am adding my own obj data to make a conveyor belt simulation but i fail to bound objects.



WARNING: DEF Simulation Robot > DEF stand\_pulley2 Solid > USE stand\_pulley2 > Shape > IndexedFaceSet: Mass properties computation failed for this IndexedFaceSet: the corresponding added mass defaults to 1kg, the added inertia matrix defaults to the identity matrix.Please check this geometry has no singularities and can suitably represent a bounded closed volume. Note in particular that every triangle should appear only once with its 'outward' orientation.



I am getting this error. What should i do ? 



Thank you



I will add another question regarding to first problem, my object does not stand on the floor it passes, anyone knows anything about it?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/23/2021 08:24:52
<@132121297277812736> your mesh file seems to be faulty. How did you generate it? There is tools and programs that can fix meshes. Webots told you the things that might be wrong


And for boundingObject, it is always better to approximate it with a simple geometric shape, such as a sphere, box, cylinder etc. It will make the simulation more stable and much faster


Should fix your second issue too

##### babaev1 01/23/2021 08:56:10
I am sorry for stupid question. Why Y axis stands for vertical direction? This I find in most of sample worlds. Conventionally Z stands for vertical. How much troubles I can encounter if I create world with Z vertical ?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/23/2021 09:30:41
In world info you can change the coordinate system from nue to enu (or create a new world through the wizard with that)

##### ssFuhrer 01/23/2021 13:56:04
Thank you Simon for your responses. I am using NX for my design and I export my design as OBJ. But it seems it doesnt work properly in webots.


`@Simon Steinmann`

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/23/2021 15:36:39
make sure the mesh is watertight

##### babaev1 01/23/2021 16:15:06
Thank you for answer. This feature is available in latest version of Webots therefore this  pushed me to update. Still I experience some inconvenience because after changing from nue to enu all protos comes imported in wrong orientation. Saving world  and re-loading world doesn’t help. Wizard doesn’t help to set up viewpoint and background light for enu. They must be changed by hands. May be I made something wrong with consequence of steps.

##### ptrepag 01/23/2021 19:58:55
Our robot has a set of omniwheels attached to encoders.  Each omniwheel can only rotate in one direction, but moves freely in the other direction.  This makes them ideal for capturing actual robot movement in the X and Y direction (we use this for position localization).  Does anyone have a suggestion for how best to model this in webots?    I could add a position sensor to a cylinder, but I worry that the drag in the other direction would mess things up.  Any assistance would be greatly appreciated.

##### Cyber Police Officer 01/24/2021 16:49:34
Hello. I tried to search this in the documentation, but found no results. 



What does it mean when the "bounding objects" turn from white into purple?



(At t=0 some of the bounding objects are white but turn purple after a few seconds)



Thank you

##### Stefania Pedrazzi [Cyberbotics] 01/25/2021 07:21:27
The red/pink bounding objects means that this objects is colliding with some other objects. Then, if the object doesn't move any longer, after a while the boundingObject will turn blue. Otherwise if there is no collision the boundingObject color is white.

[https://www.cyberbotics.com/doc/guide/the-3d-window#selecting-an-object](https://www.cyberbotics.com/doc/guide/the-3d-window#selecting-an-object)

##### yash 01/25/2021 08:32:49
Hi ! I am using def getTorqueFeedback() function to compute torque of a mani[pulator link. I set the max available torque by using the function setAvailableTorque() to 2N.  Then I supply a velocity to the motors. So when I note down the values from the torque feedback, the values are not constant ? Since providing constant velocity to the motor, the torque should also be constant right ?


Please help me regarding this.

##### Cyber Police Officer 01/25/2021 09:25:31
Thank you!

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/25/2021 10:29:08
Hold something in a stretched out arm at your side and lift it up until you hold it horizontally. The torque required at your shoulder changes constantly, even if you move at constant speed. When accelerating or compensating external forces (stretched out arm), you need more torque. Without load and a constant speed, you need no torque (or just a little bit to compensate for friction)

##### yash 01/25/2021 10:35:02
therefore , since the link has mass, so when actuating it at a constant velocity the torque will change ?

##### Bitbots\_Jasper 01/25/2021 13:54:21
if a link has no mass it theoretically requires no torque to accelerate, does any child of the link have any mass?

##### yash 01/25/2021 14:13:00
Yes the link has mass , so definitely it will require torque

##### Bitbots\_Jasper 01/25/2021 14:13:33
ah sorry i misread your message

##### yash 01/25/2021 14:14:04
I give constant velocity, so the torque should also be constant right ?


Given by gettorquefeedback() function

##### Bitbots\_Jasper 01/25/2021 14:16:36
lets assume there is no gravity, at the beginning of the motion you will need some torque to accelerate the joint, when the joint has reached its desired velocity, you might need some torque to overcome friction, at the end of the motion you will require some torque in the opposite direction to decelerate the joint


if you now have gravity, the joint has to apply torque to overcome that gravity pulling on the center of mass of the attached link, depending on the angle between the gravity vector pointing downwards and the angle of the joint to the link it will require more or less torque to overcome this force

##### yash 01/25/2021 14:21:31
Therefore as per your your statement , there will always be increase or decrease in the torque .?

##### Bitbots\_Jasper 01/25/2021 14:22:05
it depends on what external forces are acting on the link

##### yash 01/25/2021 14:22:24
Only the gravity

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/25/2021 14:22:28
Just to make sure, do you understand what forces, torques, velocities and accelerations are? And how they interact with one another?

##### yash 01/25/2021 14:23:33
Yes.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/25/2021 14:25:00
Okay, so lets assume an instance, where there is no gravity, and no friction. In this case, you only need torque to accelerate or decelerate. Remaining still,  or at constant velocity, requires no torque.


Is this clear, or is there something you want me to explain in more detail?

##### yash 01/25/2021 14:28:08
Oh okay , well understand !

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/25/2021 14:29:13
Now we turn gravity on. It will pull on the link and joint. This creates torque in the joint, assisting or fighting against the motor. So the motor torque has to change

##### yash 01/25/2021 14:30:56
Alright , because of the gravity or some exeternal force !

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/25/2021 14:31:45
Just imagine, you would suddenly turn the motor off, and it would just be a joint without friction. Would anything fall down, slow down, speed up? If the answer is yes, the motor would have to apply a torque to counter that

##### yash 01/25/2021 14:32:55
Understood, 👍

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/25/2021 14:33:07
[https://en.wikipedia.org/wiki/Newton%27s\_laws\_of\_motion](https://en.wikipedia.org/wiki/Newton%27s_laws_of_motion) Newtons laws in action 😄


Robotics is great. You actually need all the Maths and Physics you learned in school and at university

##### yash 01/25/2021 14:33:59
> Robotics is great. You actually need all the Maths and Physics you learned in school and at university

`@Simon Steinmann`  true !!!

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/25/2021 14:34:46
[https://en.wikipedia.org/wiki/Rotation\_around\_a\_fixed\_axis](https://en.wikipedia.org/wiki/Rotation_around_a_fixed_axis) this might be worth a read too.

##### yash 01/25/2021 14:35:21
Thank you !

##### Krish 01/25/2021 14:45:56
Is it possible for Webots arena/world to have pictures, animations and GIFs?

If so, how to add them?

##### Bitbots\_Jasper 01/25/2021 14:47:08
for images refer to [https://cyberbotics.com/doc/reference/imagetexture](https://cyberbotics.com/doc/reference/imagetexture)


you can also use normal and occlusion maps with PBRAppearance [https://cyberbotics.com/doc/reference/pbrappearance](https://cyberbotics.com/doc/reference/pbrappearance)


[https://cyberbotics.com/doc/reference/appearance](https://cyberbotics.com/doc/reference/appearance) might be the simples approach though

##### Cyber Police Officer 01/25/2021 14:59:52
Hello. Sorry to bother again with questions:



I'm trying to do a parallel manipulator, similar to the stewart platform, and i'm trying to close the kinematic loop. Like the stewart mechanism, this manipulator has a fixed platform and a mobile platform.



What i'm trying to do is defining the mobile platform. The mobile platform pose in space relative to the fixed is quite simple, but relative to the last joint is fairly complicated.

Is there any way to generate the mobile platform solid relative to the base (basically just making it a children of the robot node) while being a movable solid?



If I create the mobile platform as a children of the robot node, it won't move no matter how much force the actuators make. Also, if i remove all the links, the mobile platform just stays afloat, ignoring gravity. All solids in question have a physics node. On the other hand, if i create it as a child of the last joint, it behaves properly.



I tried to follow the stewart platform example, where they generate the mobile platform as a child of the last joint, but i was trying to avoid this path.



Thank you

##### alejanpa17 01/25/2021 17:26:50
Hi everyone! Hope you can help me with this, I have the 3d model of a specific traffic cone and I'm trying to put it some physics but im stuck with the bounding object.

First, I tried to use the mesh of the 3d model but when I run a sim im getting this error: "Your world may be too complex." that's obvious because the mesh got many faces.

Then I thought in using the cone geometry shape but apparently webots doesn't support the cone as a Bounding Object.

What do you suggest? Is there any way I can modify the default traffic cone of webots (because this one has physics)?

This is one of the cones I wanted to introduce in the sim:



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/803314977511440404/unknown.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/25/2021 17:28:49
Webots supports cones
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/803315455736414208/unknown.png)
%end


a cone plus box at the bottom should do the trick

##### alejanpa17 01/25/2021 17:34:58
It does supports cones but not as a  Bounding Object.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/803317001136242708/unknown.png)
%end


link: [https://cyberbotics.com/doc/reference/node-chart](https://cyberbotics.com/doc/reference/node-chart)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/25/2021 17:37:53
hmm you are right

##### AdityaDutt 01/25/2021 18:58:57
in my simulation, my simulation window is drifting around in a circular shape without me doing anything. Any ideas why?

##### DrakerDG 01/26/2021 06:50:47
<@671921336385273856> check the view menu: The Follow Object submenu allows you to switch between a fixed (static) viewpoint and a viewpoint that follows a mobile object (usually a robot). If you want the viewpoint to follow an object, first you need to select the object with the mouse and then check one of the items of the submenu depending on the following behavior you want... 

[https://cyberbotics.com/doc/guide/the-user-interface](https://cyberbotics.com/doc/guide/the-user-interface)

##### R\_ 01/26/2021 06:52:22
Is there a method Webots to create a 'ghost' robot (pure pose transformation through space without the effects of gravity and collisions affecting it?). This is shown in the video below. The robot in the darker shade is present in the physical world, whereas the other one is a ghost [https://www.youtube.com/watch?v=aiWxIjtMMFI](https://www.youtube.com/watch?v=aiWxIjtMMFI)

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/26/2021 06:56:38
Yes, this possible by creating a robot without physics and animating it from a supervisor controller.

##### JSK 01/26/2021 07:40:41
Hello  there team


i am programming a controller for drone.


it takes of at 68.5


but i have given it more than 100 but it is not flying


motors are moving i.e. showing rotation


but the drone is not taking off


what could be the possible reason?


i am stuck here. need help

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/26/2021 07:45:06
How did you create your drone model? Did you create it from the Mavic example?

##### JSK 01/26/2021 08:13:09
yes


Excuse me!

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/26/2021 08:29:10
I would recommend you to revert back to the original drone model to check if it works with your controller and make changes step-by-step to identify where did the problem come from.

##### JSK 01/26/2021 09:30:03
Actually that drone is just imported in my\_first\_Simulation. it has nothing but just a drone. it is flying with "mavic2pro.c" controller but its not flying for mine. although  i have supplied rotor inputs. here is a screenshot of my envoirment.
%figure
![Screenshot_from_2021-01-26_14-01-08.png](https://cdn.discordapp.com/attachments/565154703139405824/803557358000930826/Screenshot_from_2021-01-26_14-01-08.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 11:42:51
Perhaps share your project. It's hard to tell otherwise

##### Cyber Police Officer 01/26/2021 11:46:01
Is there a way to chain transform nodes in a bounding object? The bounding object allows a transform node with a shape child, I was wondering if there's a way to chain multiple transform nodes, where the last one has a shape child.


Or if there's a way to automatically compute the equivalent transform

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 11:48:08
With just translations you can simply add them. Otherwise you have to do matrix multiplication


I'm sure there is online calculators for that. Basically you want to do forward kinematics


You can also see the translation and rotation of a node relative to others in webots

##### Cyber Police Officer 01/26/2021 11:55:08
Yeah, i have the transform on matlab. But i was trying to dynamically generate the manipulator, for given vector of design parameters, without having to manually input the transforms


Thank you

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 12:07:31
You may be able to simply chain them. I don't know if it works or not

##### Cyber Police Officer 01/26/2021 12:14:26
Its a chain of 3 transform with both rotation and translation. I could indeed multiply the matrices to obtain the equivalent transform, and then extract the translation and rotation, but i was wondering if there was a tool, in the PROTO, to automatically compute the chain of transforms. Or if there was a way to allow the chain to be interpreted as one transform in the bounding object

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 12:15:19
try it out

##### Cyber Police Officer 01/26/2021 12:16:25
I tried tried to chain them, but webots says it only allows a transform with a shape/geometry child, not a chain of transforms with a shape/geometry child:

"A Transform node inside a 'boundingObject' can only contain one Shape or one Geometry node. The child node is ignored.!"

##### Bitbots\_Jasper 01/26/2021 12:21:20
I think the structure of the proto you want is something like this:

```
Robot {
  Solid {
    translation ...
    rotation ....
    boundingObject Transform {
      translation ...
      rotation ...
      children [
        Box {
           size ...
        }
      ]
    ...
    }
    children [
      Solid {
      next link in the kinematic chain
      }
    ]
  }
}
```


the transform of the bounding object is relative to the solid

##### Cyber Police Officer 01/26/2021 12:30:35
Exactly, my problem is that, given the kinematics, i have to describe the bounding object with a chain of Transforms, like:

```
Robot {
  Solid {
    translation ...
    rotation ....
    boundingObject Transform {
      translation ...
      rotation ...
      children [
        Transform {
          translation ...
          rotation ...
          children [
            Transform {
              translation ...
              rotation ...
              children [
                Box {
                 size ...
                }
              ]
            }
          ]
        }
      ]
    ...
    }
    children [
      Solid {
      next link in the kinematic chain
      }
    ]
  }
}
```


And he doesn't allow this chain of transforms

##### Bitbots\_Jasper 01/26/2021 12:32:34
how are your solids connected?

##### Cyber Police Officer 01/26/2021 12:33:58
with ball and hingejoints


When i try to chain transform it says:

 A Transform node inside a 'boundingObject' can only contain one Shape or one Geometry node. The child node is ignored.

##### Bitbots\_Jasper 01/26/2021 12:34:23
but there are multiple bounding boxes in each link?

##### Cyber Police Officer 01/26/2021 12:35:11
No, each link only has one bounding object

##### Bitbots\_Jasper 01/26/2021 12:36:02
then the bounding box for each link should be inside that link (or solid how it is called in webots)


otherwise they will not move if the robot's joints move

##### Cyber Police Officer 01/26/2021 12:38:36
So the bounding shape must be a child of the solid and called to be used in the bounding object?

##### Bitbots\_Jasper 01/26/2021 12:40:25
i am not quite sure what you mean by "called to be used in the bounding object"


but yes the bounding object should be a child of the solid

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 12:41:11
A solid node has the field BoundingObject, that's where you add the collision geometry

##### Cyber Police Officer 01/26/2021 12:41:27
```DEF MOBILE_PLATFORM Solid {
                          translation 0 0 0
                          rotation 1 0 0 0
                          name "mobile_plat"
                          children[
                             DEF MOBILE_PLATFORM_SHAPE Transform {
                              translation 0 %{=-d/2}% 0
                              rotation %{=up_k['x']}% %{=up_k['y']}% %{=up_k['z']}% %{=-alphap_k}%
                              children[
                                Transform {
                                translation 0 %{=-h}% 0
                                rotation 0 0 1 %{=-beta_k}%
                                  children [
                                    Transform {
                                      translation %{=-b_k["x"]}% %{=-b_k["y"]}% %{=-b_k["z"]+z}%
                                      rotation 1 0 0 1.5708
                                      children [
                                        Shape {
                                        appearance PBRAppearance {
                                          baseColor 0.12 0.56 1
                                          roughness 1
                                          metalness 0
                                        }
                                        geometry Cylinder {
                                          radius IS r_m
                                          height IS arm_r
                                          subdivision 48
                                        }
                                      }
                                ]}
                               ]}
                             ]}
                          ]
                          boundingObject USE MOBILE_PLATFORM_SHAPE
                          physics NULL
                        }
```


This is what i'm trying to do

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 12:41:40
the visual shape you add in the children field

##### Cyber Police Officer 01/26/2021 12:41:48
Sorry for the horrible indentation


But it doesn't allow me, because the MOBILE\_PLATFORM\_SHAPE has a chain of Transforms. If it was only one transform it would work

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 12:43:22
the Shape has to have the DEF

##### Cyber Police Officer 01/26/2021 12:44:53
Yeah, but if i do that the boundingObject will be in the wrong place, because it loses the transforms


I could start with the chain of transforms, and define the solid has a child of it. But this is a end-effector of a joint and it only allows the end-effector to be a solid

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 12:46:37
why do you need that chain of transforms anyways?

##### Cyber Police Officer 01/26/2021 12:50:49
Basically i'm trying to describe where the blue platform is in space. And i don't know where it is relative to the green joint, but i do know where it is relative to the red plate.



So i'm transforming the coordinate system from the green joint referential into the red plate referential.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/803607880410333194/unknown.png)
%end


I could know where the blue plate is relative to the green joint, but the direct kinematics are fairly complicated and i was trying to avoid that path

##### Bitbots\_Jasper 01/26/2021 12:52:55
you should start by defining your kinematic chain as solid->joint->....->solid

where the solid after a joint is in the endPoint field of the joint and a joint after a solid is a child to the solid

you also put in the translations and rotations between the solids in the translation and rotation field of the joint, make sure to also set the joint parameters of the joint correctly

then you can put the bounding objects into the solids where the transform you set it to is relative to the solid it belongs to

##### Cyber Police Officer 01/26/2021 12:58:57
Thank you. I'm doing that, i started by following the Stewart platform example

##### Bitbots\_Jasper 01/26/2021 12:59:03
the pose of the blue table is based on the pose of green rods, therefore it has to be defined relative to the green rods

##### Cyber Police Officer 01/26/2021 13:00:05
Yes, the problem is that implies solving the direct kinematics, which are nontrivial in parallel manipulators


For instance, in that manipulator it implies solving a system of 6 nonlinear equations


Thank you for the help!

##### Bitbots\_Jasper 01/26/2021 13:09:24
yes you are right, it's really a lot more difficult for parallel manipulators..

##### alejanpa17 01/26/2021 15:20:37
Is there any way I can modify the default traffic cone of webots with my model in order to have his physics?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/803645582152433705/unknown.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 15:21:46
of course, just add a new object and load in your model for the visual shape and Bounding object

##### alejanpa17 01/26/2021 15:28:13
the problem is that my model has too many faces so the sim is giving me "Your world may be too complex."

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 15:29:10
you should approximate the bounding object with simple geometries


you can also specify the inertia matrix yourself.

##### jejay 01/26/2021 15:47:32
Hey guys I have a few questions: There is a kinematic mode mentioned (no physics, just animations) in the docs but it is not clear to me how to use it. Can I import animated objects (e.g. a car+trajectory animation) that I exported from blender? is there curves, interpolation etc?


and the other question extends this: is there a way to import humans + skeletal animation, again in kinematics mode?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/26/2021 15:51:46
The `TrafficCone.proto` has a `physics` field which is set to `NULL`. Simply adding a `Physics` node in there should add physics to it.


The `boundingObject` of the `TrafficCone.proto` is approximated with a `Box` and 3 `Capsule` primitives.


Simply removing the `Physics` nodes will disable physics in the simulation. Then, you will be able to animate the model from a supervisor controller by changing the `translation` and `rotation` fields of the objects you want to move.


As for human, see `WEBOTS_HOME/projects/humans/pedestrian` for some simple robotic human models.


Webots also support skeletal animation, see `WEBOTS_HOME/projects/samples/rendering/worlds/animated_skin.wbt`. However we do not distribute human model with skeletal animation, but it was tested and works.

##### jejay 01/26/2021 16:00:08
interesting, thanks!


so there is no simple way to play back a prerecorded animation or one that has been included e.g. in a collada file other than implementing it on your own step by step?

##### alejanpa17 01/26/2021 16:05:41
Thank you so much  `@Simon Steinmann` `@Olivier Michel`  😀

##### jejay 01/26/2021 16:06:30
the motion functions ([https://cyberbotics.com/doc/reference/motion-functions](https://cyberbotics.com/doc/reference/motion-functions)) are for physics/actuators only? I dont understand how you record them and where the motion editor is that is mentioned in the docs

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/26/2021 16:09:39
The motion editor doesn't exist any more. The motion functions work for both physics and non-physics actuators.


We have some skin animation code in the works: [https://github.com/cyberbotics/webots-doc/pull/96](https://github.com/cyberbotics/webots-doc/pull/96) but it didn't yet reached the release.


The motion editor is actually deprecated and provided only for the Robotis-OP2 robot: [https://cyberbotics.com/doc/guide/robotis-op2#motion-editor](https://cyberbotics.com/doc/guide/robotis-op2#motion-editor)


But you can create a motion file from a simple text editor (the format is human readable).

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 16:15:27
you can also make a simple controller and use a supervisor to set jointagnles or right out the position and orientation of solids


and just read your trajectory or whatever you have and execute it in a loop

##### jejay 01/26/2021 16:16:02
thanks, yeah I guess thats kind of equivalent, thanks


Thanks a lot!

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 16:21:11
let us know if you have further questions


[https://cyberbotics.com/doc/reference/supervisor?tab-language=python](https://cyberbotics.com/doc/reference/supervisor?tab-language=python)


you'll probably need this


This trajectory follower script for ROS could help you out perhaps


[https://github.com/cyberbotics/webots/blob/master/projects/robots/universal\_robots/resources/ros\_package/ur\_e\_webots/src/ur\_e\_webots/trajectory\_follower.py#L199](https://github.com/cyberbotics/webots/blob/master/projects/robots/universal_robots/resources/ros_package/ur_e_webots/src/ur_e_webots/trajectory_follower.py#L199)


has cubic interpolation

##### jejay 01/26/2021 16:25:57
I am actually an "expert" to help students in a robotics practical at  UoEdinburgh that sadly this year is conducted virtually, hence most student will use webots. I might come back if I have any other questions. Thanks a lot again!


thats sounds good 🙂

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 16:26:52
you're welcome, and good luck 🙂

##### jejay 01/26/2021 16:37:17
ah maybe actually one final thing. If I stay in physics mode and take for example the bmw scene. Now I turn on manual mode and drive around. Can I record this into a motion file to replay the physics control in a later simulation? or would I need to write a controller that records my actions and then write those into a motion file (which in then could also be my own file format)?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 16:38:12
you can record and replay anything you want. You could log the position of the whole model for example, and the then just replay that


with a supervisor you can set anything really

##### jejay 01/26/2021 16:38:46
yeah but i need to code that supervisor, right

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 16:39:04
very simple

##### jejay 01/26/2021 16:39:07
that was my question, there is nothing out of the box


yeah makes sense


yeah I see that this is simple 🙂

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/26/2021 16:39:34
you get a handle to the node or field you want to change, and change it


instead of motors, you can directly change the joints

##### jejay 01/26/2021 16:40:52
👍 thanks again

##### chungshan 01/27/2021 06:00:25
Hi, can I get the force in the Hing2Joint?

I can get the torque by adding the rotational motor, but I also want to know how to get the force in the Hing2Joint

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/27/2021 07:42:59
Hello `@chungshan`, the Motor node returns a force feedback as well:

[https://cyberbotics.com/doc/reference/motor#wb\_motor\_get\_force\_feedback](https://cyberbotics.com/doc/reference/motor#wb_motor_get_force_feedback)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/27/2021 10:03:31
`@chungshan` What exactly do you mean by the force in the joint? A joint can only have a torque. Only linear actuators can have forces. Perhaps you can clarify 🙂

##### DrVoodoo [Moderator] 01/27/2021 11:44:16
When creating new scenery proto objects, is there an additional step to make them pickable in the viewer? For example:


```PROTO Example [
  field SFVec3f     translation            0 0 0
  field SFRotation  rotation               0 1 0 0
  field SFString    name                   "example"
]
{
  Transform {
    translation IS translation
    rotation IS rotation
    children [
      CardboardBox {
        name %{= "\"" .. fields.name.value .. " top\"" }%
        translation 0 0 1
      }
      CardboardBox {
        name %{= "\"" .. fields.name.value .. " bottom\"" }%
      }
    ]
  }
}
```


Will not be selectable in the 3D viewer although it is still selectable in the scene tree

##### Stefania Pedrazzi [Cyberbotics] 01/27/2021 12:43:24
`@DrVoodoo` only `Solid` nodes are pickable from the 3D view

##### DrVoodoo [Moderator] 01/27/2021 12:58:24
`@Stefania Pedrazzi`  excellent, thanks. I was banging my head on that one

##### pk1jk1 01/28/2021 01:27:09
Does anyone have experience creating an overhead map with a camera? And potentially stitching multiple images together to create an encompassing overhead map?

##### Laojiang 01/28/2021 06:35:05
How can I make a object whose the shape is a bowl?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 01/28/2021 07:53:16
In Webots? You can add a camera to a Supervisor robot or drone and take the images. Then, add the images to a software such as Pix4d (but probably there is a free alternative) to create a map.

##### Bitbots\_Jasper 01/28/2021 08:42:52
you can create a 3d model (many formats such as .stl, .obj, .dae and .blend are supported)


an example how to create a bowl model in open scad can be found here: [http://edutechwiki.unige.ch/en/OpenScad\_beginners\_tutorial#Simple\_CSG\_examples](http://edutechwiki.unige.ch/en/OpenScad_beginners_tutorial#Simple_CSG_examples)


then export it as stl (file->export) in open scad and import it in webots (file->import 3D model)

##### yash 01/29/2021 10:05:23
Sorry for the message before. Please ignore it.      I would like to know that if we have a structure like this -

          While robot.step(timestep)!=1

                    if (condition)                                                                                                                                                                                                                                                                                                                     for i in ----

   In this case when the execution is in the for loop will the robot.step(timestep) be updated ?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/29/2021 10:06:37
no, only when the robot.step() is executed, which is every iteration of the while loop

##### JSK 01/29/2021 10:06:50
Nop.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/29/2021 10:06:56
you could add an extra robot.step() inside the for loop

##### yash 01/29/2021 10:07:13
understood !! thanks

##### Lynerea 01/29/2021 10:43:53
Hi everyone, I recently downloaded Webots and when I went to make my own world, it comes up with an error saying that it can't find 'wbmath' and when I look at the file directory the files aren't there in any capacity - does anyone have any suggestions on how to solve that issue, would it just be a matter of reinstalling the program?

##### Bitbots\_Jasper 01/29/2021 10:45:11
how did you install webots?

##### Lynerea 01/29/2021 10:45:58
Via the cyberotics website using the newest Windows version it had available

##### Bitbots\_Jasper 01/29/2021 10:47:30
I'm sorry I don't know much about windows, maybe someone else can help with that.

##### Lynerea 01/29/2021 10:48:04
That's fine, thank you for trying anyways 🙂

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/29/2021 10:48:59
Can you run the other demos of Webots (e.g., guided tour) without any problem?

##### Lynerea 01/29/2021 10:49:43
I opened the introductory demos, all of them I believe, with no issues at all

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/29/2021 10:50:48
So what is particular with your world file? Did you add any specific object?

##### Lynerea 01/29/2021 10:52:10
I created a new project Directory via the wizard and ticked all the boxes when given the option but then the 'rectangle arena' caused an error but the rest of the world loaded successfully


I added a model of BB-8 after I created the world and the floor didn't work

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 01/29/2021 11:33:29
I tried, but cannot reproduce this problem (tested on Windows 10).

##### Lynerea 01/29/2021 11:34:53
I don't understand how, after no tampering on my part, that the file was deleted, I don't know how it happened and I doubt I could reproduce the issue but reinstalling the program seems to have solved the issue for now

##### smasud98 01/30/2021 05:29:13
Hi everyone! I am new to webots and not sure if this is the right place to be asking this question. I am considering getting the Macbook with the M1 chip which uses ARM. I was wondering if anyone knows whether Webots will work well with this. Thanks

##### Krish 01/30/2021 06:03:49
Wait for this year's lineup.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 14:23:48
`@smasud98` This is generally the right place to ask any questions regarding Webots. I think you should be fine with the macbook. While Webots is an x86 program, Apple's Rosetta 2 - x86 emulation seems to be very good and powerful. In general you should ask yourself the question though, whether a macbook is the right tool. If you want to dive deep into programming and development, engineering applications and linux, then it might be more advisable to get an x86 machine.  Perhaps you can tell me what you want to use your laptop for in general. Whether you are willing to switch to linux and/or linux is another big factor.

##### Krish 01/30/2021 15:24:37
Yeah well said.

He told me that he is more interested in getting a Mac than a Linux machine, he says that he has the 2015 Macbook and webots runs very slowly in that.

Also, not all the apps and programming languages are configured to run on the M1.



So he says if Webots does not run on it, he will just return and take the refund back.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 15:39:30
The new macbook is a completely new hardware & software product. It probably works really well for a pure consumer, but as soon as you start development and go outside of MacOS, I foresee a lot of issues. Linux can run on arm, but by far not all software and programs.  I have a Lenovo Legion laptop. It is basically a gamer laptop, that looks more professional. I bought it for robotic simulation and Reinforcement Learning. I dont see macbooks being a good choice for that. For AI you kinda need a Nvidia GPU, and a powerful x86 CPU is a godsent when compiling and running lots of parallelized code / many instances.

##### PymZoR [Premier Service] 01/30/2021 15:46:55
Hi everyone, i'm having an issue with a basic HingeJoint for a wheel


I have a differential robot, and when it's turning in place, le wheels are "moving sideways"


like on a bad bike


By looking at the tree I see small  values in the `translation` field of the wheel although the joint is configured to only allow rotation on an axis

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 15:51:18
can you share your world or a screenshot of your hingjoint? with JointParameters expanded in the scene tree

##### PymZoR [Premier Service] 01/30/2021 15:54:41

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/805103704910200854/unknown.png)
%end



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/805103881205841930/unknown.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 15:58:32
it is always good practice to have the same anchor as translation of the endPoint solid.  currently it rotates around the y-axis of in the 2nd image


move the anchor to where you want the center of rotation to be

##### PymZoR [Premier Service] 01/30/2021 15:59:18
Oh, ok


On top of my Hinge I have a Transform


I can remove the Transform, and instead use anchor & translation on the endpoint solid right ?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 16:00:19
the transform is fine.


send me your world, i can have a quick look


the issue is, if you have a translation of the endpoint, which is NOT along the rotational axis

##### PymZoR [Premier Service] 01/30/2021 16:07:04
The translation of the enpoint was the result of the bug, I did not manually edit these  values



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/805106901393604679/unknown.png)
%end


I just tried this, deleting the Transform above the Joint, no success

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 16:08:58
the translation of the endPoint changes with when the joint rotates. So at the start, the anchor and translation have to be the same (assuming you want the endPoint to rotate around its origin)

##### PymZoR [Premier Service] 01/30/2021 16:09:47
which is the case on my last screenshot right ?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 16:09:56
yes

##### PymZoR [Premier Service] 01/30/2021 16:10:30

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/805107686977830952/unknown.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 16:10:47
this should work too

##### PymZoR [Premier Service] 01/30/2021 16:10:53
Which is also the case when the transform is done above; I have the same endpoint translation and anchor


Still have the problem though 😄

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 16:11:06
what problem?

##### PymZoR [Premier Service] 01/30/2021 16:11:50
When the robot is turning in place, the wheels are slightly translating


instead of only rotating around the joint axis

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 16:12:54
I would have to see the whole thing. But just stick with the solution that works


that is the 'proper' way to do it

##### PymZoR [Premier Service] 01/30/2021 16:13:06
No solution works atm haha

##### Simon Steinmann [ROS 2 Meeting-Moderator] 01/30/2021 16:13:21
share your world file


can also pm if you want

##### ljmanso 01/31/2021 15:14:48
Hello everyone. I am new to Webots and I have a quick question that I haven't seen in the documentation. Is there any way to simulate virtual humans moving around?

##### Bitbots\_Jasper 01/31/2021 15:31:43
There is the pedestrian proto (model) and a controller for it.

##### ljmanso 01/31/2021 19:15:06
Thanks! I will try to find more information about it 👍

##### babaev1 01/31/2021 20:20:54
Hello, can’t figure out why two solids pass through each other like non-material objects during simulation running. Bounding objects are defined and physics is added for both solids. Could it be due to that dencity is -1, but mass has positive value? Could it be due to that inertia matrix is not added? Could it be due to that center of mass is not added for solids?

##### paperwave 01/31/2021 22:33:56
`@babaev1` When you click on the boundingObject node you should see the bounding region surrounding the Solid, if you don't see it, it may be covered from your graphical view.

##### babaev1 01/31/2021 22:35:44
Bounding region covers solid and I can see it

##### Bitbots\_Jasper 01/31/2021 22:37:09
the density field is used to calculate the mass by using the volume of the bounding object, if a mass is specified it should be -1


are the two solids that should collide part of the same proto?

##### babaev1 01/31/2021 22:38:07
Solids are designed by myself from primitives

##### Bitbots\_Jasper 01/31/2021 22:39:22
what i am trying to ask is if the two solids are defined in the same file?

##### babaev1 01/31/2021 22:39:41
Yes

##### Bitbots\_Jasper 01/31/2021 22:40:09
in the robot node, there is a parameter selfCollision, you might need to enable that


[https://cyberbotics.com/doc/reference/robot](https://cyberbotics.com/doc/reference/robot)

##### babaev1 01/31/2021 22:43:06
It works!


Thank you!

##### Bitbots\_Jasper 01/31/2021 22:43:39
you're welcome 👍

##### babaev1 01/31/2021 22:44:11
👏

##### paperwave 01/31/2021 22:49:14
Question, I'm working on the tutorials (on 6) but I'm getting a compile error because some nodes' name field is the same, is there a way in the IDE to change that? double click on name isn't doing anything


compile error
%figure
![wheel_not_found.PNG](https://cdn.discordapp.com/attachments/565154703139405824/805570707551027250/wheel_not_found.PNG)
%end


[https://cyberbotics.com/doc/reference/solid#unique-solid-name](https://cyberbotics.com/doc/reference/solid#unique-solid-name) indicates the IDE should make a unique name hmmm


wait those are warnings, so it should run, I just want to use better descriptive names

##### Bitbots\_Jasper 01/31/2021 22:55:09
you can right click on the robot in the node view on the left and edit the proto file(view protofile) , control+f for name and find the names that are overlapping

##### paperwave 01/31/2021 22:58:07
I don't see that, I right clicked on the robot's root node, "Robot", I'll keep looking for a view protofile option somewhere

##### Bitbots\_Jasper 01/31/2021 23:00:31

%figure
![jo.png](https://cdn.discordapp.com/attachments/565154703139405824/805573259717246996/jo.png)
%end

##### paperwave 01/31/2021 23:04:38
Here's what I'm seeing, I made it with Base nodes > Robot



%figure
![missing_settings.PNG](https://cdn.discordapp.com/attachments/565154703139405824/805574353571938334/missing_settings.PNG)
%end


I see, it doesn't have that because it was a base node.


looking for the wbt file, maybe that'll do it


It's my second day of webots so I know it's something simple

##### Bitbots\_Jasper 01/31/2021 23:10:47
The problem is that it is probably save in a location where you can not edit it (because you need sudo or admin). Copy the proto file from your webots installation to your own proto folder


maybe this helps to understand how you should structure your webots projects: [https://cyberbotics.com/doc/guide/the-standard-file-hierarchy-of-a-project](https://cyberbotics.com/doc/guide/the-standard-file-hierarchy-of-a-project)

##### paperwave 01/31/2021 23:13:19
moving the protos folder now


Looks like this part of the tutorial only uses base nodes so there was no proto on it however, I restarted Webots and was able to rename the wheels which is great. Fixing something else I messed up present, thanks for the help

## February

##### まちこ (Bailey) 02/01/2021 05:12:30
I'm trying to change the fields of an object, but it looks like this. What do I need to do so that I can edit them? (This is my first time using Webots, sorry if it's a dumb question!)
%figure
![Screen_Shot_2021-01-31_at_9.11.53_PM.png](https://cdn.discordapp.com/attachments/565154703139405824/805666868773781504/Screen_Shot_2021-01-31_at_9.11.53_PM.png)
%end

##### Stefania Pedrazzi [Cyberbotics] 02/01/2021 07:06:18
There is a dedicated window to edit the field, but it seems that you hide it.

It is located just below this scene tree.

So to show you can click on the sidget separator (indicated by the three lines on the bottom of your screenshot) and drag it up, or restore the default application layout from the `Tools > Restore Layout`

##### まちこ (Bailey) 02/01/2021 07:15:46
Thank you!

##### iagsav 02/01/2021 12:31:47
Hi! Please tell me how you can read the data from the compass of the Firebird 6 robot?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 12:35:42
`@iagsav` if you right-click on the robot, you can select "view PROTO source"


in there you will find this section, with the sensors and their names
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/805778505736519750/unknown.png)
%end

##### iagsav 02/01/2021 12:40:56
Thank you!


I wrote this code


cmp = robot.getDevice('compassXY\_01')

print(cmp)


but it returns 'None'

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 12:42:36
for i in range(robot.getNumberOfDevices()):

    device = supervisor.getDeviceByIndex(i)

    print(i, '  -  ', device.getName(), '   - Model:', device.getModel(),'   - NodeType:', device.getNodeType())


try running this


it should print all devices


also a good way to make lists of motors, sensors etc. by appending a list, if the device is of the type you want

##### iagsav 02/01/2021 12:44:35
should I set the supervisor field to true?



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/805780673922859038/unknown.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 12:45:12
oh, do it robot.getDeviceByIndex


supervisor is not needed


took it from one of my controllers


where I use a supervisor for other stuff

##### iagsav 02/01/2021 12:46:15
Thanks you! it worked



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/805781066035757117/unknown.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 12:46:47
hmm model doesnt seem to work for some reason


sensors = []

minPositions = []

maxPositions = []

for i in range(robot.getNumberOfDevices()):

    device = robot.getDeviceByIndex(i)

    print(i, '  -  ', device.getName(), '   - Model:', device.getModel(),'   - NodeType:', device.getNodeType())

    # if device is a rotational motor (uncomment line above to get a list of all robot devices)

    if device.getNodeType() == 54:  

        motors.append(device)

        minPositions.append(device.getMinPosition())

        maxPositions.append(device.getMaxPosition())

        sensor = device.getPositionSensor()

        try:

            sensor.getName()

            sensors.append(sensor)

            sensor.enable(timeStep)

        except Exception as e:

            print('Rotational Motor: ' + device.getName() + ' has no Position Sensor')


\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_

this is an example, how I initialize robotic arms. It automatically creates a list of all rotational motors, their position sensors and the corresponting min and max positions


motors = [] missing in the first line


This works regardless of what robotic arm I use. I'm sure you can adjust it to your needs

##### iagsav 02/01/2021 12:52:23
Thank you! I will try!


It is very strange, but when I run e-puck obstacle avoidance tutorial, I get such result



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/805786926274576474/unknown.png)
%end


I reinstall python 3.7.9 (64) twice


and restart webots too

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 13:10:37
ps[i] is empty in this case

##### iagsav 02/01/2021 13:11:33
I understand, but why? I use standard world file

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 13:11:58
show me the controller?

##### iagsav 02/01/2021 13:12:18

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/805787617240154132/unknown.png)
%end


I take it from tutorial


[https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers?tab-language=python](https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers?tab-language=python)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 13:13:32
put a "print(i)" in front of the ps[i].enable

##### iagsav 02/01/2021 13:14:12

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/805788096221544488/unknown.png)
%end

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 13:15:19
weird, that error is new now


you are using the correct robot?

##### iagsav 02/01/2021 13:15:32
I found similar issue here: [https://github.com/cyberbotics/webots/issues/2570](https://github.com/cyberbotics/webots/issues/2570)


I use file e\_puck.wbt, I think it is correct

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 13:16:56
you are on the newest version of webots?


you could also try running that loop I showed you earlier

##### iagsav 02/01/2021 13:17:16
I download it now


Now i install newest webots version


It is very strange, but all works: I get compass in fire bird 6

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 13:39:06
great 🙂

##### iagsav 02/01/2021 13:42:06
yee!!! Thank you!!!

##### PRVG 02/01/2021 15:33:51
Hello i would like to know if anyone as implemented a optical flow sensor in Webots. Looking through the documentation i do not think there is one available in the sensor nodes. Thank you very much.

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/01/2021 15:39:16
You are right, there is no such sensor. But basically, this can be implemented at the controller level from a standard camera flow, not inside Webots.

##### PRVG 02/01/2021 15:41:49
Thank you for the reply. I was gonna try implementing  that in matlab just wanna make sure there was no sensor for it already implemented and if someone had any experience with it so they could give some pointers. Thank you

##### Chernayaten 02/01/2021 16:39:29
I'm trying to make k-team's hemisson robot rotate around itself using the setPosition command in Python (with getTargetPosition +/- rotate\_value). However whether I try to increase the position or decrease it the robot always moves forward


I'm also facing an issue with its pen node which simply doesn't work at times. I've made a few tests starting it from random positions and it might work or it might not work. At times when it is not working, if I start manually  moving the robot around while the simulation is running it might write a bit and then stop or it might start writing normally

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 18:34:50
the setPosition most likely changes the wheel angle, not the robot rotation. So to rotate, you have to turn one clockwise and the other ccw

##### Chernayaten 02/01/2021 18:39:45
That is what I am doing. I use the motor.getTargetPosition() to get the position at that specific moment and then increase one wheel / decrease the other to make it turn. I've already used this code on a robot I made and it has been working without issues


I've performed the following simple test. Set both wheel positions at 100 and run it. The robot moves forward. Reset and set both wheel positions at -100. Robot moves forward again, even though it should be moving backwards

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 18:53:34
for wheels you should probably use velocity control


and change the angles incrementally. Also remember, that the values are radian


so 100 is a lot

##### Chernayaten 02/01/2021 18:56:41
I've been trying to make the robot turn exactly 90 degrees (and then other angles) which is why I have been using position instead. I am aware values are radian,  the 100 is just an example to show it is not working as it should


Weirdly enough, if I do use setVelocity with +/- on the wheels it does work as it should. I really don't want to have to re-work my code though. The setPosition should be working as well which is what I am trying to solve

##### PymZoR [Premier Service] 02/01/2021 19:05:13
Hi everyone, I was wondering if someone tried to use asyncio for a python controller ?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/01/2021 20:02:38
if your starting position is -200, then it will drive forward in both cases, setting the target to -100 and 100

##### Chernayaten 02/01/2021 20:38:15
That's why I said in my first message that I use the .getTargetPosition() method. The increase/decrease is always done in regards to the wheel position at that moment. That means that if my starting position was -200 then one wheel would increase to -100 and the other would decrease to -300

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/02/2021 10:57:01
`@Chernayaten`  Why are you using .getTargetPosition() instead of using the position sensor to get the current position?


positionSensor = motor.getPositionSensor()

positionSensor.enable(timestep)

pos = positionSensor.getValue()


I suspect that the issue lies here


.getTargetPosition()  gives you the last command issued with setPosition() and NOT the current position

##### Chernayaten 02/02/2021 13:12:48
I am doing it this way because I do not care about the current position of the motor. My code is working correctly in my own robot, it works correctly on a different robot I randomly picked (Lego's Mindstorms), but it doesn't work correctly on the Hemisson robot.


while self.step(self.TIME\_STEP) != -1:

    left.setPosition(left.getTargetPosition() - 10)

    right.setPosition(right.getTargetPosition() + 10)


Regardless of whether this is the smart way to do it, the following code should make the robot constantly rotate around itself. Hemisson robot doesn't do that and I can't figure out why. At the same time the other two robots I mentioned do

##### alireza\_9 02/02/2021 18:39:19
Hello everyone

dose anyone knows how can I train robot using pytorch in webots?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/02/2021 18:59:26
[https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers)


use an extern python controller

##### John520 02/02/2021 20:25:42
Hi, I'd like to create a crop field for lidar detection simulation. I found out that Webots provides Trees and Plants objects. My questions are: 1. Can the tree and plant objects (shape and leaves) be detected by a lidar in the simulation? 2. Is there a way to create a crop object by myself? Thank you!

##### LucasW 02/03/2021 00:55:54
Hi all, I'm a student working on a project using Webots for the first time. This is a pretty basic question but I'm implementing an approximation of the VL53L1X tof sensor see [https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html#overview](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html#overview) for details, or [https://www.pololu.com/product/3415/specs](https://www.pololu.com/product/3415/specs) for a quick specs overview. I was wondering what units resolution is measured in/should I just make it infinite and I don't understand what response values I should map the distances too? Any help would be appreciated thanks.

##### John520 02/03/2021 15:03:45
Hi guys, I'd like to create a crop field for a lidar detection simulation. I found out that Webots provides Trees and Plants objects. My questions are, 1. Can the Tree and Plant objects (shape and leaves) be detected by a lidar in the simulation? 2. Is there a way to create a crop object by myself? Thank you!

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 02/03/2021 15:07:50
Hello `@John520`, 1. Yes. You can create a simple robot, add a Lidar, and visualize the point cloud (option in the View menu). 2. Yes. Create a terrain, add some plants, trees...



There are a few examples on YouTube (created in a very old version of Webots):

[https://www.youtube.com/watch?v=X7YFyVZc1Eg](https://www.youtube.com/watch?v=X7YFyVZc1Eg)


Hello `@LucasW`, the returned values from the DistanceSensor node are in meters.

##### John520 02/03/2021 15:13:24
`@Darko Lukić` Thank you very much for your reply. Would be possible to create a crop or plant that is not included in the Trees and Plants objects by myself. For example, can I create a cereal object somewhere and then import it into Webots? Thank you!

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/03/2021 15:14:47
Yes, of course. It is just a matter of creating a new proto file.

##### gaitt 02/03/2021 15:16:22
Hello, I'm wondering how do I retrieve the camera behind a DEF. I'm using C++ API, supervisor->getNodeFromDef(CAMERA\_DEF) which seems to return valid pointer on a Node. But since a Device/Camera does not inherit from node I can't cast it to a camera ... any clue?

##### John520 02/03/2021 15:16:59
Oh sounds great! Thank you `@Olivier Michel`. Would you have any tutorial about creating a new proto file? Could you share a link, please? Thank you!

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/03/2021 15:17:15
You should use the `getCamera` method instead.


[https://cyberbotics.com/doc/guide/tutorial-7-your-first-proto](https://cyberbotics.com/doc/guide/tutorial-7-your-first-proto)

##### John520 02/03/2021 15:20:07
Thank you very much `@Olivier Michel` for your help!!!

##### gaitt 02/03/2021 15:25:44
Well did not find such method in the documentation, on which class the method is available?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/03/2021 15:26:52
on Robot (from which Supervisor inherits).

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/03/2021 15:28:21
[https://drive.google.com/drive/folders/1rI7xeOP5CCi\_IFTOmZu37nEZDxR-Uj6\_?usp=sharing](https://drive.google.com/drive/folders/1rI7xeOP5CCi_IFTOmZu37nEZDxR-Uj6_?usp=sharing) this could be useful

##### gaitt 02/03/2021 15:33:29
Ok thanks, I found it in the .hpp but it definitely not in the doc [https://cyberbotics.com/doc/reference/robot](https://cyberbotics.com/doc/reference/robot)

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/03/2021 15:34:06
It's there: [https://cyberbotics.com/doc/reference/robot?tab-language=c++#wb\_robot\_get\_device](https://cyberbotics.com/doc/reference/robot?tab-language=c++#wb_robot_get_device)


The C function is called `wb_robot_get_device`.

##### gaitt 02/03/2021 15:35:17
My bad, I was searching for the C equivalent *\_get\_camera

##### John520 02/03/2021 17:25:14
Hi `@Olivier Michel`, I am looking into the Cypress.proto [https://github.com/cyberbotics/webots/blob/released/projects/objects/trees/protos/Cypress.proto](https://github.com/cyberbotics/webots/blob/released/projects/objects/trees/protos/Cypress.proto). Would you please advise how to define the point values for "geometry IndexedFaceSet"? There are lots of point values there. Thank you!

##### LucasW 02/03/2021 17:30:19
Thanks `@Darko Lukić` but I don't think I was very clear. I'm a confused by the lookup table response values. I've read the Webots documentation but still not sure how you set the response values based on different sensors. For example in the webots\_ros2/webots\_ros2\_epuck/protos/E-puck\_enu.proto there is a tof sensor. Which approximates the VL53L0X sensor and I don't understand why those corresponding response values were chosen, I've looked at the sensor documentation and the Webots documentation and am still not sure why?

##### iagsav 02/03/2021 17:38:45
Hi! please explain how you can add a compass to the e-puck robot?

##### Krish 02/03/2021 18:49:01
`@John520` Did you forget to leave the voice channel?


Its been a while

##### John520 02/03/2021 19:18:15
`@Krish` Thanks. I forgot about it.

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/04/2021 07:04:37
IndexedFaceSet are usually designed as meshes in a 3D modelling software (like Blender or other) and then exported to VRML97, copy/pasted in to the PROTO file.

##### Stefania Pedrazzi [Cyberbotics] 02/04/2021 07:20:32
You should add it in the `turretSlot` filed of the e-puck.

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 02/04/2021 08:06:40
Ok, I see. It is possible that GCtronic integrated API that returns scaled values. However, if you don't have a strong reason to use the `lookupTable` parameter, you can skip modeling that part. Without the lookup table you will get actual distance. The other parameters, `numberOfRays`, `aperture`, and`resolution` are more important for modeling a realistic sensor. Also, you can use the lookup table without scaling the values, only imposing a noise at different distances.

##### Iris230 02/04/2021 08:47:29
hello guys, I'm wondering how could Logitech G29 get connected to webots and control the car  just like this?



%figure
![Screenshot_20210204_163925_com.google.android.youtube.jpg](https://cdn.discordapp.com/attachments/565154703139405824/806808172630507539/Screenshot_20210204_163925_com.google.android.youtube.jpg)
%end


I opened a sample world called 'village center' and I added a BMW node. But I can't change to this point of view, like you are sitting in this car. 🥲😭

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/04/2021 08:54:20
A sample controller for interfacing a G29 (and other racing wheels) is available in `WEBOTS_HOME/projects/vehicles/controllers/racing_wheel/`.

##### Iris230 02/04/2021 08:58:04
wow thank you so much😆

##### iagsav 02/04/2021 11:03:37
Thank you! It works!!!

##### LucasW 02/04/2021 11:20:24
Thank you very much `@Darko Lukić`

##### John520 02/04/2021 18:09:32
Thank you `@Olivier Michel` again. I am looking for a wheat model online. Do you think if I can convert the wheat field models to proto files so that I can use them in Webots? The wheat field models are in Blender formats, please check [https://www.turbosquid.com/3d-models/section-wheat-field-3d-model-1572937#](https://www.turbosquid.com/3d-models/section-wheat-field-3d-model-1572937#) and [https://www.turbosquid.com/3d-models/wheat-field-3d-model-1175816](https://www.turbosquid.com/3d-models/wheat-field-3d-model-1175816). Or I need to work on the model conversion for single wheat instead of a wheat field?

##### yash 02/04/2021 18:22:11
Hi ! This warning pops up in middle of the program running.....whats the reason behind it ?
%figure
![Screenshot_from_2021-02-04_23-50-28.png](https://cdn.discordapp.com/attachments/565154703139405824/806952763439054878/Screenshot_from_2021-02-04_23-50-28.png)
%end

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 02/04/2021 18:23:45
It means that the controller has crashed. Try to simplify it

##### yash 02/04/2021 18:26:18
oh okay ! any hints you could give me about how to do it if there's any general solution, or it depends on the way the controller code is written ?

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 02/04/2021 18:26:48
It depends a lot on the controller implementation

##### yash 02/04/2021 18:27:40
okay thanks a lot, let me try it out then.

##### Troy 02/05/2021 03:33:04
Hi everyone! I have a question about Fluid. I looked at the 'floating\_geometries' example, and in my simulation, I set the sane fluid properties, like I set shape, velocity, and boundingObj, and build a solid, set its immersionProperties, but it doesn't work, this solid doesn't even flow in the fluid.


`@Darko Lukić` I added a solid box in the 'floating\_geometries' example, and added shape, immersionProperties, boundingObj, damping, as other geometries, but this solid won't flow with the water. Can you help me? I don't understand why.



%figure
![test.jpg](https://cdn.discordapp.com/attachments/565154703139405824/807094784840826891/test.jpg)
%end


As you can see from the picture, it won't flow with the water


This happens in my another simulation. I don't know how to fix this problem.

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 02/05/2021 08:45:36
`@Troy` Please check this minimal example:

```vrml
#VRML_SIM R2021a utf8
WorldInfo {
  coordinateSystem "NUE"
}
Viewpoint {
  orientation 0.9947291283851613 0.0980963256805878 0.029850829637807465 5.68943875583334
  position -0.4920553249349089 5.987256278548413 8.041045316575675
}
TexturedBackground {
}
TexturedBackgroundLight {
}
DEF STILL_WATER Fluid {
  translation 0 0.2 0
  children [
    DEF S Shape {
      appearance PBRAppearance {
        baseColor 0 0.501961 1
        transparency 0.5
        roughness 1.1102230246251565e-16
        metalness 0
      }
      geometry Cylinder {
        height 0.4
        radius 1.8
        subdivision 24
      }
    }
  ]
  name "swimming pool"
  boundingObject USE S
}
DEF THIN_CYLINDER Solid {
  translation -0.474938 1.52181 0.0751348
  rotation 0.28982509088350455 0.7070662121554249 0.6450261919663017 4.07769
  children [
    DEF S Shape {
      appearance PBRAppearance {
        baseColor 0.501961 0.337255 0.2
        roughness 1.1102230246251565e-16
        metalness 0
      }
      geometry Cylinder {
        height 0.5
        radius 0.12
      }
    }
  ]
  immersionProperties [
    DEF SWIMMING_POOL_IMMERSION_PROPERTIES ImmersionProperties {
      fluidName "swimming pool"
      dragForceCoefficients 0.1 0 0
      dragTorqueCoefficients 0.001 0 0
      viscousResistanceTorqueCoefficient 0.005
    }
  ]
  boundingObject USE S
  physics Physics {
    density 500
    damping Damping {
      linear 0.5
      angular 0.5
    }
  }
}
```


Notice that `ImmersionProperties > fluidName`  and `Fluid > name` have to match.

##### Welsh\_dragon64 02/05/2021 11:35:16
I have been trying to code the robotis darwin-op\_2 to walk using python. I have been trying to access its packages through managers provided by webots, but i keep getting errors and some errors doesnt quite make sense. I have minor experience in python.



> **Attachment**: [robotis\_code\_.txt](https://cdn.discordapp.com/attachments/565154703139405824/807213002692493322/robotis_code_.txt)

##### John520 02/05/2021 15:10:56
Thank you `@Olivier Michel` again. I am looking for a wheat model online. Do you think if I can convert wheat field models to proto files so that I can use them in Webots? The wheat field models are in Blender formats, please check [https://www.turbosquid.com/3d-models/section-wheat-field-3d-model-1572937#](https://www.turbosquid.com/3d-models/section-wheat-field-3d-model-1572937#) and [https://www.turbosquid.com/3d-models/wheat-field-3d-model-1175816](https://www.turbosquid.com/3d-models/wheat-field-3d-model-1175816). Or Do I need to work on the model conversion for single wheat instead of a wheat field? Thanks a lot.

##### Bitbots\_Jasper 02/05/2021 18:23:02
I am a bit confused as to how some of the supervisor funtions work, maybe someone can explain or point me some documentation. The supervisor functions allow me to retrieve nodes in the world either with ID or DEF ([https://www.cyberbotics.com/doc/reference/supervisor?tab-os=linux&tab-language=c++#supervisor-functions](https://www.cyberbotics.com/doc/reference/supervisor?tab-os=linux&tab-language=c++#supervisor-functions)),



 is this the same DEF that is used in the proto file for the DEF-USE mechanism ([https://www.cyberbotics.com/doc/guide/tutorial-2-modification-of-the-environment#def-use-mechanism](https://www.cyberbotics.com/doc/guide/tutorial-2-modification-of-the-environment#def-use-mechanism)) or is it something different?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 18:23:59
`@Bitbots_Jasper` in the scene tree, you can give any node a DEF. You can then retrieve that node with the supervisor


nodes inside of proto files are a bit more tricky

##### Bitbots\_Jasper 02/05/2021 18:24:35
those are the ones I'm trying to reach

##### Welsh\_dragon64 02/05/2021 18:25:00
guys i have a question



how do i enable the robot window using roboits darwin\_op2 in webots using python?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 18:25:02
what exactly are you trying to do?

##### Bitbots\_Jasper 02/05/2021 18:25:24
finding the position of certain solids in world coordinates

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 18:26:06
what robot?

##### Bitbots\_Jasper 02/05/2021 18:26:19
my own 😄


[https://github.com/bit-bots/wolfgang\_robot/blob/master/wolfgang\_webots\_sim/protos/Wolfgang.proto](https://github.com/bit-bots/wolfgang_robot/blob/master/wolfgang_webots_sim/protos/Wolfgang.proto)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 18:26:41
you can expose fields in a proto


like you do line 8 -> line 21


you can do that for any field


ahh.. you want the node so you can get the position


should work the same

##### Bitbots\_Jasper 02/05/2021 18:29:44
i read that there is wb\_supervisor\_node\_get\_from\_proto\_def or getFromProtoDef in python, should that not work as well?


[https://www.cyberbotics.com/doc/reference/supervisor?tab-os=linux&tab-language=python#description](https://www.cyberbotics.com/doc/reference/supervisor?tab-os=linux&tab-language=python#description)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 18:30:13
ohh right


that shouuld work


I only used fields inside of protos so far

##### Bitbots\_Jasper 02/05/2021 18:30:53
sorry this link is correct: [https://www.cyberbotics.com/doc/reference/supervisor?tab-os=linux&tab-language=python#wb\_supervisor\_node\_get\_from\_proto\_def](https://www.cyberbotics.com/doc/reference/supervisor?tab-os=linux&tab-language=python#wb_supervisor_node_get_from_proto_def)

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 18:33:23
baseNode = supervisor.getSelf()

node1 = baseNode.getFromProtoDef("defName")

pos = node1.getPosition()


I think that should work

##### Bitbots\_Jasper 02/05/2021 18:38:29
awesome, it works, i only have to go back into my proto and add a DEF to each solid :D, or is there a option to do it using urdf2webots, otherwise i can work on implementing it

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 18:39:14
urdf files dont have DEF names. they have names


I guess you could add a parsable option, to turn urdf link-names into DEF names for the solids, not just solid names

##### Bitbots\_Jasper 02/05/2021 18:40:58
that is what i meant, so that they can be retrieved using the getFromProtoDef function

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 18:41:57
[https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/writeProto.py#L101](https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/writeProto.py#L101)


you would have to add it in this line


link.name contains the link-name

##### Bitbots\_Jasper 02/05/2021 18:42:43
thanks so much for your help `@Simon Steinmann` , i'll work on it

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 18:43:06
[https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/importer.py#L190](https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/importer.py#L190) here you have to add a parsable argument


you can parse a bool to the writeProto script like this: [https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/importer.py#L64](https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/importer.py#L64)

##### Welsh\_dragon64 02/05/2021 18:47:09
`@Simon Steinmann`
%figure
![Robotis_op2.PNG](https://cdn.discordapp.com/attachments/565154703139405824/807321432954634260/Robotis_op2.PNG)
%end


can you look at this error plz?


i cant seems to display the robot window?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 18:48:20
is the library at the specified path?

##### Welsh\_dragon64 02/05/2021 18:48:33
yea


for some reason it cant seem to load it


the library is in the same the file i am using to work on this robot simulation


i am trying to remotely control the robot using my laptop

##### Bitbots\_Jasper 02/05/2021 19:03:41
`@Simon Steinmann`  i made a pull request, thanks again for your help [https://github.com/cyberbotics/urdf2webots/pull/107](https://github.com/cyberbotics/urdf2webots/pull/107)


I think it depends on your use case, to convert the model to a proto you can use [https://github.com/cyberbotics/blender-webots-exporter](https://github.com/cyberbotics/blender-webots-exporter) as far as i know. If you want to very precisely place every little wheat plant you need individual models, if you only want to place it roughly you can probaly use the tiles they provide in the models you linked to. You need to pay a little attention to the amount of polygons you are using in the end, the models you linked to are quite high resolution and it might cause some issues if there is a gigantic amount of them on the screen.


if they do not come as a tile, you can create your own tile using single wheat plants. for that [https://www.cyberbotics.com/doc/reference/def-and-use](https://www.cyberbotics.com/doc/reference/def-and-use) is probably useful

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/05/2021 19:44:39
`@Bitbots_Jasper` PR looks good to me, havent tested it though. Please also add your changes to the readme [https://github.com/cyberbotics/urdf2webots/blob/master/README.md#arguments](https://github.com/cyberbotics/urdf2webots/blob/master/README.md#arguments)

##### John520 02/05/2021 21:18:25
Thank you `@Bitbots_Jasper` for your detailed instruction! I will give it a try.

##### babaev1 02/05/2021 21:42:20
Is there anybody familiar with backlashes in joints of robot?  Is there any method to define value of backlash in HinjeJoint or in RotationalMotor?

##### Bitbots\_Jasper 02/05/2021 22:29:52
done


can you give some more information on your setup? have you checked if the file that is missing `robotis-op2_window.dll` is not in your filesystem, as a last resort you could try to reinstall webots

##### Welsh\_dragon64 02/05/2021 23:16:52
`@Bitbots_Jasper` turns out some files were missing such as transfer,remote control and checkstart position. i just copied paste the webot orignal file to the file i am editing and that fixed the issue i had.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/807389310331519056/unknown.png)
%end


`@Bitbots_Jasper` `@Simon Steinmann` I am trying to work on the object detection on Robotis\_op2 on webots. I would like to know which is better YOLO or Open CV. I would also appreciate any open source codes available

##### Anand 02/06/2021 00:49:15
Hey! I have set all the env variables correctly. Yet when I type "webots" in the terminal, I get an error saying "Command not found, but can be installed with snap". Any idea how to fix this?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/06/2021 10:55:52
`@Anand` you have to add an entry in the .bash\_aliases launching the executable


`alias webots='/home/simon/webots/webots'`  something like this

##### Bitbots\_Jasper 02/06/2021 11:37:01
It depends on what you want to detect. If it is just a red circle on a flat background open cv is fine, if you want to detect different object classes you might want to use yolo, but it could be overkill

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/06/2021 11:39:07
I've only dabbled in computer vision, but I think going opencv first is the best approach. You can add YOLO to opencv


`@Welsh_dragon64` If you want to use domain randomization for training your cv, I can probably give you a starting point. I played around with it last year [https://drive.google.com/file/d/1Z2332YFU1Um1NCwx3PoTcOE2jdu6QUxP/view?usp=sharing](https://drive.google.com/file/d/1Z2332YFU1Um1NCwx3PoTcOE2jdu6QUxP/view?usp=sharing)

##### Iris230 02/07/2021 09:52:18
hi guys, I'm wondering what does this error mean?
%figure
![CHVBGQG0CMSPSG0WS0K.png](https://cdn.discordapp.com/attachments/565154703139405824/807911611695890432/CHVBGQG0CMSPSG0WS0K.png)
%end


I just opened the 'highway' sample world, and I changed the controller to the 'racing\_wheel'

##### Usama 02/07/2021 17:43:24
hi guys, i am new to webot, i am trying to add a lidar sensor to my robot but i cannot seem to find it in the base nodes.  in fact i can not find any sensor. how can i add the sensors to the node library?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/07/2021 17:44:13
Sensors have to be children of Solids, Robots or Transforms


and they only work in a robot

##### Usama 02/07/2021 17:45:04
okay thanks

##### Stefania Pedrazzi [Cyberbotics] 02/08/2021 09:35:49
Hi, to recompile the `racing_wheel` controller on Windows you can add the following line in the `Makefile` just after the `ifeq($OSTYPE),windows)` line:

```
LIBRARIES += -L"$(WEBOTS_HOME)/msys64/mingw64/bin"
```

##### yash 02/08/2021 12:21:30
Hi ! Are there any functionalities to model under-actuated behaviour ?  for example a  single motor drives multiple hinge joints.

##### bertlangton 02/08/2021 12:28:17
I have an issue starting up WeBots; it opens, but immediately shuts down after a couple of seconds. I've already tried: 

-Updating graphics card drivers (I have an integrated AMD card)

-Starting in Safe Mode 

-Older versions 

What could be the issue? My computer isn't exactly new, but I believe it does have the required minimum specifications for the program to run.

##### Stefania Pedrazzi [Cyberbotics] 02/08/2021 12:30:30
Is this the first time you start Webots or at some point it was starting?

##### robotjoaa 02/08/2021 12:34:09
Hi, I was trying to do stuff with physics ODE plugin. I wonder is there any way to create solid object that doesn't physically interact with objects but still detects collision?

##### Stefania Pedrazzi [Cyberbotics] 02/08/2021 12:34:48
Currently it is not possible to control multiple joints with the same motor, but this functionality is in our TODO list:

[https://github.com/cyberbotics/webots/issues/1365](https://github.com/cyberbotics/webots/issues/1365)

##### robotjoaa 02/08/2021 12:34:57
ex) ball collides with box but box doesn't get pushed by the ball. Still collision is detected by the box.

##### Stefania Pedrazzi [Cyberbotics] 02/08/2021 12:36:20
Yes, you should define the `boundingObject` field of a `Solid` node but not the `physics` field.

##### robotjoaa 02/08/2021 12:37:00
I tried setting physics with null, but when I moved that object around, it still interacted with other objects.

##### Stefania Pedrazzi [Cyberbotics] 02/08/2021 12:41:02
Ok, so you want to detect the collision to do your stuff but then let the physics engine ignoring it.

This is possible from the physics plugin `webots_physics_collide` function. If this function returns 1 or 2, then ODE will not handle the collision:

[https://www.cyberbotics.com/doc/reference/callback-functions#int-webots\_physics\_collidedgeomid-dgeomid](https://www.cyberbotics.com/doc/reference/callback-functions#int-webots_physics_collidedgeomid-dgeomid)

##### robotjoaa 02/08/2021 12:42:56
Thanks

##### bertlangton 02/08/2021 12:43:50
Nope, it never lasted more than a few seconds

##### Stefania Pedrazzi [Cyberbotics] 02/08/2021 12:50:06
Which OS are you using?

##### bertlangton 02/08/2021 13:24:39
Windows 10

##### Stefania Pedrazzi [Cyberbotics] 02/08/2021 13:47:01
It would be helpful if you could open the Command Prompt, navigate to the Webots installation directory, type the following instructions and post the output:

```
cd msys64\mingw64\bin
webots.exe --sysinfo
```

##### bertlangton 02/08/2021 15:21:53
sure, here it is:



System: Windows 10 64-bit

Processor: Intel(R) Core(TM) i3-6100U CPU @ 2.30GHz

Number of cores: 2

OpenGL vendor: ATI Technologies Inc. (0x1002)

OpenGL renderer: AMD Radeon (TM) R5 M430 (0x6660)

OpenGL version: 4.5.13469 Compatibility Profile Context 21.19.512.4

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/08/2021 15:28:18
Did you try to update your graphics driver?

##### bertlangton 02/08/2021 15:38:29
Yup.


After that, it stayed open for a few seconds longer, but other than that it didn't help

##### Stefania Pedrazzi [Cyberbotics] 02/08/2021 15:47:52
The OpenGL version seems rather new. I'm checking in our telemetry database to see if other users could successfully run Webots with this OpenGL version and similar setup. But for the moment it seems that most of the users that enabled the telemetry are using older OpenGL versions (usually 3.3, for example `3.3.13596 Core Profile Forward-Compatible Context`)


Maybe it could also be an option to try to downgrade the drivers to check if it works.

##### Iris230 02/08/2021 16:05:00
thank you! It works😆

##### Stefania Pedrazzi [Cyberbotics] 02/08/2021 16:07:02
Thank you for reporting this bug! I updated the Webots Makefile and the fix will be included in the next release of Webots.

##### alejanpa17 02/08/2021 16:38:47
Hey guys, i need some help here. I have DEF this boundingObject but when i want to use it in other solid It doesnt appear. Am I doing something wrong?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/808376295079018548/unknown.png)
%end



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/808376461911261244/unknown.png)
%end

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/08/2021 16:50:00
You can include a USE of this DEF only at a lower position in the scene tree. Where are you trying to include a USE of it?

##### alejanpa17 02/08/2021 16:52:41
For example in this one
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/808379791899099136/unknown.png)
%end


I just found out that you can include a USE only upper the DEF (in the way closer to the root of the scene tree). So if it is at the same level is impossible?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/08/2021 17:09:52
Yes, it should be possible at the same level, but the DEF has to come first (upper).


If you save your world file and search for the USE/DEF name, you should first find the DEF, then the USE.

##### alejanpa17 02/08/2021 17:26:24
Look, I have it the DEF there but the solids where I'm pointing It doesn't appear the USE
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/808388279052533810/unknown.png)
%end


I think is kind of a bug because I did a boundingObject DEF of a Box for testing and It works with the solids under it, just like you said


Its happening when you have this type of structure of the boundingObject DEF: Group>>Transform>>Box
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/808396782168965120/unknown.png)
%end


You can try yourself

##### Stefania Pedrazzi [Cyberbotics] 02/08/2021 19:29:02
`@alejanpa17` The DEF "test" node can only be used in `boundingObject` fields because this structure (without a `Shape` node between the `Transform` and the `Box` nodes) is a particular simplified structure not valid for graphical objects.

If you also want to use it outside the `boundingObject` to define the a graphical object, then I would suggest you to declare the DEF node for the graphical object and then the USE for the boundingObject.

##### alejanpa17 02/08/2021 20:40:19
Just found out this is working!! It turns out that you need a Shape node between Transform and Box node. Thank you!!
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/808437079990206534/unknown.png)
%end

##### AJ121 02/09/2021 06:51:52
Which sensor in webots can differentiate between colors?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/09/2021 07:04:37
A standard RGB camera.


Note: an infra-red distance sensor is also affected by colors of obstacles.

##### Bitbots\_Jasper 02/09/2021 07:05:39
if you just want a simple color sensor you can probably set the resolution to 1x1 pixels

##### AJ121 02/09/2021 07:06:00
I want to differentiate between black and white paths in a line follower

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/09/2021 07:07:00
In that case, it is simpler and more efficient to use a DistanceSensor with type set to "infra-red". You will measure a larger value for the white and a lower value for the black.

##### AJ121 02/09/2021 07:07:47
Somehow the reading of my infrared come just the opposite for white it is <1000 and for black it is 1000

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/09/2021 07:10:18
That may depend on the values set in lookupTable of the DistanceSensor.

##### AJ121 02/09/2021 07:11:31
Ok thanks !


I need a bit of help with my line following algorithm

##### popo74013 02/09/2021 09:05:00
i have a question related to the camera that works as rgb


i try to use the camera to read rgb but i am not getting the write value


my camera is close to an ir sensor, deos this affect it?

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/09/2021 09:07:09
No, it shouldn't.

##### Bitbots\_Jasper 02/09/2021 09:07:17
you can try to visualize the cameras viewframe using view->show optional renderings->show camera frustum


maybe you also need to start your camera [https://cyberbotics.com/doc/reference/camera#wb\_camera\_enable](https://cyberbotics.com/doc/reference/camera#wb_camera_enable)

##### popo74013 02/09/2021 09:09:41
i did use enable command


but i am reading the wrong value


is it possible to go on a audio call with anyone?

##### Bitbots\_Jasper 02/09/2021 09:10:45
did you try this?

##### popo74013 02/09/2021 09:10:56
doing it now


it doesnt make any difference


i am quite sure that there is an issue related  to code


def colour():

    #identify colour

    cam.enable(TIME\_STEP)

    cameraData = cam.getImage()

    # get the red component of the pixel (5,10)

    red = cam.imageGetRed(cameraData, cam.getWidth(), 1, 1)

    green = cam.imageGetGreen(cameraData, cam.getWidth(), 1, 1)

    blue = cam.imageGetBlue(cameraData, cam.getWidth(), 1, 1)

    # red = pixel[0]

    # green = pixel[1]

    # blue = pixel[2]

    cam.disable()

    return (red,green,blue)


this is the function to read the colour


but the camera most of the time reads a certain colour and only at some random moments it reads a couple of more colours

##### Bitbots\_Jasper 02/09/2021 09:21:16
enable your camera once and then let it run, dont try to enable and disable it at every simulation step

##### popo74013 02/09/2021 09:22:47
compass = robot.getDevice('compass')

compass.enable(TIME\_STEP)


i have added these bits outside of the function

##### Bitbots\_Jasper 02/09/2021 09:24:06
i think it takes one simulation step to get the image from the simulation into the frame buffer of the camera


to test you can put a step simulation between the cam.enable and the cam.getImage

##### popo74013 02/09/2021 09:27:46
how can i do this


and why does reads red when there is nothing red in front of the robot?

##### Bitbots\_Jasper 02/09/2021 09:33:09
```
simulation -> camera buffer -> your code
            1                2
```


1 only happens when you step the simulation


and you are therefore doing 2 before 1


and there might be anything in the camera buffer before it is properly initialized


that is the nature of RAM


it should be something like controller.step(TIME\_STEP)

##### popo74013 02/09/2021 09:40:16
i think i figured it out


when you click on the camera which is located at the children file of the robot


i moved the pink pyramid shape of the camera in the front and now it reads the right colour


what is actually the pink pyramid?

##### Bitbots\_Jasper 02/09/2021 09:42:32
it is called the camera frustum


basically what the camera sees

##### popo74013 02/09/2021 09:42:45
ohhh


thank you! now it works perfectly and i get the rgb values


thank you!

##### nelsondmmg 02/09/2021 13:42:16
Hi, I'm would like to write a sh file that executes multiple webots simulations (sequentially), changing the arguments of the controllers used inside the simulation. Can I modify such parameters from the command line directly or do I need to modify the .wbt file before each execution? Also, is it possible to register videos from the simulation using the command line (if possible without opening the webots window). How can I automatize these simulations .In the documentation the "Starting webots" page only gives details about launching the application. Thanks.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/09/2021 13:44:16
`@nelsondmmg` you could use extern controllers and start those seperately, parsing arguments with python for example

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 02/09/2021 13:44:17
`@nelsondmmg` You can add a Supervisor node which changes the arguments and restarts the simulation. Same for videos, you can use the Supervisor node to record videos.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/09/2021 13:46:24
you can also load the world from a python script



> **Attachment**: [launch\_webots\_world.py](https://cdn.discordapp.com/attachments/565154703139405824/808695365048991764/launch_webots_world.py)


on the bottom of the script is the example usage under \_\_main\_\_()


example on how to start your python controller
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/808697787134771261/unknown.png)
%end


this will launch the world, and make the extern controller connect to the correct webots instance

##### nelsondmmg 02/09/2021 14:06:13
Ok, but since I'm already working with C++ I would like to use it instead of python (but it's nice to know that there is an alternative). Now, if I create a supervisor node to manage the simulation (keeping in mind that all the objects in the simulation are also supervisors, since everyone recovers the position from the others directly from the node) how can I launch the other controllers (reset the controllers and the simulation and change the arguments can be done using the supervisor functions, but I do not see a start function). Also, if there a way to known that a controller ended execution, or some sort of message needs to be sent to the supervisor ?

##### popo74013 02/09/2021 14:11:23
hello, i have a question realated to motors


i try to se their position but they do some funny moves


i think it might be an issue with the reference position

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/09/2021 14:12:57
`@nelsondmmg` If it is the same world and robots, just turn the controllers into extern controllers. After a reset or world load, the simulation will only start, if all extern controllers have been started. So you can just launch them with the specific args you want


`@popo74013` you have to give us more details than that. Perhaps show us your controller code and explain exactly what it is doing, and what it SHOULD do

##### popo74013 02/09/2021 14:17:07
i made a robot and i used 2 motors to control the gripping mechanisms


and once the robot moves and detects an object it is supposed to grip it


but when i set the motors' position at an angle, they move randomy


i think it is because they are not oriented correctly

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/09/2021 14:19:08
Make sure the axis of the jont is defined properly


also the anchor and 'translation' field of the endPoint Solid for each joint


usually you want them to be identical

##### popo74013 02/09/2021 14:20:09
ohhh, thank you


i will try it and i will let you know!

##### nelsondmmg 02/09/2021 14:21:50
Is there some documentation about extern controllers? Maybe is a new feature and I'm not familiarized, but since there is some dependency related to the control of objects in the code (in my case I need to initialize the driver object to control vehicles) I should not start these controllers outside webots and then attach then to the webots process.

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/09/2021 14:22:27
[https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers)

##### nelsondmmg 02/09/2021 14:40:59
Ahh thanks, now it is much more clear.

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/09/2021 15:51:37
It could be very nice if you could share your experience using Webots with ROS at [https://discourse.ros.org/t/why-is-robotics-simulation-hard/15888/11](https://discourse.ros.org/t/why-is-robotics-simulation-hard/15888/11)

##### popo74013 02/10/2021 10:02:13
hello


is there a way to make a motor act as a servo?


i am trying to make a really simple grabbing mechanism and i just want to make 2 beams to rotate from 0 to 90 degrees

##### Darko Lukić [ROS 2 Meeting-Cyberbotics] 02/10/2021 10:03:53
Hello `@popo74013`, Yes, if you control the motor using `wb_motor_set_position` it acts as a servo.

##### popo74013 02/10/2021 10:04:15
grips = []

gripsNames = ['left\_motor', 'right\_motor']

for i in range(2):

    grips.append(robot.getDevice(gripsNames[i]))

    grips[i].setPosition(3.14/2)        

    grips[i].setVelocity(0.0)


i am writing in python but this doesnt seem to work

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/10/2021 10:39:36
`@popo74013` dont set the velocity

##### popo74013 02/10/2021 10:39:56
ohh i think it works now


it was an issue in the design


the right grip was working fine but the left not


so i modified the right motor and beam to look like the left and now it works


thank you!

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/10/2021 10:41:21
[https://cyberbotics.com/doc/reference/motor?tab-language=python#force-and-torque-control](https://cyberbotics.com/doc/reference/motor?tab-language=python#force-and-torque-control)


this table tells you exactly what the commands do. Motors can be in position-, velocity-, and torque control

##### Saud 02/10/2021 11:16:36
Hello, I have created a sphere on a new world. I want to know if and how i can increase the size of this shpere with respect to time as the simulation runs? i am very new to webots

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/10/2021 11:22:30
`@Saud` The easiest would be to use a supervisor controller. Get a handle to the radius Field of the sphere, and change the value

##### bingdong 02/10/2021 11:52:53
Hi! New to Webots and Blender. As I understand both support python programming. But is it the case that python scripts can be used in blender to create animations and in Webots to build robot controllers? 

I found a simple legged robot blender file online which makes it move when a python script. When I exported it to Webots, the model doesn't move. I'm guessing that the same script won't work for controller? What can I do to make the robot move then?

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/10/2021 12:11:05
You will have to write a controller [https://www.cyberbotics.com/doc/guide/controller-programming](https://www.cyberbotics.com/doc/guide/controller-programming)


<@&568329906048598039> Is there still separate animation functionality? Not sure about that

##### Stefania Pedrazzi [Cyberbotics] 02/10/2021 12:17:40
No, Webots doesn't have any built-in functionality for animations.

For the moment the way to go is to write a controller program that reads the animation file and using the Supervisor API moves the different robot parts accordingly.

##### bingdong 02/10/2021 12:34:31
`@Simon Steinmann` `@Stefania Pedrazzi` Thanks for the guidance. I'll try it out!

##### Simon Steinmann [ROS 2 Meeting-Moderator] 02/10/2021 12:48:51
is there an example for this? I remember this question coming up 3 times this winter already

##### Stefania Pedrazzi [Cyberbotics] 02/10/2021 12:50:57
Not yet, but it is in our short-term TODO list to add an example for the human animations (and release the experimental Skin node).

##### ChristinaP 02/10/2021 12:51:28
Hello everyone! I'm just starting using webots to simulate an autonomous vehicle. Is there an option to alter the input of the sensors (camera, lidar and gnss) while the simulation is running in order to figure out what's gonna happen at the vehicle in case one of those sensors malfunctions? Do those sensors affect the simulation on runtime?

##### csnametala 02/10/2021 12:56:31
Hello everyone! I'm using a PEN function on the e-puck robot, it leaves a trail that doesn't erase. Can I define a distance from this trail? For it to erase as the robot walks

##### Olivier Michel [ROS 2 Meeting-Cyberbotics] 02/10/2021 13:13:31
You can, at the controller level, add an extra layer that will alter sensor data before passing then to your control algorithm. Also, in Webots, depending on the sensor, you can add different types of noise. See the specific sensor descriptions in the documentation to understand what kind of noise can be added.


Yes, you can use WorldInfo.inkEvaporation, see [https://cyberbotics.com/doc/reference/worldinfo#worldinfo](https://cyberbotics.com/doc/reference/worldinfo#worldinfo)

