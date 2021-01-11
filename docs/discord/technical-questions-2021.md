# Technical-Questions 2021

This is an archive of the `technical-questions` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m) for year 2021.

## January

##### RoboCoder 01/01/2021 00:01:23
I tried 3.9 and 3.7, same result both times.

##### KajalGada 01/01/2021 00:39:04
`@RoboCoder` I assume you are using Windows. What did you set your Python command in Webots preferences?

##### RoboCoder 01/01/2021 01:32:28
I tried a few things, but right now its set to "python"

##### Simon Steinmann [Moderator] 01/01/2021 11:08:41
`@RoboCoder` What PATH variables did you set exactly? You should add something like this:

C:\Users\USERNAME\AppData\Local\Programs\Python\Python38

C:\Users\USERNAME\AppData\Local\Programs\Python\Python38\Scripts


Unless you are on Linux or Mac, then it looks differently.

##### RoboCoder 01/01/2021 16:51:01
Should both of those be set in one PATH variable, or in two separate ones?

##### Simon Steinmann [Moderator] 01/01/2021 17:08:12
just add them beneath each other. Same variable

##### RoboCoder 01/01/2021 18:03:14
Okay. Does the variable name matter?

##### Simon Steinmann [Moderator] 01/01/2021 18:03:34
you are on windows right?


You have to change the "Path" variable [https://docs.alfresco.com/4.2/tasks/fot-addpath.html](https://docs.alfresco.com/4.2/tasks/fot-addpath.html)

##### RoboCoder 01/01/2021 18:08:16
Yep, on windows. I'll try that and get back to you.


Just wondering, does the version matter? I'm on 3.7 right now.


Okay, it's working now! I restarted the computer and it solved my issue. Thank you!

##### Yang 01/02/2021 11:50:31
Hi folks, I'm new to webots, just tried out a few examples on my PC, feels great! I'm wondering whether webots accepts urdf format files and mesh files (.stl etc.)?

##### Simon Steinmann [Moderator] 01/02/2021 12:37:40
`@Yang` you can directly import 3d meshes through the webots menu. In order to convert URDF to webot's PROTO format, use this:

[https://github.com/cyberbotics/urdf2webots](https://github.com/cyberbotics/urdf2webots)


I made a more detailed tutorial on how to do that:

[https://github.com/cyberbotics/urdf2webots/blob/master/docs/tutorial.md](https://github.com/cyberbotics/urdf2webots/blob/master/docs/tutorial.md)

##### Yang 01/02/2021 12:54:10
thank you Simon, I will check that out!

##### Nicolas Y 01/03/2021 14:30:49
Hi guys I'm also very new to Webots and I'm working on creating the 'Yamor Snake' but I can't figure out how to use 'Connectors' to get the pieces to stick together does anyone know how to get it done?

##### Simon Steinmann [Moderator] 01/03/2021 16:23:54
`@Nicolas Y` the hierarchy should look something like this:

Robot > children  > Hingejoint > endPoint Solid > children > Hingejoint > endPoint Solid > children > Hingejoint  ...etc

##### pnaraltnsk 01/03/2021 16:25:38
Hi, I am using webots for my graduation project and I am working on the Nao robot. I am trying to add pen node to my robot to mark the robot's walking path but for some reason, the pen node doesn't write. I added pen node to my robot's leftFootSlot and exactly like in pen.wbt example. Could you please help me out with this situation? Or do you know any other ways to mark the robot's walking path?

##### Simon Steinmann [Moderator] 01/03/2021 16:29:13
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

##### Simon Steinmann [Moderator] 01/04/2021 08:24:47
`@PowerYdRa`  A very simple way to check for "falling off" is to check the height of the robots. If the robot is below the arena surface, it loses. That's like 3 lines of code 🙂

##### PowerYdRa 01/04/2021 08:58:07
yes, thats the idea, but how to show some notification in the arena (not in console) ? any suggestion?

##### Simon Steinmann [Moderator] 01/04/2021 09:07:55
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

##### Simon Steinmann [Moderator] 01/04/2021 09:09:11
let me know if you have questions

##### PowerYdRa 01/04/2021 09:09:24
okay, thanks for your help

##### Simon Steinmann [Moderator] 01/04/2021 09:09:29
robot\_1 = supervisor.getFromDef('Robot\_1')

robot\_2 = supervisor.getFromDef('Robot\_2')



wups, these two lines are not needed

##### Olivier Michel [Cyberbotics] 01/04/2021 09:12:27
You may use supervisor labels to display notifications in the 3D view: [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_set\_label](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_set_label)

##### Simon Steinmann [Moderator] 01/04/2021 09:13:06
ahhh that's what I was looking for :p

##### Olivier Michel [Cyberbotics] 01/04/2021 09:13:20
Or a Display device (which may be a child of your supervisor Robot): [https://www.cyberbotics.com/doc/reference/display](https://www.cyberbotics.com/doc/reference/display)

##### PowerYdRa 01/04/2021 09:14:55
noted, thanks for your reply

will try it after fix the arena view, its look weird only show white 😆

##### Simon Steinmann [Moderator] 01/04/2021 09:38:20
`@PowerYdRa` added the display
> **Attachment**: [sumo\_referee.py](https://cdn.discordapp.com/attachments/565154703139405824/795586909321560084/sumo_referee.py)


even changes the color from green to red, when a robot is out 😄

##### Master.L 01/04/2021 09:47:58
Hi I have a question. I want to rotate the coordinates of the anchor. How can you do it?

##### Simon Steinmann [Moderator] 01/04/2021 09:48:26
What do you mean by anchor?


you mean in a hingejoint?

##### Master.L 01/04/2021 09:49:34
I want to rotate the anchor in the joint parameters of the hinge joint.

##### Simon Steinmann [Moderator] 01/04/2021 09:50:02
it is relative to the parent


so you need to change the parent orientation, or add a transform node inbetween

##### Master.L 01/04/2021 09:50:53
I want to add a joint at a specific location. But I can't hold the axis of rotation in the direction I want.


Where should I add the transform node?

##### Simon Steinmann [Moderator] 01/04/2021 09:52:26
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

##### Darko Lukić [Cyberbotics] 01/04/2021 14:02:48
`@PowerYdRa` Something like that is not possible from Webots. It should be handled by your OS. For example, you can set the world file permissions to read-only mode, so only the administrator can edit it.

##### PowerYdRa 01/04/2021 14:04:29
if I setup server like in this tutorial [https://cyberbotics.com/doc/guide/web-simulation?tab-language=python](https://cyberbotics.com/doc/guide/web-simulation?tab-language=python)

can other people do some programming for robot?

##### Darko Lukić [Cyberbotics] 01/04/2021 14:05:48
Yes, in that case it would be similar to:

[https://robotbenchmark.net/](https://robotbenchmark.net/)

##### Simon Steinmann [Moderator] 01/04/2021 14:15:39
`@PowerYdRa` you can have them export their robot as a .wbo file and give you their controller as well


you can then load them into your world

##### yash 01/04/2021 14:41:58
I have set  SUPERVISOR field to TRUE, so to use  .getDevice() functions for sensors and motors ,  will robot.getDevice() be applicable ?

##### Nicolas Y 01/04/2021 15:23:14
`@Simon Steinmann` Hi Simon, thanks for you're help earlier but I can't seem to figure out how to 'bound' them so that all eight 'Yamors' move as one bigger whole

##### Stefania Pedrazzi [Cyberbotics] 01/04/2021 15:32:42
Yes

##### Simon Steinmann [Moderator] 01/04/2021 15:32:52
You have to put a "Robot" node as your base node `@Nicolas Y`

##### Nicolas Y 01/04/2021 15:33:01
I did

##### Simon Steinmann [Moderator] 01/04/2021 15:33:13
and add motors in the device section of the hingejoints


what exactly is your problem?

##### Nicolas Y 01/04/2021 15:35:06
I'm not sure how to get the components to stick
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/795676689534812170/unknown.png)
%end

##### Simon Steinmann [Moderator] 01/04/2021 15:35:37
Post your world file here


one thing I can see from the screenshot is, that you have not added JointParameters


double click on them and add a Parameter node


The individual Solids also need a physics node and a bounding Geometry

##### Nicolas Y 01/04/2021 15:40:17
This is my world file
> **Attachment**: [YamorSnake1.wbt](https://cdn.discordapp.com/attachments/565154703139405824/795677996596985856/YamorSnake1.wbt)

##### Simon Steinmann [Moderator] 01/04/2021 15:41:45
ohhh


you added all the segments as children of the robot


you have to put each next joint into the children node of the Solid before

##### Nicolas Y 01/04/2021 15:42:32
ohhh


I'll try that now

##### Simon Steinmann [Moderator] 01/04/2021 15:43:01
you can add boundingObject and physics by simply doubleclicking it


ohh wait, all the Yamor are individual robots?

##### Nicolas Y 01/04/2021 15:43:34
yes

##### Simon Steinmann [Moderator] 01/04/2021 15:43:55
are they supposed to be individually controlled?

##### Nicolas Y 01/04/2021 15:44:09
no I want them to work as one

##### Simon Steinmann [Moderator] 01/04/2021 15:44:09
Because I'm not sure about a chain of robot nodes

##### Nicolas Y 01/04/2021 15:44:37
oh?

##### Simon Steinmann [Moderator] 01/04/2021 15:45:43
Perhaps the devs know the best solution for that. I'm not sure about Robots containing other Robots

##### Nicolas Y 01/04/2021 15:47:03
oh


that complicates things

##### Simon Steinmann [Moderator] 01/04/2021 15:48:34
`@Olivier Michel` How does a hierarchy of robots work? Do the child robots act as devices if they don't have a controller assigned?

##### Olivier Michel [Cyberbotics] 01/04/2021 15:49:23
No, a robot can access only the devices it contains. If a robot contains another robot, the devices of the child robot are not available to the parent robot.

##### Simon Steinmann [Moderator] 01/04/2021 15:50:36
`@Nicolas Y` I think you are supposed to add one Yamor, then doubleclick on 'extensionSlot' and add another

##### Nicolas Y 01/04/2021 15:51:22
`@Olivier Michel` in which case how can one replicate this?
%figure
![yamor.png](https://cdn.discordapp.com/attachments/565154703139405824/795680785843748954/yamor.png)
%end

##### Olivier Michel [Cyberbotics] 01/04/2021 15:57:41
In this example (included in Webots), each module is an independent robot (there is no hierarchy).


Robots attach to each other with a Connector node.

##### Nicolas Y 01/04/2021 15:59:55
`@Olivier Michel` okay thanks I'll try that

##### Olivier Michel [Cyberbotics] 01/04/2021 16:17:39
I would recommend you to start from the yamor demo and modify it bit-by-bit to ensure you are not breaking anything.

##### Nicolas Y 01/04/2021 16:33:35
`@Olivier Michel` sorry but despite checking the website I can't seem to figure out how to use the 'Connector' node to Combine the two 'Yamor' nodes

##### Olivier Michel [Cyberbotics] 01/04/2021 16:34:45
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

##### Simon Steinmann [Moderator] 01/05/2021 09:15:59
`@Alessia Valle` a Position sensor measures the angle of a joint / motor. What you need is the Pose

[http://docs.ros.org/en/melodic/api/nav\_msgs/html/msg/Odometry.html](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)


this is the nav\_msgs definition

##### Darko Lukić [Cyberbotics] 01/05/2021 09:16:39
Hello `@Alessia Valle`,

- For odometry, see the example here: [https://github.com/cyberbotics/webots\_ros2/blob/9682e06932929ea3bfab5a6a826f6fe67471aef6/webots\_ros2\_core/webots\_ros2\_core/webots\_differential\_drive\_node.py#L128-L197](https://github.com/cyberbotics/webots_ros2/blob/9682e06932929ea3bfab5a6a826f6fe67471aef6/webots_ros2_core/webots_ros2_core/webots_differential_drive_node.py#L128-L197)

(it is for ROS2 but there is almost no difference for ROS)

- You don't have create URDF to publish transforms. If your model is simple you can publish them manually, see an example for a physical robot:

[https://github.com/cyberbotics/epuck\_ros2/blob/master/epuck\_ros2\_driver/src/driver.cpp#L167-L232](https://github.com/cyberbotics/epuck_ros2/blob/master/epuck_ros2_driver/src/driver.cpp#L167-L232)

(If you need URDF you can use [https://cyberbotics.com/doc/reference/robot#wb\_robot\_get\_urdf](https://cyberbotics.com/doc/reference/robot#wb_robot_get_urdf))

##### Simon Steinmann [Moderator] 01/05/2021 09:21:38
is the ball joint passive? If so. you have to add friction


so expand the balljoint and add a "JointParameters" node


there you can edit those values

##### yash 01/05/2021 09:34:26
I am getting some weird results when I play the simulation.
> **Attachment**: [test01.mp4](https://cdn.discordapp.com/attachments/565154703139405824/795948316768075806/test01.mp4)


what could be the issue ?

##### Simon Steinmann [Moderator] 01/05/2021 09:36:45
can you click on your robot and show a screenshot (with the white lines of the bounding box)


My first guess would be, that you used the visual mesh data as collision. And your shapes are not convex


Also it seems like it might be starting partially in the floor

##### yash 01/05/2021 09:39:27
this the bounding box
%figure
![Screenshot_from_2021-01-05_15-08-34.png](https://cdn.discordapp.com/attachments/565154703139405824/795949578347085854/Screenshot_from_2021-01-05_15-08-34.png)
%end


how could I rectify the problem ?

##### Simon Steinmann [Moderator] 01/05/2021 09:40:13
oh the grey metal part is not part of the robot?


and it looks like therre is a horizontal part, which has no visual

##### yash 01/05/2021 09:40:55
no it is not the part of the robot

##### Simon Steinmann [Moderator] 01/05/2021 09:41:11
but the robot is attached to it?


In general it is much better (for performance AND accuracy) to replace the boundingObject with simple geometries, such as boxes, spheres, cylinders and capsules

##### yash 01/05/2021 09:42:08
yes I have positioned the robot accordingly, but it is not attached to it.

##### Simon Steinmann [Moderator] 01/05/2021 09:45:33
well it is hard to say. What do you expect to happen?

##### Master.L 01/05/2021 09:45:42
Thanks for solving it!!

##### yash 01/05/2021 09:47:12
the 2 link manipulator should rotate about it's axis provided. With the metal plate at it's position only.

##### Simon Steinmann [Moderator] 01/05/2021 09:48:39
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

##### Olivier Michel [Cyberbotics] 01/06/2021 07:12:43
Webots can compile controllers in either x64 or x86. However, by default it will build x64 binaries. If you want to build a x86 controller binary, you should be sure to link it with `WEBOTS_HOME\msys64\mingw32\bin\Controller.dll` instead of `WEBOTS_HOME\msys64\mingw64\bin\Controller.dll`.

##### yash 01/06/2021 07:38:12
I am working on a certain application that involves grasping a human foot. I imported the human foot CAD using .VRML format. And used the DEF geometry indexfaceset in the shape node to define the bounding object of the human foot. But the bounding object seems to be complex which is resulting in weird physics behavior.  How to address this issue such that the human foot remains stable ?
> **Attachment**: [2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/796281451237736478/2.mp4)


Because when I replaced the foot with a simple cylindrical solid it worked fine since the bounding object of the cylinder is not complex.

##### Stefania Pedrazzi [Cyberbotics] 01/06/2021 07:48:08
The used formula comes from standard steering dynamics. Depending if the front or rear  wheels drive the car (or all four wheels) you need to compute the ratio for the left and right wheel because the do not turn using the same angle.

Here is the reference to a chapter about steering dynamics: [https://link.springer.com/chapter/10.1007/978-1-4614-8544-5\_7](https://link.springer.com/chapter/10.1007/978-1-4614-8544-5_7) but you can probably find other references on the web.

##### Olivier Michel [Cyberbotics] 01/06/2021 07:53:37
You should create a compound bounding object for the foot that is not a mesh, but a combination of simple primitives (spheres, capsules, cylinders and boxes). See this tutorial: [https://cyberbotics.com/doc/guide/tutorial-5-compound-solid-and-physics-attributes](https://cyberbotics.com/doc/guide/tutorial-5-compound-solid-and-physics-attributes) for an example of creating a compound bounding object.

##### yash 01/06/2021 08:01:33
alright ! but this wiil solve the issue right irrespective of the solid geometry ?

##### Olivier Michel [Cyberbotics] 01/06/2021 08:02:22
It should (as long as you don't include any mesh in the bounding object).

##### yash 01/06/2021 08:02:48
understood, thanks a lot !

##### Darko Lukić [Cyberbotics] 01/06/2021 08:48:56
Hello `@Diego Rojas`, I have never tried adding 7th axis linear track, but here are a few ideas. `TrajectoryFollower` can control any motor present in the robot. Therefore, you can add the linear track (built with `SliderJoint` which behaves in the same way as the prismatic joint) in the Webots robot model.



Alternatively, you can use `ConveyorBelt` but it is more complicated. You need to enable a `Supervisor` field of your robot and extend `trajectory_follower` to control `speed` field of `ConveyorBelt`. This will also require some basic integration to control the position of the belt as `ConveyorBelt` allows only a speed control.

##### Alessia Valle 01/06/2021 09:08:34
Hi `@Darko Lukić`, thank you again for the suggestions.  I also found on the ROS wiki website the diff\_drive\_controller  [http://wiki.ros.org/diff\_drive\_controller](http://wiki.ros.org/diff_drive_controller) but I don't know if it is useful for my purposes and if it can be used with Webots. Any idea? 

Moreover, I have another doubt about transforms and URDF. Actually I need the static transforms between the wheels, the base\_link and base\_laser to set up navigation. But how can I publish transforms without the URDF file? Do these frames take the name of the corresponding nodes on Webots?

##### Darko Lukić [Cyberbotics] 01/06/2021 09:20:51
You can use `diff_drive_controller` as well. Your objective is to publish `Odometry` topic and transforms that describe translation and orientation between `base_link` and `odom` frames. You can use `diff_drive_controller` or write one by yourself.



You don't need URDF to publish transforms. In general, `robot_state_publisher` uses URDF to publish `TransformStamped` ([http://docs.ros.org/en/melodic/api/geometry\_msgs/html/msg/TransformStamped.html](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TransformStamped.html)) messages. However, you can publish those messages by yourself.

