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

##### pk1jk1 01/06/2021 18:32:51
Hi! Trying to setup my environment for ros2 on my mac and am getting a file not found error, am I missing something? [https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/#environment-setup](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/#environment-setup)


This is what my directory looks like:
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/796446454096068608/unknown.png)
%end


Hi! Were you able to solve this problem? Am running into the same issue

##### Darko Lukić [Cyberbotics] 01/06/2021 18:53:25
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

##### Simon Steinmann [Moderator] 01/07/2021 08:43:56
`@Master.L` [https://gist.github.com/hugs/4231272](https://gist.github.com/hugs/4231272) this looks like what you're looking for


[https://www.marginallyclever.com/other/samples/fk-ik-test.html](https://www.marginallyclever.com/other/samples/fk-ik-test.html) or this could help

##### Olivier Michel [Cyberbotics] 01/07/2021 08:46:17
Did you try to check the monitor page http://localhost:1999/monitor to check the status of your server?

##### Luiz Felipe 01/07/2021 09:35:08
Oh... If I check it, it appears the load:



%figure
![monitor.png](https://cdn.discordapp.com/attachments/565154703139405824/796673297160601600/monitor.png)
%end


But if i click the localhost/2000 I get 'This site can't be reached'... Hm...

##### Olivier Michel [Cyberbotics] 01/07/2021 09:38:04
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

##### Darko Lukić [Cyberbotics] 01/07/2021 12:14:22
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

##### Darko Lukić [Cyberbotics] 01/07/2021 12:27:32
If you set the motor positions in the single step they all move at the same time. Your problem is therefore something else. For example, the motor in shoulder moves the endpoint faster than the motor in the wrist even though they rotate at the same speed (the motor in the shoulder is farther away from the endpoint than the motor in the wrist). One solution would be to find a lot of IK solutions along a straight line and apply the new IK solution in each step.


No. `robot = Supervisor()` is enough. It will give you methods from `Robot` and `Supervisor` nodes.

##### Alessia Valle 01/07/2021 12:34:06
Thank you. So do you think that I will have to write the SLAM algorithm from scratch on my own?

##### Darko Lukić [Cyberbotics] 01/07/2021 12:35:39
Yes, but make sure the port is correct.

##### yash 01/07/2021 12:37:14
> No. `robot = Supervisor()` is enough. It will give you methods from `Robot` and `Supervisor` nodes.

`@Darko Lukić`  thank you !

##### Darko Lukić [Cyberbotics] 01/07/2021 12:38:28
I don't know :/ Depends on your project.

##### Alessia Valle 01/07/2021 12:45:00
What do you mean by "it depends on your project?"😅

##### Darko Lukić [Cyberbotics] 01/07/2021 12:47:48
I guess if you have a good understanding of SLAM than you can implement it, but if you want a robust solution it is better to use something like `cartographer`.

##### Alessia Valle 01/07/2021 12:58:44
Ok, I will go through the documents you have shared! Thank you for your availability! 😀

##### josiehughes 01/07/2021 13:09:59
Hi - I'm wondering about the current e-puc hardware and compatability with Webots. Is the Epuc-2 compataible with Webots?

##### Luiz Felipe 01/07/2021 14:07:09
I get streaming server error... will check the debug flags of the session and simulation servers...

##### Darko Lukić [Cyberbotics] 01/07/2021 15:54:45
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

##### Darko Lukić [Cyberbotics] 01/08/2021 08:16:27
`@Luiz Felipe` There is `webots.min.js` distributed with Webots packages (`resources/web/wwi/webots.min.js`), but you can also use the one available on Cyberbotics servers.

##### Luiz Felipe 01/08/2021 08:21:10
Yes `@Darko Lukić` , I changed to use the one available on Cyberbotics servers because I am running the servers in a docker container. But the streaming server page seems a big buggy for the 2021a version in my computer... I am still having trouble following the Web Simulation documentation also, the session server and simulation server are fine, but I am not seem able to figure out how to open a simulation or send any information to the simulation server...


I just thought it is better for the default to be the one available on the Cyberbotics servers.

##### Olivier Michel [Cyberbotics] 01/08/2021 08:28:01
It should be the same anyhow.

##### yash 01/08/2021 08:33:19
Can I put touch sensor in children of solid node ? Since when the touch sensor collides with another solid, I want the contact point of the collision?

##### Luiz Felipe 01/08/2021 08:39:46
Yups... The streaming viewer however does not open... The 2020b version works fine...
%figure
![Screenshot_from_2021-01-08_17-37-04.png](https://cdn.discordapp.com/attachments/565154703139405824/797021720057741312/Screenshot_from_2021-01-08_17-37-04.png)
%end

##### Olivier Michel [Cyberbotics] 01/08/2021 08:43:19
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

##### Simon Steinmann [Moderator] 01/08/2021 09:25:36
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

##### Olivier Michel [Cyberbotics] 01/08/2021 10:19:44
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

##### Simon Steinmann [Moderator] 01/08/2021 14:44:57
are you dead set on c?

##### Maheed 01/08/2021 14:46:30
Yep

##### Simon Steinmann [Moderator] 01/08/2021 14:47:47
Then I wont be able to help much. But my approach would be to create basic functionality as "move forward" "turn left" "turn right" etc. And then combine those into algorithms for obstacle avoidance etc.

##### Maheed 01/08/2021 15:27:37
Okay thanks


Is there anyone here good with C who can help me with this please?

##### linexc 01/08/2021 23:58:08
Hello everyone, I have a problem regarding Webots and NAO robot. I am trying to write a ROS server and client in order to make the NAO in Webots perform some gestures. But when I tried to simulate it, the error showed always like this. But I really don't know why the network address in already in use. If anybody can help me, I would be very grateful.



%figure
![error.png](https://cdn.discordapp.com/attachments/565154703139405824/797253462727327784/error.png)
%end

##### Simon Steinmann [Moderator] 01/09/2021 08:26:26
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

##### Simon Steinmann [Moderator] 01/10/2021 12:37:11
`@DrakerDG` Hey, what part exactly don't you understand?

##### DrakerDG 01/10/2021 12:41:40
`@Simon Steinmann` hello! I don't understand how by duplicating or tripling the same robot (copy & paste) they can use the same controller separately.  What I mean is that I just copied and pasted them without changing anything and it worked.  Each robot can follow the line without affecting the other.  In the end I only changed the color of the chassis and nothing else.  I see that it works but I don't understand how


I understood that each robot should have its own controller, but apparently it is not entirely necessary

##### Simon Steinmann [Moderator] 01/10/2021 12:50:15
Ah okay, I understand the confusion. Each robot runs its own instance of that controller. In a separate process


does that make sense?

##### DrakerDG 01/10/2021 12:53:45
`@Simon Steinmann` yes, thank you 😁👍🏼


`@Simon Steinmann` like as running the same application several times


`@Simon Steinmann` If it were different robots if each one is configured with their controller, right?

##### Simon Steinmann [Moderator] 01/10/2021 12:56:16
yes


Usually a Controller is specific to one Robot, but you can have many instances of that Robot


you can take a look at your task-manager while running it. You should see the main Webots process, and then at least one per running controller.

##### DrakerDG 01/10/2021 12:58:23
`@Simon Steinmann` For example robots that play soccer use the same controller all, so

##### Simon Steinmann [Moderator] 01/10/2021 12:58:30
I mostly use Python and often need similar functionality. So I have sub-modules or scripts, which I dont change and can use in different controllers by simply importing them

##### DrakerDG 01/10/2021 12:59:09
Ok

##### Simon Steinmann [Moderator] 01/10/2021 13:00:01
If you have one type of robot player, then yes, it would make sense to use the same controller. Or at least a shared base-version for stuff like movement, sensors etc.


The roles for behavior, if they should differ, could be put on top of that


What Programming language do you use?

##### DrakerDG 01/10/2021 13:01:05
I will continue playing and I will try to make modules as you indicate.  It's very interesting


I am currently programming in C language, but I am learning in a course to start writing in Python


`@Simon Steinmann` My object is to use Webots to test robot designs, observe their operation, mechanical behavior, improve it and then physically build the robot

##### Simon Steinmann [Moderator] 01/10/2021 13:14:31
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

##### Darko Lukić [Cyberbotics] 01/10/2021 17:37:41
The robot window doesn't rely on webots.js. Could you tell me which world are you trying to open?

##### AdityaDutt 01/10/2021 17:39:01
I am using the erebus challenge world. [https://github.com/Shadow149/Erebus](https://github.com/Shadow149/Erebus). Under erebus-aura>game>worlds>GeneratedWorld.wbt


well i went back to 2020a rev 1 and the error is no longer there

##### Darko Lukić [Cyberbotics] 01/10/2021 21:00:41
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

##### Darko Lukić [Cyberbotics] 01/10/2021 22:03:29
Yes


You can check the e-puck example:

[https://github.com/cyberbotics/webots/blob/f6ebe9cf8ce64c30cf453655376a815deeb28744/projects/robots/gctronic/e-puck/plugins/robot\_windows/e-puck/e-puck.html#L76](https://github.com/cyberbotics/webots/blob/f6ebe9cf8ce64c30cf453655376a815deeb28744/projects/robots/gctronic/e-puck/plugins/robot_windows/e-puck/e-puck.html#L76)

##### Diego Rojas 01/10/2021 23:30:11
What are the steps of designing a model for webots simulation? What is the process of designing something in soldworks and making a PROTO from the mesh? This is likely a simple question, but I cannot seem to find a direct answer or demo.

##### DrakerDG 01/10/2021 23:40:04
`@Diego Rojas` check this play list: [https://youtube.com/playlist?list=PLbEU0vp\_OQkUwANRMUOM00SXybYQ4TXNF](https://youtube.com/playlist?list=PLbEU0vp_OQkUwANRMUOM00SXybYQ4TXNF)

##### Diego Rojas 01/11/2021 00:13:32
`@DrakerDG` Thanks!

##### Simon Steinmann [Moderator] 01/11/2021 09:15:52
Take a look at this first: [https://www.cyberbotics.com/doc/guide/tutorial-7-your-first-proto](https://www.cyberbotics.com/doc/guide/tutorial-7-your-first-proto)

In general, the simplest way in my opinion, would be to build up your model in webots, then export the robot as a .wbo


then you just need to add the .proto header (as described in the link)

##### JSK 01/11/2021 10:07:52
Hello Team Webots


i have a question related to webots


just visited the official website and then github and now i am here


Hi `@Darko Lukić`  how are you? remember me?

##### Darko Lukić [Cyberbotics] 01/11/2021 10:11:47
I am sorry, I cannot recognize your username 🙁

##### JSK 01/11/2021 10:12:22
i am Jamal Shams Khanzada, couple of days ago i had asked you a question on github and you helped me


and i am thankful to you


i am here for another problem to be solved.


can you still help me to solve it?


actually when i fetched gyro data recently, i started to make controller but it needs numpy

##### Darko Lukić [Cyberbotics] 01/11/2021 10:14:09
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

##### Darko Lukić [Cyberbotics] 01/11/2021 10:23:46
You installed `numpy` for Python2, not Python3


You need to `pip3 install numpy`

##### JSK 01/11/2021 10:24:31
ok i am installing


but how can i know that `numpy` is for python2 or python 3?


installed it


rebooting is necessary or not?

##### Darko Lukić [Cyberbotics] 01/11/2021 10:26:11
No, just restart the controller


In general (not necessary), if you install it with `pip3` it is for Python3, if you install it with `pip` it is for Python2.

##### JSK 01/11/2021 10:27:33
ok


i restaretd the controller but the problem is still there

##### Darko Lukić [Cyberbotics] 01/11/2021 10:30:08
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

##### Darko Lukić [Cyberbotics] 01/11/2021 10:34:13
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

##### Simon Steinmann [Moderator] 01/11/2021 10:48:29
`@JSK` You may also have to set environment variables, especially when using conda

##### JSK 01/11/2021 10:49:42
yeah i read about environment variables in the documentation that `@Stefania Pedrazzi`  mentioned

##### Simon Steinmann [Moderator] 01/11/2021 10:51:02
you can launch webots from a terminal. it will have the variables of that terminal

##### JSK 01/11/2021 10:51:44
to be honest i tried to set the variables but when i started to search `usr/local/webots` i was not able to find that and hence i came on this platform for help. i am trying this for 3 days

##### Simon Steinmann [Moderator] 01/11/2021 10:52:11
you can edit your .bashrc in your home folder to add variables

##### JSK 01/11/2021 10:53:05
what does "edit" mean.


?

##### Simon Steinmann [Moderator] 01/11/2021 10:53:17
open it in a text editor and add the variables


[https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=linux&tab-language=python](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=linux&tab-language=python)


here is more detail

##### JSK 01/11/2021 10:54:51
alright

##### Simon Steinmann [Moderator] 01/11/2021 10:57:26
you can add this at the end of your .bashrc

`export WEBOTS\_HOME="/home/simon/webots"

export LD\_LIBRARY\_PATH="${LD\_LIBRARY\_PATH}:${WEBOTS\_HOME}/lib/controller"

export PYTHONPATH="${PYTHONPATH}:${WEBOTS\_HOME}/lib/controller/python38"

export PYTHONIOENCODING="UTF-8"`

keep in mind to change the first line to the correct Path of your installation, and change the python version to the correct one


`python3 --version`  to get your python3 version

##### JSK 01/11/2021 11:00:44
thanks let me do it

##### Simon Steinmann [Moderator] 01/11/2021 11:00:45
the .bashrc file gets sourced every time you open a new terminal. So you have to open a new terminal for it to take effect. Then you can launch webots from the terminal, and it will have those variables

##### JSK 01/11/2021 11:03:18
can you tell me how to access .bashrc file? i cant find it

##### Simon Steinmann [Moderator] 01/11/2021 11:03:28
it's hidden


ctrl + h to see hidden files, or `gedit ~/.bashrc` in the terminal

##### JSK 01/11/2021 11:04:05
found it


updated .bashrc as you mentioned `@Simon Steinmann` .

##### Simon Steinmann [Moderator] 01/11/2021 11:07:37
now you can open a new terminal and launch webots from there. the variables should be set


Instead of setting those variables in the .bashrc, you could set them in `sudo gedit /etc/profile`. That should set them system wide

##### JSK 01/11/2021 11:08:47
error still exists

##### Alessia Valle 01/11/2021 11:09:48
Hello everyone! I would like to write a controller that allows my robot to keep the lane. What sensors do you suggest me to use?

##### MartinG 01/11/2021 11:10:33
Use a camera sensor and look into canny edge detection and hough line transform


I don't know your programming language preference, but I strongly recommend you use python since OpenCV makes it a much easier task than it seems.

##### Darko Lukić [Cyberbotics] 01/11/2021 11:11:27
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

##### Simon Steinmann [Moderator] 01/11/2021 11:23:12
Perhaps try removing things from the world one by one and hit reset after. Try to find out what is causing the crash

##### Alessia Valle 01/11/2021 11:25:21
Thank you!

##### MartinG 01/11/2021 11:26:05
I'm glad to help, I'll be around if you have some more specific questions.


Yeah, that makes sense. Thanks.


Anyone with some deeper knowledge of arctan2 and trigonometry in general want to help me out a bit?

##### Simon Steinmann [Moderator] 01/11/2021 14:16:06
`@MartinG` whaz do you want to k ow?


I got a pretty good handle on trigonometry and linear algebra, especially in the context of robotics and positions / orientations

##### MartinG 01/11/2021 14:20:37
Awesome.


So I'm trying to do a GPS guided navigation for the Tesla in Webots.


I do this by taking 5 points along the path, then converting them into a lot of points using a bspline path program.


The problem is figuring out how to calculate the angle the car needs to turn using arctan2

##### Simon Steinmann [Moderator] 01/11/2021 14:24:26

%figure
![220px-Atan2definition.png](https://cdn.discordapp.com/attachments/565154703139405824/798195623940718622/220px-Atan2definition.png)
%end

##### MartinG 01/11/2021 14:24:42
I've seen that graphic many times ahahah.

##### Simon Steinmann [Moderator] 01/11/2021 15:09:39
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

##### Simon Steinmann [Moderator] 01/11/2021 16:44:57
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

##### Olivier Michel [Cyberbotics] 01/12/2021 07:19:17
Thank you for this contribution! Once you have implemented and tested it, please open a new PR to submit your change, so that it will be integrated into Webots).

##### yash 01/12/2021 07:39:03
How can I extract the position moved by a manipulator link ? I have used a hinge joint along with position sensor .

So when a manipulator link hits a solid it stops, at this moment I want to know the position of the link .?

##### Olivier Michel [Cyberbotics] 01/12/2021 07:51:50
Can't you use the position sensor for that?

##### yash 01/12/2021 08:11:01
> Can't you use the position sensor for that?

`@Olivier Michel`  ye it’s done ! Was thinking in a wrong direction... thanks !

##### Luiz Felipe 01/12/2021 08:37:38
If I git clone the webots repository the dependencies are not downloaded right? How can I download also with the dependencies? (I cloned but the /wwi/dependencies folder does not exist)


I guess I should use --recurse-submodules... will test it!

##### Olivier Michel [Cyberbotics] 01/12/2021 08:45:02
Yes.

##### Simon Steinmann [Moderator] 01/12/2021 08:45:23
Use the build from source instructions on github


They give you a step for step process, including all dependencies

##### Olivier Michel [Cyberbotics] 01/12/2021 08:46:04
Please follow strictly the instructions at [https://github.com/cyberbotics/webots/wiki/Linux-installation](https://github.com/cyberbotics/webots/wiki/Linux-installation) (assuming you are on Linux)

##### Luiz Felipe 01/12/2021 08:49:05
Thanks 🙂 doing that now

##### Simon Steinmann [Moderator] 01/12/2021 08:51:44
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

##### Simon Steinmann [Moderator] 01/12/2021 09:23:19
what are you trying to achieve? Webots built successfully?

##### Luiz Felipe 01/12/2021 09:24:25
yups.. webots built successfully, i can open v2021a


i am trying to check a possible bug in the multimedia\_client.js file

##### Simon Steinmann [Moderator] 01/12/2021 09:24:56
okay... and what is the issue?

##### Darko Lukić [Cyberbotics] 01/12/2021 09:27:01
`@Luiz Felipe` Can you check whether you have node/npm installed, `npm --version`. If there is a missing dependency `make` skips compiling the corresponding part. Node should be included in the optional dependencies, but just to make sure whether it is properly installed

##### Luiz Felipe 01/12/2021 09:28:28
Probably that is the reason `@Darko Lukić` (npm is installed correctly but probably make skipped that when i ran the command)


does make clear works to re-build one more time?

##### Darko Lukić [Cyberbotics] 01/12/2021 09:29:20
Yes, `make clean` works

##### Olivier Michel [Cyberbotics] 01/12/2021 09:29:39
`make clean`

##### Darko Lukić [Cyberbotics] 01/12/2021 09:29:59
Correct, sorry

##### Luiz Felipe 01/12/2021 09:30:15
Thanks =), I will re-build

##### Olivier Michel [Cyberbotics] 01/12/2021 09:30:18
Or even more powerful `make cleanse` (will remove the dependencies).

##### Luiz Felipe 01/12/2021 09:31:38
For the web part, is there a list of bugs or features that need help? (I dont know much of javascript but I feel that part is really breakthrough work in  webots)


Now I got the dependencies `@Darko Lukić` . Thank you very much. I got an error during make regarding the npm but all the dependencies seems to be in the /dependencies folder.

##### Darko Lukić [Cyberbotics] 01/12/2021 09:44:24
What is the error?

##### Luiz Felipe 01/12/2021 09:47:20

%figure
![Screenshot_from_2021-01-12_18-40-36.png](https://cdn.discordapp.com/attachments/565154703139405824/798488275685605396/Screenshot_from_2021-01-12_18-40-36.png)
%end


but it works

##### Darko Lukić [Cyberbotics] 01/12/2021 09:51:21
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

##### Olivier Michel [Cyberbotics] 01/12/2021 16:41:49
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

##### Olivier Michel [Cyberbotics] 01/13/2021 07:11:38
A possible solution (without a physics plugin) would be to turn the bullet into a Robot with the supervisor field set to TRUE. Then, add a distance sensor to the bullet looking backwards. At each time step, you should get the current position of the bullet and compute the vector between the current position and the previous position. You will use this vector to change the orientation of the distance sensor so that it looks exactly at the previous position and make a sensor measurement in the same time step. If the distance measured is smaller than the distance between the current and previous position, then a collision occurred and you should set the position of the bullet at the place of collision (which can easily be computed along the distance sensor ray).

##### Wasabi Fan 01/13/2021 07:16:36
Oh wow, that's very inventive and \_sounds\_ workable. I think there are some details I'd need to work out but I shall try that in the next few days and report back if I run into issues. Thanks!

##### Olivier Michel [Cyberbotics] 01/13/2021 07:17:28
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

##### Simon Steinmann [Moderator] 01/13/2021 08:40:21
[https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-language=python&tab-os=linux](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-language=python&tab-os=linux)


you have to set your environment variablres

##### Moha 01/13/2021 08:41:58
in pycharm or windows environment variables ?

##### Simon Steinmann [Moderator] 01/13/2021 08:42:39
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

##### Simon Steinmann [Moderator] 01/13/2021 11:23:34
You're right, the documentation is not 100% clear here. When you look at the C or C++ code, you see the type is BOOL


so I'd assume setting it to True = force relative to Solid, False = force relative to world


`@Olivier Michel` We should update the documentation

##### yash 01/13/2021 11:26:15
okay , let me try that.

##### Olivier Michel [Cyberbotics] 01/13/2021 11:28:03
It seems clear to me: [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_add\_force](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_add_force)

##### Simon Steinmann [Moderator] 01/13/2021 11:28:31
when looking at the Python code, it is not clear that it is boolean

##### Olivier Michel [Cyberbotics] 01/13/2021 11:28:45
Feel free to suggest some change here: [https://github.com/cyberbotics/webots/edit/released/docs/reference/supervisor.md](https://github.com/cyberbotics/webots/edit/released/docs/reference/supervisor.md)

##### Simon Steinmann [Moderator] 01/13/2021 11:28:56
already at it 😉


[https://github.com/cyberbotics/webots/pull/2646](https://github.com/cyberbotics/webots/pull/2646)

##### Chernayaten 01/13/2021 11:34:43
From Tutorial 6 on the 4 wheeled robot, the anchor field of the HingeJointParameters node should have the same values as the Solid translation one. Is this correct?

##### Olivier Michel [Cyberbotics] 01/13/2021 11:37:19
Reviewed.


Yes.

##### Chernayaten 01/13/2021 11:41:13
There's a discrepancy in the pen.wbt device example

##### Olivier Michel [Cyberbotics] 01/13/2021 11:43:27
What is the problem?

##### Chernayaten 01/13/2021 11:44:30
The anchor field of the HingeJointParameters node is 0 0.025 0 whereas the Solid node (for the wheel) is -0.045 0.025 0 (and 0.045 for the other wheel)


This is in the Pen device example

##### Olivier Michel [Cyberbotics] 01/13/2021 11:49:28
This is correct, but is it not essential as the anchor may be anywhere on the rotation axis. However, you are right it is better to have it as close as possible to the rotating part (the wheel in this case) for stability reasons. May I let you propose a patch here: [https://github.com/cyberbotics/webots/edit/master/projects/samples/devices/worlds/pen.wbt](https://github.com/cyberbotics/webots/edit/master/projects/samples/devices/worlds/pen.wbt) ?

##### Chernayaten 01/13/2021 11:53:48
Done

##### Olivier Michel [Cyberbotics] 01/13/2021 11:55:40
Thank you. I approved it. You should be able to merge it once the CI tests are complete.

##### Chernayaten 01/13/2021 13:00:44
" Only those with write access to this repository can merge pull requests. "

##### Olivier Michel [Cyberbotics] 01/13/2021 13:06:56
OK, I merged it. Thank you.


I sent you a GitHub invitation to become "Committer" on this repo with write access.

