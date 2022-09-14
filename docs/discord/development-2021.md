# Development 2021

This is an archive of the `development` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m) for year 2021.

## January

##### Simon Steinmann [Moderator] 01/05/2021 09:27:08
The conversation in technical-questions made me think: Wouldn't it be a nice feature, to have an 'odometry' device? Which retrieves the pose of it's parent. Of course this can be done manually with a supervisor, however that is more complicated for beginners and well, requires a supervisor.

##### Olivier Michel [Cyberbotics] 01/05/2021 09:47:22
Yes, but I believe that such an odometry device doesn't exists in real life...

##### Simon Steinmann [Moderator] 01/05/2021 09:56:13
no it doesnt, but it can be a good first step / validation tool. This is used all the time in ROS and Gazebo


It could be implemented in the Robot node perhaps. "getPosition", "getOrientation"

##### Olivier Michel [Cyberbotics] 01/05/2021 10:01:28
It is already there when you turn your robot into a supervisor.

##### Simon Steinmann [Moderator] 01/05/2021 10:10:39
well I guess that's true. Still could be useful to have a device one could add to Solids/Transforms, in order to retrieve the pose


Found a strange behavior, exporting a robot as URDF uses the boundingObject as the visual geometry, not the shape. Is this on purpose? Or simply because mesh extraction is not implemented yet?

##### Darko Lukić [Moderator] 01/07/2021 08:00:15
Yes, it is described in the note under `wb_robot_get_urdf` documentation. We have a basic implementation of URDF exporter that can work for most use-cases (publish ROS transforms, calculate inverse kinematics, and similar), but only a basic visualization is possible.

##### Simon Steinmann [Moderator] 01/07/2021 10:01:47
`@Darko Lukić` launching the dynamic rviz with ros2, this Error comes up
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/796679973011718164/unknown.png)
%end



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/796680042905337867/unknown.png)
%end


this is part of the console output
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/796680283385888779/unknown.png)
%end


I suspect that the reason is rviz launching before webots and ros2 is fully loaded


loading the same config again once it is running, no errors come up

##### Darko Lukić [Moderator] 01/07/2021 10:04:06
RViz2 is a bit buggy, just uncheck/check the RobotModel checkbox and it should work

##### Simon Steinmann [Moderator] 01/07/2021 10:04:29
you're right


is there a quick and easy way to include the world frame in the tf broadcast?

##### Darko Lukić [Moderator] 01/07/2021 10:06:26
You can add a static transform publisher to your launch file:

```py
static_tf = Node(
  package='tf2_ros',
  executable='static_transform_publisher',
  name='static_transform_publisher',
  output='log',
  arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
)
```

##### Simon Steinmann [Moderator] 01/07/2021 10:07:00
what if the robot is mobile?

##### Darko Lukić [Moderator] 01/07/2021 10:07:48
Than you have to publish the transforms manually :/

##### Simon Steinmann [Moderator] 01/07/2021 10:09:00
does that work alongside the robot\_state\_publisher?

##### Darko Lukić [Moderator] 01/07/2021 10:10:07
Sure

##### Simon Steinmann [Moderator] 01/07/2021 10:10:37
maybe that should be a feature of the webots\_ros2\_core


have an option (or by default) to pulbish the world - baselink transform


static robots are one thing, but especially for mobile ones, it is kinda essential.

##### Darko Lukić [Moderator] 01/07/2021 10:14:52
But that is something that a user has to handle

##### Simon Steinmann [Moderator] 01/07/2021 10:15:37
Gazebo publishes it by default, people are used to simply selecting "world" in rviz

##### Darko Lukić [Moderator] 01/07/2021 10:17:16
Gazebo publishes transform between `world` and `base_link` for mobile robots? That sounds like cheating


In the real-world transform between `world` and `base_link` is not available

##### Simon Steinmann [Moderator] 01/07/2021 10:18:02
maybe it is just static. Has been a while, dont remember

##### Darko Lukić [Moderator] 01/07/2021 10:18:31
We can easily add it using Supervisor, but it is strange to have something like that

##### Simon Steinmann [Moderator] 01/07/2021 10:20:13
hmmm where exactly would I add that tf\_static node?


for example when using this: [https://github.com/cyberbotics/webots\_ros2/blob/master/webots\_ros2\_universal\_robot/launch/universal\_robot\_rviz\_dynamic.launch.py](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_universal_robot/launch/universal_robot_rviz_dynamic.launch.py)


line 43?


jup, works 🙂

##### Moha 01/13/2021 07:53:46
hi guys

is it possible to use "vscode" as my IDE?

##### Olivier Michel [Cyberbotics] 01/13/2021 07:55:10
Sure.

##### Moha 01/13/2021 08:07:09
👍

##### Simon Steinmann [Moderator] 01/13/2021 08:44:39
I'm curious, wouldn't it be possible to set system environment variables during installation?


`@Darko Lukić` `@Olivier Michel` I've got moveit2 working with ros2 and webots. I also made some progress of creating an easy to use template or automatic setup for robotic arms


I think one important feature would be, to automatically extract the urdf from webots, including meshes.


I already did some work in that direction with my proto-splitter script. That one worked, extracting .obj or vrml meshes, but without smoothing


ahh okay


Also I was wondering: could it be possible to launch a ROS1 or ROS2 launch file from within webots?

##### Olivier Michel [Cyberbotics] 01/13/2021 11:23:51
I am not sure this is a good idea as we want to be able to interact with ROS in the console...

##### babaev1 01/16/2021 20:00:53
Dear Sirs, recently Organizing Commitee of Humanoid League of Robocup Federation decided to host Summer World Championship in virtual format with organizing games between teams within simulator. Currently simulator platforms are under consideration for choosing. There are going to be games between 2 teams , with each team providing 4 humanoid robots  equipped by 23-25DoF servomotors, 1-2 imus, 1-2 cameras. Imortant point is streaming of images from cameras to external controller softwares of teams. Currently built in webots camera streams images in non-compressed format. Supposing simultaneously streaming images from 8 cameras with resolution 640x480 RGB  with 30 fps simulation is expected to be dramatically slowing down.  Some of teams are expected to use stereo cameras, means number of cameras in this case will be doubled.


Do you plan to add compression to image streaming in near future, or there are some implementations which I am not familiar with?

##### cnbarcelo 01/17/2021 15:43:05
Hi! Where can I find the sensors visualization code? I'm mainly looking for the the lidar visualization.

##### Darko Lukić [Moderator] 01/17/2021 17:59:21
`@cnbarcelo` You can use `View / Optional Rendering / Show Lidar Point Cloud` and `View / Optional Rendering / Show Lidar Rays Paths`:

[https://cyberbotics.com/doc/guide/the-user-interface](https://cyberbotics.com/doc/guide/the-user-interface)



Remember, to show the point cloud, you need to enable the point cloud first. You can do it from the robot window (double click on the robot) or from your controller:

[https://cyberbotics.com/doc/reference/lidar#wb\_lidar\_enable\_point\_cloud](https://cyberbotics.com/doc/reference/lidar#wb_lidar_enable_point_cloud)

##### cnbarcelo 01/17/2021 18:03:15
Yeah, I do know that, but I migrated from 2020 to 2021 and it's not working well. I wanted to check if something changed before creating a ticket.


If there's any bug to work on, I might be able to collaborate.

##### Darko Lukić [Moderator] 01/17/2021 18:06:12
It works fine for me. Can you tell more details about your issue?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/800425762247278592/unknown.png)
%end

##### cnbarcelo 01/17/2021 18:07:38
The visualization blinks. It works for a couple of seconds, stop working, and starts working again, completely randomly.


It happens in my simulation and also in the lidar world example.


I'll record a video and share it here later on today.

##### Chernayaten 01/17/2021 18:08:54
Hey Darko, I also wrote a comment few weeks ago about this in <#565154703139405824> . I run the lidar.wbt example and although it shows the point cloud at first it just stops working correctly


In my simulation it doesn't work at all

##### cnbarcelo 01/17/2021 18:09:43
That's exactly what happens, will share a video later then, so you Darko can see it.


I didn't see your comment in <#565154703139405824> , that's way I came here with the issue.

##### Darko Lukić [Moderator] 01/17/2021 18:10:38
It may be specific to GPU or OS. Could you please report the bug:

[https://github.com/cyberbotics/webots/issues/new?assignees=&labels=&template=bug\_report.md](https://github.com/cyberbotics/webots/issues/new?assignees=&labels=&template=bug_report.md)

##### Chernayaten 01/17/2021 18:12:43
What info would you like besides GPU and OS?

##### cnbarcelo 01/17/2021 18:16:21
I'm running Ubuntu and an Nvidia card.

I'm out of office now, will check if you already created the issue when I'm back, and will add the info of my system there, or will create it by my own.

##### Chernayaten 01/17/2021 18:24:03
[https://github.com/cyberbotics/webots/issues/2660](https://github.com/cyberbotics/webots/issues/2660)



I added a short video to show it

##### Darko Lukić [Moderator] 01/17/2021 19:23:47
Thank you, we will check it out

##### Chernayaten 01/17/2021 20:12:04
After a few tests it seems you are correct. The same issue presents when the robot gets close to objects because " If the range value is smaller than the minRange value then infinity is returned."

##### Azer Babaev [Starkit, TC] 01/18/2021 09:23:07
Thank you for prompt reply. For me it is clear. I shall share your answer with Robocup community for possible definitions.

##### Moha 01/19/2021 09:49:08
hi guys

Is it possible to use spot robot SDK releasing from Boston dynamics in webots ?

##### Darko Lukić [Moderator] 01/19/2021 10:01:18
Hello `@Moha`, Webots doesn't provide support for Spot SDK. It is not in the scope of Webots, but I advise you to create a discussion:

[https://github.com/cyberbotics/webots/discussions/categories/ideas](https://github.com/cyberbotics/webots/discussions/categories/ideas)

If there is a strong need (many requests), somebody from the community may create the integration.


You can create a discussion:

[https://github.com/cyberbotics/webots/discussions/new](https://github.com/cyberbotics/webots/discussions/new)



Explain why the integration is necessary, how big impact it will have on the community...


They can join the discussion, like it, suggest new ideas

##### mayank.kishore 01/20/2021 05:59:29
What is the best way to locally edit a webots package?

##### Darko Lukić [Moderator] 01/20/2021 07:30:40
You can compile as here:

[https://github.com/cyberbotics/webots\_ros2/wiki/Build-and-Install](https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install)

##### Simon Steinmann [Moderator] 01/20/2021 09:48:01
Is there documentation for custom robot window? I cannot seem to find it.

##### Olivier Michel [Cyberbotics] 01/20/2021 09:58:59
[https://cyberbotics.com/doc/guide/controller-plugin#robot-window](https://cyberbotics.com/doc/guide/controller-plugin#robot-window) (but it's very limited).


[https://cyberbotics.com/doc/guide/controller-plugin?version=master#robot-window](https://cyberbotics.com/doc/guide/controller-plugin?version=master#robot-window) (this is a newer version, with much more details)


(but refers to the master branch, not the released version)


(to be released in the next revision)

##### Simon Steinmann [Moderator] 01/20/2021 10:01:32
yeah, i'm excited for the new changes there. I'm gonna wait then until I dive back in

##### fowzan 01/21/2021 18:08:36
I would love to simulate spot mini on my terminal , could you share links if available

##### mayank.kishore 01/22/2021 16:58:40
Having trouble editing a package locally, please message me if you can help


Do you know why I am getting this error?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/802613647226044436/unknown.png)
%end

##### jasper [Moderator] 01/23/2021 19:02:50
you seem to be missiong ros-<version>-console-bridge

##### Simon Steinmann [Moderator] 01/25/2021 15:11:40
I got the suggestion from a user to add voice channels to the discord server.  Could be a nice addition

##### Olivier Michel [Cyberbotics] 01/25/2021 15:29:48
I just created one. Can you try it?

##### Simon Steinmann [Moderator] 01/25/2021 15:30:06
seems to work

##### kRiSh 01/25/2021 15:33:10
oh wow, already created.

##### mayank.kishore 01/25/2021 15:55:58
Does anyone have experience creating an overhead map with a camera? And potentially stitching multiple images together to create an encompassing overhead map?

##### Azer Babaev [Starkit, TC] 01/25/2021 16:04:06
There is good feature in Coppelia which I couldn’t find in Webots: position and orientation of each node can be measured and modified from GUI.

##### Simon Steinmann [Moderator] 01/25/2021 17:13:51
you can do the same in Webots in the scene tree

##### Azer Babaev [Starkit, TC] 01/25/2021 17:34:59
Yes, it is possible to read absolute position in scene tree, but modification of absolute position or orientation is not accessible there.

##### Simon Steinmann [Moderator] 01/25/2021 17:38:12
it is


just change the values
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/803317976853905448/unknown.png)
%end

##### Azer Babaev [Starkit, TC] 01/25/2021 17:43:41
In case if in children of parent Solid child Solid is added direct change of values of child Solid appears complex especially in case if in parent Solid rotation is not zero.


I try to compose humanoid robot with 6 DOF in each branch. Direct modification of absolute position of deep level child elements is really challenging.

##### Simon Steinmann [Moderator] 01/25/2021 17:50:44
that is the nature of kinematic chains


that is where you need inverse kinematics

##### Azer Babaev [Starkit, TC] 01/25/2021 17:56:54
In case if I select node in scene tree I can see absolute position and rotation of node, but values are not editable. Possibility of modification of absolute position and orientation through editing values could be helpful.


In Coppelia it is possible

##### Simon Steinmann [Moderator] 01/26/2021 09:11:26
`@Azer Babaev [Starkit, TC]` If I understand you correctly, you want to position a child node using absolute coordinates, which would automatically calculate the relative coordinates to the parent?

##### Azer Babaev [Starkit, TC] 01/26/2021 13:51:17
I am using Coppelia for teaching robotics in school since 2 years. There is functionally which I consider useful and I use it often. It is possible to call pop-up widow for reading and editing absolute coordinate of any node which you select on scene tree or directly on 3D rendering window. After giving new coordinate and position old link to parent node is replaced by new link. Position and orientation of parent node remain unchanged. Position and orientation of all child nodes changes together with selected node new position and orientation. In case if you want to protect complex structures from being destroyed by this action node can be protected from being selected in node properties. I started to study Webots 2 weeks ago and I didn’t find similar useful functionality.


window (not widow)

##### Simon Steinmann [Moderator] 01/27/2021 10:35:31
`@Azer Babaev [Starkit, TC]` So you want the reference frame selected here (in this case 'Absolute') to change the axis in the 3D view and the translation + rotation field in the scene tree. So they can be edited in the chosen frame. However, the saved definition is still the relative position to its parent.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/803936219175452702/unknown.png)
%end


`@Olivier Michel` this could be a nice feature and should not necessarily be too complex to include

##### Olivier Michel [Cyberbotics] 01/27/2021 10:41:53
Yes, I believe changing these position and rotation fields from read-only to read/write should be fairly easy to implement. However, I am not sure to understand why it seems useful. In general objects can be moved from the root-level translation/rotation. Moving an object inside its parent is not something we want to do from the global coordinate system. So, I am not sure to understand the use case for this...

##### Azer Babaev [Starkit, TC] 01/27/2021 10:43:34
This is useful during building new robot


In order to build 23 DOF robot from primitives I spent 2 days in Coppelia and about 1 week in Webots.

##### Olivier Michel [Cyberbotics] 01/27/2021 10:54:57
Ok, thank you for the explanation. Then, you should probably open an issue about this (or implement it by yourself if you want).

##### Azer Babaev [Starkit, TC] 01/27/2021 11:01:24
First variant is more realistic for my statement of amateur programmer.


Dear Webots developers, do you have plan to add Russian language to variety of GUI languages in 2021? Together with pandemic and moving robotic lessons and competitions to virtual format there is higher interest to use simulation software in education. Unfortunately not many teachers and students are fluent with English in Russian schools.  Adding optional Russian language to GUI can give good advantage to Webots to be widely used in Russian schools.

##### Darko Lukić [Moderator] 01/28/2021 22:15:23
Hello `@Azer Babaev [Starkit, TC]`, feel free to add translations, we will be happy to review your PR:

[https://github.com/cyberbotics/webots/tree/master/resources/translations](https://github.com/cyberbotics/webots/tree/master/resources/translations)

##### Azer Babaev [Starkit, TC] 01/28/2021 22:19:39
Thank you. Good to know that it is so easy. I shall arrange people at my side to add Russian translation.

##### Devan 01/29/2021 04:31:27
does webots use git LFS? I'm just a student who cloned the repo to try and understand some of the internal codebase for educational purposes and I noticed the file size was over 5GB

##### Olivier Michel [Cyberbotics] 01/29/2021 07:48:43
You should not need git LFS to check out Webots as Webots doesn't contain any file larger than 5GB.

##### Devan 01/29/2021 08:33:24
I know I just thought GitHub itself preferred to keep repositories under 2GB


But I think I heard wrong

##### Olivier Michel [Cyberbotics] 01/29/2021 08:34:33
Yes, there not such a limit in GitHub.

##### Simon Steinmann [Moderator] 01/29/2021 09:58:03
`@Darko Lukić` wouldn't it be possible to run a translator api over the language files and do a base translation? That seems like a fairly easy and quick way to at least offer more languages. Of course they may not be perfect, but might be bettter than nothing

##### Darko Lukić [Moderator] 01/29/2021 10:08:53
I think it is possible, but I am concern about the quality of the translations.

##### jasper [Moderator] 01/29/2021 10:10:29
I think the quality would be rather poor since there is often only a small snippet of text and translation algorithms usually work much better with context, it might be worth a shot but a native or very fluent speaker would need to rework it

##### Ginzo1 01/29/2021 12:47:01
Hello, everyone! I've just recently pick up WeBots as I have an university project that requires to be simulated. It is a robot design project for which I will need a hydraulic scissors lift. Since I have a relatively basic knowledge on mechanics and kinematics but I am still really inexperienced with WeBots. Since the scissors lift is a quite common system, I was wondering if there is some information somewhere on how to create one on my own or see how it is supposed to be done from scratch? Thanks in advance!

##### jasper [Moderator] 01/29/2021 12:49:47
`@Ginzo1` this question has been asked a while back, maybe there is something useful for you there [https://cyberbotics.com/doc/discord/development-2020#lukulus-11132020-10-48-16](https://cyberbotics.com/doc/discord/development-2020#lukulus-11132020-10-48-16)

##### Azer Babaev [Starkit, TC] 01/30/2021 10:34:05
Dear Webots developers, it is more or less clear how to translate GUI to Russian language. Students in our University started to undertake this work. Separate story is how to translate and publish User Guide and Reference Manual which  is located in website of Cyberbotics. There are 2 variants: 1. Russian translation will be published in Cyberbotics site along with English. 2. In Russian GUI link to User Guide will be different, then we can undertake providing space on web for translated User Guide.

##### Simon Steinmann [Moderator] 01/30/2021 14:18:53
`@Azer Babaev [Starkit, TC]` Awesome! Thank you for the contribution. I'm sure the devs will do everything they can to support your effort!


[https://github.com/cyberbotics/webots/tree/master/docs](https://github.com/cyberbotics/webots/tree/master/docs) this is the documentation location of Webots. "guide" and "reference" are the 2 big documentation directories. I'm sure you can make a fork and start translating these. `@Olivier Michel` Perhaps we should slightly change the structure to accommodate any new translations.

##### Azer Babaev [Starkit, TC] 01/31/2021 11:10:23
Thank you, I shall look for opportunity to make this part to be translated too.

## February

##### Olivier Michel [Cyberbotics] 02/01/2021 10:52:38
`@Azer Babaev [Starkit, TC]`: thank you for undertaking this translation effort.

For translating the GUI, please follow the procedure at [https://github.com/cyberbotics/webots/tree/released/resources/translations](https://github.com/cyberbotics/webots/tree/released/resources/translations)

We will be happy to merge your contribution into Webots.

For translating the manuals, I believe the option 2 is the best for us: we will include in the user guide and reference manual links to community contributed translations.

##### Danii 02/09/2021 10:36:01
is there any tutorial how i can start writing a program to actually move (UR) my robot in the simulation?

##### jasper [Moderator] 02/09/2021 10:37:40
[https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers](https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers)


especially the program the controller section


or if you want to do actual motion planning and more complicated motions like planned trajectories or constraints on those trajectories, I would recommend ROS with moveit, but that is quite a project to get into

##### Danii 02/09/2021 10:45:31
ye i use to work with moveit but moveit2 currently only supports c++ 

and im tryin to create a ROS2 class for my university and it should be in python because the students are learing python in prev classes. So im lookin for an alternativ so that the students can still learn ros2 but with python instead of c++

##### Simon Steinmann [Moderator] 02/09/2021 11:31:47
`@Danii` Cyberbotics and I also developed a standalone IKFast Python module


that makes ik super fast and easy to use in webots


[https://github.com/cyberbotics/pyikfast](https://github.com/cyberbotics/pyikfast)


This includes an example I made with the IRB4600 robot. You only have to recompile the pyikfast solver for your robot. Everything else should work out of the box (robot needs to be supervisor, And the init values for this example may have to be changed). I tried my best to comment the code. PM me if you have questions or suggestions
> **Attachment**: [irb4600\_ikfast\_sample\_2.zip](https://cdn.discordapp.com/attachments/565155651395780609/808666959016099840/irb4600_ikfast_sample_2.zip)

##### Danii 02/09/2021 15:09:09
Oh thanks a lot I’ll look into it!

##### Chernayaten 02/10/2021 22:52:00
Double clicking a .wbt file to start Webots, from a path-file inside a Greek folder crashes the program. To be more specific, Webots starts with the following message showing on the console window for a split second before the program terminates automatically.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/809194994279645214/unknown.png)
%end


If I initiate the webots program from its shortcut I can normally load the world without any issues


I'd argue that besides the incorrect handling of the greek characters (and possibly other characters as well), the program shouldn't crash if it fails to load the world

##### Adyasha 02/12/2021 21:32:21
Can't I set a motor speed more than 10 in webots? It shows error ! My bot is too slow

##### Stefania Pedrazzi [Cyberbotics] 02/15/2021 07:11:58
`@Adyasha` the maximum speed for a robot is defined in the `Motor.maxVelocity` field. You should check the value for your robot model and adjust it:

[https://www.cyberbotics.com/doc/reference/motor#field-summary](https://www.cyberbotics.com/doc/reference/motor#field-summary)

##### JMarple 02/16/2021 18:50:55
I wish I had more information (cause I know just saying something is broken isn't all that helpful) but I figure I'd atleast pass this along: 

Running webots on Mac OSX 10.15.6 on MBP 2020, I've been finding webots crashes quite regularly.  It can be just from starting a simulation, or moving a part within the UI, or doing nothing at all, it'll just randomly crash.  Maybe a frequency of once per hour or so. 



Is there any sort of verbose mode or *something* I can do to help track down these issues?

##### Darko Lukić [Moderator] 02/16/2021 19:05:53
Hello `@JMarple`. You can compile Webots in debug mode and you should get more details about the crashes:

[https://github.com/cyberbotics/webots/wiki/macOS-installation](https://github.com/cyberbotics/webots/wiki/macOS-installation)



Please let us know if you have any problems compiling Webots.

##### Ariel 02/16/2021 20:50:48
Hello! I am writing a program in webots for an autonomous vehicle moving around a specific path and collecting data (images) of the environment. I added a fog node on the scene and the cameras that I had already can not see it. Is this the expected behaviour?

##### DrVoodoo [Moderator] 02/18/2021 15:05:40
`@Ariel` is the fog node declared before or after the camera in the world file? If it's after, try moving the Fog node to before the camera nodes.

##### Ariel 02/18/2021 19:48:53
I thought the fog node doesn't need to be declared at all?


in the controllers code file if this is what you mean

##### Srivastav\_Udit 02/18/2021 22:38:30
Hi, I wanted to know if there was any sample code available for a robot that has to detect, collect and drop an object at a particular location? I'm trying to work towards a swarm implementation eventually. Any help will be much appreciated. Cheers!

##### Darko Lukić [Moderator] 02/23/2021 16:56:46
`@Simon Steinmann` Regarding the RAM usage. I have just tried add the UR5 in the empty world and uses like 850MB of RAM. It seems the complex models cause the high RAM usage.

##### Simon Steinmann [Moderator] 02/23/2021 16:57:58
340 -> 1.250 -> 520    (ram usage for your described scenario)


however after a while the 520 trickles down to a bit above 400


I used the UR10e back in the day for my RL testing


it sounds like there is lots of room for optimization. A Model that has 50MB at most in model data, should not need 500MB of RAM (at least in my uninformed opinion 😄 )

##### Darko Lukić [Moderator] 02/23/2021 17:05:21
Let's say you have 8 core CPU with 16 thread. To reach the full CPU utilization you should run 8 instances (2 processes per instance, 1 Webots process + 1 controller process). Let's say your simulation uses 1GB of RAM. That means that Webots parallel simulations will use 8GB of RAM in total. Considering that typical 8 core PC is equipped with 16GB of RAM (and RAM is cheap) then running the parallel simulations should be fine?

##### Simon Steinmann [Moderator] 02/23/2021 17:06:38
If it were just the simulation, yes, but running deep neural networks takes lots of resources as well


plus, if we were to scale it up on server level, the amounts of RAM get huge. A partner company in our past research project had a 128 core 256 thread server.

##### Darko Lukić [Moderator] 02/23/2021 17:10:07
I am trying to understand how import is to invest time in the RAM/VRAM optimization.

##### Simon Steinmann [Moderator] 02/23/2021 17:11:25
well, same simulation and setup in Pybullet were in the order of 100-300MB RAM

##### Darko Lukić [Moderator] 02/23/2021 17:11:45
Per instance?

##### Simon Steinmann [Moderator] 02/23/2021 17:11:48
yes


i have to check if I find the detailed data


Overall RAM usage during the benchmark run: Webots 1811 MB, PyBullet 927 MB


dont have a more detailed breakdown, but I think around 600-700MB is the portion without the simulation, so Pybullet was 200-300MB


Webots has many great advantages for RL, but this is a major downside.


PyBullet has the advantage, that you only initialize what you need, and the simulation is run on a server, completely seperate from a client, which you can connect or not. This is not super userfriendly of course (where Webots shines). But for more complex training scenarios, this is quite important

##### Darko Lukić [Moderator] 02/23/2021 17:20:53
But quality of the Webots model is much higher (pybullet model in the image)
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/813822706759499826/unknown.png)
%end



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/813822734861598720/unknown.png)
%end

##### Simon Steinmann [Moderator] 02/23/2021 17:21:27
we used the same model


directly urdf in PyBullet, conversion in Webots


the resulting protos is 9.282 KB

##### Darko Lukić [Moderator] 02/23/2021 17:39:10
I have just done some tests with UR5e in an empty world:

- 850MB with initial setup,

- 400MB with all graphics enchantments turned off,

- 177MB with meshes removed, and

- probably less with textures disabled (haven't tried).



Does it make sense that we add a CLI argument to disable meshes and textures?

##### Simon Steinmann [Moderator] 02/23/2021 17:39:37
CLI argument?

##### Darko Lukić [Moderator] 02/23/2021 17:40:05
Yes, something like, `webots --disable-meshes`

##### Simon Steinmann [Moderator] 02/23/2021 17:40:17
being able to run a simulation with all graphics turned off, would be fantastic. There is tons of simulation, where no visual sensors are needed


YES


please! THIS!

##### Darko Lukić [Moderator] 02/23/2021 17:40:35
Nice! 😄


Can you make an issue about this?

##### Simon Steinmann [Moderator] 02/23/2021 17:41:30
you already described it better :p


but I can open it if you want

##### Darko Lukić [Moderator] 02/23/2021 17:41:55
Ahh, ok, I will 🙂

##### Simon Steinmann [Moderator] 02/23/2021 17:42:01
thx ^^


- 400MB with all graphics enchantments turned off,


I mean


ohhhhhhhhhhh


yeah


perhaps it could be launched with no rendering? And only load those things, when a different mode is selected


but your suggestion is already super great 🙂

##### Darko Lukić [Moderator] 02/23/2021 17:49:04
[https://github.com/cyberbotics/webots/issues/2777](https://github.com/cyberbotics/webots/issues/2777)

##### Simon Steinmann [Moderator] 02/23/2021 17:50:20
would this include the graphics settingsß


?


perhaps "no render" mode can turn those off, if it frees up ram


ohhh makes sense


no textures and no meshes, does it still render? just with the bounding objects?

##### Darko Lukić [Moderator] 02/23/2021 17:56:48
Yes

##### Simon Steinmann [Moderator] 02/23/2021 17:57:16
thx

##### Srivastav\_Udit 02/24/2021 14:06:04
Is there a way to detect an object only when it is in the middle of the field of view?

##### pnaraltnsk 02/24/2021 20:15:27
Hi I am trying to set a specific rotation to my humanoid robot but my robot gets into weird positions. For example, I want it to rotate around z-axis so I set [0,0,1, rad] but it rotates into weird positions. Could you please help me with this problem? I am using supervisor functions to rotate.

##### Simon Steinmann [Moderator] 02/25/2021 13:21:47
what is happening instead? Is the field changing? Or is it just not behaving as expected?

##### Darko Lukić [Moderator] 02/25/2021 13:22:37
Also, what is the initial rotation of the robot?

##### pnaraltnsk 02/25/2021 20:00:04
It is not behaving as expected and the initial rotation of the robot is 1,0,0,-1.57

##### Darko Lukić [Moderator] 02/25/2021 20:02:15
So your robot is already rotated around the X axis by -pi/2. That means, the method you wrote rotates the robot, by pi/2 around the X axis + `rad` around the Z axis. Is that what you want?


To rotate around the Z axis from the initial rotation you have to:

- take the rotation,

- convert the rotation to a rotation matrix or quaternion,

- multiply by a rotation matrix/quaternion that represents desired rotation around Z axis, and 

- convert back to the angle-axis representation.

##### Simon Steinmann [Moderator] 02/25/2021 22:22:13
I highly recommend this library for the conversions: [https://matthew-brett.github.io/transforms3d/](https://matthew-brett.github.io/transforms3d/)

##### Gonçalo Carrola Silva 02/26/2021 11:20:21
Hi everyone!!! I'm totally new to using WEBOTS so I have a simple (I hope) problem of development of my robot. I want to use inverse kinematics on PR2 robot (it has to be PR2) to move the right hand to a certain position of the world. For that, I tried to use the ikpy library just like in the ABB' IRB 4600/40 example, and then adapting that code to the PR2 robot, but it seems that it does not work. What do you guys think I am doing wrong? Is there any other way that you suggest to implement an inverse kinematics solution for the PR2 robot using Python?

##### Simon Steinmann [Moderator] 02/26/2021 11:21:10
[https://github.com/cyberbotics/pyikfast](https://github.com/cyberbotics/pyikfast)


this will be the best solution


once you generated your solvers, I can help you out implementing it


or you use ROS and MoveIt

##### Gonçalo Carrola Silva 02/26/2021 11:31:48
Simon, I do not know what to replace in [base\_link], [effector] and [module\_extension] parameters on the READ\_ME file. This is my udrf file:



> **Attachment**: [Pr2.urdf](https://cdn.discordapp.com/attachments/565155651395780609/814822069497888778/Pr2.urdf)


Note that I am using Windows, so I do not use ROS

##### Simon Steinmann [Moderator] 02/26/2021 12:05:23
hmm this file looks wrong


how did you extract it?


just right click on the robot, export and then select .urdf

##### Gonçalo Carrola Silva 02/26/2021 12:08:24
Yes, I did that, but the only option to save the file is .wbo. I think that is the problem

##### Simon Steinmann [Moderator] 02/26/2021 12:09:05
what version of webots do you use?


you need R2021a

##### Gonçalo Carrola Silva 02/26/2021 12:09:46
I think mine is R2020b

##### Simon Steinmann [Moderator] 02/26/2021 12:09:55
you gotta update that


in the meantime, this is the extracted file
> **Attachment**: [Pr2.urdf](https://cdn.discordapp.com/attachments/565155651395780609/814831657579839518/Pr2.urdf)


okay, the pr2 arm has 7 DOF. Ikfast only works with 6


so there has to be an inactive link

##### Gonçalo Carrola Silva 02/26/2021 12:17:57
So, in [base\_link] I put base\_link, in [effector] I put base\_footprint?

##### Simon Steinmann [Moderator] 02/26/2021 12:18:17
base\_link = "torso\_lift\_link"

effector = "r\_wrist\_flex\_link"


I think


module extension do "\_pr2"


see if that works


`@Darko Lukić` we should look into enabling options for DOF != 6


ikfast has options to set joints as inactive


or compile different solvers for DOF = 5 or 4

##### Gonçalo Carrola Silva 02/26/2021 12:28:17

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/814836233606922240/unknown.png)
%end

##### Simon Steinmann [Moderator] 02/26/2021 12:29:10
you need to install docker

##### Gonçalo Carrola Silva 02/26/2021 14:33:40

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/814867787960418324/unknown.png)
%end

##### Ginzo1 02/26/2021 14:56:57
Hello guys! I am new to webots and for the project I am working we require the robot to receive input arguments before the start of the simulation. Is there a way to receive those arguments before the start of the simulation? The robot has to receive predefined coordinates and move towards them.

##### DDaniel [Cyberbotics] 02/26/2021 15:05:09
`@Ginzo1` you can use a supervisor to send specific commands to each robot using emitter/receivers.


you can use file > open sample world > samples > curriculum > advanced\_genetic\_algorithm as reference on how to exchange info

##### Simon Steinmann [Moderator] 02/26/2021 15:13:07
`@Gonçalo Carrola Silva` make sure docker is running


did you launch docker? you're on windows right?

##### Gonçalo Carrola Silva 02/26/2021 15:16:30
I'm on Windows yes

##### Simon Steinmann [Moderator] 02/26/2021 15:16:53
make sure docker desktop is launched

##### Gonçalo Carrola Silva 02/26/2021 15:17:20

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/814878777585827840/unknown.png)
%end


I think that is. But I tried to run that container and the message error is the same

##### Simon Steinmann [Moderator] 02/26/2021 15:18:49
when docker is running, you should be able to open windows power shell


and then use the commands on the github


ofc you have to navigate to the correct directory first

##### Gonçalo Carrola Silva 02/26/2021 15:20:54
But the directory isn't the one where the .urdf file is?

##### Simon Steinmann [Moderator] 02/26/2021 15:21:16
you have to navigate there


in your folder press "ctrl + L", copy the path


then in the powershell:

cd "<paste path>"


you put it inside of quotation marks


cd ""


the path inbetween

##### Gonçalo Carrola Silva 02/26/2021 15:31:13

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/814882271290195998/unknown.png)
%end


I don't know what is happening


The first sentence of the error message is: "Invalid characters on the way"

##### Simon Steinmann [Moderator] 02/26/2021 15:32:26
try a path with no special characters


peprhaps just your desktop or standard document folder

##### Gonçalo Carrola Silva 02/26/2021 15:34:20
Ok, I am there. I will try to run that command on the github


Nope, same error

##### Simon Steinmann [Moderator] 02/26/2021 15:36:28
try running it as administrator

##### Gonçalo Carrola Silva 02/26/2021 15:36:48

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/814883676290285608/unknown.png)
%end

##### Simon Steinmann [Moderator] 02/26/2021 15:36:50
and I dont know what your path or your characters are


is it running?

##### Gonçalo Carrola Silva 02/26/2021 15:37:38
I don't think so


I think the problem is only in the docker. I cannot run docker\_machine either, it seems that it is not installed

##### Simon Steinmann [Moderator] 02/26/2021 15:39:29
I thought you installed docker?


[https://docs.docker.com/docker-for-windows/install/](https://docs.docker.com/docker-for-windows/install/)


pr2 is not the best candidate for this all. I had to heavily modify the urdf. I'm compiling a solver right now


use this urdf
> **Attachment**: [robot.urdf](https://cdn.discordapp.com/attachments/565155651395780609/814885845462548510/robot.urdf)


`docker run -v ${PWD}:/output cyberbotics/pyikfast torso_lift_link solid_363 _pr2` and this command

##### Gonçalo Carrola Silva 02/26/2021 15:50:36

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/814887150033305660/unknown.png)
%end


I already made all these steps to install docker, except step 5. Maybe the fact that my user is not the administrator is the reason that the docker is not running



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/814887572685717544/unknown.png)
%end


This is the message error when I run powershell as an administrator

##### Simon Steinmann [Moderator] 02/26/2021 15:52:47
oh yeah, you should run the shell as admin

##### Gonçalo Carrola Silva 02/26/2021 15:59:11
`@Simon Steinmann` do I need to install docker toolbox?


I think I finally can run docker


The problem was that it misses the WSL2 container



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/814896647942832189/unknown.png)
%end


I am really sorry to be such a noob XD. I run the docker, and after the installation of many packages it appears this:



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/814921762495266896/unknown.png)
%end


I don't know if the reason was a bad introduction of the 3 arguments of base\_link, etc... or if it is something else


Then, I follow the rest of the github guidance, put "pip3 install . " command and then this appears:



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/814922508766543873/unknown.png)
%end


Although maybe both errors are correlated


Yet about yesterday, I think I made some progress. I finally can run docker but it seems that the file ikfast\_robot.cpp does not exist...



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/815203976470331392/unknown.png)
%end

##### Simon Steinmann [Moderator] 02/27/2021 12:50:21
you need python3 i'm pretty sure

##### Gonçalo Carrola Silva 02/27/2021 12:50:38
I have python3

##### Simon Steinmann [Moderator] 02/27/2021 12:50:43
can you show the command you entered?


i'm currently rerunning everything again. I had troubles doing the pip install afterwards

##### Gonçalo Carrola Silva 02/27/2021 12:51:35
docker run -v ${PWD}:/output cyberbotics/pyikfast torso\_lift\_link solid\_363 \_pr2

##### Simon Steinmann [Moderator] 02/27/2021 12:52:02
you were in the correct directory?

##### Gonçalo Carrola Silva 02/27/2021 12:52:16
yes. Absolutely sure


In the same directory as the urdf file

##### Simon Steinmann [Moderator] 02/27/2021 12:53:10
you have wsl2 installed?


and can you show the entire console output?

##### Gonçalo Carrola Silva 02/27/2021 12:53:44
I can't. It's too big

##### Simon Steinmann [Moderator] 02/27/2021 12:53:52
you can export as .txt file


oh I just got the same thing

##### Gonçalo Carrola Silva 02/27/2021 12:54:31
How?


Maybe is something missing in the urdf file?

##### Simon Steinmann [Moderator] 02/27/2021 13:02:03
`RuntimeError: maximum recursion depth exceeded`


that is the real issue


compiling the solver failed


I ran into that in the past... was a while ago though

##### Gonçalo Carrola Silva 02/27/2021 13:18:03
So, do you know an alternative way to do kinematics with Python for this robot or do you think that the best way is to define it manually? But doing manually maybe the problem maintains, because if I understand it correctly the real challenge is the urdf part.

##### Simon Steinmann [Moderator] 02/27/2021 13:19:24
inverse kinematics is not easy


pr2 is a very complicated robot


ROS and MoveIt are the most refined options


there is no good standalone solutions as far as I know. At least not in python


you could try ikpy. Perhaps it handles 7DOF somewhat well, but I always found it sluggish and difficult to work with in the past. But that was on 6DOF bots

##### row 02/27/2021 18:40:24
I just tried to run webots on a M1 MacBook Air. It runs flawlessly.


Real


I got a refurbished M1 Macbook Air base model

##### kRiSh 02/27/2021 18:45:40
Wow, Apple is catching up.


The 2 years promise would turn one and a half I guess.

##### row 02/27/2021 18:46:06
It's crazy because I think webot is translated in rosetta. it's not even running on native arm


Yeah.. I am gonna do more testings once I have the time. But it's mind blowing.


Hopefully we will see a native arm built in the future

##### Whizbuzzer 02/27/2021 21:10:39
> Tim Cook OP 😆

`@kRiSh`  

🤣

##### row 02/28/2021 01:18:23
`@Olivier Michel` Sure. 👍 Let me know how I can help

##### Ginzo1 02/28/2021 20:01:54
Hello everyone! I am sorry for the newbie question but I do not seem to understand how to do it. I've built a robot and I need it to go to a specific box, however, I do not know how to get the position of the box in the world. I've read about using the Supervisor but it does not seem to work properly. What would be the best way to get the coordinates of an object in the world? Also, is the origin (0,0,0) at the centre of my floor node? If it is not, how do I find where the origin is? Thanks!

##### Chernayaten 02/28/2021 20:04:07
The simplest way to find where your 0,0,0 is would be to set your robot (or any object's position) to those coordinates (via the Scene Tree). Similarly, if this is not an issue for you, you can hard code your object's coordinates in your code

## March

##### h.sciascia 03/01/2021 09:06:20
Hello ! How to set min and max motor limits to a negative value ?

##### jasper [Moderator] 03/01/2021 09:09:18
make sure that minPosition < maxPosition

##### h.sciascia 03/01/2021 09:25:46
I want a min at -180° and max at -45° , so minPosition<maxPosition


But I have the error : WARNING: DEF Robot3DOF Robot > HingeJoint > DEF axe0Solid Solid > HingeJoint > RotationalMotor: Invalid 'maxPosition' changed to 0. The value should be 0 or greater.


Oh sorry i'm in the wrong channel

##### jasper [Moderator] 03/01/2021 09:30:42
this is not really expected behavior but you can rotate your motor by 45° before to work around this. If you give me your proto I can quickly calculate the correct rotation

##### row 03/02/2021 12:14:01
`@Olivier Michel` will do. Thanks! 👍

##### Srivastav\_Udit 03/03/2021 03:50:10
Hi! Has anyone worked with A* path planning for robots? I'm trying to understand how I can implement the algorithm and would really appreciate some guidance.

##### row 03/03/2021 21:10:27
`@Srivastav_Udit` There is an open source project PythonRobotics ([https://github.com/AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)) and it implements many robotics related algorithms, including A*. Everything is written in Python so it's very easy to read and implement a study case.

##### cindy 03/06/2021 23:01:42
Hi I have a small coding issue that I can't figure out from the documentation alone


`LightSensor *light_sensor = robot->getLightSensor("TEPT4400");` gives 'error: 'light\_sensor' does not name a type'


though LightSensor is clearly the class name


yes



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/818470690239021116/unknown.png)
%end

##### Stefania Pedrazzi [Cyberbotics] 03/08/2021 13:11:26
Did you check that `light_sensor` is not NULL and that the device name is correct?

##### cindy 03/08/2021 13:12:08
Oh not yet. It might be a naming issue yeah


No, it shouldn't be null, I've checked the proto file of the robot

##### Stefania Pedrazzi [Cyberbotics] 03/08/2021 13:19:25
What I would suggest is to simplify it and remove everything else that it is not strictly needed (additional include statements, code, etc). Keeping only the LightSensor code you should be able to identify where does the error come from. Also you should check that in your Makefile the `USE_C_API` is not set to true.

##### cindy 03/08/2021 13:20:59
Aha that is the problem


It is set to true


Would changing it manually work?

##### Stefania Pedrazzi [Cyberbotics] 03/08/2021 13:24:21
Note that this variable tells the Makefile compilation system that in your C++ controller you will use the Webots C API. So if you are going to use Webots C++ API it seems there is no need to set it.

##### cindy 03/08/2021 13:36:12
i've deleted it completely and the issue is persisting

##### Simon Steinmann [Moderator] 03/08/2021 23:25:40
<@&568329906048598039> Is there a way to SET the values of a sensor? This would be very useful, allowing external sensors to be connected and to feed the simulation.  On a separate note: Is there a way to directly define an MF field, without having to set every item index by index. Every set call takes a certain time. When filling an array with hundreds of values, this is incredibly slow.

##### Olivier Michel [Cyberbotics] 03/09/2021 07:17:35
Yes, this is possible from the remote control library:

[https://cyberbotics.com/doc/guide/controller-plugin#remote-control-plugin](https://cyberbotics.com/doc/guide/controller-plugin#remote-control-plugin)

[https://github.com/cyberbotics/webots/blob/master/include/controller/c/webots/remote\_control.h](https://github.com/cyberbotics/webots/blob/master/include/controller/c/webots/remote_control.h)

The original purpose of this is to be able to switch a controller between the simulation and a remote controlled real robot (and display the sensor values of the remote controlled robot in Webots as it is was simulated sensors).

##### Simon Steinmann [Moderator] 03/09/2021 20:25:28
`@Olivier Michel`  thanks, I will take a look


There is  a weird issue, where the pointcloud of a lidar is only shown, if the lidar is closer.
> **Attachment**: [lidar\_issue.mp4](https://cdn.discordapp.com/attachments/565155651395780609/818943268196450314/lidar_issue.mp4)


I figured it out: As soon as a ray has no collision (no obstacle within max range), NO point is shown at all. Even though there are valid points to be drawn

##### Olivier Michel [Cyberbotics] 03/09/2021 20:46:35
That's a known bug which was fixed recently.

##### Simon Steinmann [Moderator] 03/09/2021 20:46:56
Oh good 🙂

##### Olivier Michel [Cyberbotics] 03/09/2021 20:47:20
Did you try a nightly build?

##### Simon Steinmann [Moderator] 03/09/2021 20:47:41
no, official 2021a I think

##### Olivier Michel [Cyberbotics] 03/09/2021 20:47:58
It should be fixed in all nightly builds.

##### Simon Steinmann [Moderator] 03/09/2021 20:48:18
gonna check it out later


`@Darko Lukić` For working with ros2 and moveit, which version of Webots do I use? The nightly build, or the automatic .ros install (15. Dezember)

##### Darko Lukić [Moderator] 03/09/2021 22:52:42
Any should work

##### Ginzo1 03/10/2021 23:15:11
Hello everyone! I need to draw lines on the floor that my robot will follow for a school project however I do not seem to find a proper way to do it. Could someone give me any ideas on how to do it? 🙂

##### KajalGada 03/11/2021 01:05:37
`@Ginzo1` check out this tutorial by DrakerDGRobotics on how he made his track using TinkerCAD. You can design your own style based on that.



[https://youtu.be/XUSCD7aYtQ8](https://youtu.be/XUSCD7aYtQ8)

##### Srivastav\_Udit 03/23/2021 07:12:09
Hi! I need to replace this line wb\_differential\_wheels\_set\_encoders(0, 0); with code that is accepted for Position Sensors by Webots. Are there any recommendations?

##### Ginzo1 03/25/2021 18:22:47
Hello everyone! I need to create a spraying device for my simulation - the form and size do not matter so I wondered if something similar's been done before and it is available to use. Also any suggestions for the implementation are more than welcome 🙂

##### Vial 03/25/2021 23:35:00
Hi, I'm trying to implement a new endpoint for lidars in remote control library but I have some questions about the API, I thought it would be much easier for everyone with screenshots. Hence the pdf file, it's mostly screenshots.

Thanks by advance for considering this question
> **Attachment**: [Question-remote-api.pdf](https://cdn.discordapp.com/attachments/565155651395780609/824788493434617886/Question-remote-api.pdf)

##### Spur 03/27/2021 05:05:26
dumb question but does anyone know what the lidar point cloud returns and how to access it?

##### ouur 03/28/2021 15:49:44
Hello everyone. im using python and trying to get self node but it gives me error. Any idea?

TypeError: in method 'Supervisor\_getSelf', argument 1 of type 'webots::Supervisor const *'

##### Simon Steinmann [Moderator] 03/28/2021 18:30:49
`@ouur` show us your code


I found a very strange issue. The wheel seems to move the robot in the same direction, regardless of which way it is spinning
> **Attachment**: [simple\_crashderby\_1.mp4](https://cdn.discordapp.com/attachments/565155651395780609/826536875283447818/simple_crashderby_1.mp4)

##### Darko Lukić [Moderator] 03/30/2021 19:26:39
What's that? Which world?

## April

##### Simon Steinmann [Moderator] 04/01/2021 21:13:50
it's due to kinematics mode (no physics). It works for velocity control, but not position control

##### Spur 04/02/2021 06:37:26
does getRecognitionObjects() return a pointer to an array of objects or just to one? and how do I access the id of one of those object?

##### Simon Steinmann [Moderator] 04/04/2021 01:21:49
<@&568329906048598039> The webots docs directory is huge. I know that transparency is nice with the png files, but they are only included in white documents. These images could be compressed to jpg files, reducing the file size by about 10-20 times.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/828076864353730600/unknown.png)
%end

##### DDaniel [Cyberbotics] 04/04/2021 07:27:53
`@Simon Steinmann` it's in the works, the offline documentation might be removed entirely by default (with the option of downloading it if needed) among other things. [https://github.com/cyberbotics/webots/pull/2787](https://github.com/cyberbotics/webots/pull/2787)

##### Spur 04/08/2021 01:56:50
Does anyone know how the  'position' field of a Camera Recognition Object works? it says it has 3 values but I'm unsure what they represent

##### DrVoodoo [Moderator] 04/08/2021 16:06:45
I'm having some grief on a world a ENU world with an inertial unit


I have the inertial unit aligned as per the docs (x forward, y up, z right)


And I am using getQuaternion() to get the orientation (which I assume is returning in x,y,z,w order)


Now the robot is on a flat(ish) surface so that's giving me ( 0.000950936, -0.00466466, 0.980983, -0.194035 ) for example


No wait, I think I figured this out. I'm trying to present the data in PCL which is using NUE still


As always, you bang your head on the problem for an hour and realise the issue as soon as you start explaining it.

##### Simon Steinmann [Moderator] 04/09/2021 00:24:23
axis angles usually

##### Spur 04/09/2021 06:26:06
wont there only be 3 though? xy xz yz ?

##### jasper [Moderator] 04/09/2021 06:29:30
In the axis angle notation used in webots  the first three values specify the rotation axis and the fourth specifies the rotation in radians

##### Spur 04/09/2021 06:32:35
sorry i'm a bit confused, how do the three values describe the axis, isnt an axis by definition only 1 value, ie x axis y axis or z axis

##### jasper [Moderator] 04/09/2021 06:36:12
It is not necessarily the x y or z axis. You can think of the axis being specified as a line though the origin of the coordinate system and a point on the unit sphere specified by the three coordinates. If the point is (1,0,0) it is simply a rotation around the x axis but any axis is possible

##### Spur 04/09/2021 06:38:19
oh ok, forgive my ignorance but whats the point of the rotation value then? (4th value)

##### Drake P. 04/09/2021 06:53:55
The 4th value is the roll around the specified axis to my understanding

##### Spur 04/09/2021 06:54:21
oh gotcha thanks

##### jasper [Moderator] 04/09/2021 07:16:05
Yes, thanks `@Drake P.` for explaining

##### Gotcha97 04/09/2021 19:29:53
Is it possible to add a speaker device to an existing robot template (in my case a TIAGo Titanium)?

##### Olivier Michel [Cyberbotics] 04/09/2021 20:49:33
Yes, you should be able to insert it in one of the extension slots of the TIAGo proto.

##### Westin 04/14/2021 16:39:28
I found a typo.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/831931677990191204/unknown.png)
%end

##### Simon Steinmann [Moderator] 04/14/2021 19:51:46
<@&568329906048598039> [https://github.com/cyberbotics/webots/blob/develop/src/webots/nodes/WbLidar.cpp#L728](https://github.com/cyberbotics/webots/blob/develop/src/webots/nodes/WbLidar.cpp#L728) found it. it's supposed to be 'using' right?


it comes up several times. Doing a search in VC for webots, brings up more instances. Mostly in translation files.

##### aja\_discord 04/15/2021 08:17:34
Hi Guys checkout my first own project on webots where I have made a self balancing robot which learns to balance itself using a genetic algorithm.Let me know your ideas. Thanks..LINK - [https://www.youtube.com/watch?v=l9BZQ9E5y6A](https://www.youtube.com/watch?v=l9BZQ9E5y6A)

##### Troy 04/20/2021 00:48:34
Hi sorry to bother again, we can use camera to recognize object, can we directly  get distance feedback between the camera and the object?


for example, the camera on the robot recognizes an object on top right of the image, but the distance sensor is pointing in the middle, how can I get the distance feedback of the object?

##### bobhead 04/22/2021 12:48:39
`@Olivier Michel` Thank you for linking that Master Thesis paper. I am also currently working on my Bachelor Thesis, doing research on SLAM. Is there a way to get in contact with you guys? Can I DM you on Discord?

##### Simon Steinmann [Moderator] 04/22/2021 22:06:53
does anyone here have experience compiling webots for profiling? make has the `profile` and `debug` options. This page says i have to compile in debug [https://github.com/cyberbotics/webots/wiki/Valgrind](https://github.com/cyberbotics/webots/wiki/Valgrind). I remember not being able to successfuly use valgrind in the past. But I think the profile option did not exists back then. Any ideas how to proceed?

##### Olivier Michel [Cyberbotics] 04/23/2021 06:23:58
Simply type "make profile" in the `WEBOTS_HOME/src/webots` folder. Then, you should be able to use gprof to analyse the result of a run. See [https://sourceware.org/binutils/docs/gprof/](https://sourceware.org/binutils/docs/gprof/)

##### jasper [Moderator] 04/23/2021 13:56:53
The current way of modeling noise for gyros and accelerometers using a lookup table. The noise is given as a percentage of the response value (see [https://cyberbotics.com/doc/reference/distancesensor#lookup-table](https://cyberbotics.com/doc/reference/distancesensor#lookup-table)). In my opinion this does not make much sense as a model of noise for these sensors (opposed to distance sensors for which it does) since it causes that there is very little noise around zero. 

In gazebo using a gaussian around the signal with a fixed standard deviation and an optional bias. 

Is there a way to model the noise in this way in webots as well?

##### Olivier Michel [Cyberbotics] 04/23/2021 14:45:29
Well, if you increase the noise when getting close to the 0 zero value in the lookup table, you will have somehow a constant noise level. But I agree, this is not ideal. If you have any idea to improve this, feel free to open an issue.

##### Ragemor 04/27/2021 22:33:47
ı have different colored objects in my simulation. and i want to move my robots to different colored objects. how can i perform that? basicly a leader follower concept with camera recognition. I try to use objects[].colors[] but i cant made what i want. my english is not pretty good. anyone can help me about that?


this is my simulation. reds first and greens behind them. this is working for a while but if green object robots see another colored robot it stopped moving.
%figure
![webbb.jpg](https://cdn.discordapp.com/attachments/565155651395780609/836732246350233631/webbb.jpg)
%end



> **Attachment**: [code\_greens.txt](https://cdn.discordapp.com/attachments/565155651395780609/836732424238792774/code_greens.txt)

##### Nick R 04/29/2021 21:45:23
Very general question about Webots: was it free for academic use before becoming free/open source in 2019?

##### Olivier Michel [Cyberbotics] 04/30/2021 06:44:30
No. It was not free for academic use. It became open source in December 2018.

##### Nick R 04/30/2021 14:17:27
Thank you, was just wondering.

## May

##### ShuffleWire 05/05/2021 15:47:54
Hello ! 

I'm using gdb to play with webots, and additionally to the -g option, I had to use the -ggdb one. I've read about it that it generate more "gdb friendly" symbol (and there is additional version of this flag). Is it expected or did I miss something ?

##### Olivier Michel [Cyberbotics] 05/05/2021 17:18:52
I believe it is not included in the Makefiles of Webots, but it should be easy to add. What are the exact benefits of this flag?

##### ShuffleWire [Moderator] 05/06/2021 11:26:07
I've not, but should make a PR for that ? It's seems to be a so tiny fix...


Allright, I might have some to do, then 🙂

##### Daemongear 05/17/2021 01:34:31
Hello, so as Simon (many thanks btw) asked, I'm rephrasing my question, maybe someone can help with advice. I want to publish the WeBots joints to the ROS joint\_state\_publisher. I am using the standard, tutorial ROS controller


so far the only options I've found were A) rewrite parts of the standard controller (which is heavily distributed of course) to get the joints directly published to the right topic or B) write my own node (much simpler) to subscribe to the default topics after enabling the motor sensors, and publish them to the joint\_state\_publisher. this would of course induce some delay.


Any other ideas?


how does that work? do I need a launch file like that where I manually specify the joints, or is that a service I can enable or call?


oh that's awesome, I knew there must have been a generic publisher somewhere. I'll try it in a bit, thanks so much for the info!


out of reference-making interest (I'm writing my bachelor project report right now, as it's due today, gg). are you, by any chance, one of the devs of WeBots?


is it alright if I use your name as a reference in a TAROS paper and my project? (my paper is about a comparison between CoppeliaSim and WeBots, as undergraduate viability, while my project is about a specific humanoid application)


it's more as an argument of "their team is significantly more responsive and community-involved", with the basis that CoppeliaSim has just one active dev who barely responds

##### DDaniel [Cyberbotics] 05/21/2021 06:17:07
`@Daemongear` 2021b will be release in roughly a month, but there are nightly builds of it available: [https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)

##### Daemongear 05/21/2021 06:31:41
I've downloaded the entire webots\_ros and placed it in the default location, just to test the setup, and it doesn't seem to work


so basically I need the nightly build installed, right?

##### Darko Lukić [Moderator] 05/21/2021 06:34:42
Yes

##### Daemongear 05/21/2021 06:35:45
ok, did that. is it compatible with the older one, or do I need to delete and refresh the ros thingies?

##### Darko Lukić [Moderator] 05/21/2021 06:35:47
You need the Webots nightly build and the develop branch from `webots_ros` repository for examples

##### Daemongear 05/21/2021 06:38:27
the simulation seems to work now, it does publish to a heck of a lot of rostopics, altough it does throw an error "failed to load arm\_controller". but from here I should be able to develop what I needed


ok it does update the tf, but not the joint\_states and robot\_states


what does that installation require?

##### Darko Lukić [Moderator] 05/21/2021 06:41:36
```
sudo apt install ros-noetic-ros-controllers
```

##### Daemongear 05/21/2021 06:42:01
same for melodic I assume

##### Darko Lukić [Moderator] 05/21/2021 06:42:08
Yes


`rosdep` should install the required packages automatically

##### Daemongear 05/21/2021 06:43:51
I... umm... get an error I don't understand.



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/845190202107756564/unknown.png)
%end


after I've already installed it with apt


oh wait, that's not what rosdep does, is it *facepalm*

##### Darko Lukić [Moderator] 05/21/2021 06:46:29
`rosdep` resolves dependencies based on `package.xml`, you typically run it when you install a package from source to resolve all system dependencies


Installing `ros_controller` from apt should be enough

##### Daemongear 05/21/2021 06:48:41
I've rerun roslaunch webots\_ros tiago.launch , it no longer gives an error on loading arm\_controller, just a warning for wait\_for\_srvice failed to contact, and an error for moveit\_ros\_move\_group, which I assume is a different package

##### Darko Lukić [Moderator] 05/21/2021 06:50:29
Yes, the `moveit_ros_move_group` is a different package, but it should come with the standard MoveIt installation:

[https://moveit.ros.org/install/](https://moveit.ros.org/install/)

##### Daemongear 05/21/2021 06:53:28
well, it now identifies the new install. still gives of error "failed to find 3d sensor plugin param" and "exception while loading planning adapter plugin" after using planning interface OPML.


this is all fine since I am not at the point where I can use move-it yet anyway


how would I connect this to a standard gui joint\_state\_controller?

##### Darko Lukić [Moderator] 05/21/2021 06:54:45
I am not sure about `exception while loading planning adapter plugin` but the other one is fine as Tiago doesn't have a 3D sensor

##### Daemongear 05/21/2021 06:55:25
that's a tomorrow problem for tomorrow me, as I am a long way from using that


yes, but doesn't give back anything. not sure it should tbh


oh I think I found the issue with that weird error from moveIt. upper in the log it says "could not find parameter robot\_description on parameter server" and suggests it's likely because of a remap of "robot\_description"


and after 2 retries it moves on and throws "semantic description is not specified for the same robot as the URDF"

##### Darko Lukić [Moderator] 05/21/2021 07:00:49
It should pick the `robot_description` from a corresponding topic

##### Daemongear 05/21/2021 07:02:07
robot\_description should be a topic? or sth like robot\_state


(both missing)


ok so... good news and bad news. I've started an RViz display from a launch file, now the rostopic echo returns the current positions from Joint\_state, but they don't update in webots


what's worse is that now using rosservice call set\_position doesn't do anything in the webots window


although it does return success true


the simulation is running, but at 0.00x? is this an error or a new setting??

##### Darko Lukić [Moderator] 05/21/2021 07:09:01
I can reproduce, let me check, I think we have a bug in the develop branch

##### Daemongear 05/21/2021 07:09:31
thank you ^\_^


it's part of the roslaunch package, not the webots version


from my testing, the issue seems to be in the ros controller, disabling it lets the simulation run just fine. maybe a launch service command?


umm I think I may have found the source of the bug. looking in the rosservice list output, all the motor fields are doubled.


I think this throws off an error with the services and blocks everything


never mind, that is a separate error, where closing webots and the process doesn't close the services and they get doubled


name/robot/ services are completely unresponsive, so a failed one is probably blocking the entire node

##### Darko Lukić [Moderator] 05/21/2021 08:16:06
This PR has changed the behavior of the `ros` controller:

[https://github.com/cyberbotics/webots/pull/3040](https://github.com/cyberbotics/webots/pull/3040)



As a temporary fix you can  add one more argument `"--use-sim-time"` here:

[https://github.com/cyberbotics/webots\_ros/blob/2bfc812411d82cf82427d0e414be3fc34a04e988/worlds/tiago.wbt#L544](https://github.com/cyberbotics/webots_ros/blob/2bfc812411d82cf82427d0e414be3fc34a04e988/worlds/tiago.wbt#L544)



I have to verify what is the best long-term solution and to fix it

##### Daemongear 05/21/2021 08:28:57
i'll load up the vm rightaway


ok so I have good news and bad news


the good news is that both my sim and tiago do work


the bad news is that I think it automatically requests some data?  tiago seems to be still stuck on not having a robot\_description file, since I'm not sure where that is, while my sim throws continuously errors as "rotational motor: too low requested position" and "too big" of +-pi as compared to 0. so i'm guessing it's also resetting the limits


progress update: (there are 2 services for each motor called "get\_min\_position" and max) which return the proper values, surprisingly enough. I've named some of the joints to track down the issue



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/845220870212288542/unknown.png)
%end


something keeps requesting that position, and I don't know what, since I am not using any controllers and even rviz is closed

##### Darko Lukić [Moderator] 05/21/2021 08:51:18
You are launching the simulation like this?

```
roslaunch webots_ros tiago.launch
```

##### Daemongear 05/21/2021 08:51:21
what could possibly request those positions continuously?


this is not in tiago, tiago works fine

##### Darko Lukić [Moderator] 05/21/2021 08:52:18
Oh, you are running your own robot?

##### Daemongear 05/21/2021 08:52:37
yes, I've updated its files at the same time with tiago

##### Darko Lukić [Moderator] 05/21/2021 08:53:32
You are using ros\_control?

##### Daemongear 05/21/2021 08:53:43
I keep calling from the terminal set\_position as 0.0, but something else is trying to set it weirdly


yes


we can hop on a quick call if you have time


in any case, I've check with both RViz and the proto itself for the limits, they are right, but I don't know where the initial positions are stored?


I... have no idea what that is


ah lol niice, that's a really cool feature for the software. I understand


no, it's off by pi/2 or more


and even so, when i call rosservice call /robot/LElbow/set\_position 0.0, it returns true, but the error and rostopic echo still stay it's at 2.1928

##### Darko Lukić [Moderator] 05/21/2021 08:59:22
Also, ros\_control keeps setting joint positions at each timestep, so the issue may be in ros\_control

##### Daemongear 05/21/2021 09:00:05
hmm ok that makes a lot more sense, as it overwrites at each time step. so where does ros\_control have the inputs? or does it require a controller written?


if I publish something to joint\_states, will it take those values maybe>


?

##### Darko Lukić [Moderator] 05/21/2021 09:00:44
If you delete the `"--use-ros-control"` flag it should be fine

##### Daemongear 05/21/2021 09:02:26
where does ros-control take its initial config from then? that's probably the outdated package I need to modify

##### Darko Lukić [Moderator] 05/21/2021 09:03:11
I am sorry, but this out of scope of the Webots simulation, it is more related to `ros_control`

##### Daemongear 05/21/2021 09:03:26
ooh I think I know. the proto must be generated from my previous description folder, and ros\_control takes the info from the ros package.


I'll update all the packages later and let you know if it solves the issue. it probably very much will


just to make sure I got this right: if I publish to joint\_states, it will update the webots simulation, right?

##### Darko Lukić [Moderator] 05/21/2021 09:30:20
If you use `ros_control` then you the joint control goes through the `joint_trajectory_controller`:

[http://wiki.ros.org/joint\_trajectory\_controller](http://wiki.ros.org/joint_trajectory_controller)

(typical topic name `/command`)

But you have make sure that the controller is properly configured. This is a standard interface for controlling joints utilized by packages like MoveIt.



If you don't use `ros_control` then you can rely on the `set_position` service:

[https://cyberbotics.com/doc/reference/motor?tab-language=ros#wb\_motor\_set\_position](https://cyberbotics.com/doc/reference/motor?tab-language=ros#wb_motor_set_position)

##### Daemongear 05/21/2021 09:41:51
oh ok that clears a lot of things up. However, I still don't understand how to solve my very initial issue: setting up ROS so I can use sliders from RViz (joint\_state\_publisher with gui) to modify both positions in RVIZ and the ones in WeBots.

##### Darko Lukić [Moderator] 05/21/2021 09:52:31
I don't have much experience with that, but the `joint_state_publisher` doesn't sound like something designed for control. As the name suggests, it publishes joint states, so it is not supposed to control them. It is fine for RViz as RViz is designed to visualize robot's state, but not to control a physical or simulated robot.


I believe, the correct way to control joints is through the`trajectory_msgs/JointTrajectory` topic or `control_msgs::FollowJointTrajectoryAction` action. Otherwise, you can a low-level access with the `set_position` service.

##### Daemongear 05/21/2021 09:56:32
I see🤔  thanks for all the great help!

##### Stephen 05/24/2021 00:49:30
Is anyone else having issues downloading SUMO when building webots from source? Is there another mirror somewhere for it?

I am setting up a CI/CD pipeline for our controller for robocup, which requires me to build webots from source on several computers. All of them are having issues downloading SUMO for linux64. When they get to the step where they decompress the tar archive, there is an unexpected EOF character - basically the tar archive hasn't downloaded properly. I have downloaded it manually, and decompressed it myself successfully, but that's really painful when it takes an hour to download sumo at 8 KB/s

##### Simon Steinmann [Moderator] 05/28/2021 21:38:37
<@&568329906048598039> Working with object recognition, I think it could be very useful to expose the "recognitionColors" field in all objects provided by Webots. What do you think? I can create a PR if you agree.

##### Robot 05/28/2021 21:39:12
Yeah totally agree.

##### jasper [Moderator] 05/29/2021 08:51:23
I think it's a good idea as well.

##### Olivier Michel [Cyberbotics] 05/29/2021 11:19:56
Yes, good idea. Please go ahead.

##### Simon Steinmann [Moderator] 05/29/2021 23:34:26
[https://github.com/cyberbotics/webots/pull/3108](https://github.com/cyberbotics/webots/pull/3108)


created the PR and did all the changes.

##### mmoralesp 05/31/2021 17:06:26
Does anybody know how to output in ros2 the odometry info?

##### mclzc 05/31/2021 20:31:30
Hi, regarding the urdf2proto conversion:

[https://github.com/cyberbotics/urdf2webots/blob/0136988774e7dde8d7d476c1b5e584a92d8f76df/urdf2webots/parserURDF.py#L991](https://github.com/cyberbotics/urdf2webots/blob/0136988774e7dde8d7d476c1b5e584a92d8f76df/urdf2webots/parserURDF.py#L991)



You are looking for something that starts with `libgazebo_ros_imu` which would match the `GazeboRosImu` based on the `ModelPlugin` but would as well match a `libgazebo_ros_imu_sensor`, based on the `SensorPlugin`. Is that intentional? Or did you only mean to match the first one. I don't really know the difference between both.

## June


`@Darko Lukić` I'm sorry, I made a typo mistake in that question. I didn't mean `libgazebo_ros_gps_sensor` but `libgazebo_ros_imu_sensor`. I've edited it.


No problems, I'm just trying to make my way through URDF, SDF and PROTO. Thanks `@Darko Lukić`

##### mmoralesp 06/02/2021 17:26:51
Does anybody know how to get a robot/node name using a topic?

##### chinex 06/12/2021 23:57:43
I am glad to be home, please do you have a video guide on transfer of webots files to a real robot?

##### Deleted User 06/23/2021 12:49:26
Hello! Im working on one project to transfer the gazebo ros simulation to webots. It was suggested to me to use nightly build 2021b. Problem is when I switched builds from official to 2021b the ros simulation won't start, error message-> usr/local/webots/projects/default/controllers/ros/ros: error while loading shared libraries: libboost\_system.so.1.65.1: cannot open shared object file: No such file or directory

WARNING: 'ros' controller exited with status: 127.                                                                                             

Is it the compatability issue, because I use ubuntu 20.04? the nightly build of 2021a works on same stuff without issues. Thanks for help!

##### michal.mlaticek 06/26/2021 17:38:41
Hi guys I'm new to Webots. Wanted to ask... is there/does someone have a space environment?

## July

##### Affonso 07/01/2021 19:59:04
Hey, how can I use a own lib in webots?

##### Chinwei.Chang 07/19/2021 08:21:36
The VR view worked fine in version 2019a. However, after 2019a, the VR version didn't worked. How could I use VR in 2021b?


Is it possible to interactive with robot in webots through VR

##### Nummer\_42O 07/19/2021 12:03:59
since updating to R2021b I can't seem to get the robot node via supervisor.getFromDef anymore? It just spits out None
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/866651585580630086/unknown.png)
%end



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/866651712882081813/unknown.png)
%end


ah perfect - thx. That wasn't an issue before. ^^


Another thing: How do I disable those? I found and updated the texture URLs in Spots .proto file (No textures were to be found in the left and right leg) but it still shows that to me.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/866658664274591775/unknown.png)
%end


hm ok

##### Benjamin Délèze [Cyberbotics] 07/19/2021 12:40:01
Otherwise, you can update the headers of the protos to R2021b

##### Nummer\_42O 07/19/2021 12:41:16
That works better for me. Thanks :)

##### mcitir 07/24/2021 16:49:51
Hello everybody, do you know how I can find proper material to learn the usage of Webots for Automobiles? I could not find enough source to grab the usage as quickly as possible. Also, I tried to use Visual Studio for controller design. However, I could not be successful to compile from VS. Appreciated for any source or idea. Thanks.

##### TheGiant 07/24/2021 20:36:13
`@mcitir` did you checkout [https://cyberbotics.com/doc/automobile/index](https://cyberbotics.com/doc/automobile/index) ? 

Regarding VS, Have you you read thought [https://cyberbotics.com/doc/guide/using-your-ide#visual-studio](https://cyberbotics.com/doc/guide/using-your-ide#visual-studio) ?

##### mcitir 07/27/2021 10:47:08
Yes, I checked both. I could not find solution. VS did not compile the controller even I applied everything step-by-step.


Also, unfortunately, documentation for automobiles is little bit short for me. I did not use wbeots in advance. It is first time to use for me. I started directly with automobiles, so the documentation was short to learn how the simulation works.

##### DDaniel [Cyberbotics] 07/27/2021 11:34:46
`@mcitir` Hi, there's also several samples involving vehicles, you can find them in `file > open sample world > vehicles`, the world `city.wbt` for instance can help you understand how it works

##### mcitir 07/27/2021 13:35:32
`@DDaniel` Thank you very much.

## August

##### Miguel Torres 08/06/2021 17:34:40
Hello community.

I want to add a lidar sensor to a vehicle, but when I call the lidar.getPointCloud () function from my Python controller it shows this WARNING and  it stops.

Can you help me please?
%figure
![Screenshot_2021-08-06_123327.png](https://cdn.discordapp.com/attachments/565155651395780609/873257784504877066/Screenshot_2021-08-06_123327.png)
%end


this is a video



> **Attachment**: [lidar\_error.mp4](https://cdn.discordapp.com/attachments/565155651395780609/873257945226428486/lidar_error.mp4)

##### DDaniel [Cyberbotics] 08/06/2021 20:22:07
`@Miguel Torres` I can't reproduce the error, works fine for me. Can you share the entire project?

##### Miguel Torres 08/06/2021 22:18:04
`@DDaniel` thanks for your prompt response.  This is the entire project, the world in name "tutorial6\_4\_wheeled\_robot"



> **Attachment**: [WebotsProject.7z](https://cdn.discordapp.com/attachments/565155651395780609/873329135022383135/WebotsProject.7z)


I am using windows 10

##### DDaniel [Cyberbotics] 08/09/2021 18:20:15
`@Miguel Torres` it should be accessible with `.x`. In all other languages it works fine but you're right, it doesn't for python. It appears to be a regression, I'll look into it tomorrow


`@Miguel Torres` It should be fixed in tonight's nightly build, you can download it from here tonight/tomorrow: [https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)

##### Miguel Torres 08/10/2021 13:59:55
`@DDaniel` Great news, I'll download it tomorrow. 

Thanks.

##### Deleted User 08/11/2021 08:44:19
Hello Cyberbotics team i write you again. I hadn't resolved the last problem from July 14. But I have some updates about the topic. After fresh instalation of UBUNTU 20.04 i tried to install officiall release of webots 2021b. when I do it over .deb or snap I get same Problem by connecting to ros-master. Error ist about libboost I tried to install over tarball and it connects to ros master but I have a problem with this error: 



INFO: ros: Starting controller: /home/denis/webots/projects/default/controllers/ros/ros --name=asterix

[ INFO] [1628669963.199765104]: Robot's unique name is asterix.

[ INFO] [1628669963.224707050]: The controller is now connected to the ROS master.

WARNING: ros: The process crashed some time after starting successfully.

Error: wb\_lidar\_get\_layer\_range\_image() called for a disabled device! Please use: wb\_lidar\_enable().

WARNING: 'ros' controller crashed.



I tried to enable Lidar and PointCloud on my Robot. The same code on last revision worked, here not. I looked at the example  complete\_test.cpp in webots\_ros and tried to recreate it for my project. Yes I use ROS1 .What should I do? My colleagues said that I need to work with webots R2021 but I can't resolve this over one month.  Is it maybe resolved in one of the nightly builds?





I tried to install from source but I got simmilar result as over tarball but my ros instalation got rogue and tools like rviz rqt didn't want to start. I needed to install whole Ubuntu again. 



Sorry for bothering you

Thanks in advance for help

##### Darko Lukić [Moderator] 08/11/2021 08:53:46
About the other problem let me check

##### Deleted User 08/11/2021 09:08:58
Thanks.

##### Darko Lukić [Moderator] 08/11/2021 14:55:14
`@Deleted User` I believe I fixed the issue:

[https://github.com/cyberbotics/webots/pull/3561](https://github.com/cyberbotics/webots/pull/3561)



You can download it from here:

[https://github.com/cyberbotics/webots/suites/3469232657/artifacts/82481184](https://github.com/cyberbotics/webots/suites/3469232657/artifacts/82481184)



Or wait for the nightly builds (generated an evening after a merge):

[https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)

##### Deleted User 08/16/2021 08:00:26
Thanks it finally works!

##### satyar 08/18/2021 18:35:04
Hi guys 

I want create a line follower Field in webots . Can anyone help me because I have no idea about that ?

##### DDaniel [Cyberbotics] 08/18/2021 20:30:13
`@satyar` what do you mean by follower field? You mean the terrain? There are line follower examples in the samples (search in `file > open sample world`)

##### Caio Viturino 08/21/2021 15:48:40
Hello all, I am trying to implement two external controllers in two robots using python but the controller does not start.



I set up the controller as <extern> in webots. What else can I do?



The simple code I am using to test is:

```python
#!/usr/bin/python3
import sys
sys.path.append("/usr/local/webots/lib/controller")
sys.path.append("/usr/local/webots/lib/controller/python38")
from controller import Robot

import os
os.environ["WEBOTS_ROBOT_NAME"] = "Master"

robot = Robot()

while robot.step(32) != -1:
    print("Hello World!")
```


That's that I'm trying to do but does not work:

[https://cyberbotics.com/doc/guide/running-extern-robot-controllers#single-simulation-and-multiple-extern-robot-controllers](https://cyberbotics.com/doc/guide/running-extern-robot-controllers#single-simulation-and-multiple-extern-robot-controllers)

##### DrVoodoo [Moderator] 08/21/2021 17:55:05
What exactly doesn't work? What error or debug message (if any) are you given? Is the simulation definitely running or is it paused?

##### Caio Viturino 08/21/2021 18:26:17
The simulation does not start and does not give any message


Is it possible to have 2 robots with <extern> controllers set?

##### DrVoodoo [Moderator] 08/21/2021 18:29:10
Well let's start from the beginning. Does it work if you have a single robot and a single extern controller?

##### Caio Viturino 08/21/2021 18:30:53
Yes

##### DrVoodoo [Moderator] 08/21/2021 18:33:12
Does it work if you have two robots, one extern and one 'normal'?

##### Caio Viturino 08/21/2021 18:45:11
tried to set one as a void but does not work as well

##### DrVoodoo [Moderator] 08/21/2021 18:58:59
So what are the names of your two robots?

##### Caio Viturino 08/21/2021 19:00:04
"Master" and "bar\_clamp"


I set up the robot name using

```python
os.environ["WEBOTS_ROBOT_NAME"] = "Master"
```

##### DrVoodoo [Moderator] 08/21/2021 19:02:41
Ok, what happens of you get rid of that and set the env variable using the export command before you run your extern controllers?


And you are running both controllers?

##### Caio Viturino 08/21/2021 19:16:04
Just forgot about it 😅


Thank you

##### DrVoodoo [Moderator] 08/21/2021 19:16:34
Np

##### Simon Steinmann [Moderator] 08/24/2021 19:45:32
`@satyar` what do you mean by "line filed". Do you want a different floor / room with a different looking line?

##### satyar 08/26/2021 14:33:22
I have another question,

How can I import 3D model with color to webots ? 

I export my file with color from Shapr3D with .obj but when I import in webots, there is no color and Everything is gray ☹️

##### Simon Steinmann [Moderator] 08/27/2021 00:09:01
.obj files do not contain color information. They can have linked materials or textures, but I think you have to import them seperately

## September

##### Millimeter 09/01/2021 07:34:52
Hello, I have built a dancing nao project and would like to know how I could go about contributing it to `cyberotics/community-projects`?


Hi `@Stefania Pedrazzi`! I got your email :> Hence me coming here! Thank you for the info so I just create a softbank folder inside robots and add my code there or should I add it under samples?

##### emabrey 09/06/2021 21:44:51
I wonder if it is me, or Webots homesite,  [https://cyberbotics.com](https://cyberbotics.com), often was not responsive in the past few days.

##### Fero 09/07/2021 17:47:08
Hey!! I am building my own robot but I am not familiarized with PROTO files, if anyone want to do it for me I can pay 15 usd! Thank you for reading this! 😋

##### Çağrı Kaymak 09/13/2021 12:28:08
Hi, any chance I can install webots-R2021b on Ubuntu 16.04?

##### thegodsmile 09/22/2021 18:22:37
hello all, when running this code in C i get no errors, but nothing displays on the console.



%figure
![C.png](https://cdn.discordapp.com/attachments/565155651395780609/890302975170461756/C.png)
%end

##### DDaniel [Cyberbotics] 09/22/2021 18:46:51
`@thegodsmile` are you calling `wb_robot_step` at some point after this snippet?

##### thegodsmile 09/22/2021 18:50:53
yes, this is within a larger feedback loop` while (wb_robot_step(TIME_STEP) != -1) `

##### DDaniel [Cyberbotics] 09/22/2021 18:55:17
Oh, it's just that it's `\n` not `/n`

##### thegodsmile 09/22/2021 18:59:56
ahh man. thanks Daniel. Genius. Subtle errors like that tend to get overlooked when you been coding all day. it's displaying now

## October

##### mironix<inactive> 10/12/2021 19:49:40
Hello! I would like to ask how the Python SDK is generated - would like to add type hints to - i think - generated by CI sdk


thank you `@Olivier Michel`


found it [https://github.com/cyberbotics/webots/blob/7f78b89e1f583fd87af5b1111f842e2eba6f8b1e/src/controller/python/Makefile](https://github.com/cyberbotics/webots/blob/7f78b89e1f583fd87af5b1111f842e2eba6f8b1e/src/controller/python/Makefile)

##### Justin Fisher [Moderator] 10/13/2021 09:47:11
It looks like there was some discussion of doing what `@mironix<inactive>` suggests last year in [https://github.com/cyberbotics/webots/issues/2385](https://github.com/cyberbotics/webots/issues/2385) to which I just added some ideas.

##### mironix<inactive> 10/13/2021 10:39:12
thanks `@Justin Fisher` i'll check this out

##### Meghana|GDSC 10/18/2021 02:44:57
Hi


Can we develop a mini game in webots which uses our hand gestures to control the game

##### rozgo 10/18/2021 19:47:55
You could use something like this: [https://handtracking.io/](https://handtracking.io/)


With some minimal websockets server in the controller.

## November

##### Vinay 11/09/2021 15:43:01
Can anyone help me in writing the python controller

##### mahesh 11/09/2021 15:52:06
[https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers](https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers)


Take a look at this, this is a well written documentation and let us know if you are still not able to figure things out

##### its-me 11/09/2021 16:50:25
Can I hire someone to help me get started with a project? I need help getting my robot and a chess board added so I can start writing the controller


I think I need the 3d files added and the physics done?

##### Simon Steinmann [Moderator] 11/09/2021 16:52:52
What robot? A custom one or a commercial product?

##### its-me 11/09/2021 16:53:25
Commercial, but is ancient, I posted the docs a last week


I think it was under technical questions

##### Simon Steinmann [Moderator] 11/09/2021 16:54:49
Do you have 3d models and an urdf file?

##### its-me 11/09/2021 16:55:20
I can generate step files


What is urdf

##### Simon Steinmann [Moderator] 11/09/2021 16:56:44
Google it, urdf is the most common and established way to define a robot in terms of appearance, physics and functionality


And it is fairly easy to understand and create

##### its-me 11/09/2021 16:57:52
I can work on that and try to figure it out

##### Simon Steinmann [Moderator] 11/09/2021 16:59:39
Many cad programs also allow you to export urdf

##### its-me 11/09/2021 17:00:28
I have fusion360, can look into that

##### DrakerDG [Moderator] 11/09/2021 18:45:30
[http://wiki.ros.org/urdf](http://wiki.ros.org/urdf)

##### Caio Viturino 11/14/2021 19:06:59
Guys, I made a qualitative comparison between the real UR5 and simulated UR5 in Webots:

Video: [https://www.youtube.com/watch?v=6OxeKOUKwGM](https://www.youtube.com/watch?v=6OxeKOUKwGM)

##### Simon Steinmann [Moderator] 11/14/2021 19:08:46
Awesome work!

##### BerkcanBarut 11/26/2021 13:48:01
Hi guys. Velodyne VLP-16, I am trying to do autonomous driving using this lidar. How do I filter data from lidar? How to visualize lidar data over point cloud? Can you help me ?

## December

##### tenglvjun 12/01/2021 12:37:02
Hi guys. I create a simply webots  controller for my baby two-wheels car. I add ROS to my project. I set  INCLUDE = -I"/opt/ros/noetic/include" and LIBRARIES = -L"/opt/ros/noetic/lib" -lroscpp -lroslib -lrosconsole -lrostime -lroscpp\_serialization -ltf\_conversions -ltf to the MakeFile. All of the path is correctly. It was complied correctly in my vscode editor, but failed in webots. The error is fatal error: ros/ros.h: No such file or directory.  

Did I miss something?

##### JoeDerybshire 12/01/2021 13:26:28
Hiya everyone. I have to make the the abb irp robot arm draw the letter "n"

is there any like tutorial that could explain the general idea of inverse kinematics in ikpy?


I didn't find anything in the documentation.

##### Simon Steinmann [Moderator] 12/01/2021 19:15:27
what is the exact robot arm you are trying to use?


[https://github.com/Simon-Steinmann/webots\_pyikfast\_tutorial](https://github.com/Simon-Steinmann/webots_pyikfast_tutorial)


this is the sample, using the irb4600

##### JoeDerybshire 12/02/2021 16:10:21
I'll check those out, thank you both!

