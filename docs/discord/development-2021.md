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

##### Darko Lukić [Cyberbotics] 01/07/2021 08:00:15
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

##### Darko Lukić [Cyberbotics] 01/07/2021 10:04:06
RViz2 is a bit buggy, just uncheck/check the RobotModel checkbox and it should work

##### Simon Steinmann [Moderator] 01/07/2021 10:04:29
you're right


is there a quick and easy way to include the world frame in the tf broadcast?

##### Darko Lukić [Cyberbotics] 01/07/2021 10:06:26
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

##### Darko Lukić [Cyberbotics] 01/07/2021 10:07:48
Than you have to publish the transforms manually :/

##### Simon Steinmann [Moderator] 01/07/2021 10:09:00
does that work alongside the robot\_state\_publisher?

##### Darko Lukić [Cyberbotics] 01/07/2021 10:10:07
Sure

##### Simon Steinmann [Moderator] 01/07/2021 10:10:37
maybe that should be a feature of the webots\_ros2\_core

##### Darko Lukić [Cyberbotics] 01/07/2021 10:11:42
We do it here for example:

[https://github.com/cyberbotics/webots\_ros2/blob/7aa84877e0dc5d668e923fd2f23ebd4acb865d90/webots\_ros2\_examples/webots\_ros2\_examples/khepera\_driver.py#L92-L105](https://github.com/cyberbotics/webots_ros2/blob/7aa84877e0dc5d668e923fd2f23ebd4acb865d90/webots_ros2_examples/webots_ros2_examples/khepera_driver.py#L92-L105)


The robot state publisher is in the core, what would you like to add in addition to it?

##### Simon Steinmann [Moderator] 01/07/2021 10:13:18
have an option (or by default) to pulbish the world - baselink transform


static robots are one thing, but especially for mobile ones, it is kinda essential.

##### Darko Lukić [Cyberbotics] 01/07/2021 10:14:52
But that is something that a user has to handle

##### Simon Steinmann [Moderator] 01/07/2021 10:15:37
Gazebo publishes it by default, people are used to simply selecting "world" in rviz

##### Darko Lukić [Cyberbotics] 01/07/2021 10:17:16
Gazebo publishes transform between `world` and `base_link` for mobile robots? That sounds like cheating


In the real-world transform between `world` and `base_link` is not available

##### Simon Steinmann [Moderator] 01/07/2021 10:18:02
maybe it is just static. Has been a while, dont remember

##### Darko Lukić [Cyberbotics] 01/07/2021 10:18:31
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

##### Ragemor 01/13/2021 08:06:20
[https://cyberbotics.com/doc/guide/using-your-ide](https://cyberbotics.com/doc/guide/using-your-ide)

##### Moha 01/13/2021 08:07:09
👍

##### Simon Steinmann [Moderator] 01/13/2021 08:44:39
I'm curious, wouldn't it be possible to set system environment variables during installation?


`@Darko Lukić` `@Olivier Michel` I've got moveit2 working with ros2 and webots. I also made some progress of creating an easy to use template or automatic setup for robotic arms


I think one important feature would be, to automatically extract the urdf from webots, including meshes.


I already did some work in that direction with my proto-splitter script. That one worked, extracting .obj or vrml meshes, but without smoothing

##### Olivier Michel [Cyberbotics] 01/13/2021 09:42:21
Probably yes on Windows, probably not possible on Mac and Linux...

##### Simon Steinmann [Moderator] 01/13/2021 09:44:28
ahh okay


Also I was wondering: could it be possible to launch a ROS1 or ROS2 launch file from within webots?

##### Olivier Michel [Cyberbotics] 01/13/2021 11:23:51
I am not sure this is a good idea as we want to be able to interact with ROS in the console...

##### Darko Lukić [Cyberbotics] 01/13/2021 15:44:30
It may be possible to implement events in the launch file (like here [https://github.com/cyberbotics/webots\_ros2/blob/38d88e01fe174a8a00731f554f1a8646b9127bd2/webots\_ros2\_demos/launch/armed\_robots.launch.py#L63](https://github.com/cyberbotics/webots_ros2/blob/38d88e01fe174a8a00731f554f1a8646b9127bd2/webots_ros2_demos/launch/armed_robots.launch.py#L63)), so if reset controllers in Webots it supposed to restart certain ROS2 nodes as well within the launch file. This would be useful feature and it would speed up development significantly (as you don't need to reload the world).

##### babaev1 01/16/2021 20:00:53
Dear Sirs, recently Organizing Commitee of Humanoid League of Robocup Federation decided to host Summer World Championship in virtual format with organizing games between teams within simulator. Currently simulator platforms are under consideration for choosing. There are going to be games between 2 teams , with each team providing 4 humanoid robots  equipped by 23-25DoF servomotors, 1-2 imus, 1-2 cameras. Imortant point is streaming of images from cameras to external controller softwares of teams. Currently built in webots camera streams images in non-compressed format. Supposing simultaneously streaming images from 8 cameras with resolution 640x480 RGB  with 30 fps simulation is expected to be dramatically slowing down.  Some of teams are expected to use stereo cameras, means number of cameras in this case will be doubled.


Do you plan to add compression to image streaming in near future, or there are some implementations which I am not familiar with?

##### cnbarcelo 01/17/2021 15:43:05
Hi! Where can I find the sensors visualization code? I'm mainly looking for the the lidar visualization.

##### Darko Lukić [Cyberbotics] 01/17/2021 17:59:21
`@cnbarcelo` You can use `View / Optional Rendering / Show Lidar Point Cloud` and `View / Optional Rendering / Show Lidar Rays Paths`:

[https://cyberbotics.com/doc/guide/the-user-interface](https://cyberbotics.com/doc/guide/the-user-interface)



Remember, to show the point cloud, you need to enable the point cloud first. You can do it from the robot window (double click on the robot) or from your controller:

[https://cyberbotics.com/doc/reference/lidar#wb\_lidar\_enable\_point\_cloud](https://cyberbotics.com/doc/reference/lidar#wb_lidar_enable_point_cloud)

##### cnbarcelo 01/17/2021 18:03:15
Yeah, I do know that, but I migrated from 2020 to 2021 and it's not working well. I wanted to check if something changed before creating a ticket.


If there's any bug to work on, I might be able to collaborate.

##### Darko Lukić [Cyberbotics] 01/17/2021 18:06:12
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

##### Darko Lukić [Cyberbotics] 01/17/2021 18:10:38
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

##### Darko Lukić [Cyberbotics] 01/17/2021 19:23:47
Thank you, we will check it out


It may be related to this PR:

[https://github.com/cyberbotics/webots/pull/2509](https://github.com/cyberbotics/webots/pull/2509)



It is possible that the visualization part fails to handle infinity values.

##### Chernayaten 01/17/2021 20:12:04
After a few tests it seems you are correct. The same issue presents when the robot gets close to objects because " If the range value is smaller than the minRange value then infinity is returned."

##### Olivier Michel [Cyberbotics] 01/18/2021 07:58:52
In Webots images are read from the GPU memory directly into a shared memory segment which is readable by both Webots and the local robot controller to optimize efficiency and avoid unnecessary image copy. In the context of RoboCup (and other frameworks, like ROS or ROS2), the local robot controller acts as a relay between the simulator and the actual robot controller which may run on a different machine over the network. Therefore, while implementing this relay controller, it is very easy to add JPEG compression to the camera image to be sent to the actual controller over the network. This is indeed exactly what happens when you use a ROS 2 controller with Webots to publish a camera image, it gets automatically compressed in JPEG (and you can even choose the compression ratio) before being sent to the controller.

##### babaev1 01/18/2021 09:23:07
Thank you for prompt reply. For me it is clear. I shall share your answer with Robocup community for possible definitions.

##### Moha 01/19/2021 09:49:08
hi guys

Is it possible to use spot robot SDK releasing from Boston dynamics in webots ?

##### Darko Lukić [Cyberbotics] 01/19/2021 10:01:18
Hello `@Moha`, Webots doesn't provide support for Spot SDK. It is not in the scope of Webots, but I advise you to create a discussion:

[https://github.com/cyberbotics/webots/discussions/categories/ideas](https://github.com/cyberbotics/webots/discussions/categories/ideas)

If there is a strong need (many requests), somebody from the community may create the integration.

##### Moha 01/19/2021 10:06:20
hi dear Darko

good idea, ok

but may I ask you how is it possible to request for it ?

##### Darko Lukić [Cyberbotics] 01/19/2021 10:08:39
You can create a discussion:

[https://github.com/cyberbotics/webots/discussions/new](https://github.com/cyberbotics/webots/discussions/new)



Explain why the integration is necessary, how big impact it will have on the community...

##### Moha 01/19/2021 10:14:43
I've understood what you mean in first reply, but I mean I want to know how other people can show their enthusiasm for it ?

##### Darko Lukić [Cyberbotics] 01/19/2021 10:15:56
They can join the discussion, like it, suggest new ideas

##### Moha 01/19/2021 10:52:14
ok thank you 👍

##### pk1jk1 01/20/2021 05:59:29
What is the best way to locally edit a webots package?

##### Darko Lukić [Cyberbotics] 01/20/2021 07:30:40
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

