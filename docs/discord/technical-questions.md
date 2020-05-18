# Technical-Questions

This is an archive of the `technical-questions` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m).

## 2020

##### taeyoung 05/18/2020 11:29:25
It doesn't work...

##### Rodeknopje 05/18/2020 08:51:58
`@David Mansolino` Thanks

##### David Mansolino [cyberbotics] 05/18/2020 08:51:10
> Hi, I'm testing the use of sliderjoint in webots with differnet values for the timesteps (basictimestep and wb_robot_step).

> In my simulation the speed is send to the robot controller every 50ms.  I set my basictimestep to 10ms and wb_robot_step to 10ms. It works fine.

> But when I test my simulation with basictimestep 10ms and wb_robot_step 50ms, the slider moves outside(less than minstop), the range i set for it [0,0.15] ([minstop,maxstop])

`@taeyoung` have you tried using the min/maxPosition instead?

##### David Mansolino [cyberbotics] 05/18/2020 08:50:42
`@Rodeknopje` in that case you should use the `Robot.getInertialUnit` method ([https://cyberbotics.com/doc/reference/robot?tab-language=python#wb\_robot\_get\_device).](https://cyberbotics.com/doc/reference/robot?tab-language=python#wb_robot_get_device).)

##### David Mansolino [cyberbotics] 05/18/2020 08:49:53
> `@David Mansolino`  How should I determine the wait between movements?Are there sample gait codes?

`@elkelkmuh` I am sorry, but I am not an expert in gait generation, but I am sure you will fidn a lot of literrature on the subject on internet.

##### Rodeknopje 05/18/2020 08:49:34
im using python

##### David Mansolino [cyberbotics] 05/18/2020 08:49:17
`@brianne.byer` you're welcome!

##### David Mansolino [cyberbotics] 05/18/2020 08:49:05
> which method do i have to call to get acces to my inertial unit?

`@Rodeknopje` hi, which language are you using? In any case, it works exactly like for the other sensors, it is based on the sensor name. See for example this tutorial where distance sensors are used: [https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers#program-a-controller](https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers#program-a-controller)

##### Rodeknopje 05/18/2020 08:47:19
which method do i have to call to get acces to my inertial unit?

##### brianne.byer 05/18/2020 08:36:53
`@David Mansolino` thank you so much!

##### elkelkmuh 05/18/2020 07:39:13
`@David Mansolino`  How should I determine the wait between movements?Are there sample gait codes?

##### taeyoung 05/18/2020 07:06:57
Hi, I'm testing the use of sliderjoint in webots with differnet values for the timesteps (basictimestep and wb_robot_step).
In my simulation the speed is send to the robot controller every 50ms.  I set my basictimestep to 10ms and wb_robot_step to 10ms. It works fine.
But when I test my simulation with basictimestep 10ms and wb_robot_step 50ms, the slider moves outside(less than minstop), the range i set for it [0,0.15]([minstop,maxstop])

##### David Mansolino [cyberbotics] 05/18/2020 06:58:48
Yes you should controler the motor directly like this indeed.

##### elkelkmuh 05/18/2020 06:58:17
`@David Mansolino`  Which commands should I use?
For example: self.getMotor('LegUpperL').setPosition(1.0) 
            self.getMotor('LegLowerL').setPosition(-1.0)
            self.getMotor('AnkleL').setPosition(-0.4)

##### w3. eydi 05/18/2020 06:53:04
As I understand it, there is a stable version in the only repository. I will download it manually and install it again. I will give you feedback if I get an error. If the problem is not resolved, I will continue to use it on Windows, thank you `@David Mansolino` .

##### David Mansolino [cyberbotics] 05/18/2020 06:43:31
> Hi `@David Mansolino` I want to do stair climbing work with Op2 with python. When I use the    gaitManager.setYAmplitude  command, the foot does not rise as usual.  The robot is falling.  What's your advice?

`@elkelkmuh` the gait algorithm of the darwin-op is not maid to climb stairs, unfortunately you will have to write your own gait algorithm if you want to climb stairs.

##### w3. eydi 05/18/2020 06:42:31
`@David Mansolino`  I received this error message when I used it with optirun.
%figure
![DeepinEkranGoruntusu_20200516234439.png](https://cdn.discordapp.com/attachments/565154703139405824/711831066775912455/DeepinEkranGoruntusu_20200516234439.png)
%end


##### David Mansolino [cyberbotics] 05/18/2020 06:42:28
HI `@w3. eydi`, may I ask you to try with a nightly version of R2020a-revision2 available here, the unsuported image bug should be fixed with this version (let us know if it is not the case): [https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)

##### elkelkmuh 05/18/2020 06:41:41
Hi `@David Mansolino` I want to do stair climbing work with Op2 with python. When I use the    gaitManager.setYAmplitude  command, the foot does not rise as usual.  The robot is falling.  What's your advice?

##### w3. eydi 05/18/2020 06:41:03
and error message but simulation is working !
%figure
![DeepinEkranGoruntusu_20200516234207.png](https://cdn.discordapp.com/attachments/565154703139405824/711830697127575612/DeepinEkranGoruntusu_20200516234207.png)
%end


##### w3. eydi 05/18/2020 06:40:19
Good morning `@David Mansolino`;

I was going to create an issue in Github, but since I need to upload pictures, I am writing here.

I take error : Cannot load texture and Unsupported image format.

I switched to Linux. Authority problem ?

I'm using Linux Debian9 - Pardus 17.5

##### David Mansolino [cyberbotics] 05/18/2020 06:30:55
`@dralorg` this might be due to the fact that the center of mass of the robot is not centered but rather to the front.

##### David Mansolino [cyberbotics] 05/18/2020 06:17:07
> Does any of the example use pure pursuit path follow algorithm by any chance?

`@Lussfer` here are some path plannign example in Webots: [https://en.wikibooks.org/wiki/Cyberbotics%27\_Robot\_Curriculum/Advanced\_Programming\_Exercises#Path\_planning\_[Advanced]](https://en.wikibooks.org/wiki/Cyberbotics%27_Robot_Curriculum/Advanced_Programming_Exercises#Path_planning_[Advanced])
Unfortunately none of them is using pure pursuit.

##### Stefania Pedrazzi [cyberbotics] 05/18/2020 06:14:29
> Hello, I'm working on a room-mapping robot for my final year project and was wondering if it is possible to get the grid that the robot is currently on? I'm trying to do a very basic 2d array as a topological map?

`@Conor` To get the position  of the robot you can use the GPS device or the Supervisor API. Both methods will give you the global 3D position, to map it to a XY grid map (or XZ using the default Webots coordinate system) you can simply discard the Y value.  I suggest you to check this sample simulation for an example about getting the robot position: [https://www.cyberbotics.com/doc/guide/samples-devices#gps-wbt](https://www.cyberbotics.com/doc/guide/samples-devices#gps-wbt)

##### David Mansolino [cyberbotics] 05/18/2020 06:10:26
> Hi, we are trying to simulate a simple robot in Webot with two rear driving wheels. To make it simpler among our interdisciplinary team we decided to choose python as the controller language. We are stuck with simple logic like PWM implementation, most of the sample code are written in C/C++ , shall we change it to C or there is some supporting database out there that we have not looked for. We are struggling with this sort of issue from a couple of weeks. Any suggestions would be highly appreciated. Thanks

`@Jatin Sharma` hi, in webots you don't need to implement PWM, you can directly control the motors in speed or position, I would recommend to follow our tutorials (available in Python) to get familiar with controller programming in Webots: [https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots?tab-language=python](https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots?tab-language=python)

##### David Mansolino [cyberbotics] 05/18/2020 06:08:53
> On webots i see that the object recognition is giving. X as Z, Y as Y and Z as -X. on webots does rotation match the orientations..

`@Mathie20` Yes the orientations match the 'rotation' field.
Abotu the axes inversion, I can't unfortunately reproduce this in the 'camera_recognition' sample, do you have a specific case to reproduce this? Note that the position is expressed in the camera frame and not i nthe robot or world frame.

##### Jatin Sharma 05/18/2020 06:03:41
Hi, we are trying to simulate a simple robot in Webot with two rear driving wheels. To make it simpler among our interdisciplinary team we decided to choose python as the controller language. We are stuck with simple logic like PWM implementation, most of the sample code are written in C/C++ , shall we change it to C or there is some supporting database out there that we have not looked for. We are struggling with this sort of issue from a couple of weeks. Any suggestions would be highly appreciated. Thanks

##### David Mansolino [cyberbotics] 05/18/2020 05:41:58
> Is there code available online that directly controls the wheels of the youbot? I mean, instead of what the webot demo uses (keyboard control)? The idea is for the youbot to follow a black, straight line. I'm also curious if I am required to add 4 hingejoints to the body slot of youbot. Thanks in advance, it is greatly appreciated as I am very new to coding.

`@brianne.byer` sure, here is an example: [https://github.com/cyberbotics/webots/blob/master/projects/robots/kuka/youbot/libraries/youbot\_control/src/base.c](https://github.com/cyberbotics/webots/blob/master/projects/robots/kuka/youbot/libraries/youbot_control/src/base.c)
About the HingeJoint, you can indeed add them directly in the body slot as you would do with a regular chidren field (if not already done I would suggest to read this tutorial: [https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot))

##### David Mansolino [cyberbotics] 05/18/2020 05:39:28
> hi there. if i use the supervisor to track the position of a robot. is it possible to send the position information to another controller?

`@AyresAlmada` yes of course, you can use an emitter to send it from your supervisor and a receiver node to receive it in your other controller.

##### dralorg 05/18/2020 00:31:15
Hi, I'm running the Youbots back wheels but the robot isn't moving. (The top wheels do make the robot move).

I've added the "contactProperties" inside the "WorldInfo" tag in the world file, this is what I've added.

```
  contactProperties [
    ContactProperties {
      material1 "InteriorWheelMat"
      coulombFriction [
        1.8, 0, 0.2
      ]
      frictionRotation -0.9648 0
      bounce 0
      forceDependentSlip [
        10, 0
      ]
    }
    ContactProperties {
      material1 "ExteriorWheelMat"
      coulombFriction [
        1.8, 0, 0.2
      ]
      frictionRotation 0.9648 0
      bounce 0
      forceDependentSlip [
        10, 0
      ]
    }
  ]
```

Why the bottom wheels are not making the robot move?
> **Attachment**: [first\_world.mp4](https://cdn.discordapp.com/attachments/565154703139405824/711737634514272369/first_world.mp4)


##### Conor 05/18/2020 00:01:13
`@AyresAlmada` I know you can use GPS but I'm not entirely sure how I would translate the GPS into an X,Y position that can then go into a 2d array

##### Conor 05/18/2020 00:00:35
> hi there. if i use the supervisor to track the position of a robot. is it possible to send the position information to another controller?

`@AyresAlmada` Did you ever find if this was possible? At least getting the robots position?

##### Conor 05/17/2020 23:52:53
Hello, I'm working on a room-mapping robot for my final year project and was wondering if it is possible to get the grid that the robot is currently on? I'm trying to do a very basic 2d array as a topological map?

##### Lussfer 05/17/2020 09:38:58
Does any of the example use pure pursuit path follow algorithm by any chance?

##### Mathie20 05/16/2020 13:19:21
On webots i see that the object recognition is giving. X as Z, Y as Y and Z as -X. on webots does rotation match the orientations..
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/711206156856590387/unknown.png)
%end


##### brianne.byer 05/16/2020 12:40:16
Is there code available online that directly controls the wheels of the youbot? I mean, instead of what the webot demo uses (keyboard control)? The idea is for the youbot to follow a black, straight line. I'm also curious if I am required to add 4 hingejoints to the body slot of youbot. Thanks in advance, it is greatly appreciated as I am very new to coding.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/711196323235233802/unknown.png)
%end


##### AyresAlmada 05/16/2020 06:10:34
hi there. if i use the supervisor to track the position of a robot. is it possible to send the position information to another controller?

##### David Mansolino [cyberbotics] 05/15/2020 08:17:35
See this question for example: [https://stackoverflow.com/questions/59263637/webots-write-controller-data-to-external-csv-file](https://stackoverflow.com/questions/59263637/webots-write-controller-data-to-external-csv-file)

##### elkelkmuh 05/15/2020 08:12:32
`@David Mansolino` how can I save them to a file?

##### David Mansolino [cyberbotics] 05/15/2020 07:21:10
The refresh rate of the default robot windows is not configurable, but you can get the data from your controller at the rate you want and save them to a file

##### elkelkmuh 05/15/2020 07:09:20
Hi `@David Mansolino` Can I get the engine information in the data for 0.1 seconds as data?
%figure
![Screenshot_20200515_100629_com.whatsapp.jpg](https://cdn.discordapp.com/attachments/565154703139405824/710750650992164934/Screenshot_20200515_100629_com.whatsapp.jpg)
%end


##### Troy 05/15/2020 04:55:23
I'll try the urdf2webots program again tomorrow

##### Troy 05/15/2020 04:54:59
but the URDF is correct, I imported this urdf into V-Rep, and it's correct

##### Troy 05/15/2020 04:54:08
I'm not sure if I could share this model

##### David Mansolino [cyberbotics] 05/15/2020 04:53:04
Hi `@Troy` it seems there is an issue with the generated PROTO, can you share it with us so that we can check what is exactly the problem?

##### Troy 05/14/2020 15:46:43
what should I do with the proto file or the URDF file?

##### Troy 05/14/2020 15:45:55
Hi! I have a problem. When I used the proto to import the robot, one of my model is correct, but when I import the other one using the same method, webots reminds me that " ERROR: 'D:/Lab/Simulation/5_13TestURDF/protos/MCrab.proto':19:39: error: Expected node or PROTO name, found '{'. " I both used the urdf2webots program to output the proto file, and put them in two separate wbt file. I don't what wrong...

##### David Mansolino [cyberbotics] 05/14/2020 12:32:23
`@infinity` [https://www.youtube.com/watch?v=L0FVsFD2rS4&feature=youtu.be](https://www.youtube.com/watch?v=L0FVsFD2rS4&feature=youtu.be)

##### infinity 05/14/2020 12:15:56
Anybody have any solution

##### infinity 05/14/2020 12:15:44
I wanna import my track drive robot which is developed in solid works

##### infinity 05/14/2020 12:14:41
I have a problem in webots

##### infinity 05/14/2020 12:14:11
Hlo

##### nav3549 05/14/2020 11:57:18
sorry, i got a mistake i solve the problem !! thanks

##### David Mansolino [cyberbotics] 05/14/2020 11:41:58
Have you tried the safe mode ([https://cyberbotics.com/doc/guide/starting-webots#on-windows](https://cyberbotics.com/doc/guide/starting-webots#on-windows) )?
Or to open another world file?

##### nav3549 05/14/2020 11:29:15
I tried to delete these preferences to solve the problem, but still persists. And the preferences variables are set again after i open webots

##### nav3549 05/14/2020 11:28:44
my webots clossing after loading world model
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/710453543940784158/unknown.png)
%end


##### AyresAlmada 05/14/2020 10:43:13
got it! thanks

##### David Mansolino [cyberbotics] 05/14/2020 10:31:48
You can just call the `robot.step` function with as argument the time in milisecond you want to wait.

##### AyresAlmada 05/14/2020 10:26:43
that's right, i just want it steer more. i wonder whether there are any codes have the similar function like 'delay' in c

##### David Mansolino [cyberbotics] 05/14/2020 10:12:11
Then can't you just steer more?
This is a control question, not really specific to Webots by the way.

##### AyresAlmada 05/14/2020 10:06:48
yes, i already control it in speed. i just want the rover to avoid the obstacles, but it can only steer at the moment it see the obstacles. it has to keep steering for a while

##### David Mansolino [cyberbotics] 05/14/2020 10:01:19
Yes, you simply have to control the motor in speed instead of position: [https://cyberbotics.com/doc/reference/motor#velocity-control](https://cyberbotics.com/doc/reference/motor#velocity-control)

##### AyresAlmada 05/14/2020 09:56:30
hi there. are there any codes can make the wheels on the rover keep moving at a stable velocity?

##### David Mansolino [cyberbotics] 05/14/2020 09:33:48
> What average time does it takes to create an UAV which can be manually controlled? Has anyone build it?

`@Rajesh Roy` it all depends on your UAV, but you can probably take inspiration from an UAV already existing in Webots, such as this one: [https://cyberbotics.com/doc/guide/mavic-2-pro](https://cyberbotics.com/doc/guide/mavic-2-pro)

##### Rajesh Roy 05/14/2020 08:16:25
What average time does it takes to create an UAV which can be manually controlled? Has anyone build it?

##### David Mansolino [cyberbotics] 05/14/2020 07:20:18
You're welcome

##### YCSU 05/14/2020 07:17:06
Great! Thank you so much for the information

##### David Mansolino [cyberbotics] 05/14/2020 07:15:49
Hi, the horizontal field of view is 1.0 radians, you can read this in the PROTO file: [https://github.com/cyberbotics/webots/blob/master/projects/devices/microsoft/protos/Kinect.proto#L513-L520](https://github.com/cyberbotics/webots/blob/master/projects/devices/microsoft/protos/Kinect.proto#L513-L520)

##### YCSU 05/14/2020 07:11:41
Hi,  I am using the Microsoft Kinect sensor model in Webots. Since I would like to calculate the 3D coordinates (x ,y, z) in meters from the  depth image, I need to know the intrinsic parameter of the camera or at least the FoV angle. I searched the webots document and could not find the information.  I wonder where can I find such information?

##### Troy 05/13/2020 23:21:13
I tried again, it suddenly worked. Sorry to bother you guys.

##### Troy 05/13/2020 23:12:31
when i changed the someRobot.urdf to the path of my file, it reminds me that "math_utils.py:40: RuntimeWarning: invalid value encountered in true_divide "

##### Troy 05/13/2020 23:08:42
Could you please help on this problem? How can I use this?

##### Troy 05/13/2020 23:08:18
`@Olivier Michel` Hi Olivier! When I use the urdf2webots program on github, it reminds me that "Can't determine package root path."

##### Olivier Michel [cyberbotics] 05/13/2020 20:53:36
Not to my knowledge.

##### Luftwaffel 05/13/2020 18:31:27
Is there any downside of running the webots controller in python 2.7 instead of python3?

##### Olivier Michel [cyberbotics] 05/13/2020 15:59:34
[https://pages.github.com/](https://pages.github.com/)

##### Olivier Michel [cyberbotics] 05/13/2020 15:58:59
Doing that on github pages is an option indeed.

##### Olivier Michel [cyberbotics] 05/13/2020 15:58:39
You simply need to publish the page anywhere on the web.

##### Rajesh Roy 05/13/2020 15:55:21
Alright, and how to do that?
Is there any way of doing it using GitHub?

##### Olivier Michel [cyberbotics] 05/13/2020 15:42:46
The tricks for Chrome and Firefox are only if you want to open the webgl page locally (with a `file://` URL). When you publish it on the web, e.g., with a `http(s)://` URL, you don't need these tricks.

##### Rajesh Roy 05/13/2020 15:41:25
Has anyone gone through the procedure of exporting your Webots file to a webgl file?

##### Rajesh Roy 05/13/2020 15:41:03
I want to know that isn't there any other option available? Since this part "--allow-file-access-from-files: turns your browser very vulnerable to attacks.

Also, do you have any idea of how to create a javascript block of code handling this to turn on and turn off automatically whenever calling the .html webpage?

##### Rajesh Roy 05/13/2020 15:40:49
I want to create a webgl based simulation which could just run like any other webgl application - "No add-ons, just the browser"
After referring to the materials provided on the documentation section, I found a paragraph saying

"
Chrome: launch with the --allow-file-access-from-files option
Firefox:
Open Firefox browser and in the address bar type about:config,
hit Enter button and click on I'll be careful, I promise!.
Search for privacy.file_unique_origin or security.fileuri.strict_origin_policy and double click on it to change the status from true to false.
"

Link: [https://cyberbotics.com/doc/guide/web-scene](https://cyberbotics.com/doc/guide/web-scene)

##### David Mansolino [cyberbotics] 05/13/2020 15:18:48
> `@David Mansolino` I already submited a change to the supervisor.md, which includes this code and an explanation on how to get positions of nodes relative to other nodes. Would be nice if that could get added soon to the build 🙂

`@Luftwaffel` I am sorry but I probably missed your submission, did you open a Github Pull-Request ?

##### Lussfer 05/13/2020 15:17:59
thx David & Luftwaffel

##### Luftwaffel 05/13/2020 15:17:25
I already needed to go back to reference it, luckily I still had the pastebinlink

##### Luftwaffel 05/13/2020 15:16:59
`@David Mansolino` I already submited a change to the supervisor.md, which includes this code and an explanation on how to get positions of nodes relative to other nodes. Would be nice if that could get added soon to the build 🙂

##### lojik 05/13/2020 14:53:59
> `@lojik` yes the basic time step may in rare case influence the calibration of the friction, but even with a smaller time step you should be able to find some parameters that works for your case.

`@David Mansolino` Thank you, so I will play with this example to try to find good coefficients.

##### Luftwaffel 05/13/2020 14:52:44
`@Lussfer` Check out this out [https://pastebin.com/k7kf4Ez5](https://pastebin.com/k7kf4Ez5)

##### David Mansolino [cyberbotics] 05/13/2020 14:51:21
> how to convert wgs84 coordinate to local coordinate without changing coordinate system in world info? like i want to pass since wgs84 coordinate and convert it into local coordinate

`@Lussfer` In that case you have to convert them in your controller, here is the code in Webots that allows to make the conversion, you can probably take inspiration from it: [https://github.com/cyberbotics/webots/blob/565b5aed95a2aadb73bccf2ad733d37893d5edfe/src/webots/nodes/WbGps.cpp#L85-L106](https://github.com/cyberbotics/webots/blob/565b5aed95a2aadb73bccf2ad733d37893d5edfe/src/webots/nodes/WbGps.cpp#L85-L106)

##### David Mansolino [cyberbotics] 05/13/2020 14:50:05
> `@David Mansolino` Thank you for your answer, I did not see this example. Unfortunately it does not work anymore if we put the basicTimeStep to 1ms. If I understand it correctly, it means that the physics behind the simulation has some errors due to a too high timeStep.

`@lojik` yes the basic time step may in rare case influence the calibration of the friction, but even with a smaller time step you should be able to find some parameters that works for your case.

##### Lussfer 05/13/2020 14:48:00
how to convert wgs84 coordinate to local coordinate without changing coordinate system in world info? like i want to pass since wgs84 coordinate and convert it into local coordinate

##### lojik 05/13/2020 14:44:42
> `@lojik` it all depends on your robot, but it doesn't always slide, see for example the 'accelerometer' world, where a differential wheeled robot rotates on a slope: [https://cyberbotics.com/doc/guide/samples-devices#accelerometer-wbt](https://cyberbotics.com/doc/guide/samples-devices#accelerometer-wbt)

`@David Mansolino` Thank you for your answer, I did not see this example. Unfortunately it does not work anymore if we put the basicTimeStep to 1ms. If I understand it correctly, it means that the physics behind the simulation has some errors due to a too high timeStep.

##### kwy 05/13/2020 14:06:40
`@Olivier Michel`  thanks for your help!

##### kwy 05/13/2020 14:05:45
And I have found, streaming viewer is not so smooth like webots screen.
So I decide to use its own robot-window.
It can display 3D graph with vis.js
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/710130671724789800/unknown.png)
%end


##### kwy 05/13/2020 13:59:38
😂

##### kwy 05/13/2020 13:59:23
I just rebooted my Mac… and opened a Sample on Webots, it could connect to the streaming viewer successfully. So I thought that was the error in my own JS. 
But just now my own world can also connect to the streaming viewer.
So the problem is … my Mac need to have a break.

##### Olivier Michel [cyberbotics] 05/13/2020 13:45:49
So what was the problem exactly?

##### kwy 05/13/2020 13:42:02
`@Olivier Michel`  you're right, thanks!

