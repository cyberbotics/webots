# Technical-Questions

This is an archive of the `technical-questions` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m).

## 2020

##### David Mansolino 05/18/2020 06:58:48
Yes you should controler the motor directly like this indeed.

##### elkelkmuh 05/18/2020 06:58:17
`@David Mansolino`  Which commands should I use?
For example: self.getMotor('LegUpperL').setPosition(1.0) 
            self.getMotor('LegLowerL').setPosition(-1.0)
            self.getMotor('AnkleL').setPosition(-0.4)

##### w3. eydi 05/18/2020 06:53:04
As I understand it, there is a stable version in the only repository. I will download it manually and install it again. I will give you feedback if I get an error. If the problem is not resolved, I will continue to use it on Windows, thank you `@David Mansolino` .

##### David Mansolino 05/18/2020 06:43:31
> Hi `@David Mansolino` I want to do stair climbing work with Op2 with python. When I use the    gaitManager.setYAmplitude  command, the foot does not rise as usual.  The robot is falling.  What's your advice?

`@elkelkmuh` the gait algorithm of the darwin-op is not maid to climb stairs, unfortunately you will have to write your own gait algorithm if you want to climb stairs.

##### w3. eydi 05/18/2020 06:42:31
`@David Mansolino`  I received this error message when I used it with optirun.
%figure
![DeepinEkranGoruntusu_20200516234439.png](https://cdn.discordapp.com/attachments/565154703139405824/711831066775912455/DeepinEkranGoruntusu_20200516234439.png)
%end


##### David Mansolino 05/18/2020 06:42:28
HI `@w3. eydi`, may I ask you to try with a nightly version of R2020a-revision2 available here, the unsuported image bug should be fixed with this version (let us know if it is not the case): https://github.com/cyberbotics/webots/releases

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

##### David Mansolino 05/18/2020 06:30:55
`@dralorg` this might be due to the fact that the center of mass of the robot is not centered but rather to the front.

##### David Mansolino 05/18/2020 06:17:07
> Does any of the example use pure pursuit path follow algorithm by any chance?

`@Lussfer` here are some path plannign example in Webots: https://en.wikibooks.org/wiki/Cyberbotics%27_Robot_Curriculum/Advanced_Programming_Exercises#Path_planning_[Advanced]
Unfortunately none of them is using pure pursuit.

##### Stefania Pedrazzi 05/18/2020 06:14:29
> Hello, I'm working on a room-mapping robot for my final year project and was wondering if it is possible to get the grid that the robot is currently on? I'm trying to do a very basic 2d array as a topological map?

`@Conor` To get the position  of the robot you can use the GPS device or the Supervisor API. Both methods will give you the global 3D position, to map it to a XY grid map (or XZ using the default Webots coordinate system) you can simply discard the Y value.  I suggest you to check this sample simulation for an example about getting the robot position: https://www.cyberbotics.com/doc/guide/samples-devices#gps-wbt

##### David Mansolino 05/18/2020 06:10:26
> Hi, we are trying to simulate a simple robot in Webot with two rear driving wheels. To make it simpler among our interdisciplinary team we decided to choose python as the controller language. We are stuck with simple logic like PWM implementation, most of the sample code are written in C/C++ , shall we change it to C or there is some supporting database out there that we have not looked for. We are struggling with this sort of issue from a couple of weeks. Any suggestions would be highly appreciated. Thanks

`@Jatin Sharma` hi, in webots you don't need to implement PWM, you can directly control the motors in speed or position, I would recommend to follow our tutorials (available in Python) to get familiar with controller programming in Webots: https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots?tab-language=python

##### David Mansolino 05/18/2020 06:08:53
> On webots i see that the object recognition is giving. X as Z, Y as Y and Z as -X. on webots does rotation match the orientations..

`@Mathie20` Yes the orientations match the 'rotation' field.
Abotu the axes inversion, I can't unfortunately reproduce this in the 'camera_recognition' sample, do you have a specific case to reproduce this? Note that the position is expressed in the camera frame and not i nthe robot or world frame.

##### Jatin Sharma 05/18/2020 06:03:41
Hi, we are trying to simulate a simple robot in Webot with two rear driving wheels. To make it simpler among our interdisciplinary team we decided to choose python as the controller language. We are stuck with simple logic like PWM implementation, most of the sample code are written in C/C++ , shall we change it to C or there is some supporting database out there that we have not looked for. We are struggling with this sort of issue from a couple of weeks. Any suggestions would be highly appreciated. Thanks

##### David Mansolino 05/18/2020 05:41:58
> Is there code available online that directly controls the wheels of the youbot? I mean, instead of what the webot demo uses (keyboard control)? The idea is for the youbot to follow a black, straight line. I'm also curious if I am required to add 4 hingejoints to the body slot of youbot. Thanks in advance, it is greatly appreciated as I am very new to coding.

`@brianne.byer` sure, here is an example: https://github.com/cyberbotics/webots/blob/master/projects/robots/kuka/youbot/libraries/youbot_control/src/base.c
About the HingeJoint, you can indeed add them directly in the body slot as you would do with a regular chidren field (if not already done I would suggest to read this tutorial: https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)

##### David Mansolino 05/18/2020 05:39:28
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

##### Conor 05/18/2020 00:01:13
`@AyresAlmada` I know you can use GPS but I'm not entirely sure how I would translate the GPS into an X,Y position that can then go into a 2d array

##### Conor 05/18/2020 00:00:35
> hi there. if i use the supervisor to track the position of a robot. is it possible to send the position information to another controller?

`@AyresAlmada` Did you ever find if this was possible? At least getting the robots position?

