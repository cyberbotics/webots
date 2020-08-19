# Documentation

This is an archive of the `documentation` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m).

## 2020

##### David Mansolino [cyberbotics] 08/19/2020 05:44:59
Hi `@lance`, you can get many of these information directly in Webots, if you select the drone, in the node editor (part below the scene-tree) you have a 'Mass' tab that allows you to retrieve many information. Then the simplest solution to get the rotor/propeller position, the simplest solution is to convert temporarily the PROTO node in Base node (right clik on the ndoe in the scene-tree => 'Convert to Base Node(s)') then you can select the propeller node and chek it's position (relatively to any parent node in the 'position' tab of the node editor.

##### lance 08/19/2020 00:08:18
Hi, in a project I am trying to write a adaptive controller for the DJI  mavic 2 pro drone and, therefore, require some detailed parameters of the drone model. I have read the proto file and I am still confused about where to find parameters like moment of inertia around 3 axis, total mass of the drone, and distance from center of mass to the rotor. I am wondering where I can find those parameters? Thanks in advance！

##### David Mansolino [cyberbotics] 07/20/2020 05:36:54
This is unfortunaely not possible. However you can view it offline in Webots from the 'Help' menu.

##### Laojiang 07/20/2020 02:21:35
Hello, how can I download the documentation like user guide?

##### David Mansolino [cyberbotics] 07/07/2020 11:51:08
Thank you, we will have a look soon!

##### Simon Steinmann [Moderator] 07/07/2020 11:41:29
`@David Mansolino` I created a PR [https://github.com/cyberbotics/webots/pull/1879](https://github.com/cyberbotics/webots/pull/1879)

##### David Mansolino [cyberbotics] 06/17/2020 09:42:18
You're welcome

##### ana.dospinescu 06/17/2020 09:41:50
`@David Mansolino` Thank you!

##### David Mansolino [cyberbotics] 06/17/2020 05:26:17
Hi, in R2020a, you should instead use the right and left motor instead.

```
wb_differential_wheels_set_speed
```

Becomes something like:

```
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
```

And for the encoders, you should use instead the position sensors:

```
// get a handler to the position sensors and enable them.
WbDeviceTag left_position_sensor = wb_robot_get_device("left wheel sensor");
WbDeviceTag right_position_sensor = wb_robot_get_device("right wheel sensor");
wb_position_sensor_enable(left_position_sensor, TIME_STEP);
wb_position_sensor_enable(right_position_sensor, TIME_STEP);
double left_encoder_value = wb_position_sensor_get_value(left_position_sensor);
double right_encoder_value = wb_position_sensor_get_value(right_position_sensor);
```

##### ana.dospinescu 06/16/2020 20:17:02
Hello!

Can somenone help me with the equivalent of functions in Webots R2020a version for:

wb\_differential\_wheels\_set\_speed

wb\_differential\_wheels\_enable\_encoders

wb\_differential\_wheels\_set\_encoders

##### Olivier Michel [cyberbotics] 06/08/2020 06:20:06
`@Nitish Gadangi`: Sure I will.

##### Nitish Gadangi 06/06/2020 18:47:41
Hello `@Olivier Michel` , This is regarding Season Of Docs 2020

I have mailed you the details about me and few doubts about the idea I am interested in.

Could you please check that out and ping me back

##### David Mansolino [cyberbotics] 05/20/2020 05:37:11
You're welcome

##### prubhtej Singh 05/20/2020 05:31:12
Thanks for the reply. I'll definitely check it out.

##### David Mansolino [cyberbotics] 05/20/2020 04:50:52
Hi, the documentation is updated directly from our Github repository, you can see all the latest changes here: [https://github.com/cyberbotics/webots/tree/master/docs](https://github.com/cyberbotics/webots/tree/master/docs)

##### prubhtej Singh 05/19/2020 19:10:53
When was documentation last updated ?


I just had a small query.

##### Olivier Michel [cyberbotics] 05/13/2020 15:58:07
Thank you. We will review it soon.

##### Simon Steinmann [Moderator] 05/13/2020 15:55:41
`@Olivier Michel`  thank you so much <3. I created a pull request

##### Olivier Michel [cyberbotics] 05/13/2020 15:54:00
You have to click the "Create pull request" button.


`@Simon Steinmann`: It's here [https://github.com/cyberbotics/webots/compare/master...Simon-Steinmann:patch-1](https://github.com/cyberbotics/webots/compare/master...Simon-Steinmann:patch-1)

##### David Mansolino [cyberbotics] 05/13/2020 15:40:17
Thank you, I will check and let you know

##### Simon Steinmann [Moderator] 05/13/2020 15:37:33
simon-steinmann

##### David Mansolino [cyberbotics] 05/13/2020 15:35:21
Let me check if I can find a stale branch, by the way what is your Github username?

##### Simon Steinmann [Moderator] 05/13/2020 15:34:08
I submitted it April 30th, any way to check all activity? perhaps I submited it to a weird branch. Not that familiar with github


great 😩

##### David Mansolino [cyberbotics] 05/13/2020 15:26:56
> I can't find my commit

`@Simon Steinmann` me neither


Did you fork the repo?

##### Simon Steinmann [Moderator] 05/13/2020 15:26:19
I can't find my commit

##### David Mansolino [cyberbotics] 05/13/2020 15:25:07
Ok perfect, can you then open a pull-request from the branch where you did the commit so that we can review and merge it?

##### Simon Steinmann [Moderator] 05/13/2020 15:22:20
`@David Mansolino`  I made a commit here [https://github.com/cyberbotics/webots/edit/master/docs/reference/supervisor.md](https://github.com/cyberbotics/webots/edit/master/docs/reference/supervisor.md)

##### David Mansolino [cyberbotics] 05/13/2020 15:07:12
For doc correction you can target master directly

##### lojik 05/13/2020 15:06:06
Do you prefer to do the pull request directly in master or in an other branch ?

##### David Mansolino [cyberbotics] 05/13/2020 14:56:55
Yes exactly.

##### lojik 05/13/2020 14:55:35
Thank you, I already have a github account so I will do that. The right way is to fork the repository and then purpose a pull request with my changes ?

##### David Mansolino [cyberbotics] 05/13/2020 14:52:55
Hi, thank you for pointing this out, you can edit this directly on github and create a PR for merging your changes, here is the editor (you need a Github account): [https://github.com/cyberbotics/webots/edit/master/docs/reference/worldinfo.md](https://github.com/cyberbotics/webots/edit/master/docs/reference/worldinfo.md)

##### lojik 05/13/2020 14:51:20
Hello everyone, how can we help to document webots ? I would be happy to take part to improve it.



I think there is a mistake on the following part : [https://www.cyberbotics.com/doc/reference/worldinfo](https://www.cyberbotics.com/doc/reference/worldinfo) on the basicTimeStep part it is written that the minimum value could be 0.001 (one microsecond), but in the table at the beginning, the variable should be [1, inf). If we put less than 1, the simulation does not start in the example accelerometer.

##### David Mansolino [cyberbotics] 05/13/2020 08:12:06
You're welcome

##### İchigogo 05/13/2020 08:12:00
Okay I'll check again thank you so much :)

##### David Mansolino [cyberbotics] 05/13/2020 08:11:17
but are you sure you don't have any call with a value smaller than 0? Because for me when I call with 0 I don't have any warning at all.

##### İchigogo 05/13/2020 08:10:36
but it says uses 0 instead and when I write setBrakeIntensity(1)  it gaves same warning with ,used 1 instead

##### David Mansolino [cyberbotics] 05/13/2020 08:09:14
I just tried and driver.setBrakeIntensity(0)  is not raising any warning for me, you may have another call to driver.setBrakeIntensity with a negative value somewhere in your code.

##### İchigogo 05/13/2020 08:05:14
Python

##### David Mansolino [cyberbotics] 05/13/2020 08:05:04
which language are you using?


let me check

##### İchigogo 05/13/2020 08:00:17
hi ! why setBrakeIntensity(0)  causes this warning ? How can I solve it ?
%figure
![Untitled.png](https://cdn.discordapp.com/attachments/565155720933146637/710038699077009408/Untitled.png)
%end

##### chaytanya 05/13/2020 06:46:52
Sure `@Olivier Michel`

##### Olivier Michel [cyberbotics] 05/13/2020 06:37:58
Hi `@chaytanya`, please send an introductory e-mail to support@cyberbotics.com along with you CV and we will answer you.

##### chaytanya 05/12/2020 22:12:24
I am interested in How-to Guides for Webots and How-to Guides for robotbenchmark projects to contribute


Hello everyone,My name is Chaytanya Sinha,I am an engineering student experienced in c/c++,javascript,nodejs,reactjs,html,css,python and kotlin. I am interested in contributing to webots's documentation.I have been following webots since long. I have experience of documentation as I am working on documentation of webpack v5. Please guide me how to proceed towards documentation of webots

##### David Mansolino [cyberbotics] 05/12/2020 09:04:01
You're welcome, looking forward to see your PR!

##### mgautam 05/12/2020 09:03:07
Thanks David 👍 I will make a PR on it soon

##### David Mansolino [cyberbotics] 05/12/2020 09:02:07
That would indeed be something very useful, if you want to contribute you are very welcome!

##### mgautam 05/12/2020 09:01:08
If it is needed.


Thanks for fast reply. I was thinking to set up a continuous documentation pipeline for it using sphinx.

##### David Mansolino [cyberbotics] 05/12/2020 08:59:30
Let us know if you have any specific question


Hi, the documentation is directly in the README file: [https://github.com/cyberbotics/urdf2webots/blob/master/README.md](https://github.com/cyberbotics/urdf2webots/blob/master/README.md)

##### mgautam 05/12/2020 08:58:37
Hi! I was searching for urdf2webots docs. Does it have one?

##### David Mansolino [cyberbotics] 05/08/2020 05:17:36
You're welcome

##### davisUndergrad 05/08/2020 03:56:17
`@David Mansolino` thank you for the response!

##### David Mansolino [cyberbotics] 05/07/2020 04:54:17
The you have to select the arm and in the 'Position' tab of the field editor (on the bottom of the scene-tree) you can see the position relative to the robot.


Hi `@davisUndergrad`, it is located at '0.156 0 0' from the robot origin. To see it you have to convert the robot node to base node (right click on the youbot in the scene tree, and press 'Convert to Base Node(s)'.

##### davisUndergrad 05/06/2020 17:21:50
Hello, I am trying to work with the Kuka youBot, and I am having trouble understanding where the origin of the coordinate frame used by the arm\_ik function provided in the arm.c 

library is located. Is this documented somewhere?

##### İchigogo 05/01/2020 17:31:51
Oh I understand now thank you so much

##### Olivier Michel [cyberbotics] 05/01/2020 16:53:42
which corresponds to 60°


Then you should set the aperture field value to 1.0472 rad.

##### İchigogo 05/01/2020 16:33:48
hi !  in emitter-receivers how can I calculate aperture?I couldn't understand. from documentation I understand like when it's -1 it sends to 360 degree I want it  send to only 60

##### Simon Steinmann [Moderator] 04/30/2020 10:54:50
submitted it

##### David Mansolino [cyberbotics] 04/30/2020 10:47:24
No we are using directly the Github issue mechanism: [https://github.com/cyberbotics/webots/issues](https://github.com/cyberbotics/webots/issues)

In particular for feature request:

[https://github.com/cyberbotics/webots/issues/new?template=feature\_request.md](https://github.com/cyberbotics/webots/issues/new?template=feature_request.md)

##### Simon Steinmann [Moderator] 04/30/2020 10:40:04
do you have something like a trello?


where would I propose that?

##### Olivier Michel [cyberbotics] 04/30/2020 10:37:09
😁 . But feel free to go ahead with these good idea and propose an implementation with a PR. That shouldn't be very difficult.

##### Simon Steinmann [Moderator] 04/30/2020 10:36:03
A frustrated coder is full of good ideas 😄

##### Olivier Michel [cyberbotics] 04/30/2020 10:35:08
Yes, that seems to be a good idea.

##### Simon Steinmann [Moderator] 04/30/2020 10:34:52
most commonly used in ROS and any 3D application. would really be helpfull


perhaps add get\_orientation\_quaternion while you're at it 😉


get\_orientation\_matrix


perhaps adding a function? Would be the non destructive way

##### Olivier Michel [cyberbotics] 04/30/2020 10:28:36
Yes, we may consider changing it on the develop branch.

##### Simon Steinmann [Moderator] 04/30/2020 10:26:50
but I guess changing that would break existing code


oh btw, it's kinda weird that get\_orientation returns a 1x9 list, instead of 3x3

##### Olivier Michel [cyberbotics] 04/30/2020 10:25:15
OK, looking forward to review it.

##### Simon Steinmann [Moderator] 04/30/2020 10:24:56
solved it differently


added it

##### Olivier Michel [cyberbotics] 04/30/2020 10:19:11
Great. It's the `%spoiler` keyword.

##### Simon Steinmann [Moderator] 04/30/2020 10:18:40
I think I got it 🙂

##### Olivier Michel [cyberbotics] 04/30/2020 10:18:13
Something like the **Reminder** and **Tips** here: [https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot#sensors](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot#sensors) ?


Yes, that seems to be a good idea. Let me search how to do that...

##### Simon Steinmann [Moderator] 04/30/2020 10:15:29
a 'expandable box' would be great, no idea how to implement that


in what way should I insert the code? Linked, directly in the description, or in a different way??


Okay will do. Feel free to change or edit it btw.

##### Olivier Michel [cyberbotics] 04/30/2020 10:07:41
That is great. Could you create a PR to add this contribution to the doc? [https://github.com/cyberbotics/webots/edit/master/docs/reference/supervisor.md](https://github.com/cyberbotics/webots/edit/master/docs/reference/supervisor.md)

##### Simon Steinmann [Moderator] 04/30/2020 10:03:40
[https://pastebin.com/k7kf4Ez5](https://pastebin.com/k7kf4Ez5)


Okay so I finally got it working to quickly and easily calculate any position and orientation of a node relative to any other node. This should be added to the supervisor get\_position and get\_orientation documentation

##### Olivier Michel [cyberbotics] 04/21/2020 16:04:21
From the Help menu.


Meanwhile, you can revert to the doc embedded inside Webots.


And since the doc uses that...


This URL doesn't load for me: [https://github.com/cyberbotics/webots/blob/master/docs/reference/propeller.md](https://github.com/cyberbotics/webots/blob/master/docs/reference/propeller.md)


Actually the problem is on github...


Well, it is very slow, but works for me...


Yes, that's right. Thank you for reporting...

##### Axel M [Premier Service] 04/21/2020 15:59:54
Hi ! Documentation content seems off since a few minutes

##### David Mansolino [cyberbotics] 04/21/2020 10:03:37
You're welcome

##### İchigogo 04/21/2020 09:26:51
Thank you so much 😊

##### David Mansolino [cyberbotics] 04/20/2020 06:00:37
Hi, by default lines are dashed so the last line is not diplayed in the scene-tree, to be able to change its with you have to add one more `RoadLine` node to the `lines` field.

##### İchigogo 04/19/2020 09:28:24
May I ask something? I'm trying to change width of road lines for the making lane detection more easy but when I change only one side of the road is changing. there is only 1 dashed road line node exists I couldn't understand how to change other one.
%figure
![road.png](https://cdn.discordapp.com/attachments/565155720933146637/701363563998085170/road.png)
%end

##### ClBaze 04/16/2020 13:04:43
kinetic

##### David Mansolino [cyberbotics] 04/16/2020 13:04:38
Which version of ROS are you using?

##### ClBaze 04/16/2020 13:03:52
ok

##### David Mansolino [cyberbotics] 04/16/2020 13:03:30
Ok, thank you for reporting this.

May I ask you to open a bug report here: [https://github.com/cyberbotics/webots/issues/new?template=bug\_report.md](https://github.com/cyberbotics/webots/issues/new?template=bug_report.md)

So that we can log this and try to fix it?

Thank you.

##### ClBaze 04/16/2020 12:51:05
`[ros] [ERROR] [1587023118.216799425, 4.456000000]: client wants service /tile_35_4366_pc047/supervisor/get_from_def to have md5sum ac26007a2c83bd1b38318cda0f4ce627, but it has 3f818aa8f7c2c60588c99f7ee189c5bd. Dropping connection.`


It seems that the checksum of the get\_from\_def message has changed


I haven't tested, but it works with the Webots R2020a revision1 .deb

##### David Mansolino [cyberbotics] 04/16/2020 12:42:38
Hi `@ClBaze` is it working on the master branch ?

##### ClBaze 04/16/2020 12:42:18
(oops this probably belongs to the development channel)


Hello, I'm using Webots from the develop branch, I've noticed that the ROS service supervisor\_get\_from\_def doesn't work anymore

##### David Mansolino [cyberbotics] 02/18/2020 11:46:25
The tooltip is indeed wrong, the actual hotkey is 'control + shift + t'. I am fixing the tooltip right now.


Hi `@Lifebinder (tsampazk)`, thank you for reporting this, I will check it.

##### Lifebinder (tsampazk) 02/18/2020 11:41:54
Hello, i don't know if its appropriate to tell you this in this channel, but the tooltip of reset simulation (hover over tooltip) says hotkey is control shift F, but when i press that it pops the "Fullscreen mode" window



I just installed the 2020a rev1 version

##### Lars 02/14/2020 13:20:54
`@David Mansolino` Done, made a request to you. 😄

##### David Mansolino [cyberbotics] 02/14/2020 13:12:39
`@Lars` your patch is completely correct, feel free to open a pull request to include it in Webots 🙂

##### Lars 02/14/2020 12:40:46
`@David Mansolino` We found an error in [https://github.com/cyberbotics/webots/blob/master/docs/guide/gripper-actuators.md](https://github.com/cyberbotics/webots/blob/master/docs/guide/gripper-actuators.md)

It says `palm_finger_1_joint_sensor` in both second and third row of the second column

Suggested change for third row, second column: Change to `finger_1_joint_1_sensor` (which is currently not mentioned in the table, but defined in the proto).

See [https://github.com/cyberbotics/webots/compare/master...LarsVaehrens:patch-1](https://github.com/cyberbotics/webots/compare/master...LarsVaehrens:patch-1) the fork

##### İchigogo 02/12/2020 16:36:05
I found the problem . It was about Windows i needed to use if \_\_name\_\_=='\_\_main\_\_':. 😅 😅

##### David Mansolino [cyberbotics] 02/12/2020 15:15:42
You're welcome 😉

##### İchigogo 02/12/2020 15:13:12
Don't say sorry thank you so much for helping me 🙂

##### David Mansolino [cyberbotics] 02/12/2020 14:28:30
OK, good look, sorry that I am not able to help more. But do not desesperate, it is for sure feasible (I did something similar recently with extern controllers too and it was working).

##### İchigogo 02/12/2020 14:27:19
😭 😭  okay thank you so much for helping me. I'll read more probably I'm doing something wrong because I don't know too much about multiprocessing

##### David Mansolino [cyberbotics] 02/12/2020 14:24:40
That's very strange because this alert should only be raised when you initiliaze the robot.

##### İchigogo 02/12/2020 14:10:33
Yeah.

##### David Mansolino [cyberbotics] 02/12/2020 14:07:46
Are you calling the wb\_robot\_init before this?

##### İchigogo 02/12/2020 14:01:51
in the code when p1.start() to starting processes 1

##### David Mansolino [cyberbotics] 02/12/2020 13:58:35
ok, can you identify when is the alert display? wich function cause this alert exactly?

##### İchigogo 02/12/2020 13:57:46
Yeah I set the controller <extern>. It was working fine before I try multi processing

##### David Mansolino [cyberbotics] 02/12/2020 13:56:12
Have you tried with a simpler version without multi-processing to see if you can reproduce the issue?


just to make sure, did you set the 'controller' field of the robot to <extern> ?

##### İchigogo 02/12/2020 13:53:40
In while loop I'm creating 2 process first one taking image data from camera1 finding lanes and displays it. Second one just taking image data from camera 2 and not doing anything for now cuz I didn't code it yet.  When I start it. It actually works I can see lanes on display but that no extern controller found alert shows up

##### David Mansolino [cyberbotics] 02/12/2020 13:44:37
maybe can you describe your workflow so that we can better understand where can the problem come from


Ok

##### İchigogo 02/12/2020 13:28:38
Thank you :) No I just call 2 different function 1 is for Lane detection and others for object recognition

##### David Mansolino [cyberbotics] 02/12/2020 13:26:16
Don't worry for your english, mine is not perfect too ;-)

Are you calling several time the wb\_robot\_init fucntion from different thread or so ?

##### İchigogo 02/12/2020 13:23:24
Yeah I did but it's showing it in loop when I close it shows up again until I pause the simulation (sorry for my bad English but I hope you understand me 😭   )

##### David Mansolino [cyberbotics] 02/12/2020 13:10:52
did you revert the simulation befaore starting your controller?

##### İchigogo 02/12/2020 13:09:17
Hi I came back again with my questions 😅  I kinda tried to use multiprocessing but it didn't work. it says no available extern robot controller found and when I try directly run it from simulation it just.. Nothing happens.


Thank you so much

##### Fabien Rohrer [Moderator] 02/11/2020 15:32:50
Hi, please take a look at this example: [https://cyberbotics.com/doc/guide/samples-devices#camera\_recognition-wbt](https://cyberbotics.com/doc/guide/samples-devices#camera_recognition-wbt)

##### İchigogo 02/11/2020 15:31:50
Hi! Is there any example project for object recognition from camera data? I read that camera has recognition node but I couldn't understand too much. Is it possible to recognise a specific object with it ? Or is it possible to use Yolo for this? I'm totally noob someone please help me

##### Olivier Michel [cyberbotics] 02/11/2020 11:06:26
Yes, you should use `getMotor` instead.

##### Ptosidis\_opendr 02/11/2020 11:05:17
nvm it's a Motor, getMotor works fine


but this one , even though i get no errors, returns None


`camera_roll_motor = robot.getCamera('camera roll')`


Hey again, still having some problems.

`WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll"); `

should translate to

##### Stefania Pedrazzi [cyberbotics] 02/11/2020 10:08:36
No problem!

##### Ptosidis\_opendr 02/11/2020 10:07:43
Hey Stefania, thank you for your fast reply! I might bombard you with question in the near future, so please be patient with me 🙂

##### Stefania Pedrazzi [cyberbotics] 02/11/2020 10:06:22
Hi `@Ptosidis_opendr` in Python controller you should use the method `robot.getCamera('camera roll')` [https://www.cyberbotics.com/doc/reference/robot?tab-language=python#getcamera](https://www.cyberbotics.com/doc/reference/robot?tab-language=python#getcamera)

##### Ptosidis\_opendr 02/11/2020 10:04:16
Hello, I am trying to convert the mavic2pro controller to python so I can run some experiments with it. (Just starting my webots interaction, so it might be a newbie question). 

I got the following problem: can't seem to find the camera roll device. I tried by camera\_roll\_motor = robot.getDevice('camera roll') but it doesn't seem to work. 

Any insights on how to find all devices by name, or better the names of all devices on a robot?

##### David Mansolino [cyberbotics] 01/30/2020 10:22:42
Thank you, I will have a look at this today.

##### nelsondmmg 01/30/2020 10:21:41
Just posted on stack, thanks

##### David Mansolino [cyberbotics] 01/30/2020 10:15:51
Ok thank you, you can also send us a gist if you want: [https://gist.github.com/](https://gist.github.com/)

##### nelsondmmg 01/30/2020 10:14:55
So I'm on the cellphone


I'm going to open a thread at StackOverflow, this app does not work in the company's network

##### David Mansolino [cyberbotics] 01/30/2020 10:13:26
would it be possible for you to share this part of code with us so that we can try?

##### nelsondmmg 01/30/2020 10:12:19
I didn't, I tried again with the gear 1 and the same error appears

##### David Mansolino [cyberbotics] 01/30/2020 10:04:36
Did you engage a gear before calling setThrottle ?

##### nelsondmmg 01/30/2020 10:01:26
I'm also using a Tesla model 3 as vehicle


I'm passing 0 as argument, both to setThrottle and setBreakIntensity


Error: wb\_motor\_set\_force() called with and invalid force argument (NaN)

##### David Mansolino [cyberbotics] 01/30/2020 09:59:36
What is the exact error message you have?

##### nelsondmmg 01/30/2020 09:49:51
I'm trying to use setThrottle but webots return the message "called with an invalid force argument". Should I called a function before to change the vehicle mode to torque control? I'm already waiting 30ms to initiate all sensors

##### David Mansolino [cyberbotics] 01/30/2020 08:02:43
You're welcome

##### İchigogo 01/30/2020 07:48:28
Thank you so much for your help @David Mansolino 😀

##### David Mansolino [cyberbotics] 01/30/2020 06:23:01
Hi `@İchigogo` if I am not wrong cv2 inverts X and Y coordinate compared to Webots Displays, you should therefore inverse the x an y of your array before converting it to list.

Probably the `numpy.swapaxes` function can help you, with something like this (not tested):

```Python
img= np.array(np.swapaxes(array,0,1)). tolist()  ref=display.imageNew(img,Display.RGBA,h,w)
display.imagePaste(ref,0,0,blend=false)
```

##### İchigogo 01/30/2020 00:48:26
Hi! Sorry for disturbing you again. I converted the data and made the image progressing. I want to show it with display screen. I have something like 

<<

 img= np.array( array). tolist()  ref=display.imageNew(img,Display.RGBA,h,w)

 display.imagePaste(ref,0,0,blend=false)

 >> 

 when I use cv2 to show I can see the processed image  but when I use display it shows somewhere else in the simulation with 90° rotation where I'm doing wrong? And I'm really sorry for asking too many questions I really tried for days but couldn't solve.

##### David Mansolino [cyberbotics] 01/27/2020 07:51:38
You're welcome

##### İchigogo 01/27/2020 07:51:22
Thank you so much 🙂

##### David Mansolino [cyberbotics] 01/27/2020 07:10:51
Hi `@İchigogo`, Webots provides an example of controller in python using cv2 to process a camera images: [https://github.com/cyberbotics/webots/blob/master/projects/samples/robotbenchmark/visual\_tracking/controllers/visual\_tracking/visual\_tracking.py](https://github.com/cyberbotics/webots/blob/master/projects/samples/robotbenchmark/visual_tracking/controllers/visual_tracking/visual_tracking.py)

You can't simply use 'data' as a cv2 image, you have to convert it somehow (see for example: [https://stackoverflow.com/questions/17170752/python-opencv-load-image-from-byte-string?answertab=oldest#tab-top](https://stackoverflow.com/questions/17170752/python-opencv-load-image-from-byte-string?answertab=oldest#tab-top))

##### İchigogo 01/26/2020 20:04:42
I'm trying to make image progressing. I get data with << data = camera. getImage() >> but I can't display it with cv2.imshow( data )   what should I do?

##### David Mansolino [cyberbotics] 01/22/2020 08:02:40
You're welcome

##### İchigogo 01/22/2020 08:00:31
Thank you 😃

##### David Mansolino [cyberbotics] 01/22/2020 07:01:55
Hi `@İchigogo`, the first thing to check is to make sure that you did define the properties of the contact between the ground and the wheels of the car. You should have a contactProperties with a `softCFM` around  1e-05 and a `coulombFriction` around 8. You can for example simply copy paste the contectProperties from the city world: [https://github.com/cyberbotics/webots/blob/master/projects/vehicles/worlds/city.wbt#L13-L73](https://github.com/cyberbotics/webots/blob/master/projects/vehicles/worlds/city.wbt#L13-L73)

##### İchigogo 01/21/2020 22:43:42
Hi I'm probably asking a really basic thing but I couldn't solve. I was trying to simulate a city I added roads etc everything was fine but when I added the car it's buries into the road. How can I solve it?

##### bsr.nur.bahadir 01/20/2020 09:43:13
I was talking about SUMO Exporter  sorry but I solved thank you 🙂

##### David Mansolino [cyberbotics] 01/20/2020 06:59:16
It might be possibkle that the port used by default is not free on your computer, in that case you can simply change it by changign the `port` field of the `SumoInterface` node in the scene-tree.


On Windows it should work the axact same way as on the other OS, do you have any specific issue?


Hi `@bsr.nur.bahadir` the sumo interface is described here: [https://cyberbotics.com/doc/automobile/sumo-interface](https://cyberbotics.com/doc/automobile/sumo-interface)

##### bsr.nur.bahadir 01/19/2020 14:26:48
Is there any instructions of using SUMO on Windows?

##### Stefania Pedrazzi [cyberbotics] 01/09/2020 10:54:19
`@ClBaze` Thank you for reporting this! I will fix it.

By the way you could also fix it directly in our GitHub repository: [https://github.com/cyberbotics/webots/blob/master/docs/guide/modeling.md](https://github.com/cyberbotics/webots/blob/master/docs/guide/modeling.md)

##### ClBaze 01/09/2020 10:48:17
I think it's WorldInfo.optimalThreadCount


"The number of threads used by the physics engine (ODE) can be changed either globally in the preferences or using the WorldInfo.basicTimeStep field"


There is a small mistake in the [https://cyberbotics.com/doc/guide/modeling#how-to-make-replicabledeterministic-simulations](https://cyberbotics.com/doc/guide/modeling#how-to-make-replicabledeterministic-simulations) doc.

##### TH0 01/05/2020 12:14:18
Hi, can you recommend a good webots tutorial for creating an own robot from 3d parts designed in a CAD programm (e.g. fusion 360)? I'm interessted if there is a good workflow for importing shapes and connecting all parts over joints without manually editing - for example the geometry details - in the proto files, because its so time consuming and not very efficient.

##### David Mansolino [cyberbotics] 01/03/2020 07:23:00
<@!647102363882225664>, unfortunately Webots doesn't support sound sensors for know.

But feel free to contribute to Webots to extend it and support sound sensors, here is the guidlines for adding new nodes (such as a microphone): [https://github.com/cyberbotics/webots/wiki/Adding-New-Node-and-API-Function](https://github.com/cyberbotics/webots/wiki/Adding-New-Node-and-API-Function)

## 2019

##### bsr.nur.bahadir 12/21/2019 17:51:01
I understand. Thank you so much 🙂

##### David Mansolino [cyberbotics] 12/19/2019 06:57:02
Hi `@bsr.nur.bahadir` which voice recognition documentation are you refeerig too? The Webots API provide text to speech but not voice recognition.

About path planninc, you can find some examples here: [https://en.wikibooks.org/wiki/Cyberbotics%27\_Robot\_Curriculum/Advanced\_Programming\_Exercises#Path\_planning\_](https://en.wikibooks.org/wiki/Cyberbotics%27_Robot_Curriculum/Advanced_Programming_Exercises#Path_planning_)[Advanced]

##### bsr.nur.bahadir 12/18/2019 19:40:43
Hello, need voice recognition documentation but I couldn't open it. And is there any example project for path planning? 🥺

##### David Mansolino [cyberbotics] 12/05/2019 08:03:06
`@laboris7440` you're welcome !

##### laboris7440 12/05/2019 08:02:06
`@David Mansolino`  Thanks, just what I was looking for!

##### David Mansolino [cyberbotics] 12/05/2019 07:19:46
In the meantime you can download Webots directly here: [https://github.com/cyberbotics/webots/releases/tag/R2019b-rev1](https://github.com/cyberbotics/webots/releases/tag/R2019b-rev1)


For example this file is not available yet, but we are still working on it.


Hi `@nitrow`, the server is back but we haven't restored everything yet.

##### nitrow 12/04/2019 18:56:32
This command doesn't seem to be working now that the servers are back up "curl -s -L [https://www.cyberbotics.com/Cyberbotics.asc](https://www.cyberbotics.com/Cyberbotics.asc) | sudo apt-key add -" Did it change to something else?

##### David Mansolino [cyberbotics] 12/03/2019 14:28:28
You're welcome

##### nitrow 12/03/2019 14:28:22
Thanks a lot!

##### David Mansolino [cyberbotics] 12/03/2019 14:28:03
You can find a backup of the documentation here: [https://github.com/cyberbotics/webots/blob/revision/docs/guide/installation-procedure.md](https://github.com/cyberbotics/webots/blob/revision/docs/guide/installation-procedure.md)


Hi `@nitrow`, your are right, we are currenlty updating our servers.

##### nitrow 12/03/2019 14:26:37
Hello, it seems like [https://cyberbotics.com](https://cyberbotics.com) has been down for a while now, and thereby also the documentation. Maybe you can help me anyway.. I know that it is possible to download/install Webots through the terminal, can you tell me how?

##### Fabien Rohrer [Moderator] 12/03/2019 13:00:37
`@Marian` Hi how could we help you?

##### Marian 12/03/2019 13:00:16
hello

##### Hayden Woodger 11/10/2019 23:50:48
The first Tutorial on the Webots website is also a good one to learn  to get started 🙂 [https://cyberbotics.com/doc/guide/tutorials](https://cyberbotics.com/doc/guide/tutorials)


Hi there, I used and still use this website for interactive training 🙂 [https://robotbenchmark.net/](https://robotbenchmark.net/)

##### thrilok emmadsietty 11/10/2019 23:48:26
i am new to this software where can i get the tutorials?

##### chamandana 11/07/2019 11:08:02
many thanks.

##### Stefania Pedrazzi [cyberbotics] 11/07/2019 11:06:07
`@chamandana` when using printf in C it is important to end the string with "\n" to force  flushing the buffer

##### chamandana 11/07/2019 10:58:54
another problem, I've never worked with C on Webots. and printf() function doesn't seem to be working. (2019b).

##### David Mansolino [cyberbotics] 11/07/2019 10:41:35
`@chamandana` no problem, you're welcome

##### chamandana 11/07/2019 10:38:16
`@David Mansolino` Thanks, sorry for being ignorant lol


ah elaela

##### David Mansolino [cyberbotics] 11/07/2019 10:37:28
Hi `@chamandana` an example is distributed with Webots: [https://cyberbotics.com/doc/guide/samples-devices#range\_finder-wbt](https://cyberbotics.com/doc/guide/samples-devices#range_finder-wbt)

##### chamandana 11/07/2019 10:35:59
Can anyone give me an example to use RangeFinder in C?

##### Fabien Rohrer [Moderator] 10/30/2019 16:59:34
Ok, so probably that modifying directly the Nao.proto is the best solution for your issue. (you can copy-paste the "protos" directory in your project to work on a local copy)

##### HiguchiNakamura 10/30/2019 16:57:58
But thanks


I think we're skipping the function.


I tried that out with "convert base node". The Controller wont work after that 😅 . It is planned to test it on a real NAO in the foreseeable future. If we modify, I can't tell if this work out in real life.

##### Fabien Rohrer [Moderator] 10/30/2019 16:55:18
Does this answer your question?


- or explode the Nao.proto in your simulation (from the scene tree, right click on the Nao, and select "Convert to base nodes") Then the Camera node will be accessible. But the simulation modularity will be penalized 😉


[https://github.com/cyberbotics/webots/blob/revision/projects/robots/softbank/nao/protos/Nao.proto#L797](https://github.com/cyberbotics/webots/blob/revision/projects/robots/softbank/nao/protos/Nao.proto#L797)


- edit Nao.proto, and modify the Camera node as you wish:


This is the simplest solution, but if this is problematic, you could also either:


For sure this is a supplementary camera, but it could be defined exactly as the existing camera embedded in the Nao.

##### HiguchiNakamura 10/30/2019 16:50:52
We want to stay as close as possible to the NAO (so no modifications). But this wouldn't be a mod?

##### Fabien Rohrer [Moderator] 10/30/2019 16:50:15
As-is, you will have a full control of the new camera fields.


But I think the simplest is to add a new Camera node (with the recognition feature) in the Nao.headSlot.


Yes, it's possible.


`@HiguchiNakamura` Hi

##### HiguchiNakamura 10/30/2019 16:47:51
Hello, I wanted to ask if the camera node of the NAO can be extended by a recognition node. But I don't find any children at NAO. So isn't it possible? When a Camera device has a Recognition node in its recognition field, it is able to recognize which objects are present in the camera image.

##### JoanGerard 10/28/2019 17:49:50
Thanks `@Tahir` !

##### Tahir [Moderator] 10/28/2019 17:37:05
check here [https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers)


`@JoanGerard` this option is available in R2019b not in R2019a

##### JoanGerard 10/28/2019 17:25:02
Hello, I was trying to run an external robot controller but there is no <external> option while selecting the controller of my robot. I have the Webots R2019a version. 

Maybe this options is not available any more?
%figure
![Screen_Shot_2019-10-28_at_6.22.02_PM.png](https://cdn.discordapp.com/attachments/565155720933146637/638428024743657522/Screen_Shot_2019-10-28_at_6.22.02_PM.png)
%end

##### Stefania Pedrazzi [cyberbotics] 10/28/2019 07:18:31
and you can find a simple example in the distributed samples folder `projects/samples/devices/worlds/track.wbt`

##### dreamerz 10/28/2019 07:18:05
Thanks

##### Stefania Pedrazzi [cyberbotics] 10/28/2019 07:17:23
yes, here is the documentation of the Track node: [https://www.cyberbotics.com/doc/reference/track](https://www.cyberbotics.com/doc/reference/track)

##### dreamerz 10/28/2019 07:13:11
A complete one, is there a way to make tracks?

##### Stefania Pedrazzi [cyberbotics] 10/28/2019 07:11:24
`@dreamerz` what do you mean exactly? a complete one including wheels or just a solid piece? 

Did you already look at the tutorials? [https://www.cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot](https://www.cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)

##### dreamerz 10/28/2019 07:08:03
How would you create a custom chassy

##### Stefania Pedrazzi [cyberbotics] 10/28/2019 07:07:25
then , for pitch and roll the description also specified that this is only true if the up (or gravity) vector is -y-axis


The "first" axis mentioned in the documentation corresponds to the rotation axis if the up (or gravity) vector is the default one (i.e. if the up vector corresponds to the y-axis)


`@HiguchiNakamura`, you have to take into consideration that in Webots the up vector by default corresponds to the y-axis and not the z-axis as in other coordinate systems.

##### HiguchiNakamura 10/27/2019 15:45:50
I'm not sure what's right now.


Yaw has something to do with z-axis but the documentation even doesn't include this


Pitch has something to do with y-axis, but in webots there are two axis mentioned. The first one is z-axis.


Hi. I think the inertial unit section  (function getRollPitchYaw) has wrong information. 



Some  axis are not correct at roll, pitch and yaw angle.

##### BlackPearl 10/17/2019 10:14:42
Ah ok, yes 

Thank you very much

##### Fabien Rohrer [Moderator] 10/17/2019 10:13:19
Is it more clear?


When the rays hit objects, the rays are rejected if the angle between the ray and the hit material is above 45° => this is internal Webots stuff. I think you can forget this for now 😉


The maximum angle of these rays at the sensor location is given by the DistanceSensor.aperture field, in your case, 1.04 rad = 60°. This is a parameter defined in the Nao PROTO


These rays have an origin at the sensor location.


A DistanceSensor in Webots is modeled internally as a set of rays.


I do my best to explain it 😉

##### BlackPearl 10/17/2019 10:04:56
`@Fabien Rohrer`  so can u explain what is the 45 degree now? Should we focus on 60 degree?

##### NaoTeam28 10/17/2019 09:53:05
ok, thank you

##### Fabien Rohrer [Moderator] 10/17/2019 09:52:13
The 45° occurs at another level, where the rays hit objects. It's like a property of the hit material.


This means that at the sonar point, the rays are splitted along 60°.

##### NaoTeam28 10/17/2019 09:51:11
OK

##### Fabien Rohrer [Moderator] 10/17/2019 09:49:49
Both NAO sonars have an aperture of 60°: [https://github.com/cyberbotics/webots/blob/revision/projects/robots/softbank/nao/protos/Nao.proto#L720](https://github.com/cyberbotics/webots/blob/revision/projects/robots/softbank/nao/protos/Nao.proto#L720)

##### NaoTeam28 10/17/2019 09:48:20
The NAO has two such sensors. Do they together give about this 60°? It should overlap.


is it rather the case that the NAO in webots, for example, only recognizes objects in these 45°?

##### Fabien Rohrer [Moderator] 10/17/2019 09:44:43
It seems to match quite well the Nao specs. Do you understand better these 2 angles?


The second one is specific to the sonar type, and is hard-coded in Webots to 45°. It defines the angle beyond which the rays are lost AT THE REFLECTION POINT.


The first one is given by the customizable DistanceSensor.aperture angle AT THE SENSOR POINT. In case of the Nao, it's 60°. This defines the opening angle of the cone of rays. Indeed a single sensor is modeled by several rays, shown in red if you enable the "View / Optional renderings / Show DistanceSensor rays".


There are 2 different angles in case of a sonar DistanceSensor.


Ok I see...

##### NaoTeam28 10/17/2019 09:34:49
[https://cyberbotics.com/doc/reference/distancesensor#sonar-sensors](https://cyberbotics.com/doc/reference/distancesensor#sonar-sensors)

##### Fabien Rohrer [Moderator] 10/17/2019 09:33:36
(The most recent doc about the Nao is there: [https://cyberbotics.com/doc/guide/nao](https://cyberbotics.com/doc/guide/nao) )


Hi, could you give us the link where you found 45°?

##### NaoTeam28 10/17/2019 09:27:37
The developers of the NAO specify an effective cone of 60°, but in the Webots documentary it is 45°. The DistanceSensor.getAperture() function returns the value 1.04 in radians, which is 60° in degrees.



What is meant by 45°?

##### SimonDK 10/01/2019 15:24:08
Ah yes indeed, missed that. Fortunately I remembered how to do it from one of the previous tutorials 🙂 A suggestion. It would be more intuitive if all "Hands on" followed the style of numbered bullet lists instead of just a full paragraph text. Besides that, I think the tutorials are great, love them!

##### David Mansolino [cyberbotics] 10/01/2019 14:47:56
At the beginning, in the 'Hands on #1' it mentions:

>  Modify the controller field of the E-puck node in order to associate it to the new controller.



Is this what you were looking for ?


> I just remembered, in Tutorial 4 ([https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers](https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers)) where you make a new controller for simple obstacle avoidance, I believe the tutorial never mentions to actually add the new controller to the robot in the end. It just stops and concludes.



Ok, I will check.


You're welcome. The links have been updated here: [https://github.com/cyberbotics/webots/pull/945](https://github.com/cyberbotics/webots/pull/945)

They will be updated on the live version on the website in a few minutes.

##### SimonDK 10/01/2019 14:43:20
I just remembered, in Tutorial 4 ([https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers](https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers)) where you make a new controller for simple obstacle avoidance, I believe the tutorial never mentions to actually add the new controller to the robot in the end. It just stops and concludes.


Great, thank you for the fast reply `@David Mansolino`

##### David Mansolino [cyberbotics] 10/01/2019 14:35:20
You are right, it seems this site is not the official one anymore. Thank you for the notice, we will correct this right now!

##### SimonDK 10/01/2019 14:33:47
In Tutorial 5 ([https://cyberbotics.com/doc/guide/tutorial-5-compound-solid-and-physics-attributes](https://cyberbotics.com/doc/guide/tutorial-5-compound-solid-and-physics-attributes)) the link to ODE ([http://ode-wiki.org/wiki/index.php?title=Manual](http://ode-wiki.org/wiki/index.php?title=Manual)), the website is full of advertisement and no documentation. One time I clicked it and it directly downloaded maccleaner which is a POS software. I think the link should be updated or removed.


I use it in macos and it works great

##### Fabien Rohrer [Moderator] 09/26/2019 19:02:56
Hi, yes it is tested every day in this OS.

##### elnaz 09/26/2019 16:25:23
Hi

does Webots work fine in macOS Mojave?

##### HiguchiNakamura 09/25/2019 15:55:11
Seems like a lot to do. And there is realy no example for NAO? Not even code snippets? I am not that good in c/c++

##### David Mansolino [cyberbotics] 09/25/2019 12:32:39
Yes, the remote control library should be written in C, but then the controller of the robot iteself can be written in any language.

Please find more information here:

  - [https://www.cyberbotics.com/doc/guide/transfer-to-your-own-robot#remote-control](https://www.cyberbotics.com/doc/guide/transfer-to-your-own-robot#remote-control)

  - [https://www.cyberbotics.com/doc/guide/controller-plugin#remote-control-plugin](https://www.cyberbotics.com/doc/guide/controller-plugin#remote-control-plugin)

##### HiguchiNakamura 09/25/2019 12:23:29
`@Stefania Pedrazzi` for every robot only in c or is it just the case for NAO

##### Stefania Pedrazzi [cyberbotics] 09/25/2019 06:21:48
`@HiguchiNakamura` , if you are referring to the remote-control plugin, then it is not possible to write it in Java but only in C.

##### Fabien Rohrer [Moderator] 09/24/2019 12:29:10
Does this answer your question?


But it's indeed possible to write a Java controller for the Nao.


There is currently no example of Nao controller written in Java.


You can find Nao examples in this directory\_ WEBOTS\_HOME/projects/robots/softbank/nao


You may find a java example in WEBOTS\_HOME/projects/languages/java/worlds/example.wbt


Hi,

##### HiguchiNakamura 09/24/2019 12:24:54
Hi is there an example for other Robots so I can make it for nao in java

##### Stefania Pedrazzi [cyberbotics] 09/23/2019 09:21:57
As far as I know there is no default remote controller plugin in Webots for the NAO robot. But you can write your own: [https://www.cyberbotics.com/doc/guide/controller-plugin?tab=c#remote-control-plugin](https://www.cyberbotics.com/doc/guide/controller-plugin?tab=c#remote-control-plugin)


Hi `@NaoTeam28` , as explained in the documentation ([https://www.cyberbotics.com/doc/reference/robot?version=fix-contact-properties-doc&tab=c#wb\_robot\_set\_mode](https://www.cyberbotics.com/doc/reference/robot?version=fix-contact-properties-doc&tab=c#wb_robot_set_mode)) `arg` is the argument that will be passed to the remote control `wbr_start` function.

##### NaoTeam28 09/23/2019 08:36:37
Hello Guys, 



we got a problem. 



public void setMode(int mode, SWIGTYPE\_p\_void arg);



what do we have to insert for SWIGTYPE\_p\_void arg? We want to switch from Simulated to remote control. We have a real NAO here so we want to run it in real life

##### David Mansolino [cyberbotics] 08/30/2019 10:58:05
You're welcome

##### HiguchiNakamura 08/30/2019 10:56:48
Okey, thank you

##### David Mansolino [cyberbotics] 08/30/2019 10:55:21
I just checked and the function `setMode` is indeed available in Java too, the Python and Java documentation are missing this fucntion, we will add it as soon as possible, thank you for noticing this.

##### HiguchiNakamura 08/30/2019 10:51:17
OK

##### David Mansolino [cyberbotics] 08/30/2019 10:50:46
Hi `@HiguchiNakamura`, let me check.

##### HiguchiNakamura 08/30/2019 10:44:00
[https://cyberbotics.com/doc/reference/robot?tab=java#wb\_robot\_set\_mode](https://cyberbotics.com/doc/reference/robot?tab=java#wb_robot_set_mode)


Only getMode is listed


Hi, what is the Java variant for setMode function in Robot class?

##### David Mansolino [cyberbotics] 07/10/2019 11:26:32
You're welcome

##### Luiz Felipe 07/10/2019 11:26:25
thank you 😃

##### David Mansolino [cyberbotics] 07/10/2019 11:26:13
previous versions still require a license


Webots is open source and does not require any license since the R2018a version


Hi

##### Luiz Felipe 07/10/2019 11:25:27
Hello everyone, the webots from the 2019 version is open source. Does a license is still required for older version of webots? Or they are also open source from nwo on?

##### Olivier Michel [cyberbotics] 07/01/2019 06:29:15
Hi, you can make a screenshot of this page, you are free to use this for your presentations. Or you can link your presentation directly to this web page.

##### yacinedz6 06/28/2019 17:45:03
hi, how can i use or download the khepera 4 presentation in the photo to use it in presentation
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155720933146637/594221742008827921/unknown.png)
%end

##### David Mansolino [cyberbotics] 06/12/2019 06:34:10
Hi `@TH0` and `@AnnaLefay`, please feel free to report them directly on our Github repository as a new issue: [https://github.com/omichel/webots/issues](https://github.com/omichel/webots/issues)

If you want you can also fix them directly by yourself, when you are on the doc you should have a small 'Found an error? Contribute on GitHub!' link on th etop of the page.

##### AnnaLefay 06/11/2019 18:46:49
There are some in the C++ codes too I think.

##### TH0 06/11/2019 18:12:50
Hi, there are some minor bugs in the webots tutorial java codes. how do you prefere reports about these mistakes?

##### Flo 05/17/2019 10:09:25
perfect !

##### David Mansolino [cyberbotics] 05/17/2019 10:09:19
Yes, you simply need to set the 'transperency' of the display appearance to some non-null value

##### Flo 05/17/2019 10:08:49
something like they do here : [https://techcrunch.com/wp-content/uploads/2019/01/giphy-5.gif](https://techcrunch.com/wp-content/uploads/2019/01/giphy-5.gif)


Will I be able to set  the display to be semi-transparent ? So that I can still see the floor under ?

##### David Mansolino [cyberbotics] 05/17/2019 10:07:25
You're welcome

##### Flo 05/17/2019 10:07:16
ok thanks a lot David, I will check it out !

##### David Mansolino [cyberbotics] 05/17/2019 10:06:06
Alternatively you can simply set the roughness of the floor material to 0, but this will reflect only the background not dynamic object.


You can do something similar, the concept is quite simple, your floor need to be a robot node, have one camera and one display and to attache the camera to the display.


Hi, have you looked at the mirror object? [https://www.cyberbotics.com/doc/guide/object-mirror](https://www.cyberbotics.com/doc/guide/object-mirror)

##### Flo 05/17/2019 10:03:53
Dear All, is it possible in webots to make the floor reflect (like a mirror) ?

