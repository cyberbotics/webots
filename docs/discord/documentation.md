# Documentation

This is an archive of the `documentation` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m).

## 2020

##### Olivier Michel [cyberbotics] 05/13/2020 15:58:07
Thank you. We will review it soon.

##### Luftwaffel 05/13/2020 15:55:41
`@Olivier Michel`  thank you so much <3. I created a pull request

##### Olivier Michel [cyberbotics] 05/13/2020 15:54:00
You have to click the "Create pull request" button.


`@Luftwaffel`: It's here [https://github.com/cyberbotics/webots/compare/master...Simon-Steinmann:patch-1](https://github.com/cyberbotics/webots/compare/master...Simon-Steinmann:patch-1)

##### David Mansolino [cyberbotics] 05/13/2020 15:40:17
Thank you, I will check and let you know

##### Luftwaffel 05/13/2020 15:37:33
simon-steinmann

##### David Mansolino [cyberbotics] 05/13/2020 15:35:21
Let me check if I can find a stale branch, by the way what is your Github username?

##### Luftwaffel 05/13/2020 15:34:08
I submitted it April 30th, any way to check all activity? perhaps I submited it to a weird branch. Not that familiar with github


great 😩

##### David Mansolino [cyberbotics] 05/13/2020 15:26:56
> I can't find my commit

`@Luftwaffel` me neither


Did you fork the repo?

##### Luftwaffel 05/13/2020 15:26:19
I can't find my commit

##### David Mansolino [cyberbotics] 05/13/2020 15:25:07
Ok perfect, can you then open a pull-request from the branch where you did the commit so that we can review and merge it?

##### Luftwaffel 05/13/2020 15:22:20
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

##### Luftwaffel 04/30/2020 10:54:50
submitted it

##### David Mansolino [cyberbotics] 04/30/2020 10:47:24
No we are using directly the Github issue mechanism: [https://github.com/cyberbotics/webots/issues](https://github.com/cyberbotics/webots/issues)
In particular for feature request:
[https://github.com/cyberbotics/webots/issues/new?template=feature\_request.md](https://github.com/cyberbotics/webots/issues/new?template=feature_request.md)

##### Luftwaffel 04/30/2020 10:40:04
do you have something like a trello?


where would I propose that?

##### Olivier Michel [cyberbotics] 04/30/2020 10:37:09
😁 . But feel free to go ahead with these good idea and propose an implementation with a PR. That shouldn't be very difficult.

##### Luftwaffel 04/30/2020 10:36:03
A frustrated coder is full of good ideas 😄

##### Olivier Michel [cyberbotics] 04/30/2020 10:35:08
Yes, that seems to be a good idea.

##### Luftwaffel 04/30/2020 10:34:52
most commonly used in ROS and any 3D application. would really be helpfull


perhaps add get\_orientation\_quaternion while you're at it 😉


get\_orientation\_matrix


perhaps adding a function? Would be the non destructive way

##### Olivier Michel [cyberbotics] 04/30/2020 10:28:36
Yes, we may consider changing it on the develop branch.

##### Luftwaffel 04/30/2020 10:26:50
but I guess changing that would break existing code


oh btw, it's kinda weird that get\_orientation returns a 1x9 list, instead of 3x3

##### Olivier Michel [cyberbotics] 04/30/2020 10:25:15
OK, looking forward to review it.

##### Luftwaffel 04/30/2020 10:24:56
solved it differently


added it

##### Olivier Michel [cyberbotics] 04/30/2020 10:19:11
Great. It's the `%spoiler` keyword.

##### Luftwaffel 04/30/2020 10:18:40
I think I got it 🙂

##### Olivier Michel [cyberbotics] 04/30/2020 10:18:13
Something like the **Reminder** and **Tips** here: [https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot#sensors](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot#sensors) ?


Yes, that seems to be a good idea. Let me search how to do that...

##### Luftwaffel 04/30/2020 10:15:29
a 'expandable box' would be great, no idea how to implement that


in what way should I insert the code? Linked, directly in the description, or in a different way??


Okay will do. Feel free to change or edit it btw.

##### Olivier Michel [cyberbotics] 04/30/2020 10:07:41
That is great. Could you create a PR to add this contribution to the doc? [https://github.com/cyberbotics/webots/edit/master/docs/reference/supervisor.md](https://github.com/cyberbotics/webots/edit/master/docs/reference/supervisor.md)

##### Luftwaffel 04/30/2020 10:03:40
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

