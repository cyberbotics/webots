# Development

This is an archive of the `development` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m).

## 2019

##### Thelm76 04/25/2019 10:34:24
Hi, I don't know if this is the good place to post this, tell me if I'm wrong.

I'm actually trying to implement a full support for the e-puck's missing sensors in webots (especially for V2). 

After editing the proto file to add them in the simulation, I am now trying to implement the wifi communication by following this : [http://www.gctronic.com/doc/index.php?title=e-puck2\_PC\_side\_development#WiFi\_2](http://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development#WiFi_2)



and I'm facing two problems for now : in this piece of code (e-puck\_wifi/Wrapper.cpp) I don't understand the calibration values for the accelerometer so I can't transpose them to the gyro and the magnetometer



TripleValuesSensor *accelerometer = DeviceManager::instance()->accelerometer();

  if (accelerometer->isEnabled()) {

    const double calibration\_k[3] = {-9.81 / 800.0, 9.81 / 800.0, 9.81 / 800.0};

    const double calibration\_offset = -2000.0;

    double values[3];

    for (int i = 0; i < 3; i++) {

      short int acc = sensor\_data[2 * i] + 256 * sensor\_data[2 * i + 1];

      values[i] = calibration\_k[i] * (acc + calibration\_offset);

    }

    wbr\_accelerometer\_set\_values(accelerometer->tag(), values);

  }


also, on the wiki, I've seen that the accelerometer and the gyro are encoded on 6 bytes so 2 bytes for X, Y and Z. But the magnetometer is encoded on 12 bytes, which doesn't make sense to me as the raw values are between -32460 and 32760... So are there 3 axis encoded on 4 bytes, or am I missing something?

##### Olivier Michel [cyberbotics] 04/25/2019 11:19:50
Hi, welcome.


I would propose you to open an issue on [https://github.com/omichel/webots/issues](https://github.com/omichel/webots/issues) to follow-up with this discussion.


Eventually, you will be able to open a pull request to merge your contribution.

##### Arsalan.b.r 06/04/2019 07:42:35
hello

##### Fabien Rohrer [Moderator] 06/04/2019 07:42:51
Hi again ^^

##### Arsalan.b.r 06/04/2019 07:42:56
i want to model a Tractor trailer mobile robot


its a chained combination of wheeled robots


how can i do it simply?


is there such a robot in webots?

##### Fabien Rohrer [Moderator] 06/04/2019 07:45:40
First of all it's possible to do so. There are even several possibilities to achieve this.


Is it the same robot replicated several time? How is the link between the robots? A HingeJoint?


Do you have a screenshot of a real setup?

##### Arsalan.b.r 06/04/2019 07:47:13
a passive joint between carts

##### Fabien Rohrer [Moderator] 06/04/2019 07:47:48
I think the closest robot we have in Webots yet is the yamor:


[https://cyberbotics.com/doc/guide/yamor](https://cyberbotics.com/doc/guide/yamor)

##### Arsalan.b.r 06/04/2019 07:49:01

%figure
![tractor-trailer.JPG](https://cdn.discordapp.com/attachments/565155651395780609/585374436270800906/tractor-trailer.JPG)
%end

##### Fabien Rohrer [Moderator] 06/04/2019 07:49:15
The robot model is encapsulated in a PROTO node. The Robots are linked with Connector nodes:


[https://cyberbotics.com/doc/reference/connector](https://cyberbotics.com/doc/reference/connector)

##### Arsalan.b.r 06/04/2019 07:50:21
it should be wheeled

##### Fabien Rohrer [Moderator] 06/04/2019 07:50:29
I think you could achieve something by studying yamor.wbt

##### Arsalan.b.r 06/04/2019 07:50:30
like tractor and trailer


yamor is like snake?


do you have a picture of that?

##### Fabien Rohrer [Moderator] 06/04/2019 07:51:07
Yes, but adding wheels to the yamor module is not an issue


This other example may interest you:


[https://www.cyberbotics.com/doc/automobile/car#heavy-weights](https://www.cyberbotics.com/doc/automobile/car#heavy-weights)


The trucks trailer are probably closer from your expectations.

##### Arsalan.b.r 06/04/2019 07:54:14
yes


thanks again

##### Fabien Rohrer [Moderator] 06/04/2019 07:56:42
But first of all, please do our tutorial 😉 [https://cyberbotics.com/doc/guide/tutorials](https://cyberbotics.com/doc/guide/tutorials)

##### TH0 06/12/2019 13:54:02
i tried to build webots from source under windows 10, but i got this error:



nodes/utils/WbVirtualRealityHeadset.cpp:43:10: fatal error: openvr.h: No such file or directory

   43 | #include <openvr.h>

      |          ^~~~~~~~~~

compilation terminated.

make[1]: *** [Makefile:669: WbVirtualRealityHeadset.o] Error 1





i followed the installation guide step by step ( [https://github.com/omichel/webots/wiki/Windows-installation](https://github.com/omichel/webots/wiki/Windows-installation) )

any ideas? just cloning the valvesoftware openvr sdk does not seem to work. 



is it a bug in the webots tutorial or did i miss anything?

##### David Mansolino [cyberbotics] 06/12/2019 13:55:04
this dependency should be downloaded automatically. Can you check the content of your 'dependencies' folder ?

##### TH0 06/12/2019 13:57:29
$ ls dependencies/

Cyberbotics.Webots.Mingw64.Libraries.manifest  libpico.zip  lua-gd-windows.zip  Makefile.mac      openvr-1.0.7

libOIS.zip                                     lua-5.2.3    Makefile.linux      Makefile.windows

##### David Mansolino [cyberbotics] 06/12/2019 13:58:15
Ok, perfect, you have it the includes are located in the 'openvr-1.0.7' directories.


Just to make sure, you did follow all the instructions step by step from [https://github.com/omichel/webots/wiki/Windows-installation](https://github.com/omichel/webots/wiki/Windows-installation) right ?

##### TH0 06/12/2019 13:58:48
yes


and there where no errors until the one i pasted here

##### David Mansolino [cyberbotics] 06/12/2019 13:59:16
Ok, let's first try something simple, can you do a 'make cleanse' from the 'webots' folder?


(this will clean the 'dependencies' folder so that you ca retry from a clean install)

##### TH0 06/12/2019 14:00:15
done


should i send the output in the chat?

##### David Mansolino [cyberbotics] 06/12/2019 14:00:36
Ok, can you retry to do a 'make'


if they was no error its not needed to send the output

##### TH0 06/12/2019 14:03:46
ok, same error after make -j6


is there some trick in discord to send code in a more compact way in the chat? 😄


like a block or something like that

##### David Mansolino [cyberbotics] 06/12/2019 14:04:52
Not really, but you can send gist link: [https://gist.github.com/](https://gist.github.com/)

##### TH0 06/12/2019 14:05:53
ok, how ever, is python and npm needed or only optional?

##### David Mansolino [cyberbotics] 06/12/2019 14:06:32
python is higly recommended (even so it is not required to compile the core of Webots).


Npn is completely optionnal


if you want to have npn install (and some other optionnal dependencies) you can install them by running './src/install\_scripts/msys64\_installer.sh --dev' from the webots directory

##### TH0 06/12/2019 14:07:33
ok, but i think, its not the solution for the openvr problem, right?

##### David Mansolino [cyberbotics] 06/12/2019 14:08:13
It should not impact openvr, but it is worse trying


what is the output of your make -j6 command ?

##### TH0 06/12/2019 14:09:46
first of all:

Thomas@DESKTOP-O5VO47I MSYS /C/msys64/home/Thomas/webots

$ find . -name openvr.h

./dependencies/openvr-1.0.7/headers/openvr.h

./include/openvr/headers/openvr.h



Thomas@DESKTOP-O5VO47I MSYS /C/msys64/home/Thomas/webots

$


[https://0bin.net/paste/DeUwrdi11b19n91y#XDrDCDC3pOVgGQ3a1vqIJNM6k7hJ0j1e1qgjBpNhMQh](https://0bin.net/paste/DeUwrdi11b19n91y#XDrDCDC3pOVgGQ3a1vqIJNM6k7hJ0j1e1qgjBpNhMQh)

##### David Mansolino [cyberbotics] 06/12/2019 14:15:06
The missing headers should be located in '/C/msys64/home/Thomas/webots/openvr-1.0.7/headers' after the make -j6


From the log you sent it seems they are there, can you confirm ?

##### TH0 06/12/2019 14:16:38
yes, its there


sorry for the confusion with the find command, i updated the message from above

##### David Mansolino [cyberbotics] 06/12/2019 14:18:54
Ok, no problem, then it seems they are correctly downloaded and then copied in the 'include' fodler

##### TH0 06/12/2019 14:19:04
right


seems like an error in the makefile (missing include directory?)

##### David Mansolino [cyberbotics] 06/12/2019 14:20:22
Yes, indeed, can you try changing in 'src/webots/Makefile:

OPEN\_VR\_INCLUDE = -isystem $(WEBOTS\_PATH)/include/openvr

Into

OPEN\_VR\_INCLUDE = -isystem $(WEBOTS\_PATH)/include/openvr/headers

and try to recompile ?

##### TH0 06/12/2019 14:20:46
😄


👌


look good so far

##### David Mansolino [cyberbotics] 06/12/2019 14:26:49
Perfect, we will fix this in our repo !

##### TH0 06/12/2019 14:30:43
btw.: great that webots is open source now!


make is completed and was successful!

##### David Mansolino [cyberbotics] 06/12/2019 14:31:54
Perfect !

##### TH0 06/12/2019 14:32:30
time for another question? 😉


i think the vr-function in webots is broken. i tried to view a simulation with the htc vive and steam vr but only a blue background appers in the view. when i deactivate position and rotation tracking in the webot gui, a coordinate system (webots style) is displayed in the vive in the center of the display, but no other objects from the scene)

##### David Mansolino [cyberbotics] 06/12/2019 14:36:33
That's strange we did test it a few weeks ago and it was working fine. Unfortunately we don't have any VIve available right now to test.


Where you able to view the simulation in Webots moving when you were moving the headset ?

##### TH0 06/12/2019 14:37:41
i'm using the webots 2019a Rev 1 from some weeks ago. is there a newer version?

##### David Mansolino [cyberbotics] 06/12/2019 14:39:55
Not yet, but we will release R2019b soon. Unfortunately I don't have the time right now, but if you test with R2019b when we will release it and the problem is still present, please do not hesitate to tell us.

##### TH0 06/12/2019 14:40:28
ok


when i move the headset, nothing happens


only a light blue/grey background color

##### David Mansolino [cyberbotics] 06/12/2019 14:41:33
That's indeed not normal, but we changed some part of the VRHeadset code for the next release, so please try again.

##### TH0 06/12/2019 14:41:36
whats the world you tested the VR?

##### David Mansolino [cyberbotics] 06/12/2019 14:42:11
Many of them, but I remember I tested I lot the 'robotis/soccer.wbt' world

##### TH0 06/12/2019 14:42:28
ok, thats the one i used too


i'm not sure, but isn't the version i just build from webots the 2019b you mean?

##### David Mansolino [cyberbotics] 06/12/2019 14:48:16
It depends on which git branch you are ?

##### TH0 06/12/2019 14:50:14
on the develop


i just tested the vive in the 2019b, same problem: colored background but no objects


world file was "nao\_robocup.wbt"

##### David Mansolino [cyberbotics] 06/12/2019 14:53:18
Then if you are using the 'develop' branch you are indeed very close to the R2019b version 😕

##### TH0 06/12/2019 14:54:44
ok. the color shown in the vive is the same as the "/Background/SkyColor" (i changed it to red to test it)


maybe a near-clipping problem because of different scales?

##### David Mansolino [cyberbotics] 06/12/2019 15:01:22
At least if shows that the connexion with the Vive is correctly established

##### TH0 06/12/2019 15:01:42
exact

##### David Mansolino [cyberbotics] 06/12/2019 15:02:05
Yes, maybe near-clip, or maybe a resolution issue. can you try setting the VIve resolution to 50% (you can set this in the preference of SteamVR). and then reboot Webots.

##### TH0 06/12/2019 15:04:17
set to 50%, same issue


interessting: when i click on some objects in the treeview, the coordinate systems are shown in the vive



%figure
![webotsVR01.png](https://cdn.discordapp.com/attachments/565155651395780609/588384559884926987/webotsVR01.png)
%end

##### David Mansolino [cyberbotics] 06/12/2019 15:10:35
Can you see the handle in the headset too?

##### TH0 06/12/2019 15:11:17
no handles


and the position is not tracked, but the rotation from the vive


(but i set both options to true in the webots gui)


oh, and i can rotate the cameraview per mouse while vr is on

##### David Mansolino [cyberbotics] 06/12/2019 15:23:28
that's very strange, you should see exactly the same in the Webots view and in one of the two eyes in the headset. Probably there is something not clean with OpenGL on our side.

##### TH0 06/12/2019 15:27:23
the vive preview in webots and what i see in the vive is the same

##### David Mansolino [cyberbotics] 06/12/2019 15:27:56
Ok, so you can see the handle in the preview but not in the headset ?

##### TH0 06/12/2019 15:28:37
just to be sure: you mean with "handles" the two controllers from the vive?


i do not see the rendered vive controllers in both the webots preview and in the vive

##### David Mansolino [cyberbotics] 06/12/2019 15:29:38
No sorry, I mean the red,green and blue arrows you can see in Webots.

##### TH0 06/12/2019 15:30:03
yes, i also see them in the vive (its exacly the same image)

##### David Mansolino [cyberbotics] 06/12/2019 15:30:04
> i do not see the rendered vive controllers in both the webots preview and in the vive



that's normal we did not yet interfaced the vive controller with webots.

##### TH0 06/12/2019 15:30:26
sorry, english is not my mother language, i thought you mean the vive handles (controllers)

##### David Mansolino [cyberbotics] 06/12/2019 15:30:31
Ok, than this make more sense, so the problem is really what is rendered in Webots

##### TH0 06/12/2019 15:30:40
yes


i think so

##### David Mansolino [cyberbotics] 06/12/2019 15:30:44
> sorry, english is not my mother language, i thought you mean the vive handles (controllers)



No problem that was not clear from my side too.

##### TH0 06/12/2019 15:31:10
the good news: you can simulate the behaviour without a vive, right?

##### David Mansolino [cyberbotics] 06/12/2019 15:31:11
Can you try playing with the Viewpoint, near and far value ?


> the good news: you can simulate the behaviour without a vive, right?



Kind of, but still the vive API in itself interfere with OpenGL and this is maybe the cause of the problem.

But as soon as I have time I will try to reproduve this.

##### TH0 06/12/2019 15:35:50

%figure
![webotsVR02.png](https://cdn.discordapp.com/attachments/565155651395780609/588391019444174858/webotsVR02.png)
%end



%figure
![webotsVR03.png](https://cdn.discordapp.com/attachments/565155651395780609/588391025089708196/webotsVR03.png)
%end


see difference far 0 (default) vs. far 1


that means for me, clipping is not the problem, right?

##### David Mansolino [cyberbotics] 06/12/2019 15:36:30
Can you try increasing it to something like 100 ?

##### TH0 06/12/2019 15:36:44
same result with 100

##### David Mansolino [cyberbotics] 06/12/2019 15:36:45
(value are in meters)


Ok, then yes it doesn't seems it is clipping the problem, but rather an issue with OpenGL


By the way what is your GPU ?

##### TH0 06/12/2019 15:37:26
0 means infinity?


P3200 mobile

##### David Mansolino [cyberbotics] 06/12/2019 15:40:46
> 0 means infinity?



for far yes


We haven't tested it with this GPU, but it should works fine.

Do you know which OpenGL version you are using?

Are other application working fine with the Vive headset?

##### TH0 06/12/2019 15:44:12
yes other apps work fine and i use the vive often


OpenGL 4.6 (415 ext) - Quadro P3200/PCIe/SSE2

##### David Mansolino [cyberbotics] 06/12/2019 15:50:20
Ok, then everything is perfect with your hardware. I will check when I have time and let you know

##### TH0 06/12/2019 15:51:23
ok, good luck!

##### David Mansolino [cyberbotics] 06/12/2019 15:52:00
Thank you

##### BlackPearl 06/19/2019 08:50:43
I think we found a Bug?
%figure
![image0.jpg](https://cdn.discordapp.com/attachments/565155651395780609/590825780716634132/image0.jpg)
%end



%figure
![image0.jpg](https://cdn.discordapp.com/attachments/565155651395780609/590826051312025620/image0.jpg)
%end

##### Fabien Rohrer [Moderator] 06/19/2019 08:52:39
Hi


It seems your controller called SensorController crashes.


A priori, this bug is on your side.


Do you have any way to debug your controller?

##### BlackPearl 06/19/2019 18:15:12
Actually we tried today different controllers but always getting this error

##### David Mansolino [cyberbotics] 06/20/2019 06:54:45
`@BlackPearl` do you get this error with the controllers distributed within Webots?

##### BlackPearl 06/20/2019 13:54:10
Yes


That’s why we created our own but still get this error

##### David Mansolino [cyberbotics] 06/20/2019 14:05:24
`@BlackPearl` which version of Webots and which version of Java are you using ?

##### BlackPearl 06/21/2019 04:11:49
The latest (java 8 and webots R2019-R1)

##### David Mansolino [cyberbotics] 06/26/2019 06:15:07
`@BlackPearl` can you please try with the latest version of Webots that we releeased yesterday: [https://github.com/omichel/webots/releases/latest](https://github.com/omichel/webots/releases/latest)

##### BlackPearl 06/28/2019 05:00:03
`@David Mansolino`  thanks, will try

##### Deleted User 07/20/2019 10:21:57
`@BlackPearl` hello sir, are you using nao ?

##### BlackPearl 08/21/2019 09:42:49
`@David Mansolino`  so we are using the new version but we got the controller crash again


Anything we need to pay attention?


It’s a java controller

##### Fabien Rohrer [Moderator] 08/21/2019 09:48:29
How did you installed Java? Your log seems to refer to OpenJDK while we recommend to use the Oracle JDK: [https://cyberbotics.com/doc/guide/using-java#windows](https://cyberbotics.com/doc/guide/using-java#windows)


(the java libcontroller is precompiled with the last 8 version of the Oracle JDK)

##### BlackPearl 08/21/2019 09:50:26
`@Fabien Rohrer`  oh ok thanks for the quick answer


We Will change to the oracle jdk

##### Fabien Rohrer [Moderator] 08/21/2019 09:51:44
oops, I think I'm wrong: I was sure you were on windows.

##### BlackPearl 08/21/2019 09:52:04
No we are running Linux


But yes we are using the openJDK

##### Fabien Rohrer [Moderator] 08/21/2019 09:52:20
on linux, OpenJDK is supposed to work,

##### BlackPearl 08/21/2019 09:53:19
Hm so why the error accurs

##### Fabien Rohrer [Moderator] 08/21/2019 09:56:07
not sure. could you give us more precisely your Java version (`java -version`)? It is supposed to be `1.8`. Does it matches with your `javac -version`?

##### BlackPearl 08/21/2019 09:57:46

%figure
![image0.png](https://cdn.discordapp.com/attachments/565155651395780609/613673091335323648/image0.png)
%end

##### Fabien Rohrer [Moderator] 08/21/2019 10:01:29
ok, this is certainly the issue


We are precompiling the Java library with OpenJDK 1.8.0\_222.


What OS are you using?  arch?


You should either downgrade your Java, or either recompile the library with your Java version (normally it's quite simple, just a matter to install `swig`, define `WEBOTS_HOME` and type `make` in `WEBOTS_HOME/resources/languages/java`)

##### BlackPearl 08/21/2019 10:07:58
Unbuntu


*ubuntu


Hm ok

##### Fabien Rohrer [Moderator] 08/21/2019 10:09:36
Let me know if you have troubles with this.

##### BlackPearl 08/21/2019 10:10:07
Ok thank you very much. We are downgrading

##### Fabien Rohrer [Moderator] 08/21/2019 10:10:23
(it's probably the simplest solution indeed)

##### MerySgh 08/21/2019 10:13:23
hello everyone !  has anyone have developed jamming attack in webot  before ?

##### Fabien Rohrer [Moderator] 08/21/2019 10:15:38
Hi, Webots is mainly a Desktop app. A jamming attack has not much sense to me. Do you speak about the streaming sever? Our websites?


Do you have something else in mind?

##### MerySgh 08/21/2019 10:20:57
mmm actually !! I'm supposed to develop  jamming attack in my project ... for more clarifying I have a project of autonomous vehicles and I have to make an accident with a passenger in the road according to jamming attack

##### BlackPearl 08/21/2019 10:21:14
`@Fabien Rohrer` it’s still not working ....

##### Fabien Rohrer [Moderator] 08/21/2019 10:21:46
`@MerySgh` haha makes more sense

##### MerySgh 08/21/2019 10:21:55
I  made the accident but how to develop the attack I do not know yet

##### Fabien Rohrer [Moderator] 08/21/2019 10:22:53
`@BlackPearl` so you did downgraded to java and javac 1.8 and clean/build your controller again?


`@MerySgh` are you using SUMO to manage the traffic?

##### BlackPearl 08/21/2019 10:24:53
`@Fabien Rohrer` one minute. We are restarting everything again

##### MerySgh 08/21/2019 10:25:45

> **Attachment**: [city\_2.mp4](https://cdn.discordapp.com/attachments/565155651395780609/613680130568617984/city_2.mp4)


sumo ? what do  you mean ?

##### Fabien Rohrer [Moderator] 08/21/2019 10:28:17
If I understand well, you need to simulate a traffic jam after the collision, right?


In Webots, you can simulate vehicle traffic using SUMO (it's an open-source application which simulates vehicle traffic and which can be interfaced with Webots):


Please take a look at this link and the movie: [https://cyberbotics.com/doc/automobile/sumo-interface](https://cyberbotics.com/doc/automobile/sumo-interface)


Could such traffic simulation meet your expectations?

##### MerySgh 08/21/2019 10:39:32
ok i'm going to see it .. i think i'm not clear enough ... ok the car made an accident with pedestrian because it's attacked from different attacks 

thanks to this attack that the car does not see the passenger crossing the road in my case the attack is jamming

##### BlackPearl 08/21/2019 10:42:31
It still doesn’t work `@fa`

##### Fabien Rohrer [Moderator] 08/21/2019 11:30:38
`@MerySgh` Ok, I understand more and more your scenario. In my knowledge, I'm not aware of similar projects in the Webots community. Do you have specific questions?


`@BlackPearl` Sorry to read this, the next step could be to test if the Java examples provided in Webots are working smoothly or not. Could you simply open WEBOTS\_HOME/projects/languages/java/worlds/example.wbt ?

##### MerySgh 08/21/2019 11:36:36
`@Fabien Rohrer` No thank you so much 🙂

##### Derek 09/10/2019 01:23:04
How can I add some joints in a robot by GUI operation?

##### David Mansolino [cyberbotics] 09/10/2019 06:06:06
Hi `@Derek`, I would recommend to follow our tutorial, in particular this one will explain how to add 4 joints (connected to 4 wheels) to a robot: [https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)


Do you have a particular robot in mind ?

##### SimonDK 09/15/2019 14:31:33
What models for omni-directional mecanum wheel platforms have you seen/are there in Webots? How are the mecanum wheels simulated? I would like to develop my own simulation for a platform so are looking for some examples as a starting point.

##### Stefania Pedrazzi [cyberbotics] 09/16/2019 06:10:23
Hi `@SimonDK`, the youBot robot has mecanum wheels [https://www.cyberbotics.com/doc/guide/youbot](https://www.cyberbotics.com/doc/guide/youbot)

The mecanum wheel is simulated by setting asymmetric friction  to the [ContactProperties]([https://www.cyberbotics.com/doc/reference/contactproperties](https://www.cyberbotics.com/doc/reference/contactproperties)) of the wheel.


You can find another example in samples/howto/omni\_wheels.wbt simulation:  [https://www.cyberbotics.com/doc/guide/samples-howto#omni\_wheels-wbt](https://www.cyberbotics.com/doc/guide/samples-howto#omni_wheels-wbt). In this case the wheel is simulated using two layers of joints and cylinders.

##### Anoop 09/25/2019 01:27:46
Does webots include this marine robots, exact this one? I can find salamander closest to it?
%figure
![FluidRobot.png](https://cdn.discordapp.com/attachments/565155651395780609/626228320996032512/FluidRobot.png)
%end

##### Stefania Pedrazzi [cyberbotics] 09/25/2019 06:13:39
Hi `@Anoop`, yes this robot is included in Webots. You can find it in the simulation `projects/samples/rendering/worlds/animated_skin.wbt`

[https://www.cyberbotics.com/doc/guide/samples-rendering#animated\_skin-wbt](https://www.cyberbotics.com/doc/guide/samples-rendering#animated_skin-wbt)


Just note that the `Skin` node used to implement the skin animation is still experimental, i.e. it can be used but it is not documentated as the functionality could still change.

##### Anoop 09/25/2019 17:45:20
Thanks `@Stefania Pedrazzi` for the information. Can I assume this as a soft robotics or more generally can SKIN node be used for generating soft robotics model?

##### Stefania Pedrazzi [cyberbotics] 09/26/2019 06:20:06
`@Anoop` The `Skin` node is a pure graphical functionality that only modifies the mesh of the robot but doesn't affect the physics.

Webots cannot simulation soft robotics because ODE (the Webots physics engine) doesn't support soft body dynamics.

However there are some tricks to transform a soft robot model into an hard model that could help in simulating soft robotics application.

##### ThePolymath 09/29/2019 10:08:43
how to integrate webots with an ide in macOS?

##### Fabien Rohrer [Moderator] 09/30/2019 06:35:35
`@ThePolymath` Hi, I expect you mean to create a webots controller inside an IDE, such as XCode. Could you refer to this draft page of the documentation? [https://cyberbotics.com/doc/guide/using-your-ide?version=enhancement-ide-section](https://cyberbotics.com/doc/guide/using-your-ide?version=enhancement-ide-section)

##### David Mansolino [cyberbotics] 09/30/2019 06:40:07
We also have an example with PyCharm here: [https://www.cyberbotics.com/doc/guide/using-pycharm-with-webots](https://www.cyberbotics.com/doc/guide/using-pycharm-with-webots)

##### pavlos27t 10/17/2019 14:39:46
Hello Sir, i am trying to understand supervisor programming but i found only this example in C : [https://www.cyberbotics.com/doc/guide/supervisor-programming](https://www.cyberbotics.com/doc/guide/supervisor-programming) , could you give me something similar in python?I'm trying this on a robot controller: from controller import * ;robot = Robot();robot.getFromDef(name =  "robotname")  ---> and i get error robot object has no attribute getFromDef , but i have supervisor field :TRUE on the robot


if i change robot = Robot() --> robot = Supervisor() , then webots crashes when a supervisor unction called

##### Fabien Rohrer [Moderator] 10/17/2019 14:42:43
I confirm you should use Supervisor() instead of Robot() to access the Supervisor API


Are you sure that "robotname" matches with an existing DEF name?


>>> DEF robotname Robot {}

##### pavlos27t 10/17/2019 14:43:31
yes because it crashes on every function

##### Fabien Rohrer [Moderator] 10/17/2019 14:44:01
Ok. Could you post the crash error here?

##### pavlos27t 10/17/2019 14:44:38
sorry i don't know how to post a crash error

##### Fabien Rohrer [Moderator] 10/17/2019 14:45:24
Normally, when a Python controller crash, something is displayed in the Webots console. Simply copy-paste it in this chat.


(or a screenshot ? 😉 )

##### pavlos27t 10/17/2019 14:50:35
sorry it doesn't crash now , everything works fine


😅

##### Fabien Rohrer [Moderator] 10/17/2019 15:13:45
I'm happy to read this 😉

##### threeal 10/20/2019 18:45:28
anybody ever try to use webots with ros?


and why use webots over gazebo?

##### SimonDK 10/20/2019 18:53:29
`@threeal` Going to try Webots with ROS very soon. At least for me I have grown very tired of Gazebo. It always feels unstable, cumbersome to set up and we have to use many hacks to get it to work somewhat ok with Reinforcement Learning algorithms. I hope it will be easier to speed up simulations and build simulations with Webots. Let's see 🙂

##### Flo 10/20/2019 19:09:40
`@SimonDK`  I build a gym environnement for webots. Its not released yet but let me know. If you need it for your RL I can accelerate the release

##### threeal 10/20/2019 19:11:53
anyway, is it possible to treat webots controller as ros node?

##### SimonDK 10/21/2019 04:53:16
`@Flo` sounds very interesting, will PM you

##### David Mansolino [cyberbotics] 10/21/2019 06:53:31
`@threeal`, Webots has several advantages compared to Gazebo, here is a non-exhaustive list:

  - Cross-platform [windows, linux, mac].

Stable physics engine.

  -An efficient rendering engine using Physically Based Rendering for realistic images.

  -A simple and intuitive user interface.

  -Wide range of simulated sensors and actuators available and ready to work.

  -Wide range of robot models available and ready to work.

  -Wide range of documented samples.


> anyway, is it possible to treat webots controller as ros node?



Yes of course, we have developped an interface with ROS1 which is a controller that acts as a generic bridge between Webots and ROS1: [http://wiki.ros.org/webots\_ros/](http://wiki.ros.org/webots_ros/)



And we are currently creating an interface with ROS2, for this new version we are using the Webots API directly in the ros nodes: [http://wiki.ros.org/webots\_ros2](http://wiki.ros.org/webots_ros2)



Let us know if you have any precise question


`@Flo`, that sounds very interesting, looking forward to see this too!


`@SimonDK` :

> Going to try Webots with ROS very soon. At least for me I have grown very tired of Gazebo. It always feels unstable, cumbersome to set up and we have to use many hacks to get it to work somewhat ok with Reinforcement Learning algorithms. I hope it will be easier to speed up simulations and build simulations with Webots. Let's see



For sur it will 😉 let us know if you have any issues doing so.

##### threeal 10/23/2019 10:31:58
`@David Mansolino` thank you for the answer, but is it possible to create a custom robot in Webots? As my robot is custom made and not based on sample robot provided by the Webots simulator.



And what about the ROS controller provided by Webots? can i use it in my custom made robot or should i create custom made controller for my robot that work as bridge between ROS and Webots?.

##### Olivier Michel [cyberbotics] 10/23/2019 10:33:18
Of course you can create your custom robot in Webots.


See this tutorial: [https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)


You can use the ROS controller provided in Webots with your own new robot design. There is no need to modify it. You will simply have to figure out the topics and services published by it.


See details here: [https://cyberbotics.com/doc/guide/using-ros#standard-ros-controller](https://cyberbotics.com/doc/guide/using-ros#standard-ros-controller)

##### SimonDK 10/25/2019 17:24:39
`@Flo` I sent you a PM, did you receive it? /cheers

##### threeal 11/02/2019 00:48:58
how can i launch python controller in webots?

##### Stefania Pedrazzi [cyberbotics] 11/04/2019 07:08:37
Hi `@threeal`, usually controllers are automatically started by Webots when you run the simulation if the is used by any robot node, i.e. the Robot.controller field is set to your python controller name. Does this not work for you?

##### pavlos27t 11/04/2019 13:16:17
Hello Sir, i have a strange problem: i give equal speed on left and right wheel of e-puck robot but it doesn't move linear, i print my velocities to confirm it and they are equal ,so why my robot turns?

##### David Mansolino [cyberbotics] 11/04/2019 13:17:21
Hi `@pavlos27t`, just to make sure you are speaking about simulation right? Does the robot collide with something? is the ground flat?

##### pavlos27t 11/04/2019 13:21:26
the ground is flat, but there is a possiblity for the e-puck robot to collide with another e-puch robot that i have in the simulation


i reset my robots like this [http://prntscr.com/ps9cqr](http://prntscr.com/ps9cqr)


if collision happens and forces applying should i reset the forces too?

##### David Mansolino [cyberbotics] 11/04/2019 13:26:07
you probably want to reset the physics of the robot too indeed: wb\_supervisor\_node\_reset\_physics

##### pavlos27t 11/04/2019 13:33:23
right thanks i didn't know how to reset physics..

##### David Mansolino [cyberbotics] 11/04/2019 13:33:39
You're welcome

##### pavlos27t 11/06/2019 18:54:05
hello again , i resetphysics but some forces still applying after collision



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/641711950677409813/unknown.png)
%end

##### David Mansolino [cyberbotics] 11/07/2019 07:58:34
`@pavlos27t` you should make sure that the wheels are not colliding the floor when you move the robot. It is also recommended to check what is happenning with the contact points just before and just after the reset, you can see this in the view / Optional Rendering menu.

##### pavlos27t 11/07/2019 10:41:35
how can i make sure the wheels are not colliding the floor?

##### David Mansolino [cyberbotics] 11/07/2019 16:16:08
By checking the contact points you can see if it does collide with the floor. If you don't see any contact points then it means that it is not colliding: [https://cyberbotics.com/doc/guide/the-user-interface#view-menu](https://cyberbotics.com/doc/guide/the-user-interface#view-menu)

##### pavlos27 11/07/2019 18:16:14
i see contact points with the floor but what that means?my robot can't be in the air


it will always colliding with the floor


i have reset the physics of the floor too, but still i have the same problem


i don't know what to do to fix that do you have any ideas?

##### Fabien Rohrer [Moderator] 11/11/2019 07:20:24
`@pavlos27` Would it be possible to translate it at the floor level, but to add a very small vertical offset, just in order to avoid collisions with the floor during the translation?

##### pavlos27 11/13/2019 11:20:07
unfortunately this didn't worked either

##### Fabien Rohrer [Moderator] 11/13/2019 12:29:50
Could you better describe what does not work?

##### pavlos27 11/13/2019 12:46:06
i have 2 robots and a rectangle arena . The robots are moving only in one linear direction  one behind the other and sometimes they are colliding . When i reset my robots to their initial position like this



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/644156174123532351/unknown.png)
%end

##### Fabien Rohrer [Moderator] 11/13/2019 12:47:25
it sounds very correct to me.


(the floor.resetPhysics() is certainly useless, because a floor has no physics)

##### pavlos27 11/13/2019 12:48:28
but still collisions forces are applied after reset


but this doesn't happen at first reset


this happens after 50-60 resets

##### Fabien Rohrer [Moderator] 11/13/2019 12:49:48
I fear that some error is accumulated.


did you try to revert or reload the whole simulation instead?

##### pavlos27 11/13/2019 12:52:34
no i will try

##### Fabien Rohrer [Moderator] 11/13/2019 12:54:00
I would go for this function first: [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_simulation\_reset](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_simulation_reset)

##### Ella 11/19/2019 16:04:03
Hello~ I am a real beginner of Webot!!  I want to use a* algorithm  boxes as obstacles and find the path!! but it's really hard,,,, can you give me some tips where to start??

##### Fabien Rohrer [Moderator] 11/19/2019 16:05:16
`@Ella` Hi, I confirm this task may be hard. There is currently no example about such implementation.


If I were you, I would first use an existing library to deal with the algorithm, and a predefined map stored in the controller. Webots can retrieve easily the robot position in the map.


Does this answer your question?


The objects of a scene and their size may be retrieved using a supervisor, like this:


[https://gist.github.com/fabienrohrer/543c2a9751eefc6d352e6957e6e3dc90](https://gist.github.com/fabienrohrer/543c2a9751eefc6d352e6957e6e3dc90)


But using a predefined map to start is certainly simpler.

##### Ella 11/19/2019 16:16:53
`@Fabien Rohrer`  Thank you so much!!! Maybe I should try step by step!!!

##### Fabien Rohrer [Moderator] 11/19/2019 16:38:52
you're welcome, good luck with your project 🙂

##### Dorteel 11/20/2019 09:30:13
Hi guys! Does anybody here have a model for converting the two wheel velocities of a differential-drive robot to the robot's angular and linear velocity?

##### Fabien Rohrer [Moderator] 11/20/2019 09:31:16
`@Dorteel` Hi, it's a matter to compute the robot odometry, usually using simple trigonometric functions.

##### Dorteel 11/20/2019 09:32:31
Thanks `@Fabi`en! I think I found something online!

##### SimonDK 11/27/2019 17:01:08
Regarding the Webots graphical interface, I was wondering if there are any plans to have somethings like groups or folders in the Scene Tree, or being able to drag'n'drop to reorganize items? In big simulations you easily lose overview of all items 😀

##### David Mansolino [cyberbotics] 11/28/2019 07:33:06
Yes we have some long-term plans to improve the scene-tree in Webots, but those are long term plans and will not be implemented in a near future (unfortunately we have currently more urgent things to improve), but if you want to try implementing this you are welcome to contribute 😉

##### SimonDK 11/28/2019 18:09:02
Great to hear it is in the long-term plans at least 😊 I get there are more urgent things. If time would allow it, I would give it a go 😁

##### Enger 11/30/2019 22:42:33
Hi Guys, I am new to Webots but I want to use it for a robot soccer simulation project. I would like to use the Robotis OP2 robot but I want to code in python. Is it possible?

##### Tahir [Moderator] 12/01/2019 01:44:29
Hi 

`@Enger` Yes it is possible


You can see a sample soccer simulation in webots demos as well


[https://cyberbotics.com/doc/guide/samples-demos](https://cyberbotics.com/doc/guide/samples-demos)

##### Musuyaba 12/03/2019 01:37:29
Hi, i get this error, 
```   11 | #include <webots/servo.h>
      |          ^~~~~~~~~~~~~~~~ 
```

and i think i need servo library, anyone have it? thankyou

##### Stefania Pedrazzi [cyberbotics] 12/03/2019 07:27:24
`@Musuyaba` the Servo node has been deprecated since many years now (starting from Webots 7.2). Please use the joint and motor nodes instead.


You can find some examples in the Webots samples library:

- samples/devices/motor.wbt ([https://cyberbotics.com/doc/guide/samples-devices#motor-wbt](https://cyberbotics.com/doc/guide/samples-devices#motor-wbt))

- samples/devices/motor2.wbt ([https://cyberbotics.com/doc/guide/samples-devices#motor2-wbt](https://cyberbotics.com/doc/guide/samples-devices#motor2-wbt))

- samples/devices/motor3.wbt ([https://cyberbotics.com/doc/guide/samples-devices#motor3-wbt](https://cyberbotics.com/doc/guide/samples-devices#motor3-wbt))

- samples/devices/linear\_motor.wbt ([https://cyberbotics.com/doc/guide/samples-devices#linear\_motor-wbt](https://cyberbotics.com/doc/guide/samples-devices#linear_motor-wbt))

##### Musuyaba 12/03/2019 08:40:33
I see, thank you, i will try it later

##### iloving 12/05/2019 01:29:59
Hi, I can not connect the Internet license server, and the site [https://www.cyberbotics.com/](https://www.cyberbotics.com/) is not attached. Anyone have it?

##### David Mansolino [cyberbotics] 12/05/2019 07:21:00
`@iloving`, we are currenlty running some migration on our server. This might take some time, in the meantime you can use the latest version of Webots which doesn't require any internet license: [https://github.com/cyberbotics/webots/releases/tag/R2019b-rev1](https://github.com/cyberbotics/webots/releases/tag/R2019b-rev1)

##### juanrh 12/06/2019 20:00:55
Hi, I'm a newby trying to use webots with Python. I'm on Fedora 30 with webots installed with snap. I did export WEBOTS\_HOME=/var/lib/snapd/snap/webots/current export PYTHONPATH="${WEBOTS\_HOME}/usr/share/webots/lib/python37"

export PYTHONIOENCODING='UTF-8'

export LD\_LIBRARY\_PATH="${LD\_LIBRARY\_PATH}:${WEBOTS\_HOME}/lib" and when I launch `python3` and do `import controller` I get "ImportError: libCppController.so: cannot open shared object file: No such file or directory"


I don't find that .so file in my webots installation. I have [juanrh@juanydell hello\_webots\_py]$ ls $WEBOTS\_HOME/lib

bindtextdomain.so  systemd  udev  x86\_64-linux-gnu


any idea what might be the problem here?


ah ok, it looks I should use `export WEBOTS_HOME=/var/lib/snapd/snap/webots/current/usr/share/webots` instead

##### Fabien Rohrer [Moderator] 12/06/2019 20:08:59
Good job! 👍

##### luoyu 12/09/2019 02:43:09
Hi, I have a problem that wrong license password for my account. Is It caused by the migration on the server？Can I know how long it will take？

##### Fabien Rohrer [Moderator] 12/09/2019 04:40:59
`@luoyu` could you try with the « webots » password? [https://cyberbotics.com/doc/guide/general-faq#can-i-still-use-a-webots-version-before-the-r2019a-release](https://cyberbotics.com/doc/guide/general-faq#can-i-still-use-a-webots-version-before-the-r2019a-release)


Last week, we simply open the license server (dealing with Webots licenses before R2019) as Webots is open source since one year.

##### luoyu 12/09/2019 07:09:40
Thank you very much. The magical password is useful.

##### kawaiipotato2023 12/10/2019 07:59:17
Hello! I just downloaded webots and when I log in it tells me that I am not licensed to use the program. Does anyone know why this happens or how to resolve it?

##### David Mansolino [cyberbotics] 12/10/2019 07:59:56
Hi `@kawaiipotato2023`, which version of Webots did you download?

##### kawaiipotato2023 12/10/2019 08:02:13
Webots 8 2.1.2 for windows

##### David Mansolino [cyberbotics] 12/10/2019 08:02:43
Why not using the latest version ?

##### kawaiipotato2023 12/10/2019 08:03:49
This was the only version I found, do you have a link to where I can get the latest version?

##### David Mansolino [cyberbotics] 12/10/2019 08:04:56
Yes of course, you can download the latest version from here: [https://github.com/cyberbotics/webots/releases/tag/R2019b-rev1](https://github.com/cyberbotics/webots/releases/tag/R2019b-rev1)

##### kawaiipotato2023 12/10/2019 08:05:24
Thank you!

##### David Mansolino [cyberbotics] 12/10/2019 08:05:30
You're welcome

## 2020

##### Shanaka 01/02/2020 08:15:57
I'm shanaka currently working as a research engineer in the department of electrical and electronic engineering, University of Peradeniya, Sri Lanka. These days, I engaged to design simulation using PUMA 560 robot manipulator in WEBOT and note that I'm very new to webot. 



I modified the end-tool of the manipulator by replacing the Pioneer gripper



I  control this entire manipulator (with new gripper) by modifying puma560.motion file and puma560.c file. 



But I need to add dynamics and individual joint controllers to this simulation with the idea of implementation in real world. So i check puma560.c file and there I found  functions called 



wbu\_motion\_new("puma560\_1.motion");

wbu\_motion\_play(motion);

but I was not able to find the definition of these functions anywhere. ( I checked motion.h Motion.hpp Motion.cpp files.) Although I can find the function definitions inside the above-mentioned files, still I was not able to find function definitions and how the given angles fed to motors in the manipulator by wbu\_motion\_play() function. 



The final target is to implement the controller in the real world using Raspberry pi or FPGA microcontrollers. So by using wbu\_motion\_play() functions how can I do it? Where can I find the function description? If this method is impossible, it will be great if you can give guidance to make it a success. 



your response will be highly appreciated !!
%figure
![image.png](https://cdn.discordapp.com/attachments/565155651395780609/662207440632414248/image.png)
%end

##### David Mansolino [cyberbotics] 01/03/2020 07:32:26
`@Shanaka` instead of using motion file you should control directly the motor individually  (in speed, position or torque/force) using the motor API: [https://cyberbotics.com/doc/reference/motor#motor-functions](https://cyberbotics.com/doc/reference/motor#motor-functions)

You can easily get the name of the individual motors by double-clicking on the robot to open the robot-window.

IF you didn't already did, I strongly recommend to follow our tutorial (which explains how to control the motors individually): [https://cyberbotics.com/doc/guide/tutorials](https://cyberbotics.com/doc/guide/tutorials)

##### rbhuva 01/15/2020 02:09:57
anyone has Breve Simulation Environment?

##### David Mansolino [cyberbotics] 01/15/2020 06:54:13
`@rbhuva`, no we are using the Webots simulation environment 😉

##### rbhuva 01/15/2020 23:33:21
sorry guys I am asking very silly question but I am stuck

I downloaded and try various versions of the Webot software but after installing it always shows the error of unexpected stop working webots on windows. I need the software please give some solution.

##### David Mansolino [cyberbotics] 01/16/2020 06:51:32
`@rbhuva`, what is exactly the error you get?

PLease make sure to update the driver of your GPU.


Using the safe mode might help: [https://cyberbotics.com/doc/guide/starting-webots#safe-mode](https://cyberbotics.com/doc/guide/starting-webots#safe-mode)

##### rbhuva 01/16/2020 06:52:29
Thanks `@David Mansolino`

##### David Mansolino [cyberbotics] 01/16/2020 06:52:50
You're welcome

##### Angerbot 01/17/2020 14:28:29
Apologies for the continuous questions - lines here have broken down and unable to download just yet. Is there a drag and drop interface for simple movement design? Thanks

##### David Mansolino [cyberbotics] 01/17/2020 14:29:55
No sorry, Webots doesn't provide such drag and drop interface

##### machinekoder 01/23/2020 10:05:11
I have acquired an EZgripper ([https://sakerobotics.com/](https://sakerobotics.com/)) for my robot. What's the best way to simulate such an under actuated gripper? I've written a small Python controller, but I can't really make it grasp properly. Any ideas? I don't really need the physics aspect of the grasping itself, just being able to move objects around.

##### Fabien Rohrer [Moderator] 01/23/2020 10:06:42
Hi,


Generally all our grippers have physics, collision  shapes and joints.


This is certainly the best way to implement this.


Other solutions looks like hacks to me.


Does this answer your question?

##### machinekoder 01/23/2020 10:10:52
Not really. The example grippers seem to have actuated joints. However, the ezgripper has two under-actuated joints, meaning they are not driven by a motor. What's the best way to simulate this behavior. Or shall I just actuate them in the sim? I tried to mimic the real behavior using touch sensors, but my approach wasn't successful.


[https://youtu.be/240b1ubUwMA](https://youtu.be/240b1ubUwMA)

##### Fabien Rohrer [Moderator] 01/23/2020 10:48:08
I think you need to actuate the Webots joints of the underactuated joints, yes.


I had similar issues for the PR2 simulation.


[https://www.youtube.com/watch?v=Lm0FhXAxkXg](https://www.youtube.com/watch?v=Lm0FhXAxkXg)


The most important thing was to control the gripper in force control rather than in position control.


In such case, you can master the grasping force precisely, and that matters 🙂

##### machinekoder 01/23/2020 10:58:35
`@Fabien Rohrer` Thanks, I'll try that. Is the PR2 model in the examples?

##### Fabien Rohrer [Moderator] 01/23/2020 10:59:05
Yes, here it is:


[https://www.cyberbotics.com/doc/guide/pr2](https://www.cyberbotics.com/doc/guide/pr2)

##### Hannah 01/23/2020 13:38:38
Hello, is there any way to get MES(Manufacturing Execution System) data in webots? I want to simulate a production line, and I need to work with the MES data generated by the machines in the line.

##### David Mansolino [cyberbotics] 01/23/2020 13:40:09
Hi `@Hannah` not out of the bxox, but I am pretty sure that you can implement your own interface to take these MES data as input of your robot controller.

##### Hannah 01/23/2020 13:42:27
Thanks `@David Mansolino` , I wanted to know if it is already implemented or not.

##### David Mansolino [cyberbotics] 01/23/2020 13:43:48
Unfortunately it is not.

##### Hannah 01/23/2020 13:49:07
Thanks 🙂

##### David Mansolino [cyberbotics] 01/23/2020 13:49:31
You're welcome

##### ZoRal 01/23/2020 14:12:18
Thanks

##### nitrow 01/25/2020 19:18:45
I would suggest moving the CoM a little in front of the turtlebot3 model to counteract it being very unstable when accelerating. I worked a lot better when I did this. I don't know the correct CoM, but I'm sure the real turtlebot is not that prone to doing wheelies! 😄

##### David Mansolino [cyberbotics] 01/27/2020 07:11:37
Hi `@nitrow`, thank you for the suggestion, I will check in the datasheet if I find the CoM and fix it if there is some issues.


`@nitrow`, just to let you now that this is now fixed in [https://github.com/cyberbotics/webots/pull/1300](https://github.com/cyberbotics/webots/pull/1300) and will be included in the next release (and the nightly builds).

##### nitrow 01/28/2020 10:18:03
`@David Mansolino` Great, works a lot better now!

##### David Mansolino [cyberbotics] 01/28/2020 10:19:07
`@nitrow` perfect, thank you for the feedback!

##### rbhuva 02/07/2020 16:51:33
Hi there

I want to make one model in which I want multiple sources and number of bots. And I want to see that how the bots will distribute towards the various sources.

Can anyone please help me how can I achieve that?

##### luoyu2014 02/09/2020 07:44:42
Hi, I want to know where I can download the Webots 2018b version, I need this version. Thank you.

##### David Mansolino [cyberbotics] 02/10/2020 06:36:21
Hi `@rbhuva` what do you mean by sources? Source of robots? Of light? Of energy?


Hi `@luoyu2014` you can download all the old version fo webots from here: [https://github.com/cyberbotics/webots/releases/tag/R2019a](https://github.com/cyberbotics/webots/releases/tag/R2019a)

##### junjihashimoto 02/13/2020 08:30:48
Hello, I'm Junji Hashimoto, Gree Inc. I've developed webots-bindings for haskell.  [https://hackage.haskell.org/package/HsWebots](https://hackage.haskell.org/package/HsWebots)

##### David Mansolino [cyberbotics] 02/13/2020 08:32:15
Hi `@junjihashimoto`, very nice!!!


just for the note, the link to the github project seems wrong (it seems you forgot te replace the 'githubuser') 😉

##### Fabien Rohrer [Moderator] 02/13/2020 08:34:11
That's a great news! You should post it in the <#568354695513374730> chanel for a better visibility.

##### junjihashimoto 02/13/2020 08:44:39
`@David Mansolino` Thank you for your feedback. I've fixed it.

##### David Mansolino [cyberbotics] 02/13/2020 08:46:14
You're welcome, thank you for sharing this with us.

##### junjihashimoto 02/13/2020 08:54:12
`@David Mansolino` I want to check what fields the nodes have. Does webots have the api finding fields?

##### David Mansolino [cyberbotics] 02/13/2020 08:55:49
Yes, you can use the supervisor api to get the fields of a node:

[https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_field](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_field)

##### junjihashimoto 02/13/2020 09:00:15
Thx! If the field does not exit, It returns NULL. So I should check NULL or not here. [https://github.com/junjihashimoto/HsWebots/blob/master/src/Webots/Supervisor.hs#L144](https://github.com/junjihashimoto/HsWebots/blob/master/src/Webots/Supervisor.hs#L144)

##### David Mansolino [cyberbotics] 02/13/2020 09:02:39
Both should work, as testing:

```
if (X == NULL)
```

Is equiavlent to:

```
if (!X)
```

If X = NULL.

##### junjihashimoto 02/13/2020 10:07:37
The fix is done. [https://github.com/junjihashimoto/HsWebots/blob/master/src/Webots/Supervisor.hs#L148](https://github.com/junjihashimoto/HsWebots/blob/master/src/Webots/Supervisor.hs#L148)

##### junjihashimoto 02/14/2020 04:11:57
`@David Mansolino` Thank you for inviting me on github.


Can I move HsWebots-project into cyberbotics? If you have any concerns, please feel free to contact me.


BTW, I want to do CI on both linux and macos. I know linux has  a deb package of webots. So I can install it by apt. How about macos?


Do you provide binary-tar-ball for macos?

##### David Mansolino [cyberbotics] 02/14/2020 07:43:43
> Can I move HsWebots-project into cyberbotics? If you have any concerns, please feel free to contact me.

`@junjihashimoto` 

Thanks for this, we will discuss this internally ant let you know.


Adding a CI is a very good idea and sign of quality.

For macOS unfortunately we do not provide any binary-tar-ball, however it should be feasible to install the mac package from the command line.


Jsut out of curiosity, which CI are you planning to use ?


It would also be very nice if you could include in the README the version of Webots and the supported OS.

##### junjihashimoto 02/14/2020 08:06:40
I will use github-actions, because it has 8GB memory and long running time.

##### David Mansolino [cyberbotics] 02/14/2020 08:08:09
Ok, interesting.


`@junjihashimoto` we just had a discussion and you are welcome to transfert your HsWebots-project  🙂

##### junjihashimoto 02/14/2020 09:47:18
`@David Mansolino` Thx! I'm grad to hear that. I will do it.


Setup CI is done.

##### David Mansolino [cyberbotics] 02/14/2020 13:15:00
Perfect !

##### Axel M 02/21/2020 09:28:05
Some issue I ran into when using urdf2webots:

- The resulting model in webots is laying on its right side (-90 deg roll). This seems to be related to axes differences between ROS (Z upward) and webots (Y upward)

- Large meshes are merged into a single PROTO file, which results in a (very) big file not suited to manual modifications in a text editor



I'm eager help on those issues

##### Olivier Michel [cyberbotics] 02/21/2020 09:35:46
Regarding point 2 (large meshes), we are considering implementing support for meshes in Webots in different formats (DAE, OBJ, STL) by introducing a new node called Mesh that would replace the IndexedFaceSet node by referring to a mesh file (DAE, OBJ, STL, etc.) instead of listing a large number of coordinates and coordinate indices. I will open an issue about it to explain our design ideas. You are very welcome to contribute by proposing an implementation.


See [https://github.com/cyberbotics/webots/issues/1396](https://github.com/cyberbotics/webots/issues/1396)

##### Axel M 02/21/2020 09:51:54
Thats great, in the meantime I was considering modifying urdf2webots in order to generate PROTO files containing only the IndexedFaceSet of the parsed mesh (that's what i've done manually on my model). In the light of the above issue, does that seems still relevant to you ?

##### Olivier Michel [cyberbotics] 02/21/2020 10:07:30
Yes, that's a very good idea as well.

##### rbhuva 02/24/2020 15:14:28
In my project, I would be changing the number of light sources considering them as a food source for the robot (treated like animals). The hypothesis of doing so is to observe the behaviour of an individual animal and swarm of the animals when they have different food sources available at different locations. Moreover, it is also assumed that it is possible to simulate the behaviour of the animals towards the food origin of different quality. That is, they might turn to the light(food) source of the higher intensity even though the less intense light source is nearer to them

To carry out these experiments, I will consider the intensity of the sources, number of sources, number of vehicles as my independent variables.


can any one help me to achieve this...

##### Olivier Michel [cyberbotics] 02/24/2020 15:21:47
Yes, Cyberbotics can provide you with paid support if needed. You can ask for an offer at sales@cyberbotics.com. Otherwise if you have some quick question, we may answer you here.

##### rbhuva 02/24/2020 16:48:16
okay thank you


i want code for the bot such that it senses the light and go towards that


i am ready to use either khepera-1 or epuck


because i try a lot but failed in coding


and i have very less time


I want the behaviour such that my robot go in direction of light and then stop near the light 



if someone can help me with that code I am very thankful...

##### David Mansolino [cyberbotics] 02/25/2020 06:36:38
`@rbhuva` you can find an example of robot going in the direction of the light here: [https://cyberbotics.com/doc/guide/samples-devices#light\_sensor-wbt](https://cyberbotics.com/doc/guide/samples-devices#light_sensor-wbt)

The e-puck sensor model already have light sensors so using this robot model should be quite easy to do what you want

##### cnaz 02/25/2020 08:36:14
I have a project to link a algorithm of rei forcément learning and Webots. My programming learning is python. But the problem is I don't know where to do it. Is someone want to realize the project with me or help, I'll be grateful. Thx you guys (my operating system is Ubuntu 18)

##### Olivier Michel [cyberbotics] 02/25/2020 08:44:52
Hi `@cnaz`, I would recommend you to start with simple Python controllers (see for example Webots/projects/languages/python/worlds/), define a task to be learnt by the robot and use reinforcement learning techniques in your Python controller.


I would recommend you to visit [https://robotbenchmark.net](https://robotbenchmark.net) for a series of Python-based robotics challenges that may involve reinforcement learning.


All these benchmarks are also included in Webots in the Webots/projects/samples/robotbenchmark folder.

##### cnaz 02/25/2020 11:43:03
Hi `@Olivier Michel` I already do the simple python controllers, and it works but with an algorithm of reinforcement learning I don't see where to star and what Ill do. I'll visite robotbenchmark

##### DrVoodoo [Moderator] 02/25/2020 11:45:08
Hello `@cnaz` , what approach were you planning on using for reinforcement learning?

##### David Mansolino [cyberbotics] 02/25/2020 11:45:42
`@cnaz` if you are not familliar with this page, I would suggest to have a look at it: [https://www.cyberbotics.com/doc/guide/using-numerical-optimization-methods](https://www.cyberbotics.com/doc/guide/using-numerical-optimization-methods)

##### cnaz 02/25/2020 11:56:27
`@DrVoodoo` I plan to use Q learning


`@David Mansolino` thx you, I'll look

##### rbhuva 02/26/2020 01:21:11
how can i configure proximity sensor in khepera-1?

and how to add it into code...?

Or how can I avoid collision of robot with wall of circle arena...

##### David Mansolino [cyberbotics] 02/26/2020 06:21:10
Hi `@rbhuva` the distance sensors of the khepera-1 are already present in the model: [https://cyberbotics.com/doc/guide/khepera1](https://cyberbotics.com/doc/guide/khepera1)

To use them, I strongly recommend following this tutorial: [https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers](https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers)

It uses the e-puck robot, but it works the exact same way for the khepera robot (excepts that the device names are different)

##### rbhuva 02/26/2020 06:26:57
Thank You so much

##### David Mansolino [cyberbotics] 02/26/2020 06:42:40
You're welcome

##### rbhuva 02/26/2020 06:45:11
i have attached a GPS at extension of the Khepera-1 can anyone give me the code to store the GPS co-ordinates?

##### David Mansolino [cyberbotics] 02/26/2020 06:45:38
[https://cyberbotics.com/doc/reference/gps](https://cyberbotics.com/doc/reference/gps)

##### rbhuva 02/26/2020 23:06:51
any other solution beside it?

##### David Mansolino [cyberbotics] 02/27/2020 06:19:17
Beside using a GPS ?

##### rbhuva 02/27/2020 15:36:37
thank you so much...

##### elkelkmuh 02/28/2020 12:16:35
Hi `@David Mansolino`   I saw your pricing list. On the event you can give special support. Could you give FSR support for real robot. If you give support. How much is it?

##### David Mansolino [cyberbotics] 02/28/2020 13:03:24
Hi `@elkelkmuh` yes of course we can give support for such kind of things, however, we would need to have access to the real robot with FSR to develop this.

##### elkelkmuh 02/28/2020 13:06:11
`@David Mansolino`  How much is it?

##### David Mansolino [cyberbotics] 02/28/2020 13:19:24
May I ask you to send your request to support@cyberbotics.com (precising exactly what you need (e.g. real robot only, or both real and simulated, remote-control only or cross-compilation too, which version of the robot, etc.)), I am not able to answer you here directly.


May I ask you to send your request to support@cyberbotics.com .

##### elkelkmuh 02/28/2020 13:43:16
Ok  thank you

##### KyleM 02/28/2020 23:28:34
Howdy folks!


I'd really like to use webots + ros for my autonomous vehicle project


I built a custom Car PROTO and I can drive it around with a python controller using the Driver class


However...I can't figure out how to compile the ros\_automotive to connect it my some ros nodes


Is there any documentation or projects I can study to figure how to control my robot my ros?


Was able to solve my own problem by running 'make' on the main /projects folder. It doesn't build cleanly on 16.04 Ubuntu by the way


I had comment most of the child make(s) and just compile the vehicles folder

##### David Mansolino [cyberbotics] 03/02/2020 06:39:28
`@KyleM` Welcome!

for vehicle and ROS, there is a specific controller making the bridge between Webots and ROS1 it should work out of the box: [https://github.com/cyberbotics/webots/tree/master/projects/vehicles/controllers/ros\_automobile](https://github.com/cyberbotics/webots/tree/master/projects/vehicles/controllers/ros_automobile)

If you have issue recompiling Webots, here is the guide: [https://github.com/cyberbotics/webots/wiki/Linux-installation](https://github.com/cyberbotics/webots/wiki/Linux-installation)

(It should work for Ubuntu 16.04 as I am using it on a daily basis)

##### elkelkmuh 03/02/2020 07:11:20
`@David Mansolino`  I sent an email.

##### David Mansolino [cyberbotics] 03/02/2020 07:11:56
Hi `@elkelkmuh` thank you I saw it, but haden't time to answer all my emails from the week-end yet, I will answer for sure today.

##### elkelkmuh 03/02/2020 13:20:34
Hi `@David Mansolino`   thank you for answer. I sent  an email again

##### rbhuva 03/12/2020 19:12:45
Hello!! Is it possible to measure the exact intensity of the point light source at the floor? I want to know it because I am changing the height of the fixed intense source from the floor!

##### David Mansolino [cyberbotics] 03/13/2020 06:31:37
Hello `@rbhuva`, you might use a LightSensor node: [https://www.cyberbotics.com/doc/reference/lightsensor](https://www.cyberbotics.com/doc/reference/lightsensor)

##### taeyoung 03/18/2020 10:15:13
Hi! Can I put the pointlight or spotlight over the ball? I mean make the pointlight keep the height while it follows the ball. When I put the pointlight in the children of the ball with location 0 1 0,  as ball rolls the pointlight goes down and up.

##### David Mansolino [cyberbotics] 03/18/2020 10:56:25
HI `@taeyoung`, this is unfortunately not possible out of the box, however, the solution is to use the Supervisor API, using the supervisor API you can at each step monitor the position of the ball and move the light accordingly.

Here is an example where a Supervisor moves a light at a predefined location: [https://cyberbotics.com/doc/guide/samples-devices#supervisor-wbt](https://cyberbotics.com/doc/guide/samples-devices#supervisor-wbt)

##### taeyoung 03/18/2020 12:14:23
Ok Thank you

##### David Mansolino [cyberbotics] 03/18/2020 12:14:40
You're welcome

##### Jesusmd 03/23/2020 04:32:31
Hi, I need to read the LidarPoint properties with the CluodPointEnable, but I just have one list of object pointers, so my questions is how I can set all properties with LidarPoint class imported? I mean set all values of each point into a object made by LidarPoint class.

##### David Mansolino [cyberbotics] 03/23/2020 06:19:27
HI `@Jesusmd` which lanugag are you using?

Why do you want to change the properties of the lidar points? These value should be seen are readonly value as they represent the measure of the sensor.

##### Jesusmd 03/23/2020 07:31:22
Hi, `@David Mansolino` I using python language. The idea is using the measure of Lidar to make an csv file that contains measures and corresponding angles. so I need to manipulate the data


`@David Mansolino` the global idea is make lidar to grid map

##### David Mansolino [cyberbotics] 03/23/2020 07:37:30
In that case you should be able to make a loop and modify the LidarPoint objects, somthing like (untested):

```Python
for point in lidar.getPointCloud():
    # do something with the point
```

##### Jesusmd 03/23/2020 08:20:27
`@David Mansolino`  the loop is correct but for some reason my x,y and z values give 0,  I tried something like this : Lidar\_h\_r = Lidar1.getHorizontalResolution()

    print("the getHorizontalResolution is:", Lidar\_h\_r)

    Layer = Lidar1.getNumberOfLayers()

    Layer\_id = Lidar1.getLayerPointCloud(Layer)

    for i in range(Lidar\_h\_r):

        Point = Layer\_id[i]

        print("point:",Point.z,Point.y,Point.z)


`@David Mansolino`  ho my fault, I see it is points.x or some other proprieties, thanks!

##### David Mansolino [cyberbotics] 03/23/2020 08:55:47
You're welcome 😉

##### mint 03/25/2020 12:28:12
Hello, I have been develping with webot and encountered a issue with setVelocity function for a node  in supervisor. The setVelocity function takes the angular velocity in form of rotation aroud respective x y z rotation, but my calculation for angular velocity would give the result in form of axis-angle (normal vector and the magnitude of rotation around it). I'm now troubled with converting w in axis-angle form to the x y z rotation form. Is there any formula, functions or library supported by webot?


Oh, its setVelocity function at supervisor

##### David Mansolino [cyberbotics] 03/25/2020 12:31:30
Hi `@mint` unfortunately, Webots API does not provide such function. But I am sure you can find plenty of external libraries that do this. Which language are you using ?

##### mint 03/25/2020 12:32:24
I have been using python 3..

##### David Mansolino [cyberbotics] 03/25/2020 12:33:50
In Python I personnally often use the ``transforms3d``module (available on PIP: [https://pypi.org/project/transforms3d](https://pypi.org/project/transforms3d)) which allows to convert between many 3D representations, you will probably find what you need in the doc: [http://matthew-brett.github.io/transforms3d/](http://matthew-brett.github.io/transforms3d/)

##### mint 03/25/2020 12:37:37
wow.. Thank you I couldn't find that even after hours of search. You saved a lot of time for me 🙂

##### David Mansolino [cyberbotics] 03/25/2020 12:39:30
You're welcome 😉

##### luhang 03/27/2020 01:40:47
Can we program and control the NAO robot in webots now?


I didn't find APIs

##### David Mansolino [cyberbotics] 03/27/2020 06:21:02
Hi yes of course, in Webots you can use the regular Webots API to control the simulated robot, you will find some example controllers here: [https://github.com/cyberbotics/webots/tree/master/projects/robots/softbank/nao/controllers](https://github.com/cyberbotics/webots/tree/master/projects/robots/softbank/nao/controllers)

##### luhang 03/27/2020 07:47:41
Thanks!😁

##### David Mansolino [cyberbotics] 03/27/2020 07:49:44
You're welcome.

##### imjusta23 04/01/2020 17:12:59
Hey, does someone has a code for a light follower?

##### David Mansolino [cyberbotics] 04/01/2020 17:18:23
Hi `@imjusta23`, this sample show a very simple light follower: [https://cyberbotics.com/doc/guide/samples-devices#light\_sensor-wbt](https://cyberbotics.com/doc/guide/samples-devices#light_sensor-wbt)

##### imjusta23 04/04/2020 16:23:50
> Hi `@imjusta23`, this sample show a very simple light follower: [https://cyberbotics.com/doc/guide/samples-devices#light\_sensor-wbt](https://cyberbotics.com/doc/guide/samples-devices#light_sensor-wbt)

`@David Mansolino`  thanks!!


Hey, I hope you’re doing good!! Where can I find a tutorial to change the appearance of my robot?

##### DrVoodoo [Moderator] 04/04/2020 16:51:03
Shape or colour/texture?


If shape [https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)

##### ContrastNull 04/09/2020 12:35:55
Hey, I want to use a ship mast in my simulation,  which basically has a tower like structure and a radar (dish),  which continuously moves around in a 90° angle.  

So the problem is,  I dont know how to make the part moving in the manner I need.  

There's another interesting problem,  that the whole mast should move in a sinusoidal manner (basically, the way it moves in the sea). How could these things be done?! 

Thank you!

##### David Mansolino [cyberbotics] 04/09/2020 12:37:49
About the first issue, you mean you don't now how to make the radar moving ?


About the second issue, I would use the Supervisor API to move the whole mast: [https://cyberbotics.com/doc/reference/supervisor](https://cyberbotics.com/doc/reference/supervisor)

##### ContrastNull 04/09/2020 12:43:17
`@David Mansolino`,  in the first question, after I imported the model into webots,  how should I define the movement of that radar part as needed?

##### David Mansolino [cyberbotics] 04/09/2020 12:48:38
You can either use the Supervisor API to change it's rotation field either mount the radar on a controllable joint.


Did you follow our tutorials? If not I would strongly advice to follow tutorials 1 to 6: [https://cyberbotics.com/doc/reference/supervisor](https://cyberbotics.com/doc/reference/supervisor)

##### ContrastNull 04/09/2020 12:49:57
Sure,  I will look into them and let you know if it helps! 😊


Hey, I tried importing a door hinge as a VRML97 file, which was converted from Solidworks model. 

The problem is, it is considering each part of the hinge, like screws, doors as separate individual. (As in the scene tree, it is showing each part differently with same name - "transform") 

Neither it is considered as other objects in the world's, because while I tried to click on the hinge, it won't click and show it's origin and all. 

What must be the problem here?

##### David Mansolino [cyberbotics] 04/14/2020 05:21:51
Hi `@ContrastNull`, the VRML import does indeed import only the visual meshes of the object, you then have to recreate the structure of the object yourself. I would recommend to follow these tutorials to create an objet (1 to 7): [https://cyberbotics.com/doc/guide/tutorials](https://cyberbotics.com/doc/guide/tutorials)


Note also that a customizable door is already available in Webots: [https://cyberbotics.com/doc/guide/object-apartment-structure#door](https://cyberbotics.com/doc/guide/object-apartment-structure#door)

##### İchigogo 04/14/2020 15:06:31
hello! sorry for disturbing again. I don't know why but my simulation looks like lagging (I don't know if its correct word to explain but ) it's happens in some sample projects too I tried changing fps and time step but it didn't change too much thing in simulation.is there any other way to solve this ?

##### David Mansolino [cyberbotics] 04/14/2020 15:08:50
Hi, you may try reducing the OpenGl features to speed up the simulation speed: [https://cyberbotics.com/doc/guide/preferences#opengl](https://cyberbotics.com/doc/guide/preferences#opengl)


You can also find some tips to speed up your simulation here: [https://cyberbotics.com/doc/guide/speed-performance](https://cyberbotics.com/doc/guide/speed-performance)

##### İchigogo 04/14/2020 15:09:24
okay thank you so much :)

##### David Mansolino [cyberbotics] 04/14/2020 15:13:36
You're welcome

##### Aan 04/16/2020 07:07:35
Hi, I am trying to soccer ball object recognition using camera\_recognition.c in project samples. When I add soccerball, camera does not recognize the ball but it can recognize apple, can, oil barrel. How camera can recognize the soccer ball?

##### David Mansolino [cyberbotics] 04/16/2020 07:09:05
Hi, this is because the recognition color of the soccer ball is not set, let me fix this.


Here is the  fix:

[https://github.com/cyberbotics/webots/pull/1548/files](https://github.com/cyberbotics/webots/pull/1548/files)

It will be available in the next release of Webots. But in the meantime, you might apply it locally to your Webots installation files.

##### Aan 04/16/2020 07:14:58
thank you 😀

##### David Mansolino [cyberbotics] 04/16/2020 07:15:29
You're welcome

##### Dorteel 04/18/2020 05:48:34
Hi, I have a question regarding the Nao robot. I saw WeBots used to be able to interface with Choreographe, I was wondering if the motions for the Nao that are built-in in Choreographe can somehow be imported into WeBots?

##### Stefania Pedrazzi [cyberbotics] 04/20/2020 06:14:54
`@Dorteel`, no there is no automatic procedure to import Choregraphe built-in motions in Webots. But if you have the motion joints values, then you should be able to reproduce it in Webots.

##### Dorteel 04/21/2020 09:13:59
Thank you `@Stefania Pedrazzi` ! 🙂

##### imjusta23 04/22/2020 20:20:17
Any advice about how to get the epuck moves randomly in this world
%figure
![image0.jpg](https://cdn.discordapp.com/attachments/565155651395780609/702614779767947394/image0.jpg)
%end

##### David Mansolino [cyberbotics] 04/23/2020 05:44:20
Hi `@imjusta23` you might simply use the braitenberg controller.

##### fdvalois 04/23/2020 07:34:46
You just look braitenberg controller online?

##### David Mansolino [cyberbotics] 04/23/2020 07:37:52
A breaitenberge already compatible with many robot is distributed with Webots, here is the code:  [https://github.com/cyberbotics/webots/tree/master/projects/default/controllers/braitenberg](https://github.com/cyberbotics/webots/tree/master/projects/default/controllers/braitenberg)

##### fdvalois 04/23/2020 07:38:11
Thank you!

##### David Mansolino [cyberbotics] 04/23/2020 07:38:41
You're welcome

##### mint 04/26/2020 14:38:09
One question while developing.. what is the convention webot's using for the matrix returned by wb\_supervisor\_node\_get\_orientation function?

is it Euler Angles, Rotation Matrix, or Tait–Bryan angles?

##### Olivier Michel [cyberbotics] 04/26/2020 14:43:52
It's a rotation matrix (as in OpenGL).


See [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_orientation](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_orientation)

##### mint 04/26/2020 14:54:05
Thank you Olivier!

##### Psyka 04/28/2020 13:51:04
Hey again,


I've got an other issue :


WARNING: DEF GROUND Solid > Shape > PBRAppearance > ImageTexture: 'textures/ground.jpg' not found.

A resource file can be defined relatively to the worlds directory of the current project, relatively to the worlds directory of the default project, relatively to its protos directory (if defined in a PROTO), or absolutely.


but the file is int the current directory of the current project


and it's not working

##### David Mansolino [cyberbotics] 04/28/2020 13:52:11
Is it correctly in a ``textures``  folder?

##### Psyka 04/28/2020 13:52:43
textures


ground


copy pasted

##### David Mansolino [cyberbotics] 04/28/2020 13:53:08
And where is it in your project directory exactly?

##### Psyka 04/28/2020 13:53:44
I've got a directory named "Created World" where there is all my .wbt worlds using this


and inside I've got a directory named "textures"


wbt directory


haaa sorry I'm loosing it


so all my project are in E:\...\...\Project\wbt\


there I've got a directory "controllers" for controllers and "Created World" with in it "textures" and then the "ground.jpg"


Create\_Webots\_World\wbt\Created\_World\textures\ground.jpg

##### David Mansolino [cyberbotics] 04/28/2020 13:59:47
Ok, this is the problem, the folder in which are the world can't be named 'Created World'.

You should have:

```
Create_Webots_World\wbt\
                     -> controllers
                     -> worlds
                           -> textures
                                    -> ground.jpg
```

##### Psyka 04/28/2020 13:59:59
exactly


and all my .wbt are in worlds


hooo why that can't it be ?

##### David Mansolino [cyberbotics] 04/28/2020 14:00:36
But that's not what you said:

> Create\_Webots\_World\wbt\Created\_World\textures\ground.jpg

##### Psyka 04/28/2020 14:00:46
yes


Create\_Webots\_World\wbt\Created\_World\textures\ground.jpg


is a copy past of my path

##### David Mansolino [cyberbotics] 04/28/2020 14:01:36
because the names in the project folder should respect a strict convention (you can name the project folder as you want but not the folders inside):

   [https://cyberbotics.com/doc/guide/the-standard-file-hierarchy-of-a-project](https://cyberbotics.com/doc/guide/the-standard-file-hierarchy-of-a-project)

##### Psyka 04/28/2020 14:03:05
it's "Created\_World" and not "Created World" I just writte it wrong

##### David Mansolino [cyberbotics] 04/28/2020 14:03:18
It's still problematic 😉

##### Psyka 04/28/2020 14:03:45
it was working fine like last week


how should I name that directory ?


worlds


?


ho ok

##### David Mansolino [cyberbotics] 04/28/2020 14:04:24
yes indeed

##### Psyka 04/28/2020 14:04:33
I understand now what you mean


so I just have to rename it ?

##### David Mansolino [cyberbotics] 04/28/2020 14:04:40
You should have:

> Create\_Webots\_World\wbt\worlds\textures\ground.jpg

##### Psyka 04/28/2020 14:06:34
very nice thank's 🙂


it working fine

##### David Mansolino [cyberbotics] 04/28/2020 14:09:41
You're welcome

##### Shubham D 04/29/2020 22:30:02
I am trying to create some solid using transform and shapes.

I am not able to set the objects at desired angle.

How should I set the parameters if I need specific angle.

Or is their any simple way to convert normal vector direction to euler angles

##### Simon Steinmann [Moderator] 04/29/2020 22:31:07
[https://www.andre-gaschler.com/rotationconverter/](https://www.andre-gaschler.com/rotationconverter/)

##### Jesusmd 05/10/2020 06:29:26
Is there an quickly way to get the attribute instead of  predefined values as for example in  Keyboard class.?  I would preffer enum as in c or c++

##### David Mansolino [cyberbotics] 05/11/2020 06:43:54
> Is there an quickly way to get the attribute instead of  predefined values as for example in  Keyboard class.?  I would preffer enum as in c or c++

`@Jesusmd` which language are you using?

##### Simon Steinmann [Moderator] 05/13/2020 16:02:55
<@&568329906048598039> Btw, I'm currently working on a python program, that publishes the PoseStamped of one or more nodes into a rostopic. It also allows to publish the Pose relative to a specific node.


Still a bit crude with little error correction, but it works 🙂


Btw, is there a way to get the quaternion orientation directly from webots? That would make things much simpler

##### Olivier Michel [cyberbotics] 05/13/2020 16:08:03
No, you have to get it as an axis-angle representation, but that's super easy to translate into quaternion.


```python
def axis_angle_to_quaternion(axis, theta):
    axis = numpy.array(axis) / numpy.linalg.norm(axis)
    return numpy.append([numpy.cos(theta/2)], numpy.sin(theta/2) * axis)
```

##### Simon Steinmann [Moderator] 05/13/2020 16:09:36
how do we get axis angles? I have the 3x3 matrix

##### Olivier Michel [cyberbotics] 05/13/2020 16:10:25
[https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_field\_get\_sf\_rotation](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_field_get_sf_rotation)


(gives relative rotation in axis-angle notation)


If you need absolute rotation (in axis-angle notation), use:  [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_orientation](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_orientation)

##### Simon Steinmann [Moderator] 05/13/2020 16:12:12
but that returns the 3x3 matrix

##### Olivier Michel [cyberbotics] 05/13/2020 16:12:56
Yes, you are right. Sorry, my bad.


OK, so you need to convert this 3x3 rotation matrix to a quaternion then.

##### Simon Steinmann [Moderator] 05/13/2020 16:14:33
The thing is, I want it to be able to run in the native webots environment, so no extra packages. I got it to work using their code:


[https://nipy.org/nibabel/reference/nibabel.quaternions.html](https://nipy.org/nibabel/reference/nibabel.quaternions.html)


specifically this:


[https://sscc.nimh.nih.gov/pub/dist/bin/linux\_gcc32/meica.libs/nibabel/quaternions.py](https://sscc.nimh.nih.gov/pub/dist/bin/linux_gcc32/meica.libs/nibabel/quaternions.py)


Problem is, the link to their liscense is broken 😅

##### Olivier Michel [cyberbotics] 05/13/2020 16:15:26
Oops...

##### Simon Steinmann [Moderator] 05/13/2020 16:16:25
how big of a deal would it be to add the 'tf' package to the webots ROS environment?


it is almost a must if working with ROS and robots

##### Olivier Michel [cyberbotics] 05/13/2020 16:18:13
If you use Webots with ROS, you need to install ROS, and you should get 'tf' for free, isn't it?

##### Simon Steinmann [Moderator] 05/13/2020 16:19:08
yes, but it requires runtime.ini edits. Would be nice if people can have it run as an example out of the box

##### Olivier Michel [cyberbotics] 05/13/2020 16:20:18
Yes, if you have an idea to achieve this... Now you know how to open a pull request 😉

##### Simon Steinmann [Moderator] 05/13/2020 16:20:54
oh lordy 😅


I'll give it a try

##### ContrastNull 05/13/2020 16:43:42
Hey, could you tell me how to multi-select things in scene-tree? 

I can't find a way to. I imported a VRML97 model into Webots, the model was quite large, and it's difficult to select, cut and paste each 'transform' object into a robot node's children attribute.

##### Olivier Michel [cyberbotics] 05/13/2020 16:44:11
Unfortunately, this is not possible.

##### ContrastNull 05/13/2020 16:45:26
Oh, thank you for letting me know!

Keep it as a suggestion for the next update. 😄

##### Simon Steinmann [Moderator] 05/13/2020 16:47:12
Perhaps add a 'Group' base node, put all your nodes in, and copy paste that

##### ContrastNull 05/13/2020 17:11:36
`@Simon Steinmann` that is what my problem is about. The nodes I have to copy paste one by one are in "hundreds".

##### Simon Steinmann [Moderator] 05/13/2020 18:04:16
`@ContrastNull`  perhaps direct .proto file edit can help

##### David Mansolino [cyberbotics] 05/14/2020 05:34:23
`@Simon Steinmann` instead of relying on ROS, if you are using Python you ca probably use the `transforms3d` python package which allows for example to convert from a rotation matrix to quaternions: [https://matthew-brett.github.io/transforms3d/reference/transforms3d.quaternions.html#transforms3d.quaternions.mat2quat](https://matthew-brett.github.io/transforms3d/reference/transforms3d.quaternions.html#transforms3d.quaternions.mat2quat)

##### Axel M [Premier Service] 05/14/2020 09:08:34
`@Simon Steinmann` Although working with ROS, we decided to split the simulation projects with minimal dependencies over ROS other than communication. For 3D Math in Cpp, we're using Eigen, and in python either numpy or numpy + transformations.py (which is standalone of tf)


Regarding your example of getting relative position between two nodes, that can be easily achieved with Eigen in cpp (3x3 Matrix -> Quaternion, quaternion / vector product)


The integration with ROS is super easy too with [http://wiki.ros.org/eigen\_conversions](http://wiki.ros.org/eigen_conversions) 🙂

##### Simon Steinmann [Moderator] 05/14/2020 12:48:46
> `@Simon Steinmann` Although working with ROS, we decided to split the simulation projects with minimal dependencies over ROS other than communication. For 3D Math in Cpp, we're using Eigen, and in python either numpy or numpy + transformations.py (which is standalone of tf)

`@Axel M` 



Awesome, thank you so much. That makes things easier

##### Jesusmd 05/15/2020 01:49:44
`@David Mansolino` Hi, I am using python, I would like to work with several distance sensors at the same time and comparing their data in console. But instead of obtain the type, I got a number.

##### ContrastNull 05/15/2020 03:02:49
Hey, may I know, why does a drone, (let it be DJI Mavic 2 Pro, which is already available into Webots, or a custom made) moves up and down and shifts constantly in one direction even if no such behaviour is defined in the controller code?

##### David Mansolino [cyberbotics] 05/15/2020 04:55:43
`@Jesusmd` you want to get the type of distance sensor? If so you should use the ``getType`` function: [https://cyberbotics.com/doc/reference/distancesensor?tab-language=python#wb\_distance\_sensor\_get\_type](https://cyberbotics.com/doc/reference/distancesensor?tab-language=python#wb_distance_sensor_get_type)


`@ContrastNull` the controller of the drone is very simple, it doesn't do any feedback using the GPS position or any inertial unit, it might therefore easily drift.

##### Alfian 05/21/2020 04:09:02
There is no controller for lock position of the DJI Mavic 2 Pro

##### ChapoGuzman 05/21/2020 23:41:00
Hi guys im using the urdf package to convert my files between solidworks and webots


But i have this error



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/713174651731181749/unknown.png)
%end


I did a good install because I tried the "Human" file and it worked correctly


Thanks in advance

##### Olivier Michel [cyberbotics] 05/22/2020 06:48:43
Please open an issue on [https://github.com/cyberbotics/urdf2webots/issues/new](https://github.com/cyberbotics/urdf2webots/issues/new) and upload your URDF file as a ZIP file.

##### ContrastNull 05/22/2020 13:18:06
Hey, I was trying to simulate a world on a HTTP web page. But I couldn't link the controller to the world. I can only move the viewpoint. How should I link the controllers to that file? So that the robots in my simulation should work.

##### Olivier Michel [cyberbotics] 05/22/2020 14:27:44
If your robots move in Webots, they should also move in the web view. Don't they?

##### ContrastNull 05/22/2020 15:28:54
`@Olivier Michel` you see, no other option or window opens. neither the drone moves. But in webots, it moves. Any suggestions?
%figure
![Screenshot_from_2020-05-22_20-57-46.png](https://cdn.discordapp.com/attachments/565155651395780609/713413087914491904/Screenshot_from_2020-05-22_20-57-46.png)
%end

##### Olivier Michel [cyberbotics] 05/22/2020 15:31:32
That's because you exported to HTML but you are not streaming the simulation.


Do you want to export an animation or to stream a live simulation?


Please refer to the user guide: [https://cyberbotics.com/doc/guide/web-interface](https://cyberbotics.com/doc/guide/web-interface)

##### ContrastNull 05/22/2020 15:37:09
Yes, I have already read it. I need to simulate my world on a web browser using HTML file. I mean anyone with the file can open the simulation on the web. Server streaming or already recorded animation isn't the case.

##### Olivier Michel [cyberbotics] 05/22/2020 15:41:18
Exporting a to HTML will simply export a static scene (no motion), so you probably want to either export an animation or stream a live simulation?

##### ContrastNull 05/22/2020 15:45:40
So could you tell me how can I make this static scene to moving? (by using my controller code of course).

No, as I said, its none of these two cases.

##### Olivier Michel [cyberbotics] 05/22/2020 15:46:11
This is not possible.

##### ContrastNull 05/22/2020 15:50:50
`@Olivier Michel` , would you please take a look at this. So it will be easy to help me out.

[http://www.aerialroboticscompetition.org/assets/downloads/simulation\_challenge\_rules\_1.1.pdf](http://www.aerialroboticscompetition.org/assets/downloads/simulation_challenge_rules_1.1.pdf)

Go for " Challenge Rules - 1 ".

##### Olivier Michel [cyberbotics] 05/22/2020 16:00:06
Are you a competitor or an organizer/developer of this contest?

##### ContrastNull 05/22/2020 16:01:57
I am a competitor, sir.

##### Olivier Michel [cyberbotics] 05/22/2020 16:02:54
OK, then you should probably run Webots in the cloud and stream your simulation to the web.

##### ContrastNull 05/22/2020 16:03:44
Alright, thank you for your understanding!

##### hrsh12 05/24/2020 21:21:56
Is there a way of modifying the usual steering system used in car models?

##### David Mansolino [cyberbotics] 05/25/2020 06:25:16
You can either convert your car node to base node (right click on the node in the scene-tree) and then you will be able to change the structure, either change directly the `Car.proto` file.

##### Axel M [Premier Service] 05/27/2020 16:42:51
I ported the Atom PROTO extension to VSCode : [https://marketplace.visualstudio.com/items?itemName=pymzor.language-proto-webots](https://marketplace.visualstudio.com/items?itemName=pymzor.language-proto-webots)


Review / PR / issues are welcome 🙂

##### Clara Cardoso Ferreira 05/27/2020 18:21:49
[https://pgda.gsfc.nasa.gov/products/54](https://pgda.gsfc.nasa.gov/products/54)

Im trying to retrieve height maps from the above data and import it to Webots to create a custom terrain. I am not sure if this is possible and easy to do. Please provide me some guidance.



I was thinking float img files formats would work best since I could import it to Opend CV if necessary.

##### David Mansolino [cyberbotics] 05/28/2020 05:24:05
One possible solution would be to create a procedural PROTO ([https://www.cyberbotics.com/doc/reference/procedural-proto-nodes](https://www.cyberbotics.com/doc/reference/procedural-proto-nodes)) which will read the float image and generate an ElevationGrid ([https://www.cyberbotics.com/doc/reference/elevationgrid](https://www.cyberbotics.com/doc/reference/elevationgrid)) from it.


You will find an example of this here: [https://gist.github.com/DavidMansolino/46d7e0df4c48bbaac4611b3872878347](https://gist.github.com/DavidMansolino/46d7e0df4c48bbaac4611b3872878347)

##### AngelAyala 05/28/2020 06:15:18
Is there anyway to catch the runtime errors in python in order to stop and reset the simulation? I can control the execution of the simulation with Supervisor node

##### David Mansolino [cyberbotics] 05/28/2020 06:18:04
you can always use a  `try` and `except` on the sensitive part

##### AngelAyala 05/28/2020 06:18:55
I'm currently using, but when appears this error Error: wb\_gps\_enable(): invalid device tag, the code is unable to catch it


is maybe a especial ExceptionError?

##### David Mansolino [cyberbotics] 05/28/2020 06:19:43
this s indeed just a warning which doesnt  raise any exception


but you can simply check the return value

##### AngelAyala 05/28/2020 06:20:33
oh greats, thanks

##### David Mansolino [cyberbotics] 05/28/2020 06:20:42
You're welcome

##### AngelAyala 05/28/2020 06:50:43
now, how can I access to the GPS node inside the Mavic2Pro PROTO using the supervisor node?


is there anyway to do this?


using python

##### Stefania Pedrazzi [cyberbotics] 05/28/2020 06:52:23
Is the Supervisor controller different from the Mavic2Pro controller?

##### AngelAyala 05/28/2020 06:52:31
yes

##### Stefania Pedrazzi [cyberbotics] 05/28/2020 06:52:49
if yes, then you cannot read the Mavic2Pro GPS device values from another robot/supervisor.


This is like in real context.


What you can do is to send the GPS data from the Mavic2Pro controller to the supervisor controller using for example the Emitter/Receiver devices

##### AngelAyala 05/28/2020 06:54:21
ok I get it, both running in different controllers


I wasn't considering as real context, thanks

##### Clara Cardoso Ferreira 05/28/2020 21:34:42
> One possible solution would be to create a procedural PROTO ([https://www.cyberbotics.com/doc/reference/procedural-proto-nodes](https://www.cyberbotics.com/doc/reference/procedural-proto-nodes)) which will read the float image and generate an ElevationGrid ([https://www.cyberbotics.com/doc/reference/elevationgrid](https://www.cyberbotics.com/doc/reference/elevationgrid)) from it.

`@David Mansolino`


My guess would be that his solution would work, but it would probably be extremely slow since you would have to generate each individual slide of the terrain at every render pass

##### Iguins 05/28/2020 22:10:01
I was facing that same problem

##### David Mansolino [cyberbotics] 05/29/2020 05:03:20
Will the terrain change at each step ?

##### Clara Cardoso Ferreira 05/29/2020 05:04:20
For my case, it changes every meter. I am trying to import the map of the moon to Webots


I could import the whole moon, but I plan to have at least sections of the moon


so that my robot can learn how to navigate in unknown terrain


with real data

##### David Mansolino [cyberbotics] 05/29/2020 05:06:52
For the whole moon, it is indeed going to be slow, but if you import only a reasonable sub-section, the performance should be ok. See this example: [https://cyberbotics.com/doc/automobile/ch-vens](https://cyberbotics.com/doc/automobile/ch-vens)

##### Clara Cardoso Ferreira 05/29/2020 05:20:52
Ill give it a try, thank you David!

##### Iguins 05/29/2020 05:31:28
I wanted the robots to modify an uneven terrain

##### David Mansolino [cyberbotics] 05/29/2020 05:32:59
In that case the robots should be Supervisors and they will be able to change the height values.

##### Iguins 05/29/2020 05:33:39
nice thank you!

##### David Mansolino [cyberbotics] 05/29/2020 05:33:58
You're welcome

##### Iguins 05/29/2020 05:35:32
do you think that instead of a heightmap I could use a custom mesh to also change the x and z values?

##### David Mansolino [cyberbotics] 05/29/2020 05:36:28
You can, but heightmap is much more efficient and stable than random indexedFaceSet.

##### Iguins 05/29/2020 05:36:59
I was afraid of that


thank you

##### Wesst 05/30/2020 16:24:35
does anyone have implemented some kind of pathfinding and can help me ? pls dm

##### Ans 05/30/2020 20:39:57
Hi! Webots noob here, could someone tell me how to add an odor plume in the simulation and make the robot follow it? (Kinda trying to recreate this : [https://en.wikibooks.org/wiki/Webots\_Odor\_Simulation](https://en.wikibooks.org/wiki/Webots_Odor_Simulation))

##### Sergen Aşık 05/31/2020 21:22:36
How can i run simulator using Qt Creator?


is there any depth camera that i can use for skeletal tracking?

##### David Mansolino [cyberbotics] 06/02/2020 05:51:17
> does anyone have implemented some kind of pathfinding and can help me ? pls dm

`@Wesst` you will find some examples here: [https://en.wikibooks.org/wiki/Cyberbotics%27\_Robot\_Curriculum/Advanced\_Programming\_Exercises#Path\_planning\_](https://en.wikibooks.org/wiki/Cyberbotics%27_Robot_Curriculum/Advanced_Programming_Exercises#Path_planning_)[Advanced]


> Hi! Webots noob here, could someone tell me how to add an odor plume in the simulation and make the robot follow it? (Kinda trying to recreate this : [https://en.wikibooks.org/wiki/Webots\_Odor\_Simulation](https://en.wikibooks.org/wiki/Webots_Odor_Simulation))

`@Ans` this is simply emulated from a Supervisor controller that imports shapes to visualize the plume.


The supervisor also send plume information to every robot using emitter-receivers.


> How can i run simulator using Qt Creator?

`@Sergen Aşık` do you want to start Webots from Qt Creator, or use Qt in your controller?


> is there any depth camera that i can use for skeletal tracking?

`@Sergen Aşık` yes of course, you can use range-finders: [https://www.cyberbotics.com/doc/reference/rangefinder](https://www.cyberbotics.com/doc/reference/rangefinder)

##### lojik 06/02/2020 10:48:45
Hello everyone, I do not know if someone else have the same problem as me but it is easy to verify.



I am on ubuntu 18.04, using webots R2020a-revision1. When I right-click to add new features --> search in the 'find' box and type an underscore '\_' Webots crashes.



Do someone has the same issue?

##### David Mansolino [cyberbotics] 06/02/2020 10:50:28
Hi, yes this is indeed a known bug, it was fixed very recently, you can already download a beta of R2020a-revision2 which incudes the fix here: [https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)

##### lojik 06/02/2020 10:51:08
Perfect, thank you 👍

##### David Mansolino [cyberbotics] 06/02/2020 10:51:52
You're welcome

##### hrsh12 06/04/2020 21:23:54
Hello everyone, could someone tell me if we can implement a rack and pinion system for steering in Car. For the jointsI'm trying to recreate a  steer-by-wire system. Thanks

##### David Mansolino [cyberbotics] 06/05/2020 05:15:24
Hi `@hrsh12`, this is not possible directly, however you can combine several base joint to make such system.


We plan to add a new feature which will allow to link joint together and which might help you doing this ([https://github.com/cyberbotics/webots/issues/1365](https://github.com/cyberbotics/webots/issues/1365)).

##### hrsh12 06/09/2020 11:33:40
Hi, `@David Mansolino` , To make a rack and pinion arrangement, i require more kind of joints than those that are available in the documentation. Can i model my own basic joints?

##### David Mansolino [cyberbotics] 06/09/2020 11:35:20
You can either use physics plugins: 

[https://cyberbotics.com/doc/reference/physics-plugin](https://cyberbotics.com/doc/reference/physics-plugin)

Either extend Webots with new nodes:

[https://github.com/cyberbotics/webots/blob/master/CONTRIBUTING.md](https://github.com/cyberbotics/webots/blob/master/CONTRIBUTING.md)

##### lojik 06/10/2020 16:46:38
Hello there, I imported a 3d object in webots by using webots 2020b. It happen frequently that an object is suddently not takent into account with contacts. It seems that there is some problems with the bounding object.



Could it be a bug on this nightly version or is it a problem on my side?



%figure
![Screenshot_from_2020-06-10_18-47-22.png](https://cdn.discordapp.com/attachments/565155651395780609/720318368837533766/Screenshot_from_2020-06-10_18-47-22.png)
%end


Here you have the kind of objects I imported. These are kind of sinus waves. I put a bit of each pieces down the floor to have smaller mountains.

##### David Mansolino [cyberbotics] 06/11/2020 04:57:15
Hi `@lojik` what is exactly the problem you have with the bounding object,?We can't see them on this picture.

##### lojik 06/11/2020 08:37:32
Hi `@David Mansolino` sorry, I did not explain well...  My problem is that I actually use the objects shown in the previous picture. With the bounding object as their own shape. But when my robot has to climb them, sometimes the wheel go through the object instead of climb it. It seems that the bounding object is not here.



I actually use webots nightls R2020b 9\_6\_2020



%figure
![Screenshot_from_2020-06-11_10-34-54.png](https://cdn.discordapp.com/attachments/565155651395780609/720557335977787433/Screenshot_from_2020-06-11_10-34-54.png)
%end


As seen on this picture, the wheel on the left is inside the object and the bounding object (in white) should be in red if it is well detected by the wheel.



On the right, the wheel is correctly on top of the object.


And this effect is worst when I would like to record a video. The object are never well detected.


I just try now to begin recording the video after that the robot well detected my objects. In that case it works fine, but I have to play simulation until the robot detects my objects and then begin to record video.

##### David Mansolino [cyberbotics] 06/11/2020 08:44:01
This is because the bounding object created automatically when imported are based on the actual mesh, this is known to be not very accurate and stable, for better collisions, you should re-create the key bounding objects using sets of basic geometries (e.g. boxes, spheres, etc.)

##### lojik 06/11/2020 08:45:02
Would it be better if I have a finner mesh? Or worst?


Because I would like to import custom models to have more freedom on the form of obstacles I have. It would not be a good deal if I have to redo the bounding object after importing them..

##### David Mansolino [cyberbotics] 06/11/2020 08:46:16
it all depends on the complexity of the mesh, but there is a tradeoff (which is not always easy to find).

But maybe a simpler solution for recording the video is to decrease the WeorldInfo.timestep.

##### lojik 06/11/2020 08:47:57
I work with blender to do these meshes. My goal is to have well designed unflat terrain to benchmark my robot in that condition.


I already have a basicTimeStep set to 1ms 😅

##### David Mansolino [cyberbotics] 06/11/2020 08:51:10
I am sorry but there is no magic solution for this, using meshes as bounding object is not the ideal solution.

##### lojik 06/11/2020 08:58:23
If I understand correctly it is actually not possible to create more complexe shapes with their own bounding object? If I would like to have a ramp which look like an ellipsoid more than a circle. I will always have to approximate the bounding object with a circle or take time too find a good threshold with the mesh to make it works?

##### David Mansolino [cyberbotics] 06/11/2020 09:00:04
> I will always have to approximate the bounding object with a circle

Not exaclty, you can use several geometries to approximate the shape (such as for example several circles to approximate an ellipsoid).

##### lojik 06/11/2020 09:02:53
Ok, well, I did not expected that. I have some good results with my meshes. But from one simulation to an other one the results are not the same so I was wondering where it was wrong.



Thank you for your answer, I will have a deeper look to find a good solution between approximation geometries and find a good mesh.

##### David Mansolino [cyberbotics] 06/11/2020 09:03:22
You're welcome

##### lojik 06/11/2020 10:33:03
Ok, so after playing a bit with my meshes I have a first intuition. I realize that your cylinders are composed with only parallel lines. So I did the same in blender for my sinusoid ground and it seems to work much better. Now it is detected every time instead of 1/3 run.



Unfortunately it will keep to have triangles by exportations. But since there is a huge number of parallels in  my mesh, it works fine now.



I do not know if I explain well enough .. Here is my final mesh :



%figure
![Screenshot_from_2020-06-11_12-32-27.png](https://cdn.discordapp.com/attachments/565155651395780609/720586430224793600/Screenshot_from_2020-06-11_12-32-27.png)
%end


PS: The first run after importing such an object will provoc a webots' crash.

##### David Mansolino [cyberbotics] 06/11/2020 11:45:25
The mesh looks indeed to be very clean like this. Just for the note, for the cylinder, the parallel lines are just for visualization, for the physics it is a perfect cylinder.

##### lojik 06/11/2020 11:48:58
Ok thank you, so my intuition is not right, but it works much better with this mesh 👍

##### David Mansolino [cyberbotics] 06/11/2020 11:51:11
OK, it make sense, for we are using similar meshes to create roads and it works quite well. Note that you might be interested by the road PROTO as it allows to create similar meshes (when varying the height of the road): [https://cyberbotics.com/doc/guide/object-road](https://cyberbotics.com/doc/guide/object-road)

##### lojik 06/11/2020 11:59:14
Yes, exactly, you use quite the same idea behind the road PROTO. So I will keep in mind this way to create meshes. Thank you !


I also see that uneven terrain seems to use a symetric grid mesh as a bounding object, is it right?

##### David Mansolino [cyberbotics] 06/11/2020 12:01:48
Uneven terrai is using an EleveationGrid as mesh, this mesh is way more stable/efficient than IndexedFaceSet for collision but has more constraints ([https://www.cyberbotics.com/doc/reference/elevationgrid](https://www.cyberbotics.com/doc/reference/elevationgrid))

##### lojik 06/11/2020 12:29:35
Okay, thank's 👍

##### webotspro9999 06/20/2020 22:52:55
hello all


I'm new in Webots and I want to develop speech recognition applications using Softbank Robotics NAO. Is this feature available in Webots?

##### David Mansolino [cyberbotics] 06/22/2020 05:27:14
hello `@webotspro9999`, this is unfortunately not available out of the box, you will have to implement your own speach recognition algorithm or use a library implementign this.

##### aalmanso 06/22/2020 15:36:42
Hi everyone,

I want to place E-puck randomly inside arena every time I start or reload the simulation, I need that to test some behaviour. So, i wander how can I find the help about using random numbers and 2D coordinates in webots simulator?

##### Olivier Michel [cyberbotics] 06/22/2020 15:38:09
You can simply write a supervisor controller that will move the robot to a random position at the beginning of the simulation.

##### webotspro9999 06/22/2020 22:02:26
> hello `@webotspro9999`, this is unfortunately not available out of the box, you will have to implement your own speach recognition algorithm or use a library implementign this.

`@David Mansolino` thanks for your reply, does the NAO in Webots have sound listening functions? For example, when I say hello can the NAO hear this and take it as an input data?

##### David Mansolino [cyberbotics] 06/23/2020 05:01:36
No, Webots does not simulate microphones. But you micreate an interface to get the audio from your computer michrophone.

##### black\_hammer\_67 06/23/2020 16:33:46
Can somebody help me to figure out how to reset the simulation without having to stop the controller, actually I want every time a condition is true in the controller to reset the robots initial position only, not the simulation time

##### Clara Cardoso Ferreira 06/23/2020 23:23:31
I think so`@black_hammer_67` , check the supervisor API and look for position


Does anyone know if it's possible to reset the terrain in the proto for every simulation and set up the robot's wheels to touch the ground to start the simulation?


Also, what is the benefit of using the supervisor vs a sensor (like the GPS) to get states such as position?

##### Andrei 06/23/2020 23:34:32
I guess it depends what you want to do. I wanna make a pathfinder so I wanna know the whole map when I determine the path I take not just what sensors can give me.

##### David Mansolino [cyberbotics] 06/24/2020 05:38:33
> Can somebody help me to figure out how to reset the simulation without having to stop the controller, actually I want every time a condition is true in the controller to reset the robots initial position only, not the simulation time

`@black_hammer_67` yes, you can use the `wb_supervisor_simulation_reset` Supervisor function: [https://www.cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb\_supervisor\_simulation\_reset](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb_supervisor_simulation_reset)


> Does anyone know if it's possible to reset the terrain in the proto for every simulation and set up the robot's wheels to touch the ground to start the simulation?

`@Clara Cardoso Ferreira` to reset the terrain from the proto file is indeed possible, unfortunately it is not possible to move the robot from there, but you can move it then at the first step of the simulation with a Supervisor controller.


> Also, what is the benefit of using the supervisor vs a sensor (like the GPS) to get states such as position?

`@Clara Cardoso Ferreira` no benefit, if you are trying to get your own position, GPS is the way to go, if you are trying to get the position of another node supervisor is the way to go.

##### black\_hammer\_67 06/24/2020 13:59:32
hello, is there a way to detect robot colisions in the suppervisor mode ?

##### David Mansolino [cyberbotics] 06/24/2020 14:00:20
Yes, you can get the number and location of the contact points of a node:  [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_contact\_point](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_contact_point)

##### black\_hammer\_67 06/24/2020 14:01:48
ok so if I have a pioneer 3dx robot the default contact points are 3 , these that the wheels have with the floor and if I detect 4 or more points I can assume this is a colision right ?

##### David Mansolino [cyberbotics] 06/24/2020 14:02:59
That's unfortunately not so simple, the number of contact points of the wheel can vary, you should rather check the location of the contact points.

##### black\_hammer\_67 06/24/2020 14:04:12
ok i see, so I am looking for points that they have a certain value on y axis > 0.1 for example ?

##### David Mansolino [cyberbotics] 06/24/2020 14:04:52
Yes, that sounds good.

##### black\_hammer\_67 06/24/2020 14:06:02
thank you clarifying that 😁

##### David Mansolino [cyberbotics] 06/24/2020 14:06:16
You're welcome 😉

##### Clara Cardoso Ferreira 06/24/2020 19:00:41
> `@Clara Cardoso Ferreira` to reset the terrain from the proto file is indeed possible, unfortunately it is not possible to move the robot from there, but you can move it then at the first step of the simulation with a Supervisor controller.

`@David Mansolino` 



Your answer somewhat helps. I understand I can reset the terrain, but would I have to reset it manually or is there a way to make a loop so that a different terrain is used in every simulation iteration? I imagine I would have to connect the supervisor code to the proto somehow



Another big question, I forgot a simulation running overnight on my computer and the next morning, it would not turn on. It may have been a coincidence that my computer failed even though it is under a year of use. Would you have any suggestions of what may have gone wrong?

##### David Mansolino [cyberbotics] 06/25/2020 04:53:53
You can do this directly from the proto itself, you simply have to set the 'randomSeed' field to 0 as explained here: [https://www.cyberbotics.com/doc/guide/object-floors#uneventerrain-field-summary](https://www.cyberbotics.com/doc/guide/object-floors#uneventerrain-field-summary)


About your other question, was your controller writing to a file? Would it be possibl that it saturated the hard drive?

##### Clara Cardoso Ferreira 06/29/2020 15:04:28
Thanks I'll try that when I get my computer back.

My controller was not writing anything at the moment it crashed, but it was probably storing data. Yeah, it may have been the harddrive

##### Marian 06/30/2020 09:29:18
Hi, I'm trying to create some simple cylinders in WeBots. It works but unfortunatly it only creates a wireframe but no texture. What is missing here?
%figure
![webots_cylinderr.png](https://cdn.discordapp.com/attachments/565155651395780609/727455716390076437/webots_cylinderr.png)
%end

##### lojik 06/30/2020 09:32:56
Your "shape" node should be in the "children" one and not "boundingObject". Then you can select it in "boundingObject" because bounding object is just the virtual shape taken into account when performing contact simulation. It is not the visual rendering of the object.

##### Marian 06/30/2020 09:45:52
Ok, thanks a lot for the explanation. It works!

##### Clara Cardoso Ferreira 07/03/2020 23:17:42
> Thanks I'll try that when I get my computer back.

> My controller was not writing anything at the moment it crashed, but it was probably storing data. Yeah, it may have been the harddrive



I got my computer back and it was in fact a hardrive issue. It was just a coincidence that I was running an extensive Webots  simulation

Thanks for the tips

##### henry10210 07/04/2020 16:38:53
I set the WEBOTS\_HOME to the root of the source code folder (where I ran the ./webots command).  I wonder if this is an incorrect WEBOTS\_HOME


Is it possible to use just python3 with webots?  Could not build the python2 language binding due to my libpython2 missing symbols, and since ROS2 is all python3, figured I would not need it.  But finding python2 dependenceis in webots\_ros2 package

##### Simon Steinmann [Moderator] 07/05/2020 09:34:48
`@henry10210` just select extern as the controller and run your controller outside in your own environment

##### starstuff\_0903 07/05/2020 10:26:14
i need to create a 3d environment from a 2d image..Is it possible

##### mint 07/06/2020 04:59:09
Hello, is there any ways of writing a proto file  from an external design tool like CAD? What  kind of language does it belong to?


nevermind, I just  figured out that it is VRML97. my bad XD

##### David Mansolino [cyberbotics] 07/06/2020 05:39:15
> I got my computer back and it was in fact a hardrive issue. It was just a coincidence that I was running an extensive Webots  simulation

> Thanks for the tips

`@Clara Cardoso Ferreira` you're welcome! thank you for the feedback!


> I set the WEBOTS\_HOME to the root of the source code folder (where I ran the ./webots command).  I wonder if this is an incorrect WEBOTS\_HOME

`@henry10210` yes this is correct.


> Is it possible to use just python3 with webots?  Could not build the python2 language binding due to my libpython2 missing symbols, and since ROS2 is all python3, figured I would not need it.  But finding python2 dependenceis in webots\_ros2 package

`@henry10210` which python2 dependencies do you see in the webots\_ros2 package?


> i need to create a 3d environment from a 2d image..Is it possible

`@starstuff_0903` can you tell us more, what does represent your image? The height of the terrain ?

##### Simon Steinmann [Moderator] 07/06/2020 14:32:47
`@David Mansolino` hi david, could you take a quick look at this issue? If I know how to solve it, I can write up a short tutorial


[https://github.com/cyberbotics/webots/issues/1841](https://github.com/cyberbotics/webots/issues/1841)

##### David Mansolino [cyberbotics] 07/06/2020 15:35:44
Yes, sure I will answer it tomorrow morning

##### Simon Steinmann [Moderator] 07/06/2020 17:30:54
`@David Mansolino` I seem to have figured it out. This shows joint 1 and 3 being changed in percent of their valid range. Looks correct now. Do you want me to make a PR with the new, 'correct' robotiq gripper?
> **Attachment**: [3f\_gripper.mp4](https://cdn.discordapp.com/attachments/565155651395780609/729751244406521887/3f_gripper.mp4)

##### zeynepp 07/06/2020 20:10:30

%figure
![1.JPG](https://cdn.discordapp.com/attachments/565155651395780609/729791407081914508/1.JPG)
%end



%figure
![2.JPG](https://cdn.discordapp.com/attachments/565155651395780609/729791411846774824/2.JPG)
%end


I am trying to write the kinematics for the lrb robot for ipr but it is not directed to the target


what do u think



> **Attachment**: [empty\_.mp4](https://cdn.discordapp.com/attachments/565155651395780609/729792436452196372/empty_.mp4)


where is the problem

##### ContrastNull 07/07/2020 04:53:42
I was trying to see what's inside this - "Visual\_tracking.wbt" sample world inside Webots. Unfortunately I didn't found a github link to the file for reference, but this screenshot should be helpful.

My question was, how does the RubberDuck move? I noticed it has an immersion properties node, with fluid name - "water". But I don't see any fluid node in the scene tree. Also there's no controller code, for it's movement.

Any explanation please?
%figure
![Screenshot_from_2020-07-07_10-17-01.png](https://cdn.discordapp.com/attachments/565155651395780609/729923074715025518/Screenshot_from_2020-07-07_10-17-01.png)
%end

##### David Mansolino [cyberbotics] 07/07/2020 05:52:59
> I was trying to see what's inside this - "Visual\_tracking.wbt" sample world inside Webots. Unfortunately I didn't found a github link to the file for reference, but this screenshot should be helpful.

> My question was, how does the RubberDuck move? I noticed it has an immersion properties node, with fluid name - "water". But I don't see any fluid node in the scene tree. Also there's no controller code, for it's movement.

> Any explanation please?

`@ContrastNull` here is the link: [https://github.com/cyberbotics/webots/tree/master/projects/samples/robotbenchmark/visual\_tracking](https://github.com/cyberbotics/webots/tree/master/projects/samples/robotbenchmark/visual_tracking)

The duck is moved by with a Supervisor, here is the controller of the Supervisor: [https://github.com/cyberbotics/webots/blob/master/projects/samples/robotbenchmark/visual\_tracking/controllers/visual\_tracking\_benchmark/visual\_tracking\_benchmark.py](https://github.com/cyberbotics/webots/blob/master/projects/samples/robotbenchmark/visual_tracking/controllers/visual_tracking_benchmark/visual_tracking_benchmark.py)


> `@David Mansolino` I seem to have figured it out. This shows joint 1 and 3 being changed in percent of their valid range. Looks correct now. Do you want me to make a PR with the new, 'correct' robotiq gripper?

`@Simon Steinmann` yes sure if you have any correction to bring to the model any PR is highly appreciated!

##### Simon Steinmann [Moderator] 07/08/2020 15:55:06
I had a weird issue with urdf2webots. I converted the kinova gen3 6dof arm and everything worked. However when implementing it with moveit, I noticed that several hingeJoints were rotating in the wrong direction. I had to switch the axis manually to negative (or positve) to change direction.  It might have something to do with the urdf using the z-axis and the proto using the y-axis for the joint.

##### David Mansolino [cyberbotics] 07/09/2020 05:33:58
Hi, just to make sur I understood correctly, the joint axes were correct, but the direction wrong right? Is the error similar to [https://github.com/cyberbotics/urdf2webots/issues/42](https://github.com/cyberbotics/urdf2webots/issues/42) ?

##### Simon Steinmann [Moderator] 07/09/2020 08:43:40
Yes, that's exactly the same issue. For some joints the direction of rotation is flipped. I fixed it manually, by flipping the rotational axis (0 1 0 --> 0 -1 0)


I think I had to flip 4/6 joints. Inverse kinematics was broken before, now it works. This should be looked into though in my opinion. It takes deeper knowledge to be able to figure out the issue and fix it. Would be great if it works out of the box and control + IK code can be directly used from the existing urdf based repositories.


Btw, successfully created a working GEN3 kinova arm. Even got IK through moveIt to run. Any news on the 'unofficial' robot model repo?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/730719273768452136/unknown.png)
%end

##### David Mansolino [cyberbotics] 07/09/2020 09:41:27
> I think I had to flip 4/6 joints. Inverse kinematics was broken before, now it works. This should be looked into though in my opinion. It takes deeper knowledge to be able to figure out the issue and fix it. Would be great if it works out of the box and control + IK code can be directly used from the existing urdf based repositories.

`@Simon Steinmann` ok thank you for the feedback, we will try to fix this in the URDF exporter directly.


> Btw, successfully created a working GEN3 kinova arm. Even got IK through moveIt to run. Any news on the 'unofficial' robot model repo?

`@Simon Steinmann` Very nice! No news yet (several of us are in holidays this week, so we will wait next week to discuss about this), but I will for sure let you know, it would be nice to include your model 🙂

##### Simon Steinmann [Moderator] 07/09/2020 09:44:14
I might make a kinova\_webots git. A bunch of the launch files have to be altered in order for the IK to work. Plus it needs a custom ROS controller for webots.


On that node, I highly modified the universal\_ROS controller you guys created. Made it extern and modified the trajectory and pose\_publisher slightly. Now all it takes to switch from ur10e to kinova-Gen3, is to change the joint names and the same controller works.


A universal(common, not the company) trajectory-arm controller might be something interesting


For our simulation benchmark project, we have to decide on a  version of webots. Could you give a short explanation on how your versions work?  For example, what is 2020a vs 2020b? When does the official version get updated? And lastly, which version would you recommend? I know that I needed to install a nightly build at the end of mai, to fix the simulation reset issue, and that some proto file updates are essential for our benchmark.


Should I just get the latest nightly build of the master branch?

##### David Mansolino [cyberbotics] 07/14/2020 14:16:37
We usually do 2 version per year, first version a, then aroudn the middle of the year we create version b, then in between these version if needed we create patch release (e.g. a-revision1, a-revision2, etc.). If you need the patch, I would stick to the version b nightly, we will soon release an official version of R2020b when this is the case, I woudl stick to this official and stable R2020b.

##### Simon Steinmann [Moderator] 07/14/2020 14:17:20
version b is the develop branch?

##### David Mansolino [cyberbotics] 07/14/2020 14:17:25
yes

##### Simon Steinmann [Moderator] 07/14/2020 14:18:33
do nightly build files stay up indefinetely?

##### David Mansolino [cyberbotics] 07/14/2020 14:18:49
no, we keep only the one of the last 3 days.

##### Simon Steinmann [Moderator] 07/14/2020 14:19:05
hmm

##### David Mansolino [cyberbotics] 07/14/2020 14:19:20
But once we will release the official stable version of R2020b, this one will stay up indefinetely.

##### Simon Steinmann [Moderator] 07/14/2020 14:19:30
is there a ETA?


collegues are setting up their systems and environments, we need to be able to install the same version, and preferably not have to change it frequently

##### David Mansolino [cyberbotics] 07/14/2020 14:21:22
ok, in that case, only the official stable versions will be kept (e.g. latest R2020a-rev1).

##### Simon Steinmann [Moderator] 07/14/2020 14:21:39
that one doesnt have the simulation reset fix included yet right?

##### David Mansolino [cyberbotics] 07/14/2020 14:22:25
Unfortunately not.


But we expect to release the official version of R2020b in ~3 weeks from now.

##### Simon Steinmann [Moderator] 07/14/2020 14:23:02
hmm, perhaps we have to copy and save the setup file ourselves then


thanks for the infos 🙂

##### David Mansolino [cyberbotics] 07/14/2020 14:31:19
You're welcome

##### Emiliano Borghi 07/22/2020 19:33:46
Hey, I saw you merged this PR: [https://github.com/cyberbotics/webots/pull/1643](https://github.com/cyberbotics/webots/pull/1643)

I would recommend you to make a post in ROS Discourse because it will help the ROS community with their simulations (included myself).



Thanks for merging it BTW!

##### Olivier Michel [cyberbotics] 07/23/2020 05:33:41
You are welcome. We plan to release Webots R2020b (including this feature) in the next couple of days. We will make sure we announce it (including this ROS-friendly feature and a few others) on ROS Discourse.

##### JSON Derulo 07/23/2020 08:54:46
Hi, does WeBots have to the ability to be integrated with by planners?

##### Darko Lukić [cyberbotics] 07/23/2020 08:57:02
`@JSON Derulo` Yes, you can check this example: [https://www.youtube.com/watch?v=gO0EXKv8x70](https://www.youtube.com/watch?v=gO0EXKv8x70)



In general, Webots is compatible with ROS and ROS2, therefore you can use ROS packages. A few more links:

- Webots with ROS1 tutorial: [https://cyberbotics.com/doc/guide/tutorial-8-using-ros](https://cyberbotics.com/doc/guide/tutorial-8-using-ros)

- Webots support for ROS2: [http://wiki.ros.org/webots\_ros2](http://wiki.ros.org/webots_ros2)

##### JSON Derulo 07/23/2020 08:57:35
Awesome, thanks

##### csnametala 07/27/2020 16:11:10
Hello everyone!

I have an e-puck robot in Webots simulations, and I wanted to detect when it hits a wall. How can I solve this?

##### Simon Steinmann [Moderator] 07/27/2020 16:30:05
read out the appropiate sensor

##### Lukulus 07/29/2020 10:33:21
Hello I am trying to use multiple threads for different tasks for an robot. 

But it seems like calling the robot->step() funktion in diffenrent threads leads to stop the Simulation.

Sometimes it runs, sometimes time stops and also the simulation.

So can you tell me if its possible to use different threads in a robot controller and how to use them in a right way?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/737981083752202241/unknown.png)
%end

##### David Mansolino [cyberbotics] 07/29/2020 10:43:48
Hi `@Lukulus`, Webots APi supports well multi-threading, however, it is strongly recommended to call the step function from one thread only, doing is contrary is extremely error-prone.

##### Lukulus 07/29/2020 10:46:56
ok, thank you 🙂

##### David Mansolino [cyberbotics] 07/29/2020 10:48:29
You're welcome

##### Yanick Douven 07/31/2020 07:57:06
Hi everyone!

I'm trying to implement a ROS controller for a 3D lidar (like the Velodyne). Does anything like this already exist?

Thank you!

##### David Mansolino [cyberbotics] 07/31/2020 08:25:34
hi, for ROS 1 or 2?

##### Yanick Douven 07/31/2020 08:28:12
Hi David,

For ROS 1

##### David Mansolino [cyberbotics] 07/31/2020 08:29:32
Ok, yes in that case the default ros controller does implement an interface for any kind of lidars: 

  - [https://www.cyberbotics.com/doc/guide/using-ros#standard-ros-controller](https://www.cyberbotics.com/doc/guide/using-ros#standard-ros-controller)

  - [https://www.cyberbotics.com/doc/reference/lidar?tab-language=ros#lidar-functions](https://www.cyberbotics.com/doc/reference/lidar?tab-language=ros#lidar-functions)

##### Yanick Douven 07/31/2020 08:44:34
Thanks! I didn't know there was a standard ROS controller. I'm currently writing a custom one in Python, since I need some non-generic interfacing with other components. Is it possible to combine the two? I.e. run the standard and custom ROS controllers side by side?


Or, if not, is there some Python implementation of the generic lidar controller?

##### David Mansolino [cyberbotics] 07/31/2020 08:45:16
Yes and no, you can split your robot into two robots (one been a child of the other) and assign one controller per robot.


> Or, if not, is there some Python implementation of the generic lidar controller?

`@Yanick Douven` unfortunately not, but you should be able to convert it quite simply, here is the source code: [https://github.com/cyberbotics/webots/blob/master/projects/default/controllers/ros/RosLidar.cpp](https://github.com/cyberbotics/webots/blob/master/projects/default/controllers/ros/RosLidar.cpp)

##### Yanick Douven 07/31/2020 08:46:31
Perfect, thank you very much!

##### David Mansolino [cyberbotics] 07/31/2020 08:46:40
You're welcome

##### Simon Steinmann [Moderator] 08/04/2020 15:17:17
Found a strange behavior. Running EXACTLY the same simulation, I get several of these errors:

WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.



if the timeStep=1    Timestep 4-16  are fine

##### Stefania Pedrazzi [cyberbotics] 08/04/2020 15:46:43
As reported by the warning message, the ODE physics engine has issues in computing the simulation step due to the complexity of the world.

Reducing the time step also reduces the complexity of the computations. Other suggestions to improve the stability of the simulation and avoid this error can be found here:

[https://www.cyberbotics.com/doc/guide/modeling#my-robotsimulation-explodes-what-should-i-do](https://www.cyberbotics.com/doc/guide/modeling#my-robotsimulation-explodes-what-should-i-do)

##### Simon Steinmann [Moderator] 08/04/2020 15:47:22
the issue is, that I get these errors at timestep 1, which is much smaller than let's say 8, where everythign runs fine

##### Stefania Pedrazzi [cyberbotics] 08/04/2020 15:54:36
it depends on the simulation and the collisions between the objects at the computation time. Maybe in your case increasing the time step makes skipping some problematic contact or object's intersection.

##### Simon Steinmann [Moderator] 08/04/2020 15:56:09
a simple stacking task
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/740236647781630022/unknown.png)
%end


with cylinder objects, and gripper tips are boxes


at something like 4 or 8 timestep, the simulation outcome is also exactly the same. At like 32 there is a bit of variance, which is expected. I find it weird that at timestep 1 it has issues

##### Stefania Pedrazzi [cyberbotics] 08/04/2020 15:58:04
it's a simple task, but it is complex from the physical computation

##### Simon Steinmann [Moderator] 08/04/2020 15:59:52
is there a way to get more detailed info? where exactle the issue is?

##### Stefania Pedrazzi [cyberbotics] 08/04/2020 16:01:05
unfortunately not, ODE doesn't give any details. But you can find it by simplifying your simulation until the problem disappears.

##### Simon Steinmann [Moderator] 08/04/2020 16:04:35
I just dont understand how decreasing the timestep can cause issues. That's how you're supposed to get rid of these issues

##### Olivier Michel [cyberbotics] 08/04/2020 16:05:46
In general yes, but your case seems tricky...

##### Simon Steinmann [Moderator] 08/04/2020 16:16:42
Suuuper weird issue. So I run the exact same task several times with increasing timesteps (4, 8, 16, 32). If everything is set correctly, it takes 43 seconds. I use the supervisor to reset the simulation after each run, and set the timestep before each run. Now here is the strange part. If the timestep is 4 or smaller, the time for each run is the same. However, if reset the controller and manually put the timestep to 32 BEFORE i start my script, it uses 32ms increments for the simulation time, even if the steps are actually smaller. This results in times of like 5min, 2,5 min 1,2 min and then finally 43 seconds when actually using 32 timestep again.


There seems to be a something strange going on with the sim\_time and timestep


Okay I figured out the issue:

robot.getBasicTimeStep()

This always returns the timestep that a controller was initialized with. Changing the timestep does NOT change the return of this function.


looking at the source code, the robot would have to be re-initialized. I think this function should get this value directly from the world instance instead from a value that get's defined on controller initialization

##### Buzzer 08/05/2020 15:57:51
Hi, how can I get a node position


?


I want to built a bot to go to a specific node location


there is something like GetNode(Node*)


?

##### Simon Steinmann [Moderator] 08/05/2020 15:58:21
[https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_node\_get\_orientation](https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_node_get_orientation)


there is. The controller has to be a supervisor though

##### Buzzer 08/05/2020 15:58:49
Ow nice, thanks

##### Simon Steinmann [Moderator] 08/05/2020 16:00:36
there is a python example I wrote, on how to get the position of an object relative to another. That might come in handy
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/740600155841101904/unknown.png)
%end


because the function will only return values relative to the world

##### Buzzer 08/05/2020 16:03:09
thanks `@Simon Steinmann`, I will follow this

##### Simon Steinmann [Moderator] 08/06/2020 11:19:38
Since I have done quite a few urdf2webots conversions lately, I might as well create a tutorial for it, as the process can be complicated if one has not much experience. Is there any robotic arm that people want to see added to the community projects?

##### željko 08/06/2020 11:23:39
Hey! I'm just in the middle of trying to wrap my head around urf2webots and a tutorial would be of great help to me! The arms I would like to use are KUKA LBR iiwa and Kinova Jaco and I belive that they would be a good addition as they are quite popular. BTW right now reading older messages as I see that you've had similar problems 🙂

##### Simon Steinmann [Moderator] 08/06/2020 11:24:34
Yesterday I fixed a huge issue in the converter. The joints should not be wonky anymore, so make sure you pull the newest version of urdf2webots


you got a link to the kuka repo?

##### željko 08/06/2020 11:25:49
OK, thanks a lot 😄 You've just saved me countless hours hahah :)

I found a github repo with the kuka arm which I'm trying to get to work

##### Simon Steinmann [Moderator] 08/06/2020 11:26:00
link?

##### željko 08/06/2020 11:26:05
just a sec


[https://github.com/ros-industrial/kuka\_experimental](https://github.com/ros-industrial/kuka_experimental)

The repo has other arms as well, all in separate folders

##### Simon Steinmann [Moderator] 08/06/2020 12:09:16
`@David Mansolino` is the new fixed version of urdff2webots already available on pip install? because I get this error:

importer.py: error: no such option: --multi-file

##### David Mansolino [cyberbotics] 08/06/2020 12:09:32
Not yet.


I will create it right now 😉

##### Simon Steinmann [Moderator] 08/06/2020 12:09:58
hehe good


making a  tutorial right now


would be easier with pip. please ping me when it's done

##### David Mansolino [cyberbotics] 08/06/2020 12:11:46
I just launched the process, now Github Action will create and upload the pip package


It's now live (it reflects the current state of master): [https://pypi.org/project/urdf2webots/#history](https://pypi.org/project/urdf2webots/#history)

##### Simon Steinmann [Moderator] 08/06/2020 13:43:39
It worked, but needed to specify no-cache. For anyone else who wants to use the new and fixed version of urdf2webots, install it like this:

pip install --no-cache-dir --upgrade urdf2webots


I ran into an issue, where 

robot.getBasicTimeStep()


returns the Timestep on robot init, not the actual timestep of the simulation


is there already an issue related to this? My suggestion would be to add a function:

robot.setBasicTimeStep

##### Olivier Michel [cyberbotics] 08/07/2020 13:34:10
Yes, this is a known issue.

##### Simon Steinmann [Moderator] 08/07/2020 13:34:36
or add it to the supervisor and have 

robot.getBasicTimeStep()

grab the value directly from the world instance

##### Olivier Michel [cyberbotics] 08/07/2020 13:34:39
It's pretty rare to dynamically change the time step of a simulation. Why do your need to do that?

##### Simon Steinmann [Moderator] 08/07/2020 13:35:21
benchmark runs at different timesteps. I worked around it, by not using 

robot.getBasicTimeStep()  for steps, but the manually set interval

##### Olivier Michel [cyberbotics] 08/07/2020 13:35:55
A solution could be to read the "basicTimeStep" field of the WorldInfo node from a supervisor process.

##### Simon Steinmann [Moderator] 08/07/2020 13:36:18
that would be great


Supervisor function to set and get the BasicTimeStep


because setting it is a bit clonky


want me to open request?

##### Olivier Michel [cyberbotics] 08/07/2020 13:37:27
No, I believe we won't implement it.

##### Simon Steinmann [Moderator] 08/07/2020 13:37:52
ohh I know what you mean

##### Olivier Michel [cyberbotics] 08/07/2020 13:37:57
Because this is a rare use case and a workaround exists using a supervisor.

##### Simon Steinmann [Moderator] 08/07/2020 13:45:50
I could add this example code to the robot.getBasicTimeStep documentation. In case someone else runs into that issue
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/741291016308523018/unknown.png)
%end

##### Olivier Michel [cyberbotics] 08/07/2020 13:47:32
Yes, that would be great.

##### Simon Steinmann [Moderator] 08/12/2020 13:40:49
When I include images in a tutorial, where do I put them? I'm currently creating a 'tutorial.md' for urdf2webots. My plan is to have it in the base directory and link it in the README.md.


I could create a new directory called 'docs' and put everything, including the images there

##### David Mansolino [cyberbotics] 08/12/2020 13:42:14
Yes, a 'docs' folder with an 'images' folder inside look good.


Like it is currently the case for the 'webots' repository: [https://github.com/cyberbotics/webots/tree/master/docs](https://github.com/cyberbotics/webots/tree/master/docs)

##### Simon Steinmann [Moderator] 08/12/2020 13:46:01
sounds good


PR is ready for review [https://github.com/cyberbotics/urdf2webots/pull/68](https://github.com/cyberbotics/urdf2webots/pull/68)

##### Olivier Michel [cyberbotics] 08/12/2020 15:43:19
Reviewed.

##### Simon Steinmann [Moderator] 08/12/2020 16:07:54
can't commit the last one


outdated?!

##### Olivier Michel [cyberbotics] 08/12/2020 16:16:35
Probably...

##### Simon Steinmann [Moderator] 08/12/2020 16:17:16
i'll do it manually

##### Olivier Michel [cyberbotics] 08/12/2020 16:17:30
OK, thank you.

##### Simon Steinmann [Moderator] 08/12/2020 16:21:17
okay done. Accepted all changes except 2 minor ones

##### Olivier Michel [cyberbotics] 08/12/2020 16:58:51
Thanks. I will check it tomorrow.

##### MarioAndres7 08/12/2020 23:14:33
Hello Im new. Does anyone knows if exists and example of a movile robot? i want to give it 

coordinates

##### David Mansolino [cyberbotics] 08/13/2020 05:17:10
Hi `@MarioAndres7`, welcome? Do you mean mobile robot?

##### Simon Steinmann [Moderator] 08/13/2020 06:55:32
converting the kuka repo atm. Getting these errors:

WARNING: KukaLbrIiwa14R820 (PROTO) > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > Solid: Undefined inertia matrix: using the identity matrix. Please specify 'boundingObject' or 'inertiaMatrix' values.

WARNING: KukaLbrIiwa14R820 (PROTO) > Solid: Undefined inertia matrix: using the identity matrix. Please specify 'boundingObject' or 'inertiaMatrix' values.



I'm pretty sure, these are due to the two 'empty' links in the urdf:

<link name="tool0"/>

<link name="base"/>


Question is, how should empty links be handled properly? And can we automate this

##### David Mansolino [cyberbotics] 08/13/2020 06:58:39
We should probably improve the importer to handle this exception and avoid exporting the physis in this case.

##### Simon Steinmann [Moderator] 08/13/2020 06:59:14
I agree. giving them mass, doesnt seem logical

##### David Mansolino [cyberbotics] 08/13/2020 06:59:43
Please open an issue on the urdf2webots repo about this.

##### Simon Steinmann [Moderator] 08/13/2020 07:00:02
will do

##### David Mansolino [cyberbotics] 08/13/2020 07:00:07
thank you

##### Simon Steinmann [Moderator] 08/13/2020 07:05:30
done, opened issue

##### David Mansolino [cyberbotics] 08/13/2020 07:06:45
perfect, thank you

##### Simon Steinmann [Moderator] 08/13/2020 07:25:26
DEF link\_4\_0 IndexedFaceSet: Cannot create IndexedFaceSet because: "'coordIndex' is empty.".
> **Attachment**: [link\_4\_0Mesh.proto](https://cdn.discordapp.com/attachments/565155651395780609/743369609666625587/link_4_0Mesh.proto)


The file looks correct. I have no idea why this error comes up

##### David Mansolino [cyberbotics] 08/13/2020 07:30:05
To me it works fine, without any error in the console
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/743370782125719562/unknown.png)
%end

##### Simon Steinmann [Moderator] 08/13/2020 07:30:26
uploading to community, one sec


[https://github.com/cyberbotics/community-projects/pull/9](https://github.com/cyberbotics/community-projects/pull/9)


the kr5 robot


I can load the link itself as well, but for some reason, it doesnt work loading it into the robot link

##### David Mansolino [cyberbotics] 08/13/2020 07:39:11
ok, that's strange, will have a look as soon as I have the time to.

##### Simon Steinmann [Moderator] 08/13/2020 07:42:29
removing and adding the hidden tag again seemed to have fixed the issue


[https://tenor.com/view/elmo-shrug-gif-5094560](https://tenor.com/view/elmo-shrug-gif-5094560)

##### David Mansolino [cyberbotics] 08/13/2020 07:43:14
That's strange, I will try thsi too if I can reproduce the problem

##### Simon Steinmann [Moderator] 08/13/2020 08:14:03
another strange issue. This is the kr5 robot, loaded from the community projects folder
%figure
![Screenshot_from_2020-08-13_10-09-47.png](https://cdn.discordapp.com/attachments/565155651395780609/743381846217457694/Screenshot_from_2020-08-13_10-09-47.png)
%end


simply copy pasting the meshes folder into my current project's protos folder, fixes all the issues and looks like this:
%figure
![Screenshot_from_2020-08-13_10-10-20.png](https://cdn.discordapp.com/attachments/565155651395780609/743381994280321034/Screenshot_from_2020-08-13_10-10-20.png)
%end

##### David Mansolino [cyberbotics] 08/13/2020 08:48:53
I am currently reviewing your PR and checking this.

##### Simon Steinmann [Moderator] 08/13/2020 08:49:28
There is also a issue with the standard maxTorque  of 100 being too low


on the kr5 I changed it to maxTorque 1000, and it fixes it (not pushed yet)

##### David Mansolino [cyberbotics] 08/13/2020 08:49:52
the maxtorque is not defined in the urdf ?

##### Simon Steinmann [Moderator] 08/13/2020 08:49:57
no it is not

##### David Mansolino [cyberbotics] 08/13/2020 08:50:12
ok, and do you know what is the default maxtorque in URDF ?

##### Simon Steinmann [Moderator] 08/13/2020 08:50:22
nope


not defined

##### David Mansolino [cyberbotics] 08/13/2020 08:50:49
ok, maybe if we find it, we can export to the standard urdf maxtorque if not specified in the urdf file.

##### Simon Steinmann [Moderator] 08/13/2020 08:51:17
what do you mean?

##### David Mansolino [cyberbotics] 08/13/2020 08:54:08
In the urdf2webtos, if the maxtorque is not specified in th eurdf file, we just do not specify it in the PROTO file, Webots will then use the fedault value  which is quite small and probably doesn't correspond to the URDF default value. Instead, if the maxtorque is not defined in the URDF file, we can still export the maxTorque to a higher value (ideally corresponding to the URDF default maxTorque).

##### Simon Steinmann [Moderator] 08/13/2020 08:54:45
ahhh I understand


from my understanding, if not defined in urdf, it is handled as not having a limit


I mean, Gazebo handles it that way

##### David Mansolino [cyberbotics] 08/13/2020 08:59:06
> simply copy pasting the meshes folder into my current project's protos folder, fixes all the issues and looks like this:

`@Simon Steinmann` for me it works fine from the commuity project without any changes too. Maybe you need to close and re-open Webots? It is possible that if you made some changes to the 'extern project' files they are updated only next time you restart Webots.

##### Simon Steinmann [Moderator] 08/13/2020 08:59:31
restarting webots didnt fix the issue weirdly enough

##### David Mansolino [cyberbotics] 08/13/2020 08:59:57
> from my understanding, if not defined in urdf, it is handled as not having a limit

`@Simon Steinmann`ok then in that case, if not defined we should probably set a very high value (e.g. 10'000)


Ohhh I can reprocude the issue with the `lrb_iiwa` but not with the `kr5` is it the same for you?

##### Simon Steinmann [Moderator] 08/13/2020 09:01:23
hold on

##### David Mansolino [cyberbotics] 08/13/2020 09:02:47
Ok, I think I found the reason. The problem is that both robot are defining the same PROTO names, and therefore in the case of the `lrb_iiwa` some meshes of the `kr5` are used.

##### Simon Steinmann [Moderator] 08/13/2020 09:03:06
yes, same

##### David Mansolino [cyberbotics] 08/13/2020 09:03:12
We should probably improve the name of the sub-PROTO to include as prefix the name of the robot.

##### Simon Steinmann [Moderator] 08/13/2020 09:03:56
ohhhh, it doesnt start searching at the location of the proto file, but at the base level


okay, then names have to be unique


fix should be easy enough

##### David Mansolino [cyberbotics] 08/13/2020 09:04:54
Yes, that shouldn't be too dificult to include this prefix

##### Simon Steinmann [Moderator] 08/13/2020 09:05:56
but perhaps it would be better to (or in addition), to change the mesh-lookup algorithm. So it starts in the same directory as the main proto file

##### David Mansolino [cyberbotics] 08/13/2020 09:09:05
We are are going to re-work this indeed (that's on our medium term plan), but in any case avoiding duplcating PROTO names is a good practice (think for example if a user then move the files to put them all in the same folder).

It should be sufficient to prepend the name of the robot to the `name` variable here: [https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/writeProto.py#L461](https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/writeProto.py#L461)

Would you give a try? Or I can do this if you want.

##### Simon Steinmann [Moderator] 08/13/2020 09:09:34
already at it 😄

##### David Mansolino [cyberbotics] 08/13/2020 09:09:47
Perfect, thank  you 🙂

##### Simon Steinmann [Moderator] 08/13/2020 09:29:45
[https://github.com/cyberbotics/urdf2webots/pull/70](https://github.com/cyberbotics/urdf2webots/pull/70)


implemented and tested. works 🙂

##### David Mansolino [cyberbotics] 08/13/2020 09:33:28
That looks all good, I will try and update the test.

##### Simon Steinmann [Moderator] 08/13/2020 09:36:21
I can add the torque fix while we are at it.


already tested it. change 1 indet + change 2 lines

##### David Mansolino [cyberbotics] 08/13/2020 09:36:50
(I was going to do it ^^, but if you can do it while I am testing it's cool

##### Simon Steinmann [Moderator] 08/13/2020 09:39:21
added it


now we just have to remove the physics nodes from epmty liks

##### David Mansolino [cyberbotics] 08/13/2020 09:39:37
Perfect, I will check it just after lunch

##### Simon Steinmann [Moderator] 08/13/2020 10:09:27
implemented the physics node thing too. Not sure it is the cleanest solution, but at least it only needs 4 lines of extra code

##### David Mansolino [cyberbotics] 08/13/2020 11:06:55
I just finished, if you're fine with the changes feel free to merge it.

##### Simon Steinmann [Moderator] 08/13/2020 11:25:35
max torque implementation doesnt work for me


the torque limits are still 0.0, not 10000

##### David Mansolino [cyberbotics] 08/13/2020 11:29:32
oups, will try

##### Simon Steinmann [Moderator] 08/13/2020 11:30:43
line 825 in parserURDF needs a condition I think


jup, adding this line works:


if float(limitElement.getAttribute('effort')) != 0:


pushed it


merged


... Sorry to do this, but one final PR 😄


[https://github.com/cyberbotics/urdf2webots/pull/71](https://github.com/cyberbotics/urdf2webots/pull/71)


after this I'll be happy, I swear


No manual edits needed after converting

##### David Mansolino [cyberbotics] 08/13/2020 12:13:26
> No manual edits needed after converting

`@Simon Steinmann` 🎉

##### Simon Steinmann [Moderator] 08/13/2020 12:13:46
well at least for robot arms 🙂

##### David Mansolino [cyberbotics] 08/13/2020 12:14:29
That's already a good start

##### Simon Steinmann [Moderator] 08/13/2020 12:26:03
done


converted and added bunch of KUKA arms:

[https://github.com/cyberbotics/community-projects/pull/11](https://github.com/cyberbotics/community-projects/pull/11)

##### David Mansolino [cyberbotics] 08/13/2020 14:02:09
Can you also update the one of [https://github.com/cyberbotics/community-projects/pull/9](https://github.com/cyberbotics/community-projects/pull/9) ?

##### Simon Steinmann [Moderator] 08/13/2020 15:45:04
Oh I didn't see that one still being open. I replaced all files. pull/9 can be discarded


Can this be an issue? The base node of every robot is called 'base\_link'. Could this lead to problems?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/743863005908959352/unknown.png)
%end

##### David Mansolino [cyberbotics] 08/17/2020 06:10:33
Oh yes indeed, we should open the 'name' field too.


Just opened an issue about this: [https://github.com/cyberbotics/urdf2webots/issues/72](https://github.com/cyberbotics/urdf2webots/issues/72)

##### Simon Steinmann [Moderator] 08/17/2020 17:25:40
I created a universal IK controller. only parameters that need to be changed are the urdf filename and which axis points out of the toolSlot. Works really well


I wonder if the link-setup information could be extracted straight from the protofile. Then it would truly be an universal controller, that wouldn't need any adjustments between robot arms



> **Attachment**: [ik3.mp4](https://cdn.discordapp.com/attachments/565155651395780609/744971358995349504/ik3.mp4)

##### Dennet 08/17/2020 19:31:23
> I wonder if the link-setup information could be extracted straight from the protofile. Then it would truly be an universal controller, that wouldn't need any adjustments between robot arms

`@Simon Steinmann` where did you upload the controller?

##### Simon Steinmann [Moderator] 08/17/2020 19:36:55
`@Dennet` have not uploaded it yet


do you need help with IK?

##### David Mansolino [cyberbotics] 08/18/2020 05:47:49
That looks very cool, I have been trying to do the same using IKPY last week ([https://github.com/cyberbotics/webots/blob/master/projects/robots/abb/irb/controllers/inverse\_kinematics/inverse\_kinematics.py](https://github.com/cyberbotics/webots/blob/master/projects/robots/abb/irb/controllers/inverse_kinematics/inverse_kinematics.py)) but I am not 100% happy with the result, I would be very interested to see which library you are using and how.


> I wonder if the link-setup information could be extracted straight from the protofile. Then it would truly be an universal controller, that wouldn't need any adjustments between robot arms

`@Simon Steinmann` what about using the new `wb_robot_get_urdf` function?

##### Simon Steinmann [Moderator] 08/18/2020 07:33:02
😮 that exists? That would be awesome, I'll check it out later when I have time

##### David Mansolino [cyberbotics] 08/18/2020 07:34:51
Yes, that's new in R2020b 😉


(and improved in the nightly)

##### Simon Steinmann [Moderator] 08/18/2020 07:49:13
the version I have uses the sensor names for the joints. has this been fixed in the new nightlies?

##### David Mansolino [cyberbotics] 08/18/2020 07:50:52
No, it uses indeed the sensor as names, but you should be able to easily switch from one to the other (removing the 'sensor' postfix).

##### Simon Steinmann [Moderator] 08/18/2020 07:51:22
particular reason it does this?


oh and can you link me the IK controller you're working on?

##### David Mansolino [cyberbotics] 08/18/2020 07:51:53
No, it was either the motor or sensor name, we decided sensor name.


Sure, here it is: [https://github.com/cyberbotics/webots/blob/master/projects/robots/abb/irb/controllers/inverse\_kinematics/inverse\_kinematics.py](https://github.com/cyberbotics/webots/blob/master/projects/robots/abb/irb/controllers/inverse_kinematics/inverse_kinematics.py)

##### Simon Steinmann [Moderator] 08/18/2020 07:52:41
converting urdf2webots uses joint names as motor names, that's why I'd assume the other way around would do the same

##### David Mansolino [cyberbotics] 08/18/2020 07:53:02
Makes sense indeed.

##### Simon Steinmann [Moderator] 08/18/2020 07:53:16
if you were to convert it back, the motors would be called 'joint\_sensor' and the sensors 'joint\_sensor\_sensor'

##### David Mansolino [cyberbotics] 08/18/2020 07:54:21
indeed good point, I should admit that the name of the joint was chosen without particularly thinking about this, it need indeed a bit more thinking about this.

##### Simon Steinmann [Moderator] 08/18/2020 07:56:31
glad to be of service 😄


In the controller I sent you, there is the extra file 'get\_relative\_position.py'. I think this is a much needed functionality for the supervisor. Could look something like this:

supervisor.getPositionRelative(self, node)


Doing all this linear algebra manually is a big obstacle for many people. Took me a while to figure out too. First time I needed linear algebra since uni pretty much 😄

##### David Mansolino [cyberbotics] 08/18/2020 08:11:04
It is indeed useful, but we somehow have to keep the number of API functions low (as to keep the quality level high it requires maintenance time) so we have to be careful when adding function to get information that can already be extracted from other API functions.

##### Simon Steinmann [Moderator] 08/18/2020 08:13:33
true I guess, still would be usefull. I might update the documentation and link to a implementation. Or we update the irb inverse kinematics world with this function, showing a general implementation on how to get relative positions and orientationis


universal IK controller nearly done :D. already requires no urdf import (used your armChain creation). only thing left is to figure out, which axis points out of the robot


automatically


is there a easy way to get translation and orientation of a specific solid in a proto file?


I especially need the rotation matrix, urdf only has rpy

##### Olivier Michel [cyberbotics] 08/18/2020 10:09:24
Yes, you should be able to do this from a supervisor controller.


See [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_from\_proto\_def](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_from_proto_def)


And [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_position](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_position)

##### Simon Steinmann [Moderator] 08/18/2020 10:10:25
but  the solid needs to have a defined DEF right?

##### Olivier Michel [cyberbotics] 08/18/2020 10:10:30
Yes.

##### Simon Steinmann [Moderator] 08/18/2020 10:10:35
damn

##### Olivier Michel [cyberbotics] 08/18/2020 10:11:05
If an upper parent has a DEF name, this may be workable though.

##### Simon Steinmann [Moderator] 08/18/2020 10:11:23
I would have to go through the whole tree right?

##### Olivier Michel [cyberbotics] 08/18/2020 10:11:29
Yes.

##### Simon Steinmann [Moderator] 08/18/2020 10:11:32
problem is, I need it for the last solid 😄

##### Olivier Michel [cyberbotics] 08/18/2020 10:11:54
A fairly simple loop should do the job.

##### Simon Steinmann [Moderator] 08/18/2020 10:12:08
trying to make a universal-IK controller for you guys


could you help me with that loop?

##### Olivier Michel [cyberbotics] 08/18/2020 10:12:41
Which programming language are you using?

##### Simon Steinmann [Moderator] 08/18/2020 10:12:47
python

##### Olivier Michel [cyberbotics] 08/18/2020 10:13:06
OK, let me have a look.

##### Simon Steinmann [Moderator] 08/18/2020 10:16:30
thank you

##### Olivier Michel [cyberbotics] 08/18/2020 10:18:38
How will your recognize the node you want to get the orientation/position? By its name, any other field?

##### Simon Steinmann [Moderator] 08/18/2020 10:19:02
i guess name

##### Olivier Michel [cyberbotics] 08/18/2020 10:19:08
OK.

##### David Mansolino [cyberbotics] 08/18/2020 10:19:30
> I especially need the rotation matrix, urdf only has rpy

`@Simon Steinmann` can't you convert the rpy into matrix? (in python e.g. `transforms3d` allows to do this: [https://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#terms-used-in-function-names](https://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#terms-used-in-function-names))

##### Simon Steinmann [Moderator] 08/18/2020 10:19:51
we might have to update all official robotic arms with the new urdf2webots convention


would have liked to prevent more libraries, but that is an option as well of course


but the supervisor loop would be great in general, and perhaps preferable here


okay, so the ur robots will have to be created again.


the structure of the proto is not like the official urdf. mainly the tool link is missing.

##### David Mansolino [cyberbotics] 08/18/2020 10:33:30
We can indeed add the tool link, but we should be carreful to not break compatibility (e.g. changing motor/sensor names) when updating it.

##### Simon Steinmann [Moderator] 08/18/2020 10:34:57
those are correct. the issue will be the tool slot. with the webots model, the y-axis points out, with the urdff model, the x-axis points out. But we can adjust that to be the same, like the old webots model

##### Olivier Michel [cyberbotics] 08/18/2020 10:35:29
```python
function getNode(node, name):
    nameField = node.getField('name'):
    if nameField:
        if nameField.getSFString() == name:
            return node
    children = node.getField('children')
    if children:
        for i in range(children.getCount()):
                child = children.getMFNode(i)
                n = getNode(child, name)
                if n:
                    return n
    return None

proto = supervisor.getFromDef('MY_PROTO_ROBOT')
root = proto.getFromProtoDef('MY_ROBOT_ROOT')
node = getNode(root, 'my_searched_name')
```


Something like this should do the job (not tested).

##### Simon Steinmann [Moderator] 08/18/2020 10:37:03
thx I will test it

##### David Mansolino [cyberbotics] 08/18/2020 10:38:01
I think there is an issue:

n = getSFNode(child, name)  = > n = getNode(child, name)

##### Simon Steinmann [Moderator] 08/18/2020 10:44:32
root = proto.getProtoFromDef('MY\_ROBOT\_ROOT') 

this is not a valid function


what would be 'MY\_ROBOT\_ROOT'


is the getProto a new develop-branch function?

##### David Mansolino [cyberbotics] 08/18/2020 10:45:39
no, it is in R2020b

##### Simon Steinmann [Moderator] 08/18/2020 10:46:00
oh, running a

##### David Mansolino [cyberbotics] 08/18/2020 10:46:08
But the code is probably wrong:

=> root = supervisor.getProtoFromDef('MY\_ROBOT\_ROOT')

##### Simon Steinmann [Moderator] 08/18/2020 10:52:47
would have to install 2020b, With the tar install, I just have to extract and change environment variable right?

##### David Mansolino [cyberbotics] 08/18/2020 10:53:05
Yes exactly

##### Simon Steinmann [Moderator] 08/18/2020 10:54:40
for ubuntu 18.04 and up I take 

webots-R2020b-rev1-x86-64.tar.bz2 

right?

##### David Mansolino [cyberbotics] 08/18/2020 10:55:13
For ubuntu 18.04 you can take the webots-R2020b-rev1-x86-64\_ubuntu-16.04.tar.bz2


For up you can take webots-R2020b-rev1-x86-64.tar.bz2

##### Simon Steinmann [Moderator] 08/18/2020 10:55:45
can or should? :p

##### David Mansolino [cyberbotics] 08/18/2020 10:55:50
That's indeed not clearly said


*should 😉

##### Simon Steinmann [Moderator] 08/18/2020 10:56:00
okay, thx ^

##### David Mansolino [cyberbotics] 08/18/2020 10:56:23
You're welcome

##### Simon Steinmann [Moderator] 08/18/2020 12:28:27
is there a quick way to set max and min position of a motor from code? or to ignore limits

##### David Mansolino [cyberbotics] 08/18/2020 12:29:15
No, you can only get the min and max, but the robot is not able to change them (except using the Supervisor API).

##### Simon Steinmann [Moderator] 08/18/2020 12:29:47
node from device is not implemented in the main build yyet right?

##### Olivier Michel [cyberbotics] 08/18/2020 12:30:40
I believe it is implemented in the nightly build of R2021a.

##### David Mansolino [cyberbotics] 08/18/2020 12:30:45
Only on the develop branch

##### Simon Steinmann [Moderator] 08/18/2020 12:31:05
oh man, ikpy uses >= for limits, webots uses just >


WARNING: UR10e (PROTO) > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > RotationalMotor: too low requested position: -6.28319 < -6.28319

WARNING: UR10e (PROTO) > HingeJoint > RotationalMotor: too low requested position: -6.28319 < -6.28319

WARNING: UR10e (PROTO) > HingeJoint > Solid > HingeJoint > RotationalMotor: too low requested position: -6.28319 < -6.28319

WARNING: UR10e (PROTO) > HingeJoint > RotationalMotor: too low requested position: -6.28319 < -6.28319

WARNING: UR10e (PROTO) > HingeJoint > Solid > HingeJoint > RotationalMotor: too low requested position: -6.28319 < -6.28319

WARNING: UR10e (PROTO) > HingeJoint > RotationalMotor: too low requested position: -6.28319 < -6.28319

WARNING: UR10e (PROTO) > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > RotationalMotor: too big requested position: 6.28319 > 6.28319

WARNING: UR10e (PROTO) > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > RotationalMotor: too low requested position: -6.28319 < -6.28319

WARNING: UR10e (PROTO) > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > RotationalMotor: too low requested position: -6.28319 < -6.28319

WARNING: UR10e (PROTO) > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > RotationalMotor: too big requested position: 6.28319 > 6.28319


😄


finished with the universal IK controller


can take it for a spin
> **Attachment**: [webots\_IK\_controller2.py](https://cdn.discordapp.com/attachments/565155651395780609/745258824138489926/webots_IK_controller2.py)


only requires the robot to be supervisor, and a target node, where it gets position and orientation



> **Attachment**: [lbr\_iiwa\_IK.zip](https://cdn.discordapp.com/attachments/565155651395780609/745260355437199380/lbr_iiwa_IK.zip)


full world, easier to test 🙂


that urdf export function is awesome!


perhaps adding it to the gui would be good. Right click robot - export - as urdf


...testing with irb... the whole calling the joints after the sensors is really screwing with the ik controller here, because it is not using the convention of the sensor being named 'motorname\_sensor'


is there a way to get the motor name, given the sensorname?

##### David Mansolino [cyberbotics] 08/18/2020 13:20:11
Yes: [https://cyberbotics.com/doc/reference/motor#wb\_motor\_get\_position\_sensor](https://cyberbotics.com/doc/reference/motor#wb_motor_get_position_sensor)

##### Simon Steinmann [Moderator] 08/18/2020 13:20:54
[https://tenor.com/view/yay-market-yellow-man-gif-6195464](https://tenor.com/view/yay-market-yellow-man-gif-6195464)


omg it works!!!

##### David Mansolino [cyberbotics] 08/18/2020 13:30:32
🙂

##### Simon Steinmann [Moderator] 08/18/2020 13:31:04
added a fix for singularities, and generating the active link mask is now independant of the naming scheme

##### Olivier Michel [cyberbotics] 08/18/2020 13:31:59
I just fixed a couple of errors in my code snippet (it should be slightly more correct now).

##### Simon Steinmann [Moderator] 08/18/2020 13:32:42
is it compatible with 2020a? I'm not running it, but would be nice for other people to be able to use it

##### Olivier Michel [cyberbotics] 08/18/2020 13:33:01
No. It requires R2020b.

##### Simon Steinmann [Moderator] 08/18/2020 13:33:14
can you paste it again?

##### Olivier Michel [cyberbotics] 08/18/2020 13:33:52
```python
def getNode(node, name):
    nameField = node.getField('name')
    if nameField:
        if nameField.getSFString() == name:
            return node
    children = node.getField('children')
    if children:
        for i in range(children.getCount()):
            child = children.getMFNode(i)
            n = getNode(child, name)
            if n:
                return n
    return None

proto = supervisor.getFromDef('MY_PROTO_ROBOT')
root = proto.getFromProtoDef('MY_ROBOT_ROOT')
node = getNode(root, 'my_searched_name')
```

##### Simon Steinmann [Moderator] 08/18/2020 13:38:29
what is MY\_ROBOT\_ROOT?

##### Olivier Michel [cyberbotics] 08/18/2020 13:40:02
It's the DEF node inside your proto definition.


The one that contains the node you are looking for.

##### Simon Steinmann [Moderator] 08/18/2020 13:43:32
oh so the DEF of the robot with the supervisor controller?


would that not be the same as MY\_PROTO\_ROBOT?

##### Olivier Michel [cyberbotics] 08/18/2020 13:45:57
No, it's not the same. In your world file, you will have:


```
DEF MY_PROTO_ROBOT MyRobot { ... }
```


And in your `MyRobot.proto` file:


```
PROTO MyRobot [ ... ] { DEF MY_ROBOT_ROOT Robot { ... } }
```

##### Simon Steinmann [Moderator] 08/18/2020 13:48:50
there is no DEF in most proto files



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/745278094939783168/unknown.png)
%end

##### Olivier Michel [cyberbotics] 08/18/2020 13:49:48
You should add one. Or use one if any at a lower level.

##### Simon Steinmann [Moderator] 08/18/2020 13:53:11
trying to make it work for general and existing protos


the screenshot above is your model 😉

##### Olivier Michel [cyberbotics] 08/18/2020 13:54:19
I know. Then we probably need another API function to get the root proto node even if it has no DEF...

##### Simon Steinmann [Moderator] 08/18/2020 13:55:14
requireing no argument would be good, as we arleady specify which node.

##### Olivier Michel [cyberbotics] 08/18/2020 13:57:28
Yes, something like `wb_supervisor_node_get_from_proto_root`.

##### Simon Steinmann [Moderator] 08/18/2020 14:00:12
exactly 🙂


\# ikpy includes the bounds as valid, In Webots they have to be less than the limit

        new\_lower = armChain.links[i].bounds[0] * 0.9999999

        new\_upper = armChain.links[i].bounds[1] * 0.9999999        

        armChain.links[i].bounds = (new\_lower, new\_upper)


😄


problem fixed :p still makes me smile

##### Olivier Michel [cyberbotics] 08/18/2020 14:01:40
It's maybe less dangerous to do:

##### Simon Steinmann [Moderator] 08/18/2020 14:03:49
hmm, so my IK controller now works flawlessly with all the robots (at least kuka) in the community projects. Is it possible to add a controller on a higher abstraction level and not a per-world basis?

##### Olivier Michel [cyberbotics] 08/18/2020 14:03:54
```python
        new_lower = armChain.links[i].bounds[0] - 0.0000001 if armChain.links[i].bounds[0] > 0 else armChain.links[i].bounds[0] + 0.0000001
        new_upper = armChain.links[i].bounds[1] - 0.0000001 if armChain.links[i].bounds[1] > 0 else armChain.links[i].bounds[1] + 0.0000001
        armChain.links[i].bounds = (new_lower, new_upper)
```

##### Simon Steinmann [Moderator] 08/18/2020 14:04:30
i'm not dividing, so there should be no issues?!


ohhhhh


because of 0

##### Olivier Michel [cyberbotics] 08/18/2020 14:04:50
Yes, I would recommend you to add such a controller in a folder named "generic\_inverse\_kinematic" or the like.

##### Simon Steinmann [Moderator] 08/18/2020 14:04:50
yeah

##### Olivier Michel [cyberbotics] 08/18/2020 14:05:20
Yes, there might be rounding problems with small values.

##### Simon Steinmann [Moderator] 08/18/2020 14:05:27
where would that folder be? I added the whole comm.proj. git to extra paths


new\_lower = armChain.links[i].bounds[0] + 0.0000001

        new\_upper = armChain.links[i].bounds[1] - 0.0000001       

        armChain.links[i].bounds = (new\_lower, new\_upper)

this should be enough, since lower is always smaller than upper

##### Olivier Michel [cyberbotics] 08/18/2020 14:07:55
Probably we should create a `controllers` folder at the level and put it in there.

##### Simon Steinmann [Moderator] 08/18/2020 14:08:13
that would be good

##### Olivier Michel [cyberbotics] 08/18/2020 14:08:31
Like [https://github.com/cyberbotics/community-projects/tree/master/controllers/generic\_inverse\_kinematic/](https://github.com/cyberbotics/community-projects/tree/master/controllers/generic_inverse_kinematic/)

##### Simon Steinmann [Moderator] 08/18/2020 14:08:46
404

##### Olivier Michel [cyberbotics] 08/18/2020 14:08:54
Sure, you need to create it.


Feel free to open a PR to do so.

##### Simon Steinmann [Moderator] 08/18/2020 14:11:58
does not find the controller

##### David Mansolino [cyberbotics] 08/18/2020 14:13:38
> Like [https://github.com/cyberbotics/community-projects/tree/master/controllers/generic\_inverse\_kinematic/](https://github.com/cyberbotics/community-projects/tree/master/controllers/generic_inverse_kinematic/)

`@Olivier Michel` this does not respect the Webots project hierarchy

##### Simon Steinmann [Moderator] 08/18/2020 14:14:10
wait, it doesnt find anything in the extra path now

##### David Mansolino [cyberbotics] 08/18/2020 14:14:16
You should rather put it in somethig like 'default/controllers/generic\_inverse\_kinematic'

##### Simon Steinmann [Moderator] 08/18/2020 14:15:14
that was it


putting controller in the base directory, removes everything

##### Olivier Michel [cyberbotics] 08/18/2020 14:15:37
OK, in that case, I would prefer something like `generic/controllers/inverse_kinematic`

##### David Mansolino [cyberbotics] 08/18/2020 14:16:27
That's fine too, I was suggesting 'default' to be like in Webots ([https://github.com/cyberbotics/webots/tree/master/projects](https://github.com/cyberbotics/webots/tree/master/projects)), but it is not mandatory to be exactly the same.

##### Simon Steinmann [Moderator] 08/18/2020 14:16:58
I like consistency


but up to you guys

##### David Mansolino [cyberbotics] 08/18/2020 14:17:18
And as we already have a n'inverse\_kinematic' controller in Webots looks safer to avoid duplicating it.

##### Olivier Michel [cyberbotics] 08/18/2020 14:17:59
OK, so let's go for the solution proposed by `@David Mansolino`.

##### Simon Steinmann [Moderator] 08/18/2020 14:19:43
perhaps that target sphere can be generated by the controller. that would make the implementation even simpler

##### Olivier Michel [cyberbotics] 08/18/2020 14:20:29
Yes, that could be a nice demo.

##### Simon Steinmann [Moderator] 08/18/2020 14:20:50
i can't upload to github through the webpage 😦

##### Olivier Michel [cyberbotics] 08/18/2020 14:21:01
(it may be optional, triggered by a controllerArg)


You need to create a new branch

##### Simon Steinmann [Moderator] 08/18/2020 14:21:55
usually I can upload and crerate a new branch from those files


you guys do it 😉
> **Attachment**: [default.zip](https://cdn.discordapp.com/attachments/565155651395780609/745286472424489010/default.zip)

##### Olivier Michel [cyberbotics] 08/18/2020 14:22:43
Did you checked out locally the community-projects repo?

##### Simon Steinmann [Moderator] 08/18/2020 14:23:10
I dont wanna pull, because I have many working changes, guess I could clone again for upload

##### Olivier Michel [cyberbotics] 08/18/2020 14:24:13
I see. So from GitHub web page, you should be able to create a branch and upload your controller files on this branch (creating the folder structure at the same time).


1. Go on the "Code" tab.


2. Click on the "master" pop-up menu and create a branch (give it a name)


3. On this branch, click "Add files" popup menu.


4. Upload your two python files.

##### Simon Steinmann [Moderator] 08/18/2020 14:26:11
still something wrong


doesnt let me


Something went really wrong, and we can’t process that file.

##### Olivier Michel [cyberbotics] 08/18/2020 14:26:21
Yes, I see.


You should create a temporary dummy file.


Click "Add file" / "Create new files".


Enter "default/controllers/generic\_inverse\_kinematics/dummy.txt"

##### Simon Steinmann [Moderator] 08/18/2020 14:28:06
that seems to work

##### Olivier Michel [cyberbotics] 08/18/2020 14:28:18
Then, you will be able to upload your files and delete the dummy.txt file.

##### Simon Steinmann [Moderator] 08/18/2020 14:28:55
[https://github.com/cyberbotics/community-projects/pull/12](https://github.com/cyberbotics/community-projects/pull/12)


how do I revert commits?

##### Olivier Michel [cyberbotics] 08/18/2020 15:26:02
Simply commit the previous version of the file.

##### Simon Steinmann [Moderator] 08/18/2020 15:42:15
finally done 😄


how would I spawn that target sphere?

##### Olivier Michel [cyberbotics] 08/18/2020 15:44:20
Probably with `importSFNodeFromString`: [https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_field\_import\_sf\_node\_from\_string](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_import_sf_node_from_string)

##### Simon Steinmann [Moderator] 08/18/2020 15:46:14
as a string I can just use the world file part of the node then?

##### Olivier Michel [cyberbotics] 08/18/2020 15:46:37
Yes.

##### Simon Steinmann [Moderator] 08/18/2020 15:46:44
gonna try 🙂


how can I get the number of children? I wanna insert the target node on the bottom

##### Olivier Michel [cyberbotics] 08/18/2020 15:58:13
Get the root node, its children field and get the number of items (see my code sample)


`children.getCount()`

##### Simon Steinmann [Moderator] 08/18/2020 15:59:13
just saw, thx 🙂


okay works beautifully


can I link textures relative to the controller? or would it be easier to copy the files into the world folder?

##### Olivier Michel [cyberbotics] 08/18/2020 16:06:54
It's better to copy them in the worlds folder under a textures subfolder.

##### Simon Steinmann [Moderator] 08/18/2020 16:07:39
how can I get the path to the world?

##### Olivier Michel [cyberbotics] 08/18/2020 16:09:38
There must be a project path environment variable... Let me search...


Can you try to getenv and print WEBOTS\_PROJECT environment variable? Then append "/worlds" to it should do the trick.


No, that won't work...

##### Simon Steinmann [Moderator] 08/18/2020 16:13:36
it's not a system variable

##### Olivier Michel [cyberbotics] 08/18/2020 16:14:29
Otherwise, you can assume it's `../../worlds/` relative to your controller.

##### Simon Steinmann [Moderator] 08/18/2020 16:14:49
well, using it for the generic controller 😉

##### Olivier Michel [cyberbotics] 08/18/2020 16:17:17
Unfortunately, I don't see any better way to do this...

##### Simon Steinmann [Moderator] 08/18/2020 16:17:36
shucks


I think I'll just remove the texture and make the sphere yellow


is there a conflict, if I inialize a supervisor in my main controler, and also in a imported module?


meaning, do I have to parse the supervisor for the module-class \_\_init\_\_ function or can it just import it directly


figured it out, have to parse it. Can't init supervisor twice


btw, figured out the world path problem



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/745346993257185430/unknown.png)
%end

##### David Mansolino [cyberbotics] 08/18/2020 19:25:45
> Can you try to getenv and print WEBOTS\_PROJECT environment variable? Then append "/worlds" to it should do the trick.

`@Olivier Michel` `@Simon Steinmann`  we have an API function for this: Robot.getWorldPath()

##### Simon Steinmann [Moderator] 08/18/2020 19:26:29
what I used 😉


that returns the worldpath including the filename though, that's why I needed the next 2 lines to remove it

##### David Mansolino [cyberbotics] 08/18/2020 19:27:23
> what I used 😉

`@Simon Steinmann` just saw it (I was reading messages from top to bottom)

##### Simon Steinmann [Moderator] 08/18/2020 19:28:29
it works super well now. you just have to assignt the controller and do nothing else. If there is no target node, it adds it for you. Even tells you to enable supervisor, if not done so already

##### David Mansolino [cyberbotics] 08/18/2020 19:29:52
> that returns the worldpath including the filename though, that's why I needed the next 2 lines to remove it

`@Simon Steinmann` a simplest solution would be:

```Python
worldPath = os.path.dirname(supervisor.getWorldPath())
```

##### Simon Steinmann [Moderator] 08/18/2020 19:30:08
that's cleaner

##### David Mansolino [cyberbotics] 08/18/2020 19:30:11
> it works super well now. you just have to assignt the controller and do nothing else. If there is no target node, it adds it for you. Even tells you to enable supervisor, if not done so already

`@Simon Steinmann` looks very nice 🙂

##### Simon Steinmann [Moderator] 08/18/2020 19:35:45
but we will have to check and recompile several official robot arms, as they dont include the last link, when converting to urdf. Good idea anyways, using multi-file, so viewing and editing PROTO-source is less laggy


`@alxy` you can merge your PR

`@Olivier Michel` `@David Mansolino` I created a new PR for the Mirobot, already including alxy's changes:

[https://github.com/cyberbotics/community-projects/pull/14](https://github.com/cyberbotics/community-projects/pull/14)

##### alxy 08/20/2020 18:09:14
Did you check if it works if the bounds are not set?

##### Simon Steinmann [Moderator] 08/20/2020 18:09:30
the bounds get exported


I now found an issue, where attached things to the toolSlot fall down 😄


added a bit of code that improves the singularity handling
> **Attachment**: [ik\_module.py](https://cdn.discordapp.com/attachments/565155651395780609/746068745415557170/ik_module.py)

##### alxy 08/20/2020 18:11:08
Ok, so I will now merge my PR, however I guess it doesnt really matter....

##### Simon Steinmann [Moderator] 08/20/2020 18:11:46
you can attach grippers without falling down?

##### alxy 08/20/2020 18:11:56
yes?


you can check my world in my repo, my gripper works...

##### Simon Steinmann [Moderator] 08/20/2020 18:45:22
figured it out and fixed it


you can check out my model in the PR above. I'd recommend using it, as it is super lightweight and easy to edit, due to the meshes being in seperate files

##### alxy 08/20/2020 19:38:05
yeah, will see if I can do that tomorrow. I also plan to export my gripper to a proto file again

##### Simon Steinmann [Moderator] 08/21/2020 10:54:52
`@David Mansolino` Could you tell me why [https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/writeProto.py#L462](https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/writeProto.py#L462) this condition is needed? I created a batch-convertion, automatically converting a whole nested directory of urdf files (with options defined in .json configs). And for some reason, this line stops SOME, but not all Mesh protos to be created. I spent 5 hours yesterday trying to figure that out.  Setting this line to 

if True: 

fixes this issue. My question is, why is it needed?

##### David Mansolino [cyberbotics] 08/21/2020 11:40:50
This is part of the automatic DEF-USE mechanism to avoid defining twice the same mesh, if you set the condition to True, then it will an any case export the mesh and disable DEF-USE mechanism

##### Simon Steinmann [Moderator] 08/21/2020 11:43:39
Once we are through with the current PR (just updated it), I'll upload the batch conversion. Perhaps you can then take a look. The issue is super weird


the issue came up, when I rant the batch conversion inside a class. removed the class and just used normal functions. Seems to work now... super strange

##### alxy 08/21/2020 12:48:55
What do you guys think? Is it somehow possible to interface the original firmware ( [https://github.com/wlkata/WLkata-Mirobot-Firmware](https://github.com/wlkata/WLkata-Mirobot-Firmware) ) for the microcontroller of the real robot with webots? I have no idea if this is could even somehow be possible, but I imagine much more realistic movements and trajectories of the robot, as it uses the original algorithms to calculate motor positons...

##### Simon Steinmann [Moderator] 08/21/2020 12:50:05
of course it's possible, but not easy. Feel free to do it 😉


I'm a python guy. c scares me 😅


I can read it if I absolutely have to, but I can't code in it

##### alxy 08/21/2020 12:52:03
Yeah, Im on the same side, but some C code I can really hardly read


the steps would be to first kind of build the program and see how to map input and output I assume?


On a first glance it appears these guys who coded the firmware in C are wizards 😄

##### Simon Steinmann [Moderator] 08/21/2020 12:55:35
`@David Mansolino` do you have the urdf files and meshes of the robots you created?

##### David Mansolino [cyberbotics] 08/21/2020 12:55:50
Which ones?

##### Simon Steinmann [Moderator] 08/21/2020 12:56:02
as many as possible 😉

##### David Mansolino [cyberbotics] 08/21/2020 12:56:37
Unfortunately, most (almost all) of the robots are designed from scratch without using the URDF files 😕

##### Simon Steinmann [Moderator] 08/21/2020 12:56:45
ohh okay


right now, I can convert the entire community projects repo, plus universal robots ur3, 5 and 10 in like 10 seconds


making improvements to the convertion tools, allows to reconvert and update all robots

##### David Mansolino [cyberbotics] 08/21/2020 12:58:06
That sounds very nice 😄

##### Simon Steinmann [Moderator] 08/21/2020 12:58:11
The difference with mutli-file for the user-experrience is HUGE

##### David Mansolino [cyberbotics] 08/21/2020 12:58:28
> The difference with mutli-file for the user-experrience is HUGE

`@Simon Steinmann` yes indeed, it much simpler for the maintenance too!

##### Simon Steinmann [Moderator] 08/21/2020 12:58:31
editing a 5MB proto file is just so horrible


approved the changes

##### David Mansolino [cyberbotics] 08/21/2020 13:03:31
Cool, remains just to update:

  - [https://github.com/cyberbotics/urdf2webots/blob/Simon-Steinmann-nameFixes%26more/README.md](https://github.com/cyberbotics/urdf2webots/blob/Simon-Steinmann-nameFixes%26more/README.md)

  - [https://github.com/cyberbotics/urdf2webots/blob/Simon-Steinmann-nameFixes%26more/docs/tutorial.md](https://github.com/cyberbotics/urdf2webots/blob/Simon-Steinmann-nameFixes%26more/docs/tutorial.md)

##### Simon Steinmann [Moderator] 08/21/2020 13:03:46
😅  oh yeah right


pushed update doc

##### David Mansolino [cyberbotics] 08/21/2020 13:37:00
Just a small typo.

##### alxy 08/21/2020 13:44:49
Ok, after googling around it appears more or less impossible for me 😄

##### Simon Steinmann [Moderator] 08/21/2020 13:45:17
i'm sure the real robot has position control right?


if your max vel is low enough, it should be pretty much 1 to 1


where you run into differences is when using IK. Most real robots also have cartesian control, using IK in their controller. That might be quite different

##### alxy 08/21/2020 13:46:42
position control as in your provide coordinates and it moves?

##### Simon Steinmann [Moderator] 08/21/2020 13:46:54
position control is joint angles

##### alxy 08/21/2020 13:47:13
ah, yeah you can do that, obviously that is something we dont use at all 😄

##### Simon Steinmann [Moderator] 08/21/2020 13:47:15
cartesian control is xyz for the tool


but you can use your own ik, use it for both, the simulation and the real robot


and control the robot by adressing the joint angles directly

##### alxy 08/21/2020 13:48:15
that sounds good

##### Simon Steinmann [Moderator] 08/21/2020 13:50:31
where did you get the mirobot vel limits?


and are there torque limits too?

##### alxy 08/21/2020 13:51:08
I linked the document in the PR


[https://uploads-ssl.webflow.com/5edc22ea8e58c13e2883e09b/5efd245acdbcfb27deb727ce\_WLKATA%20Mirobot%20User%20Manual%20V1.2.pdf](https://uploads-ssl.webflow.com/5edc22ea8e58c13e2883e09b/5efd245acdbcfb27deb727ce_WLKATA%20Mirobot%20User%20Manual%20V1.2.pdf) page 55


but I have to say the docuemtnation is not entirely correct, I have already found some quite major errors

##### Simon Steinmann [Moderator] 08/21/2020 13:53:10
like what?

##### alxy 08/21/2020 13:53:15
They certainly do have torque limits, but they are not specified

##### Simon Steinmann [Moderator] 08/21/2020 13:53:34
yeah ofc they have. they are NOT 200Nm, that would be very impressive 😄

##### alxy 08/21/2020 13:53:49
They use gcodes to communicate, and there is one gcode to open the gripper, but it actually closes it


return values of the firmware to the serial connection are not correct


or not documented


or entirely different from what is documented

##### Simon Steinmann [Moderator] 08/21/2020 13:57:37
well i'm gonna add the velocities to the .urdf, so they get converted automatically

##### alxy 08/21/2020 13:57:52
Nice, thanks!

##### Simon Steinmann [Moderator] 08/21/2020 14:00:28
works, gets converted automatically now 🙂

##### alxy 08/21/2020 14:01:05
you have taken the converter to a whole new level! 🙂

##### Simon Steinmann [Moderator] 08/21/2020 14:01:43
really have over the past month. It was the reason I didnt switch a year earlier. Getting your own robot into the simulator is so important

##### alxy 08/21/2020 14:02:17
what did you use earlier?

##### Simon Steinmann [Moderator] 08/21/2020 14:02:22
Gazebo

##### alxy 08/21/2020 14:02:48
Ok, for me its my first experience with a robot simulation tool, or robotics in general 😄

##### Simon Steinmann [Moderator] 08/21/2020 14:04:02
you picked a good one. For me the main advantages of Webots are:

- excellent and detailed documentation

- native API in multiple languages (not incomplete and slow wrappers)

- this discord, where you get answers immediatley

##### alxy 08/21/2020 14:04:32
yeah, the support is really impresive

##### Simon Steinmann [Moderator] 08/21/2020 14:04:54
pllus it's easy to get changes implemented if you make them 😄


wooo, PR merged


`@alxy` `@David Mansolino` [https://github.com/cyberbotics/urdf2webots/pull/74](https://github.com/cyberbotics/urdf2webots/pull/74)


please check it out and see if it works for you


was interesting creating this, super fun. Sat at it till from like 4pm to 5am 😅


FINALLY!!!!!!!!!!! figured out the issue with some meshes not being created. Collision and Material references get added. Have to manually reset them:

importer.urdf2webots.parserURDF.Geometry.reference = {}

importer.urdf2webots.parserURDF.Material.namedMaterial = {}


so the <line> part in collada meshes dont get correctly converted.  It tries to create a triangle mesh, which of course fails (coordIndex is empty)
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/746425729868103701/unknown.png)
%end


fixed it, assuming that LineSets cannot get properly imported into Webots:


[https://github.com/cyberbotics/urdf2webots/pull/75](https://github.com/cyberbotics/urdf2webots/pull/75)

##### alxy 08/22/2020 14:47:49
yeah, the PRs definetly need fixing 😄


I'd also suggest to use your own fork for the PRs, otherwise the main repo gets cluttered with all these branches


`@Simon Steinmann` Sorry to bug you again, but I have found the original gripper for Mirobot in github as step (stp) file: [https://github.com/CBerauer/Mirobot-CAD-Files/tree/master/Gripper](https://github.com/CBerauer/Mirobot-CAD-Files/tree/master/Gripper) I just installed a viewer and it indeed looks good, however I am now struggling how to convert that to webots? Is it possible?

##### Simon Steinmann [Moderator] 08/22/2020 15:57:10
you gonna have to convert it to .obj, stl or .dae for proto conversion. Or check the import in webots itself. it supports a bunch of formats

##### alxy 08/22/2020 16:03:56
Mh, exported to stl and was able to successfully import, but nothing shows up. It created a bunch of new elements under a solid node though, but I dont see the gripper

##### Simon Steinmann [Moderator] 08/22/2020 16:06:05
having a urdf file or something that dictates the structure would be good

##### alxy 08/22/2020 16:07:15
yeah, I have just realized it really only contains the 3d mockup, no joints defined


So I cannot really use it, sadly 😦

##### Simon Steinmann [Moderator] 08/22/2020 16:08:40
what is the name of that gripper?


you could try to manually assemble them

##### alxy 08/22/2020 16:09:56
I dont think it has a dedicated name?

##### Simon Steinmann [Moderator] 08/22/2020 16:10:23
except the one you linked, I find no mirobot package containing a gripper

##### alxy 08/22/2020 16:10:25
It comes with Mirobot in the satndard package


yeah, they did not release the urdf for the gripper, just for the arm itself


Not sure where that guy got the 3d model from, maybe he has assembled it on his own

##### Simon Steinmann [Moderator] 08/22/2020 16:12:19
well, having the parts, it should not be too complicated assembing it

##### alxy 08/22/2020 16:12:55
given I have no experience with any cad/3d software, I can imagine it is very hard 😄

##### Simon Steinmann [Moderator] 08/22/2020 16:13:26
no, in webots, or with urdf.

##### alxy 08/22/2020 16:14:31
yeah that might be possible, but not if it gets created as a signle IndexedFaceSet, the parts are not separate nodes in webots

##### Simon Steinmann [Moderator] 08/22/2020 16:15:40
that would not be ideal :p


perhaps you can convert it to another format, which includes data for the different components

##### alxy 08/22/2020 16:16:17
I checked out the other file (catpart) in inventor, that looks more promising


do you know which formats preserve the structure?

##### Simon Steinmann [Moderator] 08/22/2020 16:17:10
collada (.dae) does. But i'm not an expert in this


.dae has the advantage to be readable in a text editor

##### alxy 08/22/2020 16:19:36
of course that is not available as an export option 😄

##### Simon Steinmann [Moderator] 08/22/2020 16:20:20
A new Mesh node was added to support the inclusion of a large variety of external 3D mesh files: 3D Studio mesh, Blender, Biovision Hierarchy, Collada, Filmbox, STL, Wavefront, X3D. It is also possible to import such files as Webots primitives.

##### alxy 08/22/2020 16:21:09
Yeah, I checked the list on the import wizzward with the list of available export options


I try .obj now


lol, scales dont seem to match:
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/746766447845310506/unknown.png)
%end


can you find my mirobot 😄

##### Simon Steinmann [Moderator] 08/22/2020 16:25:13
nope :p


perhaps it uses millimeters as its unit


so that would be 1000x


perhaps you can change the unit somewhere before exporting?

##### alxy 08/22/2020 16:26:03
yeah, its in that black whole facing the front. You can see it


but the problem persists with the obj, it is just one single node in webots then

##### Simon Steinmann [Moderator] 08/22/2020 16:29:18
what program did you load the file into?

##### alxy 08/22/2020 16:30:08
autodesk inventor


and I can see the hierarchy of different parts there


so it is somehow encoded in the file

##### Simon Steinmann [Moderator] 08/22/2020 16:30:43
can you export individual parts?

##### alxy 08/22/2020 16:30:49

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/746768350914281633/unknown.png)
%end


No idea, as said I have no prior experience with CAD programs. The interface looks rather complicated to me

##### Simon Steinmann [Moderator] 08/22/2020 16:32:43
great opportunity to learn 😉

##### alxy 08/22/2020 16:33:20
Its a bit far from what I wanted to achieve originally 😄

##### Simon Steinmann [Moderator] 08/22/2020 16:33:35
can never hurt

##### alxy 08/22/2020 16:34:02
probably not, if just time permits to do all the things... 😄


the easier thing is to first try all the other available cad programs and see if they can export to other formats (I will try blender now)

##### Simon Steinmann [Moderator] 08/22/2020 16:36:36
fusion 360 might be a good optioni

##### alxy 08/22/2020 16:44:45
[https://github.com/rjvallett/URDF-Converter](https://github.com/rjvallett/URDF-Converter) that looks promising, but hasnt been updated in a while and lacks any documentation

##### Simon Steinmann [Moderator] 08/22/2020 16:45:56
try it 🙂


here we go
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/746773437116186765/unknown.png)
%end


used this: [https://cadexchanger.com/downloads](https://cadexchanger.com/downloads)

to convert it to a .dae file. It needed some editing, but managed to import it

##### alxy 08/22/2020 17:05:47
but is it in separate parts?


I mean importing was not an issue, that worked with either of the format


it is a sigle IndexedFacedSet for me


`@Simon Steinmann`

##### Simon Steinmann [Moderator] 08/22/2020 17:16:03
seperate parts


I'm adding jonits and motors atm

##### alxy 08/22/2020 17:16:21
wow, thanks 🙂

##### Simon Steinmann [Moderator] 08/22/2020 17:16:48
never done it that way, learning opportunity 😉

##### alxy 08/22/2020 17:17:48
I really dont know why the manufacturer does not provide the files for the gripper. It is basically useless to do simulations without the gripper I assume

##### Simon Steinmann [Moderator] 08/22/2020 17:18:02
[https://tenor.com/view/elmo-shrug-gif-5094560](https://tenor.com/view/elmo-shrug-gif-5094560)


if you're curious, we could zoom and I show you what i'm doing

##### alxy 08/22/2020 17:18:36
sure, are you from germany btw?

##### Simon Steinmann [Moderator] 08/22/2020 17:18:41
jep

##### alxy 08/22/2020 17:19:10
ok, so let me install zoom then

##### Simon Steinmann [Moderator] 08/22/2020 19:10:02
preliminary proto for the gripper. names for meshes are still bonked, and of course motors have no limits
> **Attachment**: [MirobotGripper.proto](https://cdn.discordapp.com/attachments/565155651395780609/746808418400206853/MirobotGripper.proto)


you owe me a beer once rona is over 😄
> **Attachment**: [for\_alex.zip](https://cdn.discordapp.com/attachments/565155651395780609/746859366459899914/for_alex.zip)


`@alxy` wrote a very simple and easy to include controller too

##### alxy 08/23/2020 08:02:43
Ok, yeah, sure. Will test it now 🙂


The Mirobot.proto is the current one from the community project?

##### Simon Steinmann [Moderator] 08/23/2020 08:04:58
either that or a newly converted, but with correct values


it doesnt contain the mirobot, so it is the same model you have

##### alxy 08/23/2020 08:08:24
Nice, works very well 🙂


but it really contains a lot of motors 😄

##### Simon Steinmann [Moderator] 08/23/2020 08:10:03
uses 3 virtual motors per gripper side, instead of 1


but all 6 gripper motors can be moved with the same position command to have it open and close correctly

##### alxy 08/23/2020 08:12:44
yeah, thats indeed vbery easy to use like that. I had thought ahout a rather complex controller script when you told me about the virtual motors yesterday...


I am very curious how good it matches the real bot in the end. It looks really close now

##### Simon Steinmann [Moderator] 08/23/2020 08:25:07
you gonna have to adjust the torques perhaps


the problem is, that the motors should be exactly the same value


perhaps use torque feedback

##### alxy 08/23/2020 08:27:29
ok, will see, I needed to do that for the old gripper as well to properly grab the wooden boxes

##### Simon Steinmann [Moderator] 08/23/2020 08:28:04
I have a script somewhere to do that. I can check later, when I'm on linux


basically I use high torque on the motors, but with torque feedback, and they incrementally close each timestep, as long as the torque is lower than a threshhold

##### alxy 08/23/2020 08:29:26
why dont you set it too some lower, more realistic torque?

##### Simon Steinmann [Moderator] 08/23/2020 08:29:28
then they all stop


because you have 3 virtual motors, they have to have the EXACT same value, for them to be parallel


and they have different level action. so torques will be different


you could manually synchronize all motors to let's say on main motor, but then all the 'slave' motors will be one timestep behind


perhaps you can come up with a better solution 😄


if you do, let me know 😉

##### alxy 08/23/2020 08:33:59
Im not entirely sure if I understand the problem, but I will see what happens. I am going to integrate the new gripper with my IP based controller first and then go on to see if I can grab something 🙂

##### Simon Steinmann [Moderator] 08/23/2020 08:35:10
good luck. Let me know if you have any changes or improvements for the model. We should probably add it to community projects

##### alxy 08/23/2020 08:36:48
yeah, I can do it if you want me to, but as its your work, you should probably do it 😄

##### Simon Steinmann [Moderator] 08/23/2020 08:57:32
well, let me know if you have any improvements or feedback, so I can improve the model and then upload it

##### alxy 08/23/2020 09:01:59
I think the wrist motor is not necessary and not in the real robot as well, as the last joint of the arm has the same axis and can thus be used to achieve the same functionality. Maybe be usedfull if used with another arm though

##### Simon Steinmann [Moderator] 08/23/2020 09:28:17
I think you're right


[https://www.robotshop.com/media/catalog/product/cache/image/1350x/9df78eab33525d08d6e5fb8d27136e95/w/l/wlkata-6-axis-mini-robot-arm-mirobot-education-kit-2.jpg](https://www.robotshop.com/media/catalog/product/cache/image/1350x/9df78eab33525d08d6e5fb8d27136e95/w/l/wlkata-6-axis-mini-robot-arm-mirobot-education-kit-2.jpg)


you can try to do it yourself. Otherwise I'll do it later

##### alxy 08/23/2020 09:29:33
its not an issue, I just dont initialize or use this motor

##### Simon Steinmann [Moderator] 08/23/2020 09:29:37
little tip. You can convert it to basenodes, adjust it, then save the world. If you open the world file, you have the .proto definition in there


(open in text editor)

##### alxy 08/23/2020 09:33:24
but you are right, in this case its not enoug to simply change the troque limits, teh gripper is not able to grab like that


right\_1 is the actual motor which is actuated in the real robot?


> WARNING: Mirobot (PROTO) > MirobotGripper (PROTO) > Solid > HingeJoint > Solid > HingeJoint > RotationalMotor: wb\_motor\_enable\_torque\_feedback(): cannot be invoked for a Joint whose end point has no Physics node.


mh, simply adding Physics nodes to all solides makes it look a bit weird 😄

##### Simon Steinmann [Moderator] 08/23/2020 10:28:10
I already did that


my work in progress
> **Attachment**: [for\_alex2.zip](https://cdn.discordapp.com/attachments/565155651395780609/747039918563262504/for_alex2.zip)


torque feedback almost done

##### alxy 08/23/2020 12:11:52
Ok, I think I understand the code, but getTorqueFeedback() appears to always return nan? Why is that?

##### Simon Steinmann [Moderator] 08/23/2020 12:12:24

> **Attachment**: [MirobotGripper\_controller.py](https://cdn.discordapp.com/attachments/565155651395780609/747065705785983046/MirobotGripper_controller.py)


line 15 and 16 were wrong


have to enable the sensors, was late last night ^^


should work now

##### alxy 08/23/2020 12:14:52
yes, nice 🙂


thanks a lot!


I just think about rewriting the controller so that the gripperPosTorqueLimited does not call supervisor.step(), will see if I can get it working...

##### Simon Steinmann [Moderator] 08/23/2020 12:17:51
that is intentional


I guess you could add another function, that moves it closer to the target location for one increment

##### alxy 08/23/2020 12:19:22
I was thinking about a watcher pattern, that just monitors the torque at each step and fixes the positions to the current positions once a threshold is reached


I do something similar for the joint angles to determine if the robot is busy or idle

##### Simon Steinmann [Moderator] 08/23/2020 12:20:19
you will have to call it every time though. the current implementation, you only have to call once


sorry, typo
> **Attachment**: [MirobotGripper\_controller.py](https://cdn.discordapp.com/attachments/565155651395780609/747070845209280572/MirobotGripper_controller.py)


this one

##### alxy 08/23/2020 13:34:47
will see tomorrow how I can integrate it with my code. I was jsut a bnit concerned about having blocking calls inside the main controller loop, as I have other things which have to continuously work inside the loop, like the tcp/ip server

##### Simon Steinmann [Moderator] 08/23/2020 13:35:34
I added a function at the bottom, that only moves it incrementally, and you call the timestep in your main loop

##### alxy 08/23/2020 14:37:00
Minor issue, if the gripper only needs to move a very tiny bit, itnervals might be zero, hwich leads to an error (division by zero). I'll make sure to check if intervals > 0 🙂

##### Simon Steinmann [Moderator] 08/23/2020 14:38:01
they should not be, as i'm using math.ceil(), which always rounds up


perhaps there needs to be an abs() in there.

##### alxy 08/23/2020 14:38:35
then probably pos\_end exactly matched pos\_start

##### Simon Steinmann [Moderator] 08/23/2020 14:38:52
unlikely but possible

##### alxy 08/23/2020 14:38:56
if you call the function again with the exact same position


in your test code the sin always gives a different value for each time step


but if intervals > 0: really fixes the problem, so everything alright

##### Simon Steinmann [Moderator] 08/23/2020 14:42:01
rahter add a min(1, ...)

##### alxy 08/23/2020 14:42:37
do you mean max()  ?

##### Simon Steinmann [Moderator] 08/23/2020 14:42:46
oh yeah

##### alxy 08/23/2020 14:43:40
I have by the way now done it the following way. Added a target\_pos attribute to the class and the open and close actions set that, and in each iterationf og the controller loop I call this function: ``def run(self):

        self.gripperMoveIncrement(self.target\_pos, 2)``

##### Simon Steinmann [Moderator] 08/23/2020 14:45:23
works too

##### alxy 08/23/2020 14:47:31
yeah like that I can just use your GripperXController class in my main controller and just pass the calls for the gripper to that instance 🙂

##### Dave 08/24/2020 10:05:38
Hello everyone,

I would need some guidance please.



I am currently simulating en EPUCK on Webots with Python.



As I understand, the ground sensors functions are only available on C language but not on Python because it is an "extension" and it is not implemented in the API.



[https://cyberbotics.com/doc/guide/epuck](https://cyberbotics.com/doc/guide/epuck)



`In particular, the ground sensors module extension of the real e-puck robot is modelled using the "E-puckGroundSensors.proto" PROTO in Webots to provide 3 optional infra-red sensors pointing to the ground in front of the robot. `



How can I add the python functions for the ground sensors in my code please ? 



Thank you in advance for your help !



I have been searching for a solution, but it seems that I need to go trough PROTO ? Not sure what is Proto

##### David Mansolino [cyberbotics] 08/24/2020 10:06:29
HI `@Dave`, the ground sensor is available for Python too in simulation.


Here is an introduction about PROTO: [https://cyberbotics.com/doc/reference/proto](https://cyberbotics.com/doc/reference/proto)

##### Dave 08/24/2020 10:06:57
I don't understand how I can import in Python 😓

##### David Mansolino [cyberbotics] 08/24/2020 10:08:44
In any language, but you have to make sure that the 'groundSensorsSlot' field contains an 'E-puckGroundSensors' node.


Then they names of the ground sensors devices are 'gs0', 'gs1' and 'gs2'


they can be get the same way as the other distance sensors.

##### Dave 08/24/2020 10:10:03
Ok thank you! I will try again and be more persistence!


Thank you! At least I know that it is already implemented 😁

##### David Mansolino [cyberbotics] 08/24/2020 10:10:33
You're welcome

##### alxy 08/24/2020 16:35:04
lol, I now tried to figure out why the gripper isnt working in the greater setup for like an hour. Turns out I have used an old version of he gripper there without the Physics node, meaning torque is alsways nan


interestingly, it does not throw an error when compared to a number

##### Simon Steinmann [Moderator] 08/24/2020 16:36:55
doh

##### alxy 08/24/2020 16:42:50
Yeah, adding it an everything works as expected. I was really starting to become desperate 😄


`@Simon Steinmann` Whoops, I added the physics nodes and now I see the following phenomenon when moving the arm. What could that be?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/747497736135376906/unknown.png)
%end

##### Simon Steinmann [Moderator] 08/24/2020 16:49:32
what model are you using?

##### alxy 08/24/2020 16:50:01
The second one from yesterday, I just made the changes to the Solid proto node as well (adding the physics nodes)

##### Simon Steinmann [Moderator] 08/24/2020 16:50:10
i added physics nodes

##### alxy 08/24/2020 16:50:10
As they were only present in the Robot proto

##### Simon Steinmann [Moderator] 08/24/2020 16:50:32
did you convert it to base nodes?

##### alxy 08/24/2020 16:51:06
nope, I copied the children attribute of the robot node directly to the other proto file


[https://github.com/alxy/webots-mirobot/commit/209eae53e1233c5e59491447de03029aeb713093#diff-5f0cbd16163d674dc1f6dd994a25cf05](https://github.com/alxy/webots-mirobot/commit/209eae53e1233c5e59491447de03029aeb713093#diff-5f0cbd16163d674dc1f6dd994a25cf05)


ver weird that the preview is my own portrait

##### Simon Steinmann [Moderator] 08/24/2020 16:55:07

> **Attachment**: [MirobotGripper.zip](https://cdn.discordapp.com/attachments/565155651395780609/747499242016473188/MirobotGripper.zip)


both, the robot and the solid version have physics nodes


The issue with the meshes is, they are all in reference to the origin of the wrist mesh, so I needed to add transform nodes before the mesh nodes

##### alxy 08/24/2020 16:56:46
Thanks! I will check this version out


mh, same issue. It contains the same file I was using, just with one additional max position set

##### Simon Steinmann [Moderator] 08/24/2020 16:59:53
send me your project as a zip

##### alxy 08/24/2020 17:01:03
[https://github.com/alxy/webots-mirobot/archive/master.zip](https://github.com/alxy/webots-mirobot/archive/master.zip)


The controller wont work as it has a lot of dependencies, but the same happens when just using the manual motor control


mirobot\_gripper\_original is my world, I need to definetly clean up the projext once it is working, sorry for that


I can create a more minimal example if you want

##### Simon Steinmann [Moderator] 08/24/2020 17:03:31

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/747501356654526505/unknown.png)
%end

##### alxy 08/24/2020 17:03:53
as said, just use void controller

##### Simon Steinmann [Moderator] 08/24/2020 17:04:42
looks fine to me

##### alxy 08/24/2020 17:05:27
so if you move joint1 (which rotates the base) the gripper moves along with the arm?

##### Simon Steinmann [Moderator] 08/24/2020 17:05:40
ohh I see


it does, but turning wrist of gripper screws it

##### alxy 08/24/2020 17:06:07
I think its the wrist joint somehow not connected to the rest of the gripper

##### Simon Steinmann [Moderator] 08/24/2020 17:06:42
lol


lemme investigate


fixed it. I removed the extra layered Solid node (now wrist is first solid) and added a physics node to it
> **Attachment**: [Mirobot.proto](https://cdn.discordapp.com/attachments/565155651395780609/747506528566902874/Mirobot.proto)

##### alxy 08/24/2020 17:26:04
So you needed to change the Mirobot?


Not the gripper?

##### Simon Steinmann [Moderator] 08/24/2020 17:26:13
wups


wrong file



> **Attachment**: [MirobotGripper.proto](https://cdn.discordapp.com/attachments/565155651395780609/747507112984313996/MirobotGripper.proto)


😅

##### alxy 08/24/2020 17:27:24
Yeah I have the same issue, way too many files with the same or similar names 😄


uh yeah, very nice, a big thanks again! Definetly worth at least one beer, if we ever meet in person 🙂

##### Simon Steinmann [Moderator] 08/24/2020 17:31:18
heh, I'll hold you to it 😉


but interesting project, learned a lot doing it

##### alxy 08/24/2020 17:33:21
I eventually want to use it for my job, as we have the mirobot there, but its really time consuming to test the control code. Right now its only a weekend/after work project, but I hope it will really help to speed up development


Looks very promising now

##### Simon Steinmann [Moderator] 08/24/2020 18:07:07
take a look at this
> **Attachment**: [Mirobot\_noAttachment.zip](https://cdn.discordapp.com/attachments/565155651395780609/747517360113451048/Mirobot_noAttachment.zip)

##### Dave 08/24/2020 19:29:54
> You're welcome

`@David Mansolino` Finally did it (haha thank you again for the help !) Indeed I did not added in the field of the groundSensorSlot.



Wish you a nice evening !

##### David Mansolino [cyberbotics] 08/25/2020 05:14:42
`@Dave` you're welcome, have a nice day too.

##### Simon Steinmann [Moderator] 08/25/2020 12:56:20
Is there a reason that the inertia matrix from a urdf file does not get used in the converter? I've noticed that no .proto model has those values. Instead it get's calculated from the boundingObject, which, in almost all cases, is a box.


I tested it, and the calculated inertia matrices are quite different from the ones in the urdf file.  It might not be an issue for pure simulation, but I think for the transfer of Sim-to-real, this could be a factor


But you might have additional info that i'm missing

##### David Mansolino [cyberbotics] 08/25/2020 13:05:19
It seems th einertia get parsed here: [https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/parserURDF.py#L601](https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/parserURDF.py#L601)


But then there is a problem when exporting the Physics: [https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/writeProto.py#L158](https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/writeProto.py#L158)

##### Simon Steinmann [Moderator] 08/25/2020 13:07:08
exactly, that's the area I found

##### David Mansolino [cyberbotics] 08/25/2020 13:07:19
It seems there is a misunderstanding there betwen center of mass and inertia

##### Simon Steinmann [Moderator] 08/25/2020 13:08:07
inertia is never actually added

##### David Mansolino [cyberbotics] 08/25/2020 13:08:07
This seems indeed to be a bug (not sure if it was always like this or if a wrong conflict merge introduced it).


Yes and it is used to check if center of mass should be set (which is wrong).

##### Simon Steinmann [Moderator] 08/25/2020 13:08:35
I found it due to the error message, when the inertia had 'rpy'


in that case it could just be multiplied by the rotation matrix derived from the rpy I think


I might make a PR later if I have some time

##### David Mansolino [cyberbotics] 08/25/2020 13:13:31
Ok, thank you

##### Simon Steinmann [Moderator] 08/25/2020 16:08:56
`@David Mansolino` I implemented the fix, you can try it out with this urdf file. [https://github.com/cyberbotics/urdf2webots/pull/76](https://github.com/cyberbotics/urdf2webots/pull/76)
> **Attachment**: [UR10e\_urdf.zip](https://cdn.discordapp.com/attachments/565155651395780609/747850006773366814/UR10e_urdf.zip)

##### David Mansolino [cyberbotics] 08/26/2020 05:24:39
Perfect, I will try it, thank you

##### Simon Steinmann [Moderator] 08/26/2020 12:04:18
Hey, I want your opinion. If no mass or inertia is specified, wouldn't it be better to use a density calculation instead of assuming a mass of 1 kg? Especially for very small or very large objects, this seems like a much better approach

##### David Mansolino [cyberbotics] 08/26/2020 12:20:32
That sounds good indeed.

##### Olivier Michel [cyberbotics] 08/26/2020 12:21:18
Sure, by default Webots assumes a density of 1000 kg/m^3 for objects (based on the volume of the bounding object). This is default value of the Physics node.


Maybe you can re-use this default value?

##### Simon Steinmann [Moderator] 08/26/2020 12:25:19
Okay so this gets into specific cases, where urdf-links have collision values, but no inertia tag. We dont want empty links (which are common) to get a physics node


So if collision exists, and mass == None, we add an empty physics node, which should calculate the mass of the bounding object with a default density of 1000 right?

##### Olivier Michel [cyberbotics] 08/26/2020 12:26:36
Yep.

##### Simon Steinmann [Moderator] 08/26/2020 12:27:01
and the center of Mass gets automatically taken from the bounding box, or does it have to be specified?

##### Olivier Michel [cyberbotics] 08/26/2020 12:28:23
No, if left empty, the center of mass will be automatically computed from the bounding object.

##### Simon Steinmann [Moderator] 08/26/2020 12:28:33
ok great

##### Olivier Michel [cyberbotics] 08/26/2020 12:29:00
This is documented here: [https://cyberbotics.com/doc/reference/physics](https://cyberbotics.com/doc/reference/physics)

##### Simon Steinmann [Moderator] 08/26/2020 12:29:19
yeah read that yesterda, just double checking 🙂


`@David Mansolino` consolidated all the changes [https://github.com/cyberbotics/urdf2webots/pull/79](https://github.com/cyberbotics/urdf2webots/pull/79)

