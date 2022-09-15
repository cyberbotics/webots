# Development 2022

This is an archive of the `development` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m) for year 2022.

## January

##### samuel.nametala 01/12/2022 23:26:39
Goodnight!



Does anyone know any way to stop the Webots simulation by code?

##### Olivier Michel [Cyberbotics] 01/13/2022 07:28:23
[https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_simulation\_quit](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_simulation_quit)

##### samuel.nametala 01/13/2022 11:21:49
Thank you so much!

##### tokia 01/17/2022 00:30:20
Hello, is there a 2 finger gripper compatible with webots simulation UR5 available to download ? I can't use the 3 finger one for my project.

##### Simon Steinmann [Moderator] 01/18/2022 00:16:40
okay so the webots update definitely broke some things


this is now how the world loads, which has been created in 2021
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/932790761718698014/unknown.png)
%end


the IK solvers for pyikfast are also broken


[https://github.com/Simon-Steinmann/webots\_pyikfast\_tutorial](https://github.com/Simon-Steinmann/webots_pyikfast_tutorial) this worked in 2021b


Did the robot models change?

##### Mat198 01/18/2022 00:24:18
Is this the ABB IRB 4600? The coordenate system changed.

##### Simon Steinmann [Moderator] 01/18/2022 00:24:30
it is


well I guess I will have to recompile the solvers 😩


[https://tenor.com/view/schoooool-noooo-stressed-problematic-gif-18852540](https://tenor.com/view/schoooool-noooo-stressed-problematic-gif-18852540)


hmm the irb seems to work, but I still have weird issues... not sure I wanna dive super deep in right now


I think I found the solution. Now I am working on a Inverse Kinematics repository, including all 6DOF arms in webots and providing solvers for each, thus only requiring a simple `pip install .` to install the solver for a particular robot

##### Olivier Michel [Cyberbotics] 01/18/2022 10:06:53
Yes, the robot models were also changed to respect the ENU/FLU standards and be more compliant with ROS and other robotics frameworks. See details here: [https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a](https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a)

##### Simon Steinmann [Moderator] 01/19/2022 00:17:45
Hi, I wanna gauge your interest and opinion. Through interest form `@shpigi` I have gotten back into the whole inverse kinematics and working on my sample project and pyikfast (cyberbotics repository). I created a lot of solvers in the past (all 6DOF arms at the time). I think it would be very beneficial for Webots, if those solvers and a simple controler + install instructions (pip install .) could be provided officially in webots. Setting these things up and getting it to work is a major pain in the behind. Especially for newcomers.


Currently I have a somewhat complexer controller, providing an algorithm to pick the best solution out of all possible solutions provided by ikfast (This is actually a big deal and makes it work much better than any other ik solution I tried). I also added cartesian velocity control.


I think it would make sense to put those 2 features into the c++ portion. Making it much simpler to use for the user. (and faster)

##### Mat198 01/19/2022 00:33:56
I'm feeling that pain right now...

##### Simon Steinmann [Moderator] 01/19/2022 00:34:22
feel free to pm me, if you want help right now

##### JoséeMallah 01/20/2022 13:46:41
Hello everyone!

I am looking to modeling the human gait cycle so to represent the human body and make it walk.

I got the c3d body representation proto, and I was wondering how to actually control the body and make it walk. I am new to webots, and thought that a supervisor might be able to actuate the joints and simulate walking.

Is there any better way to do so - like to control the body itself without using a supervisor? And is the c3d model good for biomechanical simulations?

Thank you

##### Simon Steinmann [Moderator] 01/21/2022 02:10:26
you have to add motors to the joints

##### the-french-bunny 01/21/2022 06:48:51
hello well met everyone first of all i LOVE the concept of this software a great place to prototype and test robot designs and concepts before attempting production FANTASTIC for both hobbyists, newbies(like myself) and professionals. however as a newbie im having some trouble understanding the design proccess for the robots themselves as i kinda want to design a robot from scratch based on the design of cubli.

##### amna 01/28/2022 08:09:16
What values shall i give to trajectory, speed and step in human.py?

## February

##### Simon Steinmann [Moderator] 02/22/2022 17:25:36
This reply might be a bit late, but if you still want help, could you clarify your question a bit? Do you have troubles with how to add nodes together in Webots? Or is the issue  unrelated to software and more on general design of a robot?


<@&568329906048598039> the void controller seems to be broken in the latest nightly build (21/2/2022). On ubuntu 20.04 installed with tarball.  this test world just has unmodified ur3e with void controller and it crashes. EDIT: ENV variable was set incorrectly, sorry for the confusion

## March

##### Craig 03/01/2022 22:17:50
I have built a custom robot and set all the joints to have a minStop/maxStop value. The joints are using velocity control and work as expected until they reach one of the stops, at which point I get robot confetti as every joint explodes out. My basicTimeStep is 1.

Does anyone know what is happening? This is my first robot and I don't have a lot of experience with Webots so I expect the issue is on my end.

##### DrakerDG [Moderator] 03/01/2022 22:20:37
If is possible, share your world to check it and understand it better

##### Craig 03/01/2022 22:39:16
wbt file?

##### Simon Steinmann [Moderator] 03/01/2022 22:39:57
`@Craig` zip your project folder

##### Craig 03/01/2022 22:47:11
The controller is external to the project and can't be shared, so to reproduce command the "Shoulder Motor" to move 1.745 rad/s until it jams before its maxStop, then -1.745 rad/s. The confetti occurs when it returns to its initial position. My best guess is the inertia is carrying it past the stop
> **Attachment**: [ConfettiBot.zip](https://cdn.discordapp.com/attachments/565155651395780609/948350720397148200/ConfettiBot.zip)

##### Simon Steinmann [Moderator] 03/01/2022 22:48:30
cant you share the controller?

##### Craig 03/01/2022 22:49:03
No, but I can rewrite with a simple demo one

##### Simon Steinmann [Moderator] 03/01/2022 22:49:26
and you hard linked many assets, you should include all assets in your project folder and link relative paths

##### Craig 03/01/2022 22:50:16
The mesh files? My bad, I'll send them with the basic controller

##### Simon Steinmann [Moderator] 03/01/2022 22:50:52
There are many warnings, that you specify both, the mass and density. So it uses the mass instead of density. When using mass, it is vital to have the correct inertia matrix


If you dont want to do that, you can specify a density and webots will calculate the inertia matrix assuming a uniform material filling your collision geometry


This might be your culprit. A torque of 10 million Nm.... that is just WAY too much
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/948352397980344340/unknown.png)
%end


check what a realistic value would be for the type of motors you would use

##### Craig 03/01/2022 22:56:44
I was having trouble getting things moving so I used order of magnitude values. I apparently did not go back and fix when I set the weights.

##### Simon Steinmann [Moderator] 03/01/2022 22:57:26
you should also add these limits to the motors, not just the joints

##### Craig 03/01/2022 22:58:10
I originally set the weight when I was working with primitives, I did not update when I got the mesh modules. I can correct this and the torque values

##### Simon Steinmann [Moderator] 03/01/2022 22:58:42
just use density, unless you calculate the exact weight and moment of inertia matrix


for density values ranging from 200-1000 are realistic  for arms and structural elements (1000 being the density of water)

##### Craig 03/01/2022 22:59:59
That explains part of my confusion, the documentation was talking about min/max stops and positions, but I couldn't find any min/max position fields. I'll definitely correct that as well

##### Simon Steinmann [Moderator] 03/01/2022 23:00:52
[https://cyberbotics.com/doc/reference/motor?version=R2022a](https://cyberbotics.com/doc/reference/motor?version=R2022a)

##### Craig 03/01/2022 23:01:43
It sounds like there is a lot to fix after a quick first pass, I'll go back and rework the model to use more realistic values. My bet is that it will fix the issue. Thanks! This was very helpful

##### Simon Steinmann [Moderator] 03/01/2022 23:02:12
you're welcome 🙂

##### Craig 03/02/2022 19:57:45
I have implemented the changes we discussed yesterday but I am still experiencing a robot explosion. I included the controller generated by the wizard, the settling of the robot is enough to initiate the explosion, no further code was required.

• maxTorque updated with realistic values, all of which are in the hundred thousand Nm range

• minStop, maxStop, minPosition, and maxPosition are now both set with the stops wider than the positions

• Everything is using density only (steel 7850 kg/m3)

• All warnings have been cleaned up

• All mesh files are referenced and not absolute

• Mesh files are included
> **Attachment**: [ConfettiBot.zip](https://cdn.discordapp.com/attachments/565155651395780609/948670469404512256/ConfettiBot.zip)

##### Simon Steinmann [Moderator] 03/03/2022 03:01:04
One mayor issue I see is that you use the meshes as collision geometry. Try replacing the boundingObjects with primitive boxes. Another potential issue is that the masses and forces are still incredibly high. Why are the densities so high? 7850 for the first arm? You would never build a robot arm as a solid chunk of metal. The effective density should be less than 1000 in most cases. And your torques are still WAY to high. Just bring it into perspective, a torque of 800 000 Nm like in your first joint is poerfull eough to lift 8 metric tons at the end of a 10m arm. That is absurd. I suggest you 1. fix the bounding objects and 2. research what the physical properties of real world robots are, that are similar in size and function


Also, you dont need the transform nodes between parts. You can just use the translation and rotation field of the solid nodes to do the same


`Solid > children > hingejoint > EndPointSolid > children > hingejoint > EndPointSolid > children ....`

##### Craig 03/03/2022 16:23:07
Valid points, but I must refute some of them. We are building a robotic arm capable of lifting a 1000kg mass and placing it ~12m away, so the densities, lengths, and torques were given to me by the mechanical engineer. The one concession I'll make is that I simplified the joints to use rotational motors instead of linear motors to emulate the hydraulics so I could start work on the control system sooner rather than later. My intention is to circle back and correct the model at a later date as modeling it was not intuitive to me


I included the transforms as quality of life nodes. The STL files I was given had arbitrary origins so I dissociated the center of the joint from the center of the model. If they are causing issues I can strip them out

##### DrakerDG [Moderator] 03/03/2022 16:32:27
My advice is the same as Simon's. You should simplify your bounding objects, avoid using the ones provided by the 3D model and change them to simpler shapes, which provide a simpler way of calculating the physical behavior of your robot. Look at the example of the e-puck robot, it is quite simple
%figure
![2022-03-03.png](https://cdn.discordapp.com/attachments/565155651395780609/948981189182631957/2022-03-03.png)
%end
%figure
![2022-03-03_1.png](https://cdn.discordapp.com/attachments/565155651395780609/948981189451083776/2022-03-03_1.png)
%end

##### Craig 03/03/2022 16:33:56
Is weight based off of shape or bounding box?

##### DDaniel [Cyberbotics] 03/03/2022 16:34:09
What is the timestep you're using? For something this heavy you'll likely need a rather low value

##### Craig 03/03/2022 16:34:32
1, I tried fractional but it didn't run

##### DDaniel [Cyberbotics] 03/03/2022 16:35:06
Either you specify the physics yourself (mass, center of mass, inertia), or alternatively you just specify the mass and a bounding object and webots will infer/approximate the rest

##### DrakerDG [Moderator] 03/03/2022 16:35:06
Yes, mass center


Your robot design is very simple, It is easier make the tree structure and make it works well

##### DDaniel [Cyberbotics] 03/03/2022 16:38:46
Also make sure all intermediary solids in your chain have a physics node, you can't mix physical and non physical stuff

##### DrakerDG [Moderator] 03/03/2022 16:53:07
I manage to understand that 2 parts are attached to the red base (pink and green), which are joined to each other that can slide (light blue and blue). Finally these two sliding parts converge in one that unites them (yellow). Check this example of how to couple two motors (Hinge joins)



[https://www.cyberbotics.com/doc/guide/samples-devices#coupled\_motors-wbt](https://www.cyberbotics.com/doc/guide/samples-devices#coupled_motors-wbt)
%figure
![2022-03-03_2.png](https://cdn.discordapp.com/attachments/565155651395780609/948986395370852372/2022-03-03_2.png)
%end

##### Craig 03/03/2022 16:57:13
Thanks, I actually started looking at that example for doing the hydraulics a couple weeks ago. I couldn't spend as much time on it as I would have liked so I filed it away for further investigation during my CI time. I think I'm going to increase it's priority as it sounds like it has the potential to help my model work better


It sounds like to improve my model I need to convert to use simpler bounding boxes which is fairly easy. I think I will need to convert from density to mass as the steel parts are mostly hollow and thus lighter than they appear. I'll let Webots determine center of mass and inertia as a box approximation is actually quite close to reality.

I'll also look into replacing the rotary motors with linear ones in order to reduce the forces involved. I assume the ultimate issue is float point rounding due to orders of magnitude differences of the magnitudes involved. Thank you everyone for taking the time to help me with this

##### DrakerDG [Moderator] 03/03/2022 19:45:34
It is possible to make a bounding object of the pieces that are hollow, using 4 thin boxes

##### Craig 03/03/2022 23:05:37
I started by stripping out the transform nodes and following the strict 

> Solid > children > hingejoint > EndPointSolid > children > hingejoint > EndPointSolid > children .... 

pattern that Simon suggested. The robot as a whole is looking haggard as I hack and slashed my way through it but the model is much more stable now and I can't get it to enter confetti mode.

The mesh bounding boxes have not been changed and I'm thinking I will leave it alone at this point

##### Zemo 03/05/2022 15:59:23
Hi everyone. I'm trying to create an arm with a gripper for a project, but I don't know how to set up the gripper itself (I'm using the P-grip). Can anyone help me?

I'm using Python to create the controller and I don't know how to set up the P-grip in order to make it open and close

##### DrakerDG [Moderator] 03/05/2022 16:05:23
if it is possible you can share your world, including the controller, so I review it to understand what is happening

##### Zemo 03/05/2022 16:08:24
This is the world
> **Attachment**: [myworld.wbt](https://cdn.discordapp.com/attachments/565155651395780609/949699916488376380/myworld.wbt)


and this is the controller
> **Attachment**: [my\_controller.py](https://cdn.discordapp.com/attachments/565155651395780609/949700003453095976/my_controller.py)


It's very simple

##### DrakerDG [Moderator] 03/05/2022 16:09:04
Ok, let me check it

##### Zemo 03/05/2022 16:10:11
I think I need to add a kind of motor to the gripper or something like that

##### DrakerDG [Moderator] 03/05/2022 16:19:42
The gripper is a PROTO that have one motor for every finger:  "motor 7" and "motor 7 left", you can get every one of this  like your linear motor


Like this:



f1 = robot.getDevice('motor 7')

f2 = robot.getDevice('motor 7 left')

##### Zemo 03/05/2022 16:35:01
Oh ok I didn't know the PROTO thing, and to open/close the gripper what should I do?

##### DrakerDG [Moderator] 03/05/2022 16:37:15
if open == 1:

            f1.setPosition(0.7854) # 45°

            f2.setPosition(0.7854) # 45°

            open = 0

        else: 

            f1.setPosition(0)

            f2.setPosition(0)

            open = 1


Check it please!
> **Attachment**: [myworld.mp4](https://cdn.discordapp.com/attachments/565155651395780609/949708984225501184/myworld.mp4)
> **Attachment**: [my\_controller.py](https://cdn.discordapp.com/attachments/565155651395780609/949708984728830042/my_controller.py)

##### Zemo 03/05/2022 16:51:45
Wow thank u so much, you are the best!

##### DrakerDG [Moderator] 03/05/2022 16:52:22
You welcome!


You can view the PROTO source or can convert to Base Node(s) to see every device like motors, position sensors and distance sensors
%figure
![2022-03-05.png](https://cdn.discordapp.com/attachments/565155651395780609/949713599608279080/2022-03-05.png)
%end

##### Zemo 03/06/2022 09:38:12
So the PROTO is more like a conjunction of pieces and you can take it all together, but if it is converted to base nodes all the pieces are made explicit and become like the ones I added before

##### DrakerDG [Moderator] 03/06/2022 09:41:18
[https://www.cyberbotics.com/doc/guide/tutorial-7-your-first-proto](https://www.cyberbotics.com/doc/guide/tutorial-7-your-first-proto)

##### Zemo 03/06/2022 09:50:44
When I was trying to solve the problem by my self I was like "wtf there are no devices here" expecially because I was looking at the WeBots example that uses this particular grip (p-rob3.wbt) and there where no devices to modify the bots and I couldn't find a solution 😆

##### DrakerDG [Moderator] 03/06/2022 09:52:17
Step by step you will learn more and more 👍

##### Zemo 03/06/2022 09:52:48
I was like "in the example there are motors but in the world building on the left there is nothing like that but in my code I added the motor for the join and I can see it" 😆

##### DrakerDG [Moderator] 03/06/2022 09:52:52
Here you can find a lot of information:

[https://www.cyberbotics.com/doc/guide/index](https://www.cyberbotics.com/doc/guide/index)


And here too: [https://www.cyberbotics.com/doc/reference/index](https://www.cyberbotics.com/doc/reference/index)

##### Zemo 03/06/2022 09:54:16
I searched and searched on internet but I always saw only the pages of the gripper itself and the ones that teach how to import Motor and Robot from controller


Let me ask another thing: I created a program on Jupyter Notebook that make sign recognition and I wanted to merge it to the arm to make an arm that can be controlled using signs, if I copy the text from jupyter to Webots, will it work? Obviously I will need to make some "IF" to recognize the gesture and make the arm move like the gesture imposes

##### DrakerDG [Moderator] 03/06/2022 10:02:38
I understand that the gestures have to be captured by a camera. It may work, but the code would have to be adapted, but I'm not sure how

##### Zemo 03/06/2022 10:02:58
Exactly


I followed this tutorial to make it [https://www.youtube.com/watch?v=pDXdlXlaCco&t=1540s](https://www.youtube.com/watch?v=pDXdlXlaCco&t=1540s)


If you want to see the code

##### DrakerDG [Moderator] 03/06/2022 10:08:26
I recently started to familiarize myself with OpenCV, I'm practically at kindergarten level. I based on this example to make a little line following bot: [https://github.com/lukicdarkoo/webots-example-visual-tracking](https://github.com/lukicdarkoo/webots-example-visual-tracking)

##### Zemo 03/06/2022 10:13:00
So the fact the opencv is installed on the device make the import possible on webots too I think

##### DrakerDG [Moderator] 03/06/2022 10:13:39
I it possible

##### Zemo 03/06/2022 10:15:55
Will cv2.VideoCapture() work? 🤔

##### DrakerDG [Moderator] 03/06/2022 11:23:43
I understand that there should be an external program that captures the images and stores them in some buffer that can be accessed and processed by the webots controller to identify the gestures. It's just an idea, but honestly I don't know how to do it

##### Simon Steinmann [Moderator] 03/06/2022 20:33:21
You can directly use cv2 in webots and handle the video stream from the simulated cameras

##### DrakerDG [Moderator] 03/06/2022 20:35:56
Ok, I understand, but in this case the camera is physical (real) and takes the gestures of a person. Is it possible that somehow the capture of a camera outside of webots can be processed in webots?

##### Simon Steinmann [Moderator] 03/06/2022 20:37:53
Well of course, just use cv2.


Probably best to use external controller

##### DrakerDG [Moderator] 03/06/2022 20:38:37
Ok, thanks

##### Simon Steinmann [Moderator] 03/06/2022 20:44:56
You can directly access connected cameras with cv2. It is actually quite easy to do

##### Zemo 03/07/2022 18:08:43
So the editor of py is just like jupyter? The are a lot of import and also I needed to install somethings to make tensorflow run


The code is just like the one in this video

##### Simon Steinmann [Moderator] 03/07/2022 21:05:21
You can use any editor or IDE you want. For example I use Visual Studio Code (with python extensions). Make sure you set your environment variables when running webots controller externally. [https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers)

##### Zemo 03/07/2022 22:13:57
I already made a basic controller that makes the robot go up, down, open, close and now I need to add the py sign recognition to the controller. The idea was that when the python program I wrote is activated it recognizes hand signs and to each sign it will be assigned one of the commands


I'm a newbie so I don't know what do you mean with "Make sure you set your environment variables when running webots controller externally" 😅

##### Simon Steinmann [Moderator] 03/07/2022 22:22:09
Check out the link I posted

##### Zemo 03/07/2022 22:33:52
So you suggest instead of bringing in WeBots the code I can make everything extern?

##### Simon Steinmann [Moderator] 03/07/2022 22:34:22
Yes


Select extern as the controller in webots and run the simulation. It will pause until you start your controller externally from console etc., attach and start the simulation

##### Zemo 03/07/2022 22:35:55
But I need a downloadable IDE because Jupyter runs in a strange way using the cmd to create a sort of local server 🤔


I don't think it is compatible

##### Simon Steinmann [Moderator] 03/07/2022 22:36:14
I'd recommend vsc


You can add jupyter to vsc if you need it, but it's the wrong tool for webots

##### Zemo 03/07/2022 22:38:22
If I export the .py file will it work on vsc?

##### Simon Steinmann [Moderator] 03/07/2022 22:38:54
Yes, no need to export


Just open the project folder in vsc and edit the files directly

##### Zemo 03/07/2022 22:40:11
And since I already have py and conda on my pc will vsc automatically work or do I need something else?

##### Simon Steinmann [Moderator] 03/07/2022 22:40:37
Nope, should all work


It is basically just a powerful editor

##### Zemo 03/07/2022 22:41:27
Okok because you said "with python extensions" so I didn't know if I needed something else


and then I can also import something like "from controller import Robot" without any problem?

##### Simon Steinmann [Moderator] 03/07/2022 22:44:07
You have to set the environment variables for that


As described in the link

##### Zemo 03/07/2022 22:51:36
Usually when I did something like this I only modified the "Path" one, in the first table WEBOTS\_HOME needs to be a new one?

##### Simon Steinmann [Moderator] 03/07/2022 22:52:42
Yeah, just add everything as described in the link

##### Zemo 03/07/2022 22:53:43
I have Python 3.9 so I only need %WEBOTS\_HOME%\lib\controller right?

##### Simon Steinmann [Moderator] 03/07/2022 22:54:17
Don't remember, try it

##### Zemo 03/07/2022 23:13:18
I'll try everything as soon as I can


`@Simon Steinmann` VSC gives me the error "No module named 'controller'" 😐

##### Simon Steinmann [Moderator] 03/08/2022 20:41:44
Then you did not set your environment variables correctly


What OS are you on

##### Zemo 03/08/2022 20:45:01
Windows


I followed the instruction on the site you linked me


I need to create a new environment variable for each one it says in the tables right? (except Path)

##### Simon Steinmann [Moderator] 03/08/2022 22:21:19
`@Zemo` you need WEBOTS\_HOME, PATH and PYTHONPATH



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/950881079026413578/unknown.png)
%end

##### Zemo 03/08/2022 22:22:28
I only had Path so I created the other two

##### Simon Steinmann [Moderator] 03/08/2022 22:23:23
you added `%WEBOTS_HOME%\lib\controller` without defining the variable `WEBOTS_HOME`. That is probably the issue


everything with % % is a variable in windows and needs to be defined

##### Zemo 03/08/2022 22:24:53
I created WEBOTS\_HOME too

##### Simon Steinmann [Moderator] 03/08/2022 22:25:19
obviously you will have to adjust the python version


to the correct one

##### Zemo 03/08/2022 22:25:56
Yeah I'm using 3.9.7 so I wrote python39

##### Simon Steinmann [Moderator] 03/08/2022 22:26:15
this is an excerpt from my PATH variable
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/950882166018031647/unknown.png)
%end

##### Zemo 03/08/2022 22:26:26
But I installed it using conda, maybe is that the problem?

##### Simon Steinmann [Moderator] 03/08/2022 22:26:30
yeah, 39


i only use conda on linux, works fine there


remember to restart the program


how do you launch the controller? With the terminal in vsc?

##### Zemo 03/08/2022 22:28:52
I created a new python file and selected the version

##### Simon Steinmann [Moderator] 03/08/2022 22:29:06
but how do you run it?

##### Zemo 03/08/2022 22:29:24
Run Python File in the top right

##### Simon Steinmann [Moderator] 03/08/2022 22:29:31
try it from the terminal


`python3 my_controller.py`


in the correct directory ofc

##### Zemo 03/08/2022 22:32:02
Nothing

##### Simon Steinmann [Moderator] 03/08/2022 22:32:13
did you restart vsc?

##### Zemo 03/08/2022 22:32:17
Yea

##### Simon Steinmann [Moderator] 03/08/2022 22:32:25
perhaps try launching it in powershell


perhaps you have to load conda first

##### Zemo 03/08/2022 22:35:13
Always no module named controller

##### Simon Steinmann [Moderator] 03/08/2022 22:36:36
print your env variables


are they being loaded?

##### Zemo 03/08/2022 22:37:23
How do I do that?

##### Simon Steinmann [Moderator] 03/08/2022 22:40:17
echo $env:PATH


echo $env:WEBOTS\_HOME

##### Zemo 03/08/2022 22:42:45
Oh wait maybe I did it


Now the arm is moving in WeBots and the print "Hello World" that I added is showing in vsc

##### Simon Steinmann [Moderator] 03/08/2022 22:43:55
then it works as expected

##### Zemo 03/08/2022 22:44:37
The PYTHONPATH that was in the table of the site was different from yours, I tried with the one you sent me and worked


but I changed / with \

##### Simon Steinmann [Moderator] 03/08/2022 22:45:56
mine is the same as the website, isn't it?

##### Zemo 03/08/2022 22:46:23
I see ${WEBOTS\_HOME} in the website


You used %

##### Simon Steinmann [Moderator] 03/08/2022 22:46:54
ohhhh


yeah, it needs the windows syntax


<@&568329906048598039> [https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-language=python](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-language=python) the variable entry for python should change to the windows format, if windows is selected above

##### Zemo 03/08/2022 22:55:46
Now if I copy the code from jupyter to vsc I should be able to also the webots command in theory 🤔

##### Simon Steinmann [Moderator] 03/08/2022 22:56:08
yeah

##### Olivier Michel [Cyberbotics] 03/09/2022 07:09:58
Unfortunately, this is not easy to change... The table label is "Typical value" and is given as an example with the Linux format (for Python and Java) and Windows format (for MATLAB). Maybe we should rework the Windows/Linux/macOS table instead?

##### Simon Steinmann [Moderator] 03/09/2022 07:11:19
I think that is the right apporach. Just put all in the first table

##### Olivier Michel [Cyberbotics] 03/09/2022 08:52:06
I just fixed it here: [https://github.com/cyberbotics/webots/pull/4330](https://github.com/cyberbotics/webots/pull/4330)


Can you please review the PR and approve it if you believe it is better now?

##### Simon Steinmann [Moderator] 03/10/2022 01:26:40
`@Olivier Michel` done. Much better now

##### kimmcg 03/18/2022 15:19:02
Hi all, There wasn't really a show and tell channel, but I figured I place it here. Here is the Crazyflie flying with velocity control by the keyboard 😁 I still want to work on some ROS twist message intergration and some general fixes, but this has an python and c controller.  It is currently at this github repo: [https://github.com/bitcraze/crazyflie-simulation/](https://github.com/bitcraze/crazyflie-simulation/) but in the coming months I'll work on a PR for the webots repo.
> **Attachment**: [crazyflie\_world\_2.mp4](https://cdn.discordapp.com/attachments/565155651395780609/954398533111394304/crazyflie_world_2.mp4)


I also want to make a better environment for it as well then this arena. perhaps something with some range finders, rooms and wall following?

##### Max\_K 03/28/2022 11:29:26
Hi all, I am working on a teleop for the spot with the developed Kinematic from SoftServe. The walking/running does not work yet, but you can move in x,y,z u. roll, pitch, yaw. 

I uploaded the repo here: [https://github.com/METEORITENMAX/webots\_ros2\_spot](https://github.com/METEORITENMAX/webots_ros2_spot)

Maybe someone has suggestions for improvement or wants to participate. I appreciate any comments.
> **Attachment**: [webots\_teleop.mp4](https://cdn.discordapp.com/attachments/565155651395780609/957964631421386802/webots_teleop.mp4)

##### merdim 03/30/2022 10:44:26
Check out this procedurally generated elevator model. It has a ROS controller for floor doors and elevator cabin. It will be used for our new multi-floor delivery robot
> **Attachment**: [smart\_elevator\_kazam.mp4](https://cdn.discordapp.com/attachments/565155651395780609/958678081403518976/smart_elevator_kazam.mp4)

## April

##### Lucas Waelti 04/01/2022 09:34:49
Hi, can I still find somewhere the nightly build release for `nightly_30_9_2021`?

##### Olivier Michel [Cyberbotics] 04/01/2022 12:35:18
I don't think so... Unless somebody around here as a backup of it...

##### amna 04/06/2022 06:06:47
I need to add a robot which talks `@Mat198`

##### Max\_K 04/21/2022 12:21:58
Hello, we have released a ROS 2 package that enables to control the spot in webots.



[https://github.com/MASKOR/webots\_ros2\_spot/](https://github.com/MASKOR/webots_ros2_spot/)
> **Attachment**: [spot\_teleop.mp4](https://cdn.discordapp.com/attachments/565155651395780609/966675159937327104/spot_teleop.mp4)

## May

##### Sonsonroro17 05/07/2022 04:08:49
That’s awesome, how long have you been working on it. Would like to contribute

##### kimmcg 05/10/2022 07:07:33
Hi! Thanks 😄 It's been a couple of months now but I've not worked on it for a few weeks now due to other commitments. The idea is to push it to Webots' repo this summer, so I hope to start on that soon. You can try out the current state of the simulator! There are now python bindings  of the crazyflie's onboard controller

##### cnbarcelo 05/11/2022 15:45:05
Hi! I just realized R2022a is up to date with master, which I'm not sure is correct. In particular, I have some worlds created with R2022a that are not able to find the mountains\_back.png image. The fix is easy because they're still there in jpg, but I'm not sure if that happened on purpose.

##### DDaniel [Cyberbotics] 05/12/2022 07:10:16
Thank you for the report, yes indeed there was a mixup between branches and tags which made the asset temporarily unavailable. It should be fixed now and you shouldn't need to do anything about it. Let us know if it fails to retrieve the .png again

##### cnbarcelo 05/16/2022 17:15:27
Works like a charm now, thanks!


Hi!

Is there any way on current version of opening the robot window on the browser? Or at least see its console.

##### DDaniel [Cyberbotics] 05/18/2022 06:08:40
Currently I don't think so, but in the new version that will be released in a couple of months the robot windows will open in the browser instead

##### Olivier Michel [Cyberbotics] 05/18/2022 06:12:17
You can test it in the Webots nightly builds of Webots R2022b.

##### amna 05/18/2022 13:25:33
Hi `@Mat198`


Is there anyone who can help me in adding a drone that does not cross the boundary?

##### goch [Moderator] 05/18/2022 17:45:26
If you canot switch to the nightly, here is how I helped myself out



I surrounded part of my Robot Window JS  Code in a try catch and if an error occurs I display the error in a <div> area. Not the best solution but it helps to pinpoint the error by knowing what kind of exception happened 



If that does not help I spin up a local http server and open the robot window manualy in a browser to check the console for errors.


You will need to provide more information about your problem. What kind of boundry?  Do you already have a robot model? ...

##### cnbarcelo 05/18/2022 19:36:18
The problem with the nightly is that, AFAIK, robot window is not retrocompatible, so while I like its new implementation, I'd need to do the migration to workaround that, not sure how hard that would be.

##### Olivier Michel [Cyberbotics] 05/19/2022 05:36:47
There is nothing to change with your robot window. The only difference is that it will open in a web browser instead of inside Webots.

## June

##### kimmcg 06/16/2022 11:59:55
Hi! Just a question related to the upcoming release. The 2022b is projected to come out soon right? In July?  I would like to add a new robot platform to it (the Crazyflie) so my question is: Would that still be possible to before the release and if so, what would be the deadline to not interfere with your testing too much?

##### DDaniel [Cyberbotics] 06/16/2022 12:08:27
Hi, that'd be great! The timing is a bit tight, but depending on how "ready" the robot is it might still be possible. I'd say if you can manage to have the PR ready for review by next Wednesday (and assuming it's complete with respect to what is mentioned here: [https://github.com/cyberbotics/webots/wiki/Adding-a-New-Robot](https://github.com/cyberbotics/webots/wiki/Adding-a-New-Robot)) we might still be able to include it

##### kimmcg 06/16/2022 12:13:04
Yeah I realize it is a bit tight.. was on a holiday before as well or else I would have done it sooner! There were also some issues with image textures with the nightly builds which prevented me to use the latest to try it out (I wanted to use the fixed collada import). Other than that, the robot itself is mostly done (albeit with a simple environment), so I think I will be able to make it!


Thanks for letting me know the deadline and that it is still possible! It will be worth to get the Crazyflie in the next release 🙂 I'll give an headsup once the PR is ready


For the new robot PR, there are no instructions of generating the icon [https://github.com/cyberbotics/webots/tree/master/scripts/icon\_studio](https://github.com/cyberbotics/webots/tree/master/scripts/icon_studio). I assume that I open the world, add the robot proto and run it?


For the web component studio, I think I 'only' have to add the proto to the components.json.. but let me know if I'm mistaken

##### Olivier Michel [Cyberbotics] 06/20/2022 10:25:26
Yes, that seems correct. Let us know if you experience any difficulty while doing this.

##### kimmcg 06/20/2022 10:52:15
Great! yes I'll try it:)


congrats on the construct award by the way!

##### DDaniel [Cyberbotics] 06/20/2022 11:37:46
normally I think you need add an entry to objects.json. However when you run the world, it will re-generate all icons which take a considerable amount of time. It might be better to empty the objects.json file leaving only yours first, and then run it

##### kimmcg 06/20/2022 12:11:00
ah yes thanks, I indeed came that far already.


the issue now is that the 2022b nightly had errors with the icon creator. Either it's not able to upload the proto or issues with cropping the image.. problem is that I use CADshape for the robot so can't use 2022a stable


I still have a non cadshape version of that, so perhaps I should use that. It looks completely identical anyway.

##### Olivier Michel [Cyberbotics] 06/20/2022 12:22:00
Yes, it is better to go for the cadshape version.

##### kimmcg 06/20/2022 13:02:49
Alright, with some detours I finally managed to get model pictures and icons. time for documentation 😁  Thanks for the help so far
%figure
![icon.png](https://cdn.discordapp.com/attachments/565155651395780609/988428712406032394/icon.png)
%end

##### DDaniel [Cyberbotics] 06/20/2022 13:21:50
awesome, yes sorry about that, there have been some big changes recently and some of the tools in the periphery haven't been updated accordingly

##### kimmcg 06/20/2022 13:23:13
ah yeah I understand. Btw, I saw there that possibly 2022b nightly will be fixed tonight right, looking at the latest commit on master?

##### DDaniel [Cyberbotics] 06/20/2022 13:23:57
yes it should

##### kimmcg 06/20/2022 13:24:43
ah great! then hopefully I'll be able to test the final things on that tomorrow


I started the PR for the Crazyflie on [https://github.com/cyberbotics/webots/pull/4703](https://github.com/cyberbotics/webots/pull/4703). I put it in draft for now as I still can see some CI failure (I couldn't build webots locally). but you can already take a look


`@Olivier Michel` I see that you are currently reviewing it and pushing some changes. I think it only still fails on clang formatting and the licensing. Let me know once you are ready looking at it


I had a question about the copyright notice in the string that the CI is checking though. Would it be possible to have a shared copyright here? How have you done it in the past?

##### Olivier Michel [Cyberbotics] 06/21/2022 13:59:43
Sure, this is possible. We should simply put your own copyright and disable the copyright test for your files.

##### kimmcg 06/21/2022 14:02:12
yes that sounds good, then I'll do that. Can I then do it like: 'Copyright 2022 Bitcraze AB' (since we added it this year)


I'll wait until you finished your review

##### Olivier Michel [Cyberbotics] 06/21/2022 14:03:18
I will commit something to disable the license check for your files.


But I'd appreciate if you can fix the clang-format error meanwhile.


See [https://github.com/cyberbotics/webots/wiki/CPP-Coding-Style#cs300-cyberbotics-clang-format-style-compliance](https://github.com/cyberbotics/webots/wiki/CPP-Coding-Style#cs300-cyberbotics-clang-format-style-compliance)

##### kimmcg 06/21/2022 14:06:12
ah yes, I have formated them now already. I was just waiting a bit until everything was commited. At the moment that I wanted to push I got some divergences errors from github 😄  because I didn't noticed you merged the latest develop branch

##### Olivier Michel [Cyberbotics] 06/21/2022 14:06:39
Oops... Sorry for this.

##### kimmcg 06/21/2022 14:07:46
no worries! but I'll push the clang formatting now then with the updated licencing


Seems like it is no longer failing on clang! but the licence test still fails.


btw, the instructions of adding a robot also said I should add something to the changelog, but there is no file at $WEBOTS\_HOME/change\_logs/ChangeLog.html ([https://github.com/cyberbotics/webots/wiki/Adding-a-New-Robot](https://github.com/cyberbotics/webots/wiki/Adding-a-New-Robot)). does this mean that this is no longer necessary or do you have to add this to something else

##### Olivier Michel [Cyberbotics] 06/21/2022 14:29:26
The changelog actually moved...


Let me fix the URL...

##### kimmcg 06/21/2022 14:31:34
ah it's probably changelog-r2022.md

##### Olivier Michel [Cyberbotics] 06/21/2022 14:31:42
That's it.


Is is currently a mess regarding new robots... Let me clean this up...

##### kimmcg 06/21/2022 14:35:15
ah yea no worries. No hurry here. Let me know in the PR if you like to see any changes

##### Olivier Michel [Cyberbotics] 06/21/2022 14:36:12
The code looks pretty good. I only have a couple of minor change requests. I will test it now...

##### kimmcg 06/21/2022 14:48:02
thanks! I've tried to address all of them with the latest commits.


all merged! Thanks again for your guidance 😄


Must say that I'm quite impressed with the CI action going on! We can learn a thing or two at bitcraze

## July

##### starscream 07/15/2022 11:32:29
Hope this finds you well. I am new to robotics and webots. If  someone would really direct me in the right direction it would be really helpful. I am trying to map a maze configuration file to a volumetric map like occupancy grid. Any one can recommend how to start approaching to this. TIA

##### row 07/15/2022 14:19:33
For the grid approach, I believe you need to discretize the workspace right? I think you can initialize a 2d array based on your workspace dimension and then ask the robot to travel along cells. You might need a controller that maps desired location to each wheel's velocity.


Here is a tutorial: [https://towardsdatascience.com/occupancy-grid-mapping-with-webots-and-ros2-a6162a644fab](https://towardsdatascience.com/occupancy-grid-mapping-with-webots-and-ros2-a6162a644fab)

##### DepressedCoder 07/19/2022 05:58:24
A controller program written in python is not working in the latest nightly build of Webots 2022b. I am using a Macbook with Apple Silicon.
%figure
![Screenshot_2022-07-19_at_11.18.01_AM.png](https://cdn.discordapp.com/attachments/565155651395780609/998831153240887316/Screenshot_2022-07-19_at_11.18.01_AM.png)
%end

##### Benjamin Délèze [Cyberbotics] 07/19/2022 06:21:39
I cannot reproduce this bug. Can you check that the python path that your are using in Webots (`/usr/local/bin/python3.7`) is correct?

##### DepressedCoder 07/19/2022 06:22:23
No I am using python which comes as a part of the conda environment. Default python is the one i installed through homebrew.


When i deactivate the environment and type python it is not working. On typing python3 it works

##### Benjamin Délèze [Cyberbotics] 07/19/2022 06:26:04
Can you try to change the python command path in `Webots->preferences`to match an existing python install?

##### DepressedCoder 07/19/2022 06:26:13
let me try


Error came.
%figure
![Screenshot_2022-07-19_at_11.57.28_AM.png](https://cdn.discordapp.com/attachments/565155651395780609/998838606657490975/Screenshot_2022-07-19_at_11.57.28_AM.png)
%end
%figure
![Screenshot_2022-07-19_at_11.57.35_AM.png](https://cdn.discordapp.com/attachments/565155651395780609/998838607404073000/Screenshot_2022-07-19_at_11.57.35_AM.png)
%end

##### Benjamin Délèze [Cyberbotics] 07/19/2022 06:56:50
Can you try with `/Library/Frameworks/Python.framework/Versions/3.9/bin/python3.9` instead?

##### DepressedCoder 07/19/2022 07:29:18
It is not working. The installation of python itself is not there. I used homebrew to install python when I bought the Macbook.


`@Benjamin Délèze` Still same error with the latest nightly build. Could you have a possible answer on why the error is happening? I tried all solutions I could think of .

##### Stefania Pedrazzi [Cyberbotics] 07/20/2022 06:06:38
It seems to be expected that it doesn't work with the Anaconda Python.

A workaround is to recompile the Webots controller library:

[https://www.cyberbotics.com/doc/guide/using-python#use-an-alternative-python-version](https://www.cyberbotics.com/doc/guide/using-python#use-an-alternative-python-version)

Regarding the issue with Python installed with homebrew this should be inspected but for the moment recompiling the Webots controller library should work in this case too.

##### DepressedCoder 07/20/2022 06:20:07
Let me try


I installed python manually by downloading it from python.org. The controller worked finally. However, it would be good and helpful if the issue related to python installed through homebrew be looked upon. Thank you once again `@Stefania Pedrazzi` `@Benjamin Délèze`

## August

##### PabloDavidAR 08/01/2022 17:30:29
Hey everyone, does anyone know if there is any plan to add some functionality between ROS and Webots to be able to spawn robots through a launch file of ROS2? I have been trying to make something like that, but I found nothing that could work within the webots\_ros2 package

##### Olivier Michel [Cyberbotics] 08/03/2022 11:48:57
This was implemented in [https://github.com/cyberbotics/webots\_ros2/pull/345](https://github.com/cyberbotics/webots_ros2/pull/345) and merged in the develop branch of `webots_ros2`.

##### Din96\_Boy 08/17/2022 10:12:23
[https://streamable.com/1c74of](https://streamable.com/1c74of) , Hello and good day everyone , I'm trying to achieve a rope like simulation in Webots. I have done some changes and I get some issues. In this video I have explained the issue that I'm having? , Does anyone know how can I resolve this one? Thank you

##### Olivier Michel [Cyberbotics] 08/17/2022 10:20:37
Does your pink cylinder have a Physics node?

##### Din96\_Boy 08/17/2022 10:21:43
no it only has bounding object

##### Olivier Michel [Cyberbotics] 08/17/2022 10:22:23
I believe adding a Physics node should resolve the problem.

##### Din96\_Boy 08/17/2022 10:31:38
Thank you for the quick reply on my question. I have tried it but it didn't made any changes?

