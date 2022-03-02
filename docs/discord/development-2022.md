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

##### Luftwaffel [Moderator] 01/18/2022 00:16:40
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

##### Luftwaffel [Moderator] 01/18/2022 00:24:30
it is


well I guess I will have to recompile the solvers 😩


[https://tenor.com/view/schoooool-noooo-stressed-problematic-gif-18852540](https://tenor.com/view/schoooool-noooo-stressed-problematic-gif-18852540)


hmm the irb seems to work, but I still have weird issues... not sure I wanna dive super deep in right now


I think I found the solution. Now I am working on a Inverse Kinematics repository, including all 6DOF arms in webots and providing solvers for each, thus only requiring a simple `pip install .` to install the solver for a particular robot

##### Olivier Michel [Cyberbotics] 01/18/2022 10:06:53
Yes, the robot models were also changed to respect the ENU/FLU standards and be more compliant with ROS and other robotics frameworks. See details here: [https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a](https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a)

##### Luftwaffel [Moderator] 01/19/2022 00:17:45
Hi, I wanna gauge your interest and opinion. Through interest form `@shpigi` I have gotten back into the whole inverse kinematics and working on my sample project and pyikfast (cyberbotics repository). I created a lot of solvers in the past (all 6DOF arms at the time). I think it would be very beneficial for Webots, if those solvers and a simple controler + install instructions (pip install .) could be provided officially in webots. Setting these things up and getting it to work is a major pain in the behind. Especially for newcomers.


Currently I have a somewhat complexer controller, providing an algorithm to pick the best solution out of all possible solutions provided by ikfast (This is actually a big deal and makes it work much better than any other ik solution I tried). I also added cartesian velocity control.


I think it would make sense to put those 2 features into the c++ portion. Making it much simpler to use for the user. (and faster)

##### Mat198 01/19/2022 00:33:56
I'm feeling that pain right now...

##### Luftwaffel [Moderator] 01/19/2022 00:34:22
feel free to pm me, if you want help right now

##### JoséeMallah 01/20/2022 13:46:41
Hello everyone!

I am looking to modeling the human gait cycle so to represent the human body and make it walk.

I got the c3d body representation proto, and I was wondering how to actually control the body and make it walk. I am new to webots, and thought that a supervisor might be able to actuate the joints and simulate walking.

Is there any better way to do so - like to control the body itself without using a supervisor? And is the c3d model good for biomechanical simulations?

Thank you

##### Luftwaffel [Moderator] 01/21/2022 02:10:26
you have to add motors to the joints

##### the-french-bunny 01/21/2022 06:48:51
hello well met everyone first of all i LOVE the concept of this software a great place to prototype and test robot designs and concepts before attempting production FANTASTIC for both hobbyists, newbies(like myself) and professionals. however as a newbie im having some trouble understanding the design proccess for the robots themselves as i kinda want to design a robot from scratch based on the design of cubli.

##### amna 01/28/2022 08:09:16
What values shall i give to trajectory, speed and step in human.py?

## February

##### Luftwaffel [Moderator] 02/22/2022 17:25:36
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

##### Luftwaffel [Moderator] 03/01/2022 22:39:57
`@Craig` zip your project folder

##### Craig 03/01/2022 22:47:11
The controller is external to the project and can't be shared, so to reproduce command the "Shoulder Motor" to move 1.745 rad/s until it jams before its maxStop, then -1.745 rad/s. The confetti occurs when it returns to its initial position. My best guess is the inertia is carrying it past the stop
> **Attachment**: [ConfettiBot.zip](https://cdn.discordapp.com/attachments/565155651395780609/948350720397148200/ConfettiBot.zip)

##### Luftwaffel [Moderator] 03/01/2022 22:48:30
cant you share the controller?

##### Craig 03/01/2022 22:49:03
No, but I can rewrite with a simple demo one

##### Luftwaffel [Moderator] 03/01/2022 22:49:26
and you hard linked many assets, you should include all assets in your project folder and link relative paths

##### Craig 03/01/2022 22:50:16
The mesh files? My bad, I'll send them with the basic controller

##### Luftwaffel [Moderator] 03/01/2022 22:50:52
There are many warnings, that you specify both, the mass and density. So it uses the mass instead of density. When using mass, it is vital to have the correct inertia matrix


If you dont want to do that, you can specify a density and webots will calculate the inertia matrix assuming a uniform material filling your collision geometry


This might be your culprit. A torque of 10 million Nm.... that is just WAY too much
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/948352397980344340/unknown.png)
%end


check what a realistic value would be for the type of motors you would use

##### Craig 03/01/2022 22:56:44
I was having trouble getting things moving so I used order of magnitude values. I apparently did not go back and fix when I set the weights.

##### Luftwaffel [Moderator] 03/01/2022 22:57:26
you should also add these limits to the motors, not just the joints

##### Craig 03/01/2022 22:58:10
I originally set the weight when I was working with primitives, I did not update when I got the mesh modules. I can correct this and the torque values

##### Luftwaffel [Moderator] 03/01/2022 22:58:42
just use density, unless you calculate the exact weight and moment of inertia matrix


for density values ranging from 200-1000 are realistic  for arms and structural elements (1000 being the density of water)

##### Craig 03/01/2022 22:59:59
That explains part of my confusion, the documentation was talking about min/max stops and positions, but I couldn't find any min/max position fields. I'll definitely correct that as well

##### Luftwaffel [Moderator] 03/01/2022 23:00:52
[https://cyberbotics.com/doc/reference/motor?version=R2022a](https://cyberbotics.com/doc/reference/motor?version=R2022a)

##### Craig 03/01/2022 23:01:43
It sounds like there is a lot to fix after a quick first pass, I'll go back and rework the model to use more realistic values. My bet is that it will fix the issue. Thanks! This was very helpful

##### Luftwaffel [Moderator] 03/01/2022 23:02:12
you're welcome 🙂

