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

##### [Red Dragons] Mat198 01/18/2022 00:24:18
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

##### [Red Dragons] Mat198 01/19/2022 00:33:56
I'm feeling that pain right now...

##### Luftwaffel [Moderator] 01/19/2022 00:34:22
feel free to pm me, if you want help right now

