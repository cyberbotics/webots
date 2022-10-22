# Technical-Questions 2022

This is an archive of the `technical-questions` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m) for year 2022.

## January

##### GeoCSBI 01/02/2022 14:45:37
Hey everyone and happy new year! I was working with MuliSense21 in an older version of Webots (2020 i believe). The range finder object could successfully retrieved according to the documentation, i.e., getDevice("Multisense21 meta range finder"). 



However in 2022a version, the Multisense21.proto missing the name field in the meta range finder object ->    

  RangeFinder {

        translation 0.05 0 0

        fieldOfView IS 

        cameraFieldOfView

        minRange 1.5

        maxRange IS 

        rangeFinderMaxRange

        width IS cameraWidth

        height IS cameraHeight

        noise IS rangeFinderNoise

      }



So it is not possible to get the object according documentation. Am I missing something?

##### DDaniel [Cyberbotics] 01/02/2022 15:04:18
You're right, it's a bug. For the time being you can retrieve the device called "range-finder" (after setting the `metaRangeFinder` field to `TRUE`), we'll fix it on the next nightly build.

##### GeoCSBI 01/02/2022 15:05:27
`@DDaniel` Thank you!

##### tokia 01/04/2022 02:05:27
hello, I am quite new. I am trying to put some models from blender into a world, but when I add physics they are broken and fall in the floor or bounce around weirdly. What could be the issue?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/927744508744065094/unknown.png)
%end



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/927744519330480138/unknown.png)
%end

##### DrakerDG [Moderator] 01/04/2022 02:08:26
You use the original meshes for bounding objects?

##### tokia 01/04/2022 02:08:40
yes

##### DrakerDG [Moderator] 01/04/2022 02:10:20
Try don't used it and make your bounding objects in webots, using simple shapes like boxes, cylinders, etc.

##### tokia 01/04/2022 02:10:21

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/927745740481126440/unknown.png)
%end

##### DrakerDG [Moderator] 01/04/2022 02:13:03
Check this example, you can see it, is very simple bounding objects. In this only 3 cylinders.
%figure
![e-puck_webinar_4.png](https://cdn.discordapp.com/attachments/565154703139405824/927746422227480576/e-puck_webinar_4.png)
%end

##### tokia 01/04/2022 02:13:55
Oh yeah, so weBots just cannot handle more complicated mesh very well?


and will make it unstable

##### DrakerDG [Moderator] 01/04/2022 02:15:05
Webots use the bounding objects to calculate the physics behavior,  if your bounding objects are more complex, the calculations is too


Other solution is set the basicTimeStep of the WoldInfo in low value


Maybe between 8 to 16

##### tokia 01/04/2022 02:17:18
Okay, how can I create a group of bounding objects to represent a object with multiple parts?

##### DrakerDG [Moderator] 01/04/2022 02:19:41
You can use a group node and put in children your shapes thru transform nodes, for you can move or rotate every part using transform nodes

##### tokia 01/04/2022 02:20:40
Okay I will try doing that, thank you for your help 😃

##### DrakerDG [Moderator] 01/04/2022 02:21:42
You welcome!


You can add more transform (step 2) in children Group of the bounding object
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/927749416201367572/unknown.png)
%end

##### tokia 01/04/2022 03:12:35
worked good thankyou 🙂
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/927761402842734622/unknown.png)
%end

##### DrakerDG [Moderator] 01/04/2022 03:13:04
Excellent!

##### DDaniel [Cyberbotics] 01/04/2022 07:46:37
The fix should be included in the latest nightly version available here: [https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)

You can get the device with `getDevice("MultiSense S21 meta range finder")`

##### Jan Weber 01/04/2022 13:39:50
Hello,

I have a problem with meshes and their appearance. When I create a Solid with a Box as geometry and for example RedBricks as appearance, everything works as expected: I get a box of red bricks.

However, when I use a Mesh as geometry, I get the correct geometry, but it is gray and the RedBrick texture is not visible.

Does anyone have any idea what could be the reason for this?

Here is my code:



Solid {

  children [

    Shape {

      # Does not work:

      geometry Mesh { url "walls.stl" }

      # But this works:

      # geometry Box { size 2 2 2 }

      appearance RedBricks{}

    }

  ]

}



Jan


That is the stl for my question
> **Attachment**: [walls.stl](https://cdn.discordapp.com/attachments/565154703139405824/927920023857688576/walls.stl)

##### Olivier Michel [Cyberbotics] 01/04/2022 13:45:27
It is likely because the STL file format doesn't contain any texture coordinate information. Instead you should use the OBJ format and ensure your OBJ file does contain correct texture coordinates (you can add them using a 3D modeling software like Blender).

##### Max\_K 01/04/2022 15:06:50
hello i have a question about the coordinates system and transforms. The default setting of the coordinate system in the spot world and all other sample projects is NUE. Default should be ENU or? Anyway, I believe that ROS uses ENU. 

In webots I set the coordsystem to ENU and rotated the floor so that nothing falls. The transforms from the spot in webots point in the wrong direction and that's why I think it looks wrong in rviz.

Do I have to rotate all the transforms from the spot manually?
%figure
![Screenshot_from_2022-01-04_15-54-31.png](https://cdn.discordapp.com/attachments/565154703139405824/927941149493182514/Screenshot_from_2022-01-04_15-54-31.png)
%end

##### DDaniel [Cyberbotics] 01/04/2022 15:08:01
In R2022a everything was converted to FLU/ENU

##### Max\_K 01/04/2022 15:11:36
is it possible to have 2021b and 2022a installed at the same time? I currently still have a "world" that does not run under 2022a.

##### DrakerDG [Moderator] 01/04/2022 15:15:30
Hi! Maybe you need check this documentation [https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a](https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a)

##### Max\_K 01/04/2022 15:21:05
Yes thanks maybe i try this script. But i think i first try the new version.

##### Olivier Michel [Cyberbotics] 01/04/2022 15:22:55
On Linux, it is easy to install different versions (from the tarballs), on Windows or macOS, it is a bit more tricky as you will have to manually move files or rename folders to allow the installation of multiple versions.

##### Max\_K 01/04/2022 15:31:22
Ok thanks, I have ubuntu 20.04. I would try that then.

##### black\_hammer\_67 01/04/2022 20:21:14
Hello, I'm trying to setup an underwater scenario, do you have any idea for how to make floating objects on the water ? I already tried by changing the density of the object but it did not paid off.

##### DDaniel [Cyberbotics] 01/04/2022 20:26:25
You also need to define the `immersionProperties` field of the Solids. You can look at the sample world: `file > open sample world > floating_geometries` as an example

##### black\_hammer\_67 01/04/2022 20:45:25
Ok, I'll check it out and I will return, thank.

##### weyho 01/05/2022 09:33:11
Hello,

currently I am getting a segfault, when i am trying to open my world build in 2021b with 2022a, is there anyway to recover my world and continue to use it in 2022a since I would like to stay up to date?

##### DDaniel [Cyberbotics] 01/05/2022 09:35:17
If you open the world with a text editor and replace the first line from `#VRML_SIM R2021b utf8` to `#VRML_SIM R2022a utf8`, does it still segfault?

##### Jan Weber 01/05/2022 13:36:24
Thanks for your quick reply! Now I understand the problem. Then I'll just make the walls monochrome for now.

##### weyho 01/05/2022 15:35:32
It does not segfault anymore, but my world is still not loading sadly

##### mcitir 01/05/2022 18:28:36
Hello, I want to apply some autonomous driving functions. So, I started to use webots. I am working on highway\_overtake tutorial. Do you know how I can extract or record point data cloud of lidars? I want to work on data set outside of webots. Also, I want to visualize point cloud data. But I am not familiar with ROS. Any idea how to possible recording with python script only?

##### Tyhe/Tyook 01/06/2022 13:15:16
Hello everyone,

I'm trying to simulate a hexapod. Like the "mantis" in the sample files. However, the design of my robot has a more complex structure and the robot fails. The warning from the console is as follows; "WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep." 

Can I give it a rectangular physics like in the "mantis" robot, without changing the robot's design? Or how can I give?


this is my robot geometry
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/928638950610976828/unknown.png)
%end


and this is the mantis geometry
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/928639449921884180/unknown.png)
%end

##### DDaniel [Cyberbotics] 01/06/2022 13:22:31
Have you tried lowering the `basicTimeStep` of the simulation? That might solve it already. Otherwise it's usually not a good idea to use a complex mesh as `boundingObject`, in your case you can just use a `Box` node where the leg will contact with the ground

##### Tyhe/Tyook 01/06/2022 13:24:00
I tried lowering basicTimeStep but the error persisted


i guess i should use box geometry but i dont know how can i do it, when i add box in the geometry, the appearance is changing

##### DDaniel [Cyberbotics] 01/06/2022 13:26:02
you need to set in the `boundingObject` field of the yellow Solid, not geometry

##### Tyhe/Tyook 01/06/2022 13:27:35
the position of the box I added must be in the same position as the leg, right?

##### DDaniel [Cyberbotics] 01/06/2022 13:28:11
You probably need to add a `Transform` node before the `Box` node in order to place it at the right spot

##### Tyhe/Tyook 01/06/2022 13:34:32
Should I add the transform inside children?

##### DDaniel [Cyberbotics] 01/06/2022 13:35:57
```
boundingObject Transform {
  translation a b c
  children [
    Box {
      size x y z
    }
  ]
}
```

##### Tyhe/Tyook 01/06/2022 13:43:40
I can't add box in children. What am i doing wrong?


there is no box in list

##### DDaniel [Cyberbotics] 01/06/2022 13:46:32
Should be possible. What about `Shape`? Can you add that before the `Box`?

##### Tyhe/Tyook 01/06/2022 13:47:01
oops, i find the problem, my mistake


I was trying to add in the wrong place

##### •́ 01/07/2022 10:43:37
Hello all I am facing an issue when loading code into vs code

##### Simon Steinmann [Moderator] 01/09/2022 00:18:11
what issue?

##### Rizalfauz 01/09/2022 04:03:30
Hello everyone, i want to make a plunger like solenoid, any idea to make it?  because using a linear motor does not have much effect on the ball



> **Attachment**: [Tak\_berjudul\_5\_540p.mp4](https://cdn.discordapp.com/attachments/565154703139405824/929586918663024650/Tak_berjudul_5_540p.mp4)

##### •́ 01/09/2022 04:56:40
[https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot?tab-language=c](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot?tab-language=c)
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/929599534982647838/unknown.png)
%end

##### giunas97 01/09/2022 10:48:20
Hi, i'm using a kuka youbot and an e-puck in my project. But the youbot needs the contact properties defined in the WorldInfo node to work properly (for example, in my case, if these aren't defined, the youbot cannot reach the precise target point with the gps node). So, i entered the contact properties, now the youbot works properly, but the e-puck doesn't move straight. Any suggest to solve this problem? Thanks in advance


Not matter, solved. The problem was the basicTimestep

##### Ranga Kulathunga 01/09/2022 15:10:17
Hi everyone! Can help to solve this issue?
%figure
![pyerror.PNG](https://cdn.discordapp.com/attachments/565154703139405824/929753957256011856/pyerror.PNG)
%end

##### furkann 01/09/2022 16:54:32
hi guys


I have a question


are there any body here

##### Lavi 01/09/2022 17:26:52
Hi.

I'd like to add a stationary camera to my scene that does not move with my robot (arm). Trying to add a camera at the top level I get a message saying this is not possible.

If there some way to add a stationary camera?

##### shpigi 01/09/2022 17:49:59
I guess this is my answer:

[https://discordapp.com/channels/565154702715518986/565154703139405824/887352420110319656](https://discordapp.com/channels/565154702715518986/565154703139405824/887352420110319656)


I wish to control a ur10e arm and two stationary cameras with a single supervisor.

I tried nesting the arm and cameras in a single robot node as in the image below.



I can now get and enable the cameras within the supervisor `__init__` like so:

```
        self.st2_cam = self.getDevice("st2_cam")
        self.st2_cam.enable(self.timestep)
        self.st1_cam = self.getDevice("st1_cam")
        self.st1_cam.enable(self.timestep)
```

but I can not see the ur10 sensors. 

this:

```
        shoulder_pan_joint = self.getFromDef("ARM").getDevice("shoulder_pan_joint_sensor")
```

fails with `AttributeError: 'Node' object has no attribute 'getDevice'`



and this:

```
        shoulder_pan_joint = self.getDevice("shoulder_pan_joint_sensor")
```

returns None with a warning `Warning: "shoulder_pan_joint_sensor" device not found.`



Same for:

```
        shoulder_pan_joint = self.getDevice("ROBOT.ARM.shoulder_pan_joint_sensor")
```



How can I get both the arm and cams to work under the same supervisor?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/929846568708882472/unknown.png)
%end

##### Mat198 01/09/2022 21:42:23
I did something like this last week and had no problem using the IBR 4600. I wrote supervisor.getDevice("joint sensor name") if I remember correctly. I will check my code later


You need to call the motors first and then the sensors with the motors.



for link in armChain.links:

        if 'motor' in link.name:

            motor = supervisor.getDevice(link.name)

            motor.setVelocity(1.0)

            position\_sensor = motor.getPositionSensor()

            position\_sensor.enable(timeStep)

            motors.append(motor)

    return armChain, motors

##### shpigi 01/10/2022 17:00:26
Thanks `@Mat198` . In my case, doing `armChain = self.getFromDef("ARM")` returns a Node which does not have `links` so I'd need to investigate further. (Is it "OK" to nest a Robot under a Robot so that it shares the same thread as the cameras)?

##### DDaniel [Cyberbotics] 01/10/2022 17:21:48
In principle it is allowed to have a robot within a robot, but I don't believe access to the devices is shared among them. So each controller will only have access to its own devices.

If however the arm is defined as a proto, the reason you aren't able to get the node reference might be because this node is internal to the PROTO, hence it requires using `getFromProtoDef` method instead of `getFromDef`

##### shpigi 01/10/2022 17:28:43
Thanks `@DDaniel` . The arm is a UR10e, not something I myself imported from a PROTO file (so I guess it is not defined as a proto) 

Edit: or is it? [https://github.com/cyberbotics/webots/blob/990e8291b3cb18c0709d984c8e4b72ebfbc7e641/projects/robots/universal\_robots/protos/UR10e.proto](https://github.com/cyberbotics/webots/blob/990e8291b3cb18c0709d984c8e4b72ebfbc7e641/projects/robots/universal_robots/protos/UR10e.proto)

##### DDaniel [Cyberbotics] 01/10/2022 17:32:00
If it's shipped with webots then indeed is defined as a PROTO


I don't know what you intend to do, but as a heads up internal nodes cannot be modified (not even with a supervisor). So if you intend to do something beyond just reading its position/state, you might want to convert it from a PROTO to a normal node (by right click and "convert to base node")

##### shpigi 01/10/2022 17:38:29
I did plan to `setPosition` and `setVelocity` on the motors

##### DDaniel [Cyberbotics] 01/10/2022 17:42:47
That you can do from the arm's controller yes

##### shpigi 01/10/2022 17:44:24
can I do it from supervisor that runs on the robot that contains the UR10e as a child? (see the image in this post [https://discordapp.com/channels/565154702715518986/565154703139405824/929846569493229588](https://discordapp.com/channels/565154702715518986/565154703139405824/929846569493229588))


.. i.e. with this controller hierarchy
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/930156388129312768/unknown.png)
%end

##### Simon Steinmann [Moderator] 01/11/2022 00:59:28
you could convert the robot to basenodes (right click on it). Then Turn the Robot basenode into a transform. That should do the trick


This code-block shows you all devices, and this specifically puts all motors and position sensors in a list. But it prints ALL devices, so you can add other sensors.



> **Attachment**: [webots\_init\_devices.py](https://cdn.discordapp.com/attachments/565154703139405824/930266974196498472/webots_init_devices.py)


If you want to use an inverse kinematics controller I made (and which works very well), check out this repository. I have not checked it recently with the UR10e, but i did in the past and it worked flawlessly


[https://github.com/Simon-Steinmann/webots\_pyikfast\_tutorial](https://github.com/Simon-Steinmann/webots_pyikfast_tutorial)


I uploaded a solver for the ur10e I had lying around, but I have not tested building the module and testing it

##### shpigi 01/11/2022 01:15:31
I really appreciate it `@Simon Steinmann` . Going to give all of it a try


so far, this code returns the two cams (`st1_cam` and `st2_cam`) but not the robot (converted from proto). I'm gueesing this is because it's a node, not a device and the getNumberOfDevices / getDeviceByIndex doesn't drill into the Nodes.

##### Simon Steinmann [Moderator] 01/11/2022 01:26:13
did you turn the robot node of the ur10e into a solid?


perhaps you can post another screenshot of your scene tree

##### shpigi 01/11/2022 01:27:31
I converted to Base Node and now this is my tree:
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/930271674924941382/unknown.png)
%end

##### Simon Steinmann [Moderator] 01/11/2022 01:28:45
create a new solid next to the 2 cameras. Then cut - copy all the ur10e's children to the solids children


perhaps also put the translation and rotation to the same values


then delete the "DEF ARM Robot" node


The issue is that you have nested Robot nodes. A robot is a node that starts a controller and has access to its own devices.


so you should only have your highest level robot node, if you want to control everything with one controller

##### shpigi 01/11/2022 01:37:22
`@Simon Steinmann` , you're a genius! I went into a text editor, replaced the `DEF ARM Robot` with `Solid` and it worked!

This is your code's output:

```INFO: ur10_supervisor_manager: Starting controller: python -u ur10_supervisor_manager.py
n: 16
shoulder_pan_joint    - NodeType: 55
shoulder_pan_joint_sensor    - NodeType: 50
shoulder_lift_joint    - NodeType: 55
shoulder_lift_joint_sensor    - NodeType: 50
elbow_joint    - NodeType: 55
elbow_joint_sensor    - NodeType: 50
wrist_1_joint    - NodeType: 55
wrist_1_joint_sensor    - NodeType: 50
wrist_2_joint    - NodeType: 55
wrist_2_joint_sensor    - NodeType: 50
wrist_3_joint    - NodeType: 55
wrist_3_joint_sensor    - NodeType: 50
WRISTCAM    - NodeType: 36
distance sensor    - NodeType: 40
st1_cam    - NodeType: 36
st2_cam    - NodeType: 36
```

##### Simon Steinmann [Moderator] 01/11/2022 01:38:00
😎  Awesome! that is a much smarter way to do it


If you want to get IK to work properly, I highly suggest you use my repository. I spent half a year with that stuff, and it is by far the best solution that is out there and compatible without complex other frameworks

##### shpigi 01/11/2022 01:40:31
Sounds great. Will do. I'm aiming at some reinforcement learning work and controlling the end-effector in Cartesian space (instead of doing actions in joint space) is very promising IMO

##### Simon Steinmann [Moderator] 01/11/2022 01:41:16
then you kind of HAVE to use my repo. That was exactly what I was doing. I added support for cartesian velocity control without error drift


and it is very fast, in the dozens to hundreds of microseconds per solution instead of milliseconds


but dont expect too much from RL with robotic arms. Spend about 2 years with it

##### shpigi 01/11/2022 01:42:36
I'll be sure to let you know how it goes

##### Simon Steinmann [Moderator] 01/11/2022 01:43:02
If you wanna have a chat, feel free to pm me. Literally what I did my master thesis on and worked at the german NASA for 1 year 🙂

##### shpigi 01/11/2022 01:43:33
I'll be getting back to you. For sure !

##### Jan Weber 01/11/2022 11:21:28
Hello, is it possible to have a proto inherit from another proto? I want to extend the "TurtleBot3Burger.proto" with some sensors. So far I just copied it and made my changes in it (not in a field, but in children). But due to the change of the coordinate systems in R2022a I would now have to maintain different versions of the proto file for the different webots versions. Therefore it would be easier to let my modified proto inherit from the "official" proto and put only my changes there.

Or is the only way to do that via the extensionSlots?

Jan

##### DDaniel [Cyberbotics] 01/11/2022 11:34:12
You can inherit from another PROTO (just by nesting them, proto B calls proto A and providing some specific parameters to it). By doing it this way you can't however alter the base proto more than what it's exposed parameters allow for. However the extension slot was made for just that purpose, so you can put your sensors there

##### Jan Weber 01/12/2022 11:24:31
Thanks for the answer! I have now implemented it in the extensionSlot. However, with this I don't see any way to easily change a proto, like for example changing the lookup table from an already installed sensor. 

Anyway - my problem is solved, thank you!


I have another question: According to [https://cyberbotics.com/doc/blog/Webots-2022-a-release](https://cyberbotics.com/doc/blog/Webots-2022-a-release) webots\_ros2\_driver replaces the older webots\_ros2\_core. In the webots\_ros2\_core/launch/robot\_launch.py there was the possibility to specify the robot name so that the robot controller finds the associated robot in webots. I am missing this possibility in webots\_ros2\_driver.

I want to launch multiple external robot controllers from one ROS launchfile.

Also the possibility to work with environment variables (as described in [https://cyberbotics.com/doc/guide/running-extern-robot-controllers](https://cyberbotics.com/doc/guide/running-extern-robot-controllers)) fails in the context of ROS launchfiles, because the nodes are not started in a fixed time sequence.

The old way of specifying the robot name in the launchfile worked very well. Does anyone have an idea how to solve the problem?

##### Benjamin Hug [Moderator] 01/12/2022 12:57:54
I suggest you to check this launch file [https://github.com/cyberbotics/webots\_ros2/blob/master/webots\_ros2\_universal\_robot/launch/multirobot\_launch.py#L40-L68](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_universal_robot/launch/multirobot_launch.py#L40-L68) which is what you are looking for.

##### Jan Weber 01/12/2022 15:17:37
Thank you, the "additional\_env" parameter worked.

##### moebius 01/12/2022 23:42:51
Hi I have a supervisor file that is generating the arena for the environment and managing running the sims for multiple robot designs , and I want to read some data from a json file for a given robot and pass those parameters to the customData field for the robot  using 

```
supervisor.getRoot().getField('children')
topLevelNodes.getMFNode(-1)
robot.getField('customData').setSFString(data['geometry_params'])
```

 but i do not see this reflected in the customData field in the GUI tree for the robot node, and hence I am not able to access the data in the controller file



What am i doing wrong, and how am i supposed to be passing the data to the controller?

##### Olivier Michel [Cyberbotics] 01/13/2022 07:32:13
Using the `customData` field this way should work. Are you sure you are pointing to the right robot? Can you for example change the `description` field for that robot and see the change in the scene tree? Alternatively, you may also use an `Emitter` on the supervisor side and a `Receiver` device on the robot side.

##### pyphais 01/13/2022 08:09:15
Hi all, I'm trying to make a skid-steer vehicle simulation accurate to the physical vehicle I have. I've set it up so that the available torque for each wheel at each step is based on the speed of the wheel, and approximated a linear torque-speed curve, then I attempt to set the velocity for each wheel and it uses the available torque to do it. I figured this is more accurate than simply picking an acceleration as the motors do not have that information. The vehicle seems to be getting to the correct speed doing this (its max speed is dependent on the torque available at that speed), but it is accelerating much too quickly. The physical vehicle takes about 1/4 of the wheel to get up to speed, and another quarter turn of the wheel to come to a complete stop when the throttle is let go, but the simulated vehicle seems to do both nearly instantaneously. Lowering the available torque just lowers the max speed that the vehicle gets to instead of affecting the accelerations, increasing friction makes it worse, decreasing friction makes it slide instead of continuing turning the wheel. I'm not sure what parameters to edit in order to make it have the same behavior as the real vehicle. The only thing that gets me that behavior is increasing the mass of the wheels from 0.7kg to like 50kg each, but this is definitely not the case for the real vehicle. Does anyone have any advice?

##### moebius 01/13/2022 08:09:49
yes I'm pointing to the right robot, but  i'll check once more. I'll try the methods that you've mentioned and get back to you, thanks you so much!

##### Amrita 01/15/2022 07:05:36
I am using NAO robot in WEBOT. I am controlling it with c coding. Can anyone help m to bring it to stand position or standzero position after walking for few steps.

##### AmilaChinthaka 01/16/2022 12:57:57
Hi guys, Is there any method to extract the camera output of a robot in a webots simulation and send it to a web interface using python?

##### Mat198 01/16/2022 13:27:23
I know how to send image data from webots to opencv. Is that useful? I never worked with web interfaces

##### Amrita 01/16/2022 13:30:38
How we can send it?

##### mouselet2017 01/16/2022 13:31:56
When I used webots to simulate led lights, I used the following code:

from controller import Robot

from controller import LED



robot = Robot()

timestep = int(robot.getBasicTimeStep())

led = robot.getDevice("led")

print(led)

print(led.get())

led.set(1)

print(led.get())

while robot.step(timestep) != -1:

    led.set(1)

    pass



But unfortunately, my led doesn't light up, and I don't know why (console output is fine). Hope someone can help me! THANKS!!!
%figure
![20220116213136.png](https://cdn.discordapp.com/attachments/565154703139405824/932265920557121536/20220116213136.png)
%end

##### Mat198 01/16/2022 13:48:28
That's a code to get webots images and send them to openCV:



from controller import Robot

from controller import Camera

import cv2 

import numpy as np





\# create the Robot instance.

robot = Robot()



\# get the time step of the current world.

timestep = int(robot.getBasicTimeStep())



\# Get camera devices

cam\_right = robot.getDevice('cam\_right')

cam\_left = robot.getDevice('cam\_left')





\#Enable the cameras

Camera.enable(cam\_right, timestep)

Camera.enable(cam\_left, timestep)



\# Get camera parameters

width = cam\_right.getWidth()

height = cam\_right.getHeight()

fov = cam\_right.getFov() 



cv2.startWindowThread()



while robot.step(timestep) != -1:

    

    # Read the sensors:

    img\_right = cam\_right.getImage()

    img\_left = cam\_left.getImage()

    

    # Convert images to openCV

    img\_right\_cv = np.frombuffer(img\_right, np.uint8).reshape((height, width, 4))

    img\_left\_cv = np.frombuffer(img\_left, np.uint8).reshape((height, width, 4))

    

    # Showing images

    cv2.imshow("Direita",img\_right\_cv)

    cv2.imshow('Esquerda',img\_left\_cv)

    cv2.waitKey(timestep)

\# Enter here exit cleanup code.

##### DrakerDG [Moderator] 01/16/2022 21:59:39
Hi, you need put a pointlight or spotlight in children LED node and define the color that you need. 



This is an example in Python:



\# Initialize LED

led = robot.getDevice('led\_name')



\# Turn on LED

led.set(1)

   

\# Turn off LED

led.set(0)



Reference: [https://cyberbotics.com/doc/reference/led](https://cyberbotics.com/doc/reference/led)


You must be use pointlight with castShadows setting in TRUE to obtain more realistic results. 



Check the world samples, in devices a LED example to understand that I say



%figure
![Screenshot_20220116-160913_Chrome.jpg](https://cdn.discordapp.com/attachments/565154703139405824/932396343970922556/Screenshot_20220116-160913_Chrome.jpg)
%end

##### mouselet2017 01/17/2022 03:02:36
Thank you very much!!!That's OK!

##### waynemartis 01/17/2022 06:05:56
I notice that the yaw values of the compass has a range of (-1, 1). Can it be made to (-pi, pi)?

##### Tom\_Wolf 01/17/2022 09:11:23
Hi everyone ! I'm new user on webots simulateur and im currently using ros2 galactic to launch my world and develope my stuff. so i've an issue with it when i try to launch my world it say "Error: can't copy 'worlds/my\_world.wbt' dioesn't exist or not a regular file" if you now why thanks to let me now

##### Olivier Michel [Cyberbotics] 01/17/2022 09:31:42
Sure, it is simply a matter of defining a `lookupTable` for the `Compass` node, see documentation here: [https://cyberbotics.com/doc/reference/compass](https://cyberbotics.com/doc/reference/compass)

##### Tom\_Wolf 01/17/2022 09:44:37
thanks a lot !

##### Benjamin Hug [Moderator] 01/17/2022 10:00:15
I will need to see your launch file in order to help you, hard to say without any code. You might check some of our tutorials if you begin with Webots ([https://github.com/cyberbotics/webots\_ros2/wiki/Tutorials](https://github.com/cyberbotics/webots_ros2/wiki/Tutorials)). Finally you can check this launch file ([https://github.com/cyberbotics/webots\_ros2/blob/master/webots\_ros2\_mavic/launch/robot\_launch.py](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_mavic/launch/robot_launch.py)), which is very simple.

##### Tom\_Wolf 01/17/2022 10:09:35
Hi ! there is my launch file
%figure
![Capture_decran_2022-01-17_a_11.09.00.png](https://cdn.discordapp.com/attachments/565154703139405824/932577384295698492/Capture_decran_2022-01-17_a_11.09.00.png)
%end

##### Benjamin Hug [Moderator] 01/17/2022 10:17:49
I assume your world is in the folder `world`, but you are searching into the folder `worlds` at line 16.

##### Tom\_Wolf 01/17/2022 10:19:37
daim it s a stupide error thanks

##### JosjaG 01/17/2022 16:02:20
I'm having issues using Webots R2022a, when I try `from controller import Robot` I get the following error: `ImportError: my_webots_home/webots/lib/controller/python38/_controller.so: undefined symbol: _ZNK6webots3GPS14getSpeedVectorEv`. The error does not occur when I revert back to Webots R2021b and the issue shows up for the deb as well as the tar package of Webots 2022a. Does anyone know what may be causing this?

##### DDaniel [Cyberbotics] 01/17/2022 16:17:08
When installing from tar, did you follow these steps? [https://cyberbotics.com/doc/guide/installation-procedure#installing-the-tarball-package](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-tarball-package)

##### Olivier Michel [Cyberbotics] 01/17/2022 16:18:38
`GPS.getSpeedVector` is a new API function in Webots R2022a. Are you sure, you don't have two versions of Webots installed on your machine and `WEBOTS_HOME` is pointing to R2021b while you are try to run R2022a?

##### JosjaG 01/18/2022 09:37:01
Ah, I see my `PYTHONPATH` also referenced to an old location of webots, removing the old reference fixed the issue!

##### Spy Guy 01/19/2022 22:22:01
Hello, I'm having issues trying to run the example python webots world. I'm not using PyCharm and just trying to do it within Webots itself. This is the same error I keep getting and I haven't had any luck researching the issue
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/933486485947187291/unknown.png)
%end


I'm on Windows using Webots R2022a and my Python 3.9 installation is located at C:\msys64\mingw64\bin\python.exe (already specified in my Path environment variable)

##### AmilaChinthaka 01/20/2022 06:10:17
It would be helpful indeed. Can you explain how it is done?

##### Olivier Michel [Cyberbotics] 01/20/2022 08:05:06
You should install Python 3.9 from Python.org instead (or recompile the python wrapper for msys64 python).

##### JosjaG 01/20/2022 09:57:29
I'm using webots with ros\_control, but since that means I have to use the default `"ros"` controller (right?), I can no longer access the motor devices within the controller to set the initial position. How can I provide initial positions to motor joints?

##### Mat198 01/20/2022 11:45:21
I send the code above. I got it from an exemple wich I don't remember the name


I'm trying to use the robot.worldSave() command without an argument as explained here: [https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_world\_save](https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_world_save)



I keep getting this error msg:

Error: wb\_supervisor\_world\_save() called with a NULL 'filename' argument.



I don't want to explicitly give the world name as I want to use the same code for multiple worlds.



I need to generate multiple objects and save so I don't need to create then every single time I run the sim.



Thanks in advance

##### Olivier Michel [Cyberbotics] 01/20/2022 13:03:36
It looks like a bug. Please open an issue about it.

##### Spy Guy 01/20/2022 17:23:19
I got that to work, thank you!

##### Mat198 01/20/2022 17:26:25
How I do that?

##### Benjamin Hug [Moderator] 01/20/2022 19:37:42
Click on `New issue` here [https://github.com/cyberbotics/webots/issues](https://github.com/cyberbotics/webots/issues), thank you!

##### Mat198 01/20/2022 19:41:14
Thanks

##### DDaniel [Cyberbotics] 01/21/2022 10:12:46
No need to open an issue, it should be fixed already. The corrected version should become available with tonight's nightly build here: [https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)

##### Jan Weber 01/21/2022 14:12:42
I am currently converting my project from the old webots\_ros2\_core to the webots\_ros2\_driver. I am using webots\_ros2\_turtlebot. I noticed that the tf2-tree of the turtlebot is not connected: (see tf2\_tree\_splitted.png)

In webots 2021a with webots\_ros2\_core it was fully connected. 

My understanding is that the tree should be completely built by the driver, right? I have now extended the robot\_launch.py from webots\_ros2\_turtlebot:



    extension\_slot\_publisher = Node(

        package='tf2\_ros',

        executable='static\_transform\_publisher',

        output='screen',

        arguments=['-0.03', '0', '0.153', '0', '0', '0', 'base\_link', 'extension\_slot'],

    )



    lidar\_publisher = Node(

        package='tf2\_ros',

        executable='static\_transform\_publisher',

        output='screen',

        arguments=['0', '0', '0.02', '0', '0', 'extension\_slot', 'LDS-01'],

    )



Now the tree is fully connected and the navigation works: (see tf2\_tree.png)



I am unsure if this is the right way. I rarely get this warning, which to my understanding indicates that someone is publishing the transformation between LDS-01 and base\_link:



[controller\_server-11] [INFO] [1642773577.942406284] [local\_costmap.local\_costmap\_rclcpp\_node]: Message Filter dropping message: frame 'LDS-01' at time 0.864 for reason 'the timestamp on the message is earlier than all the data in the transform cache'.



Jan
%figure
![tf2_tree_splitted.png](https://cdn.discordapp.com/attachments/565154703139405824/934088121136603166/tf2_tree_splitted.png)
%end


tf2\_tree.png
%figure
![tf2_tree.png](https://cdn.discordapp.com/attachments/565154703139405824/934088351307403394/tf2_tree.png)
%end

##### Benjamin Hug [Moderator] 01/21/2022 14:53:56
When I run  `ros2 launch webots_ros2_turtlebot robot_launch.py` I have this tree, so I guess you are missing something.

Maybe by looking here ([https://github.com/cyberbotics/webots\_ros2/blob/master/webots\_ros2\_turtlebot/launch/robot\_launch.py](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_turtlebot/launch/robot_launch.py)) you will find out what is wrong / missing, otherwise I think it would be easier to open an issue on `webots_ros2` so we can take a look more in depth.
> **Attachment**: [frames.pdf](https://cdn.discordapp.com/attachments/565154703139405824/934098497035190282/frames.pdf)

##### Jan Weber 01/21/2022 15:15:34
When I run the command you suggested, I get exactly the same unconnected tree as shown above. There are also many frames missing that are included in your tree.

Then I will create an issue in the next few days and describe the problem in more detail.

Thank you!

##### Max\_K 01/21/2022 15:31:09
Hello, i try to add a TouchSensor to the spot robot. I followed the tutorial setting-up a Robot Simulation. I am able to add a Distance Sensor as described in updating my\_robot.urdf and had the ros topic also available. But when i try to add a TouchSensor i get the error:

```[driver-2] wb_device_init(): node not handled
[ERROR] [driver-2]: process has died [pid 132208, exit code -11, cmd '/home/max/webots_ws/install/webots_ros2_driver/lib/webots_ros2_driver/driver --ros-args --params-file /tmp/launch_params_hfe_jwra'].
```

what could be the reason? I am using ros foxy

##### Benjamin Hug [Moderator] 01/21/2022 16:35:29
Webots tries to init the TouchSensor with `wb_device_init`, but it shouldn't.



In any case, the TouchSensor is not yet implemented in the ROS 2 interface for Webots :/

But it is on our to do list!

##### kindleforjc 01/21/2022 21:49:33
Hello all, just started with Webots Automotive and python. I am trying to make small example where car can reach up to some solid node.



I am trying to use the supervisor API in  python but it says "Only one instance of the Root class should be created".



So can anyone help me understand the error? Does this mean that when we add any Car/Vehicle and set the Supervisor field to True... In background one instance of that robot is already created?



Is there any way to access the self instance in python for vehicle?



Thank you and sorry if my questions are not clear.


I think I got it. I had created Driver() instance already and hence no need to create Supervisor () instance. 



Thank you.

##### tokia 01/22/2022 01:30:57
How to increase camera sensor resolution?

##### DrakerDG [Moderator] 01/22/2022 01:33:18
You can change de width and height to increase resolution (pixels) and set to TRUE antiAliasing
%figure
![Screenshot_2022-01-21_193210.png](https://cdn.discordapp.com/attachments/565154703139405824/934259399705387108/Screenshot_2022-01-21_193210.png)
%end

##### tokia 01/22/2022 01:37:28
Cool, thank you! I have been looking for 2 finger grippers compatible for universal robots in Webots as I cant use the 3 finger one for my project. If you know any available please let me know😃

##### DrakerDG [Moderator] 01/22/2022 01:46:00
Maybe the sample world coupled\_motors.wbt works for you.  Take a look to this...
> **Attachment**: [coupled\_motors.mp4](https://cdn.discordapp.com/attachments/565154703139405824/934262592447324160/coupled_motors.mp4)



%figure
![2022-01-21.png](https://cdn.discordapp.com/attachments/565154703139405824/934262724827963434/2022-01-21.png)
%end

##### giaco\_mz 01/22/2022 18:44:12
Hi i am running robocup\kid.wbt simulation. I want a new simulation with a lower number of robot but if i try to delete some of them webots crash. Where i can find the file to lower the number of robots?

##### DDaniel [Cyberbotics] 01/22/2022 18:54:21
Just by deleting them it crashes? Which OS and webots version are you using? I have no issues on my side. Either way the adult.wbt only has 4 robots

##### giaco\_mz 01/22/2022 18:55:46
Windows - 477.46 MB

18 Dec 2021 - R2022a

##### DDaniel [Cyberbotics] 01/22/2022 18:57:42
You mean the included sample world kid.wbt, or it's a custom one (from a previous version of webots)?

##### giaco\_mz 01/22/2022 18:58:01
the included one


Now setting field size on kid to the adult one i have this error, it could be related
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/934522977343668274/unknown.png)
%end


after this error also the adult one with the field size parameter set to kid crash

##### DDaniel [Cyberbotics] 01/22/2022 19:05:19
That error is from the physics engine, I think something is very wrong with your version, something might have been corrupted. I suggest you try to completely uninstall webots and reinstall it

##### giaco\_mz 01/22/2022 19:06:18
ok, one question can you give me some advice to speed up this kind of simulation please?


from webots settings point of view

##### DDaniel [Cyberbotics] 01/22/2022 19:15:10
these simulations are meant to be very realistic, so they are especially heavy. If you don't need that much, disabling `backlash` modeling would help (set the corresponding flag to false), and disable `turfPhysics` for the ground. Disabling rendering also helps, but there's not much else you can do, it depends a lot on your system too

##### giaco\_mz 01/22/2022 19:18:31
Ok perfect thx a lot you were very helpfull


i have try to reinstall webots but i have the same problem

##### Yosi F 01/22/2022 20:32:01
Is their a way to run unit test for Python controllers in webots?

##### Max\_K 01/24/2022 12:37:32
ok 😅 .. Do you have an idea for a workaround? I want to detect if the feet of the spot are on the ground. When the spot move/sits the feet turns red, so maybe i could just read out the boundingbox?
> **Attachment**: [2022-01-24\_13-25-55.mp4](https://cdn.discordapp.com/attachments/565154703139405824/935151332325724220/2022-01-24_13-25-55.mp4)

##### Benjamin Hug [Moderator] 01/24/2022 12:50:20
That could be something to do. With ROS 2 you should be able to create a plugin and use the Supervisor API in case your robot is a Supervisor.

##### Max\_K 01/24/2022 18:01:52
When i try to import the Supervisor module i get this error: 
```[driver-2] Traceback (most recent call last):
[driver-2]   File "/home/max/webots_ws/build/webots_spot/webots_spot/spot_driver.py", line 8, in <module>
[driver-2]     from controller import Supervisor
[driver-2]   File "/usr/local/webots/lib/controller/python38/controller.py", line 31, in <module>
[driver-2]     import _controller
[driver-2] ImportError: /usr/local/webots/lib/controller/python38/_controller.so: undefined symbol: _ZNK6webots3GPS14getSpeedVectorEv
[driver-2] terminate called after throwing an instance of 'std::runtime_error'
[driver-2]   what():  The webots_spot.spot_driver.SpotDriver plugin cannot be found (C++ or Python).
[ERROR] [driver-2]: process has died [pid 73385, exit code -6, cmd '/home/max/webots_ws/install/webots_ros2_driver/lib/webots_ros2_driver/driver --ros-args --params-file /tmp/launch_params_zgpmbvdv'].
```

maybe my env vars are still wrong

##### Benjamin Hug [Moderator] 01/25/2022 09:45:08
Just take example on the mavic plugin here ([https://github.com/cyberbotics/webots\_ros2/blob/master/webots\_ros2\_mavic/webots\_ros2\_mavic/mavic\_driver.py#L40](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_mavic/webots_ros2_mavic/mavic_driver.py#L40)). (Edit: check the line 40, this is the way to get the instance of the robot)

##### Max\_K 01/25/2022 12:06:44
Ok, i have an instance of my robot, but i cant execute supervisor functions. I want to use getFromDef to access a boundingObject Group that i could use to detect the collision.

I have the same error as here: [https://github.com/cyberbotics/webots/issues/4134](https://github.com/cyberbotics/webots/issues/4134)



I exported the env vars, but the error remains

##### Benjamin Hug [Moderator] 01/25/2022 13:08:13
Please open an issue on Github so we can look at this together more in depth.

##### Yosi F 01/25/2022 14:38:43
I had a similar issue


I was using an older version of webots. when I upgraded, everything worked

##### Max\_K 01/25/2022 14:46:07
[https://github.com/cyberbotics/webots/issues/4151](https://github.com/cyberbotics/webots/issues/4151) 

I opened an issue.

##### moebius 01/26/2022 03:47:32
if a .wbo file i am importing into webots has the coordinate system with the x axis pointing up and the arena has the coordinate system with the y axis pointing up (a +90 deg rotation about the z axis basically) , is there anyway I can change the coordinate systems of either of them so they match? I need to use a compass and the values are obviously not usable at the moment. Or should I do some transformation in the controller itself to use the compass values? would really appreciate the help, i am a bit stuck here.

##### Rico Schillings[Sweaty] [Moderator] 01/26/2022 09:57:47
I'm trying to rebuild a RC-car in webots which we are using for several university projects to test self-driving algorithms. Its a Audi q2 scaled with 1:8 (see picture). I am using the Car.proto to allow working with the webots driver library, but when i down-scaled everything with the real dimensions of our car, I have the following problem: Using 4x4 is working but it seems that the car is "slipping" over the front wheels as soon as I steer into a curve (like the friction of the front wheels is to low and the rear wheels are pushing the car ahead instead of steering correctly..). Since our real car has only a rear motor, I would like to use the "propulsion" type of car.proto. But this is not working for such small-scaled cars like I want. With propulsion the front tires are blocking/fixed and do not rotate. 



I also tested the example proto of the saeon altino car. This is by default a 4x4 type, but when I change it to propulsion, its front wheels are also blocking. 



Could it be that there is a kind of a bug with the car.proto which only work for (nearly) realistic dimensions but not when working with small-scaled cars?
%figure
![audi_model.jpeg](https://cdn.discordapp.com/attachments/565154703139405824/935835909113413642/audi_model.jpeg)
%end

##### DDaniel [Cyberbotics] 01/26/2022 09:59:38
Are you using the `scale` parameter to scale it down? This scaling should only be used on geometries, not at the robot level

##### Rico Schillings[Sweaty] [Moderator] 01/26/2022 10:02:58
No I just use `scale` on my created mesh/stl files. I have measured all dimensions from the real car and modified the values in the car.proto (e.g. `trackFront = 0.2` or `wheelbase=0.36`) and also created a new wheel.proto with measured values

##### Olivier Michel [Cyberbotics] 01/26/2022 15:42:38
Maybe you need also to adapt some other parameters, like the various spring constants, torques, break coefficients, etc.?

##### Rico Schillings[Sweaty] [Moderator] 01/26/2022 15:56:20
Well, this could be since I dont have adapt all of them. But 1. For some values I dont know exactly what plausible values would be (some are unitless so i dont know the plausible range or on what they depend) and 2. This would Not explain, why the saeon altino have the same Problem when changing from 4x4 to propulsion. I think the other parameters of the altino should be adapted well for the mini car?

##### Timple 01/26/2022 18:33:07
Importing a python supervisor fails for webots 2022a:



```
$ python3 --version
Python 3.8.10
$ echo $PYTHONPATH 
/home/tim/ws/install/my_robot/lib/python3.8/site-packages:/opt/ros/galactic/lib/python3.8/site-packages:/home/tim/repos/ros_command::/usr/local/webots/lib/controller/python38
$ webots --version
Webots version: R2022a
$ echo $WEBOTS_HOME 
/usr/local/webots
$ python3 -c 'import controller'
Traceback (most recent call last):
  File "<string>", line 1, in <module>
  File "/usr/local/webots/lib/controller/python38/controller.py", line 31, in <module>
    import _controller
ImportError: /usr/local/webots/lib/controller/python38/_controller.so: undefined symbol: _ZN6webots5Robot19internalGetInstanceEv
```

Also when using SNAP version. Can someone confirm this?

##### Olivier Michel [Cyberbotics] 01/27/2022 07:59:10
I just tried that and it worked fine for me. The only difference is that my `PYTHONPATH` contains only `/usr/local/webots/lib/controller/python38`. Could you also check your `LD_LIBRARY_PATH` and `PATH` environment variables?


Even after I typed `source /opt/ros/galactic/setup.bash`, `PYTHONPATH` is changed to `/opt/ros/galactic/lib/python3.8/site-packages:/usr/local/webots/lib/controller/python38` and everything works as expected. So, there is apparently not conflict with `galactic`.

##### Timple 01/27/2022 08:52:00
Interesting. Colleagues have the same issue. I'll report back later today on your suggestion

##### Rico Schillings[Sweaty] [Moderator] 01/27/2022 11:47:14
Ok I've tested a lot with different parameters and you're right. After decreasing the wheelDampingConstant to a very low value it seems to work with propulsion. Nevertheless I still have to find more realistic values for those parameters 🙂 Maybe I write a simple genetic optimizer for this 😉 Thanks for your hint to check them

##### Olivier Michel [Cyberbotics] 01/27/2022 11:48:16
Let me know whatever you find. It would be nice to be able to document it somehow.

##### Rico Schillings[Sweaty] [Moderator] 01/27/2022 11:51:58
Alright. Since we want to try to use webots for further autonomous plattforms at our university I will optimize them further. As soon as its accurate enough I will also start to use webots to test several learning algorithms to evaluate if we can use it for transfer learning progress (sim2real) 🙂

##### amna 01/28/2022 08:03:18
Why am I unable to find skin animated characters in R2022a?

##### DDaniel [Cyberbotics] 01/28/2022 08:04:35
An example should be available under `file > open sample world > humans > skin_animated_humans`

##### amna 01/28/2022 08:05:00
ok great!


What values shall i give to trajectory, speed and step in human.py?

##### DDaniel [Cyberbotics] 01/28/2022 08:33:55
what is human.py? your own controller? normally you need .BVH files that define the animation the character will play

##### amna 01/28/2022 08:45:48
`@DDaniel` Yes, it is a controller of pedestrian.


It is in python and documentation says to add values of trajectory, speed and step to make the pedestrian move

##### DDaniel [Cyberbotics] 01/28/2022 08:55:02
pedestrian is a different thing, a sample of that is also available (`file > open sample world > humans > pedestrian`). If you use the pedestrian.py controller you need to specify in the `controllerArgs` field these properties, `--trajectory` specifies the path the human will take, `--speed` the speed it should run at and `--step` the timestep. If you look at that example it should be fairly clear

##### amna 01/28/2022 08:55:52
ok `@DDaniel` 

got it. Thank You so much


`@DDaniel`   I am facing this issue: Traceback (most recent call last):

  File "pedestrian.py", line 16, in <module>

    from controller import Supervisor

  File "C:\Program Files\Webots\lib\controller\python38\controller.py", line 31, in <module>

    import \_controller

ImportError: DLL load failed while importing \_controller: The specified module could not be found.

WARNING: 'pedestrian' controller exited with status: 1.

Traceback (most recent call last):

  File "pedestrian.py", line 16, in <module>

    from controller import Supervisor

  File "C:\Program Files\Webots\lib\controller\python38\controller.py", line 31, in <module>

    import \_controller

ImportError: DLL load failed while importing \_controller: The specified module could not be found.

WARNING: 'pedestrian' controller exited with status: 1.

##### JoséeMallah 01/28/2022 12:59:58
Hello everyone

Are there any tutorials out there, or anyone who could help me, in doing walking simulations with c3d models?


Can I convert it to a robot? Or how to add motors to the joints as someone recommended on the <#565155651395780609> channel?

##### amna 01/28/2022 13:42:38
`@JoséeMallah` everything is available in samples, check them. File>open sample world>human>c3d

##### Ranga Kulathunga 01/28/2022 16:20:16
Hi all, how to change makefile when we use C++ for the controller?

##### JoséeMallah 01/29/2022 11:16:26
Thank you. But I am getting this error:

Traceback (most recent call last):

  File "C:\Program Files\Webots\projects\humans\c3d\controllers\c3d\_viewer\c3d\_viewer.py", line 262, in <module>

    c3dfile = c3dFile(sys.argv[1])

  File "C:\Program Files\Webots\projects\humans\c3d\controllers\c3d\_viewer\c3d\_viewer.py", line 200, in \_\_init\_\_

    for i, points, analog in self.reader.read\_frames():

  File "C:\Program Files\Webots\projects\humans\c3d\controllers\c3d\_viewer\c3d.py", line 853, in read\_frames

    offsets = param.int16\_array[:self.analog\_used, None]

  File "C:\Program Files\Webots\projects\humans\c3d\controllers\c3d\_viewer\c3d.py", line 345, in int16\_array

    return self.\_as\_array('h')

  File "C:\Program Files\Webots\projects\humans\c3d\controllers\c3d\_viewer\c3d.py", line 329, in \_as\_array

    elems.fromstring(self.bytes)

AttributeError: 'array.array' object has no attribute 'fromstring'

WARNING: 'c3d\_viewer' controller exited with status: 1.



How to solve it?

##### amna 01/29/2022 11:17:34
`@JoséeMallah` download python 3.8 and add its path to environment variables.

##### JoséeMallah 01/29/2022 11:21:57
I have 3.9 and it's already in the path
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/936944254335479868/unknown.png)
%end

##### amna 01/29/2022 11:22:46
change the slash

##### JoséeMallah 01/29/2022 11:47:01
Okay, it is solved now. The problem is that apparently python 3.9 does not work here. Downgrading to python 3.8 solved the problem. Thank you `@amna`.

##### amna 01/29/2022 13:08:21
`@JoséeMallah` 👍🏻

##### Timple 01/31/2022 07:02:22
So I removed `lld` as linker on my computer. Solved the issue... Thanks for thinking along

## February

##### JoséeMallah 02/01/2022 12:39:01
Hello everyone

I am working on a prosthesis project. Therefore, I would like to have a human model, maybe a CharacterSkin (simulated using .bvh files), "cut" its leg, and replace it with our prosthesis model.

Is that even possible in Webots? And any idea on how to do so?

##### DDaniel [Cyberbotics] 02/01/2022 17:51:09
.bvh defines an animation, there's no physics involved. For a physically driven simulation you need to specify the solids corresponding to the bones (and joints)

##### ToKi 02/01/2022 21:06:34
Hey there,

so I have this older Webots-project for demonstrating line follower robots for a course I'm holding. The model is based on the e-puck model with a normal "infra-red" type sensor set in the groundSensorsSlot. I use a black and white png set as the baseColorMap under RectangleArena/floorAppearance PBRAppearance as the track to follow. I previously had a R2020-version of Webots installed, where the model was still working. Since the students will most likely use the newest version, I today also upgraded to R2022a. Otherwise the model opens and runs fine, but the irSensor now delivers a constant value (1000 with the standard lookup table) Does anyone have any pointers, what might have happened in the last couple of versions? Is there some setting I need to tweak to get it working again? I couldn't find anything directly helpful in the release notes nor online, and playing aroud with the parameters didn't seem to do much either. Thanks!


Here's a better picture:
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/938180217795645550/unknown.png)
%end

##### DrakerDG [Moderator] 02/01/2022 21:37:01
Something like that?
> **Attachment**: [e-puck\_LF\_V1\_2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/938186202140528710/e-puck_LF_V1_2.mp4)

##### ToKi 02/01/2022 21:38:12
Yes, just with IR sensors

##### DrakerDG [Moderator] 02/01/2022 21:42:46
How much IR sensors used in your robot?

##### ToKi 02/01/2022 21:47:14
I would like to use anywhere from 1-5, I tried with 1 and 5, for the 1 sensor I re-created the whole project from scratch but still got the same issue


I was originally using robot.getDistanceSensor() to get the handle to the object, now getDevice, which was deprecated somewhere along the way, not sure if there were some other changes apart from the API-change along the way, and whether it has anything to do with my problem 🙂

##### DDaniel [Cyberbotics] 02/01/2022 21:50:08
Can you share the world? Can't think of a reason it shouldn't work

##### ToKi 02/01/2022 21:50:57
Sure, can I just send it to you here?

##### DDaniel [Cyberbotics] 02/01/2022 21:51:02
Sure

##### ToKi 02/01/2022 22:13:13
And in case someone with the same problem finds this discussion, check your sensor translation/rotation kids.. Thanks everyone!

##### Rico Schillings[Sweaty] [Moderator] 02/02/2022 16:40:48
Hey. As already mentioned I'm working with the car library to simulate our RC cars. As a next step i want to use our DRL framework to start learning. Therefore i would need some supervisor functions (e.g. reset simulation or groundtruth information..) currently i have implemented my own ros2 node which also acts as external webots controller with the car library. As Ive seen `car` seems to be the "Supervisor" of the `driver` but it has no real supervisor access. My question is: what would be the "nicest" solution? Define my car proto as supervisor and do a internal typecast on the supervisor node to also have access to the car/driver library OR add a separate Supervisor robot to my world as supervisor? In the last case i would have 2 external controllers since I also would like to use ros2 for the supervisor functions🤨

##### DDaniel [Cyberbotics] 02/02/2022 17:12:03
The Car class inherits from the Supervisor class so you have all powers with a Car instance. If you rely on the Car.proto for your model, then you might need to set the Supervisor flag to true

##### Rico Schillings[Sweaty] [Moderator] 02/02/2022 17:39:56
oh dear, shame on me... I rely on the Car.proto but forgot to add the supervisor field to it so I was unable to test it before i asked my (stupid) question.. Now it works fine and I have all supervisor power like I need. Thanks for this hint 🙂

##### mclzc 02/02/2022 23:30:49
Hi, I'd like to use Webots to test different cases of a lifting mechanism subjected to various loads.

Could you point me out with anything that comes to your mind regarding the following please?

1) I'd like to work mostly in 2D (but I guess I'll just go with 3D)

2) How to restrict an arbitrary number of degrees of freedom in one end of a bar (e.g. pin joint restricting x and y movement; fixed joint restricting x, y and rotation movement, etc).

3) Run multiple simulations, varying programmatically certain parameters of the Proto file

##### Oxmoon 02/03/2022 01:10:04
```
/usr/local/bin/python3 "/Users/oxmoon/Documents/Project Files/Webots/MyProject/controllers/Test.py"
/Users/oxmoon/.zshrc:102: /opt/homebrew/bin:/Library/Frameworks/Python.framework/Versions/3.9/bin:/Library/Frameworks/Python.framework/Versions/3.7/bin:/opt/homebrew/bin:/opt/homebrew/sbin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/share/dotnet:~/.dotnet/tools:/Library/Frameworks/Mono.framework/Versions/Current/Commands:/Library/Frameworks/Python.framework/Versions/3.9/bin:/Library/Frameworks/Python.framework/Versions/3.7/bin:/opt/homebrew/bin:/opt/homebrew/sbin not found
➜  ~ /usr/local/bin/python3 "/Users/oxmoon/Documents/Project Files/Webots/MyProject/controllers/Test.py"
Traceback (most recent call last):
  File "/Users/oxmoon/Documents/Project Files/Webots/MyProject/controllers/Test.py", line 1, in <module>
    from controller import Robot
  File "/Users/oxmoon/Library/Python/3.9/lib/python/site-packages/controller.py", line 31, in <module>
    import _controller
ImportError: dlopen(/Users/oxmoon/Library/Python/3.9/lib/python/site-packages/_controller.so, 0x0002): Library not loaded: @rpath/lib/controller/libController.dylib
  Referenced from: /Users/oxmoon/Library/Python/3.9/lib/python/site-packages/_controller.so
  Reason: tried: '/Users/oxmoon/Library/Python/3.9/lib/python/site-packages/../../../lib/controller/libController.dylib' (no such file), '/Users/oxmoon/Library/Python/3.9/lib/python/site-packages/../../../lib/controller/libController.dylib' (no such file), '/usr/lib/libController.dylib' (no such file)
```



I'm getting this error on an M1 mac when running a controller script in python 3.9.9. If anyone could point me in the right direction it would be greatly appreciated

##### amna 02/03/2022 01:11:09
`@Oxmoon` install python 3.8. it works.

##### Oxmoon 02/03/2022 01:11:23
Ok i will try it, thank you


```
Traceback (most recent call last):
  File "/Users/oxmoon/Documents/Project Files/Webots/MyProject/controllers/Test.py", line 1, in <module>
    from controller import Robot
  File "/Users/oxmoon/Library/Python/3.8/lib/python/site-packages/controller.py", line 31, in <module>
    import _controller
ImportError: dlopen(/Users/oxmoon/Library/Python/3.8/lib/python/site-packages/_controller.so, 0x0002): Library not loaded: @rpath/lib/controller/libController.dylib
  Referenced from: /Users/oxmoon/Library/Python/3.8/lib/python/site-packages/_controller.so
  Reason: tried: '/Users/oxmoon/Library/Python/3.8/lib/python/site-packages/../../../lib/controller/libController.dylib' (no such file), '/Users/oxmoon/Library/Python/3.8/lib/python/site-packages/../../../lib/controller/libController.dylib' (no such file), '/usr/local/lib/libController.dylib' (no such file), '/usr/lib/libController.dylib' (no such file)
```

I am having the same problem with 3.8 as well

##### amna 02/03/2022 01:20:20
`@Oxmoon` have you passed the path to environment variables?

##### Oxmoon 02/03/2022 01:20:38
I don't believe so how can i do that?

##### amna 02/03/2022 01:20:59
are you a Mac or windows user?

##### Oxmoon 02/03/2022 01:21:03
mac

##### amna 02/03/2022 01:21:45
I dn know how to do in Mac, try to google “how to set path in environment variables in Mac”. Follow the steps and it will work fine then.

##### Oxmoon 02/03/2022 01:21:55
ok will do, thanks

##### amna 02/03/2022 01:22:01
👍🏻

##### Oxmoon 02/03/2022 02:11:40
I edited the path but then it couldn't find \_controller.so so i tried putting all the files in my python directory and it still doesn't work

##### amna 02/03/2022 08:10:42
How can we add a timer to create robot (cleaning robot) to stop its movement after certain time? `@DrakerDG`

##### DrakerDG [Moderator] 02/03/2022 08:16:52
You can use in c :

float timeX = wb\_robot\_get\_time() //  obtain the run time simulation

In python is similar:

timeX =  robot.getTime()  #  obtain the run time simulation



You can combine with supervisor node to stop the run simulation, for example


[https://www.cyberbotics.com/doc/reference/robot?tab-language=python#wb\_robot\_get\_time](https://www.cyberbotics.com/doc/reference/robot?tab-language=python#wb_robot_get_time)

##### amna 02/03/2022 08:35:36
Great


Thank You

##### DrakerDG [Moderator] 02/03/2022 08:35:55
You welcome


[https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_simulation\_set\_mode](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_simulation_set_mode)

##### amna 02/03/2022 08:38:53
`@Oxmoon` Go to Webots, click on Tools —-> Preferences..—> paste the directory of Python in python command. Add python.exe in the end of directory. Something like this: C:\user\………..\python38\python.exe —-> click ok and reload world. It is working.


Sharing a tip <#565154703139405824> we can export the specific module like any robot or human or object from sample worlds and save them in our current project folder. Next, open your project and click on + then click on use proto —> import —> select the saved object and add it in your project. It works fine.  No need to write codes again or alter body slots or any properties.

##### Penoctikum 02/03/2022 13:03:11
Hi, I would like to create a web based interactive simulation and would like to be able to send commands to webots via the web page.Is this possible with buttons?

##### Benjamin Délèze [Cyberbotics] 02/03/2022 13:15:07
Yes, you can see an example here: [https://cyberbotics1.epfl.ch/open-roberta/](https://cyberbotics1.epfl.ch/open-roberta/)

##### Penoctikum 02/03/2022 13:19:42
Thank You. But can i move the Bot there?

##### Benjamin Délèze [Cyberbotics] 02/03/2022 13:32:19
It should be possible. You can use the `sendMessage` function documented there: [https://cyberbotics.com/doc/guide/web-simulation#how-to-embed-a-web-scene-in-your-website](https://cyberbotics.com/doc/guide/web-simulation#how-to-embed-a-web-scene-in-your-website), to send a message from the web view to the controller of a robot and then use something like that:

```
while(wb_robot_step(time_step) != 1) {
  const char *message = wb_robot_wwi_receive_text();
  if (message == NULL)
    continue;
    if (strncmp(message, "forward:", 7) == 0) {
      //code to move forward
    else if (...)
  }
}
```

in the controller to handle the messages.

##### Penoctikum 02/03/2022 13:33:55
Thank You very much. I will try it.

##### JoséeMallah 02/03/2022 14:35:15
Oh.. can't I simulate the intact body parts using .bvh and apply physics to the prosthesis part only?

##### waynemartis 02/04/2022 09:33:41
I am trying to give yaw to the mavic drone while hovering, in NUE. I notice that when I give a yaw greater than 1.57 rad, the drone becomes unstable and crashes. Can anyone help me with this?

##### 안정수 02/04/2022 10:24:19
I'm going to use ros\_automobile as a controller. However, with the "--robot-description" option, robot\_state\_publisher does not work. Has anyone used the car as ROS, not the Skidbot?

##### Rico Schillings[Sweaty] [Moderator] 02/04/2022 10:31:10
I'm using the car with ros2, but not using anything of the provided ros code from webots, complete own implementation of my node and interaction with webots.


I have another question. Is there some kind of function to get the lanes of a roadsegment? I want to check on which lane of the road the car is driving and just ask if there already exist some functionality for this case or I should implement it from scratch by getting the fields and calculate it manually? 🙂

##### 안정수 02/04/2022 12:24:27
I also tried implementing my own. However, when I visualize my vehicle in Rviz, the PointCloud data is not visible due to the TF problem.

Answer to your additional question: I'm not sure of the answer because I'm experimenting in a field environment. Sorry for not giving you an answer.

##### joachim honegger [Moderator] 02/04/2022 13:49:13
You can have a look to this code, it notably purposes to move the mavic to specific coordinates: [https://github.com/cyberbotics/webots-projects/blob/master/projects/forest\_firefighters/controllers/autonomous\_mavic/autonomous\_mavic.py](https://github.com/cyberbotics/webots-projects/blob/master/projects/forest_firefighters/controllers/autonomous_mavic/autonomous_mavic.py)

##### Benjamin Hug [Moderator] 02/04/2022 14:27:22
Do you mean that you stated `--robot-description` in the controllerArgs of the robot in Webots, and that your `robot_state_publisher` node does not publish although you started it ?

##### 안정수 02/04/2022 16:18:49
Yes, this option is used in the wbt file and TF is not generated properly when robot\_state\_publisher is executed.

##### Benjamin Hug [Moderator] 02/04/2022 16:24:19
Can you see if robot\_state\_publisher receive an urdf as parameter?


In case it doesn't receive the urdf, could you open on issue on [https://github.com/cyberbotics/webots\_ros/issues](https://github.com/cyberbotics/webots_ros/issues) and, if possible, upload your world so I can take a look?

##### 안정수 02/04/2022 16:28:16
It is not receiving your urdf. Would it be okay if I upload it later?

##### Benjamin Hug [Moderator] 02/04/2022 16:28:31
Of course 🙂

##### 안정수 02/04/2022 16:29:02
Thank you 😭😭

##### Rico Schillings[Sweaty] [Moderator] 02/04/2022 17:38:46
Np, my question was rather meant to the community, not directed to you;)

##### 안정수 02/05/2022 12:27:24
Thanks for waiting.

I used ros\_automobile as controller. The webots\_ros package is required to run.
> **Attachment**: [ugv\_pkg.zip](https://cdn.discordapp.com/attachments/565154703139405824/939497439419846686/ugv_pkg.zip)


roslaunch ugv\_pkg ugv.launch <- this is my package

##### mclzc 02/06/2022 06:34:15
Hi, how could I fix a solid to the ground?

The closest thing I can imagine is adding some joint as a children to the Solid that is that cylinder, and for its endpoint, defining a SolidReference with a value of "<static environment>". But the problem is that the joints I know how to use (Hinge and Slider) would give a degree of freedom that I don't want.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/939770951254564874/unknown.png)
%end

##### DDaniel [Cyberbotics] 02/06/2022 09:25:15
If the uppermost solid doesn't have physics enabled, it will be fixed to the environment, so either remove physics from the cylinder or add an upper solid without physics. If it's a robot the same applies

##### Benjamin Hug [Moderator] 02/06/2022 10:20:30
Perfect I will investigate that in the next days.

##### mclzc 02/06/2022 15:56:05
Thank you very much.


Is it possible to force a body to have a velocity in certain direction and then read the torque values of attached motors? 

I have a mechanism and want to know the input torque profile that would be needed to obtain certain motion in the other end of the mechanism.


My Webots was crashing; it was stuck thinking that it needed to run void.exe as a controller, even though in my world there is no supervisor==True nor controller "something" defined anymore.

It is related to my usage of SolidReference and "<static environment>", which I tried using because removing the physics node wasn't working anymore to fix a body for some reason (that's a different story).

Removing the SolidReference fixes the issue, but I still wonder why was it crashing in that way.
> **Attachment**: [lifting\_platform.zip](https://cdn.discordapp.com/attachments/565154703139405824/940123406760837160/lifting_platform.zip)

##### Benjamin Hug [Moderator] 02/07/2022 10:00:27
On my side it is working, I can see the publication on the `/tf` topic.

##### Rico Schillings[Sweaty] [Moderator] 02/07/2022 10:33:29
I'm trying to create several tracks with RoadSegments. When I add these in the world-file I can access the fields of each segment without problems (`getFromDef("ROAD1").getField("length").getSFFloat()`) But to keep my world file clean and make several track files, I introduced a new proto were I have some fields to re-use (e.g. `width/ lines`)  for all road segments and just list the roadSegments in a Group node (also with DEF). But when I try to access the fields of the grouped segments (`getFromDef("TRACK").getFromProtoDef("ROAD1").getField("length")...` I get invalid values for the fields of these segments. Is there a kind of access limitation?

##### waynemartis 02/07/2022 10:50:47
In an NUE coordinate system, when I orient the mavic facing positive z axis(yaw=90degree), I notice that the imu gives noisy values for roll and pitch, compared to the the normal orientation facing x axis. How do I avoid this? Also, can I get the internal model of the mavic?

##### 안정수 02/07/2022 14:33:22
On my side, /tf is being published, but the buffer is zero.

and also velodyne tf is not published

Can you check in Rviz if the PointCloud of velodyne HDL32E is being published? I'm trying to create a new /tf because of this problem.

##### Max\_K 02/07/2022 15:25:40
hello I am trying to control the spot via ros2 with teleop. I have ported parts of the SoftServe repo: [https://github.com/SoftServeSAG/robotics\_spot/blob/temp\_robomaker/robot\_ws/src/rs\_inverse/scripts/SpotKinematics.py](https://github.com/SoftServeSAG/robotics_spot/blob/temp_robomaker/robot_ws/src/rs_inverse/scripts/SpotKinematics.py)

 

There is a SpotModel class which are initialized with parameters for the legs. As comment it says "Spot Micro Kinematics". 

Is the Spot in webots the "Micro" version or another one?

I don't know if these values are correct:

``` shoulder_length=0.110945,
elbow_length=0.3205,
wrist_length=0.37,
hip_x=2*0.29785,
hip_y=2*0.055,
foot_x=2*0.29785,
foot_y=2*0.2,
height=0.6,
com_offset=0.016,
shoulder_lim=[-0.78539816339744827899, 0.78539816339744827899],
elbow_lim=[-0.89884456477707963539, 2.2951079663725435509],
wrist\_lim=[-2.7929, -0.254801]
```



For the \_lim parameters, I have so far only specified the min/max position of the 3 motors.

The rest of the values I should be able to look up in the "spot.proto" or?
> **Attachment**: [2022-02-07\_13-46-38.mp4](https://cdn.discordapp.com/attachments/565154703139405824/940267079301492786/2022-02-07_13-46-38.mp4)

##### mclzc 02/07/2022 18:26:23
Hi, I'm not having luck fixing the endpoint of a slider joint. I'm trying for the endpoint to have a solid without a physics node, but the solid isn't being fixed.

I have:

`Robot (the beam) with physics and boundingObject -> HingeJoint -> Solid with physics (blue cylinder) -> SliderJoint -> Solid without physics (white cylinder)`

The blue cylinder seems to be attached to the white cylinder and dragging it down when the simulation starts.

Any ideas please?
%figure
![before.png](https://cdn.discordapp.com/attachments/565154703139405824/940312550015377418/before.png)
%end
%figure
![after.png](https://cdn.discordapp.com/attachments/565154703139405824/940312552389373962/after.png)
%end

##### DrakerDG [Moderator] 02/07/2022 19:09:45
Can you share your world?

##### mclzc 02/07/2022 19:12:54
Yes, this is it
> **Attachment**: [lifting\_platform.wbt](https://cdn.discordapp.com/attachments/565154703139405824/940324261204197396/lifting_platform.wbt)

##### DrakerDG [Moderator] 02/07/2022 22:58:40
Hello, I see some points to review:

The orientation of the axis must be configured depending on the movement required. 

Maybe the correct orientation is vertical.

The boundingObject must be included if physics is included
%figure
![2022-02-07_1.png](https://cdn.discordapp.com/attachments/565154703139405824/940381076713508884/2022-02-07_1.png)
%end
%figure
![2022-02-07_2.png](https://cdn.discordapp.com/attachments/565154703139405824/940381076977766450/2022-02-07_2.png)
%end

##### mclzc 02/07/2022 23:11:47
Thank you for having reviewed it `@DrakerDG` . 

- The axis that you mention is a hinge joint that I want. You may be thinking that such axis is the one of the SliderJoint, but that one is in the correct orientation, just hidden behind the shape.

- I've added the Shape of the blue cylinder as a BoundingObject, but found no difference in the results: the blue cylinder still drags down the white cylinder.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/940384377190350848/unknown.png)
%end

##### DrakerDG [Moderator] 02/08/2022 00:01:44
OK, I understand, let me check again


What is the purpose of the hingejoint?

##### mclzc 02/08/2022 00:46:05
If I don't use it, my system is statically indeterminate to the first degree. (The program I use there is called GIM: [http://www.ehu.eus/compmech/software/](http://www.ehu.eus/compmech/software/))
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/940408112798662666/unknown.png)
%end

##### DrakerDG [Moderator] 02/08/2022 01:01:50

> **Attachment**: [lifting\_platform.wbt](https://cdn.discordapp.com/attachments/565154703139405824/940412073857413150/lifting_platform.wbt)



> **Attachment**: [lifting\_platform.mp4](https://cdn.discordapp.com/attachments/565154703139405824/940412112994459648/lifting_platform.mp4)


I reversed the order of children



> **Attachment**: [lifting\_platform\_1.mp4](https://cdn.discordapp.com/attachments/565154703139405824/940414558214975529/lifting_platform_1.mp4)


For the other side, you can use the next example that couple both slider joins


[https://www.cyberbotics.com/doc/guide/samples-devices#coupled\_motor-wbt](https://www.cyberbotics.com/doc/guide/samples-devices#coupled_motor-wbt)

##### mclzc 02/08/2022 02:02:43
Thanks for your help `@DrakerDG`.

##### DrakerDG [Moderator] 02/08/2022 02:02:54

> **Attachment**: [lifting\_platform1\_1.mp4](https://cdn.discordapp.com/attachments/565154703139405824/940427442269614180/lifting_platform1_1.mp4)

##### mclzc 02/08/2022 02:05:55
I see that you started the model using the white cylinder as the base Robot node. It works but I still wonder if starting from the read beam would work somehow. Starting the modelling from the center read beam would be better because all the joints would descend from it, and would be easier to manage. Having the Robot node in the start or last element would create a huge tree underneath it.

##### Houstone 02/08/2022 06:02:23
I'd like to use elevation grid node for my 100*100 map. According to webots documentation, I should get about 10000 value of height with just type every value (because of xSpacing 1, ySpacing 1) . Would I know any other convenience way to get height value? For example, get value from .stl file... etc. My computer is hard to import .stl file cause of my gpu.

##### Olivier Michel [Cyberbotics] 02/08/2022 07:42:15
The spot model in Webots corresponds to the real one from Boston Dynamics, not the "Micro" version. You should indeed look inside the Spot.proto file to get the values you need.

##### 안정수 02/08/2022 08:44:55
Has anyone published a topic PointCloud2 as ROS via the controller package in python?

##### Flo\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_ 02/08/2022 12:30:08
Hi, when trying to use the xacro2proto or urdf2proto function in ros2 i get the following error. I installed and built webots\_ros2 according to the wiki, any idea why this error occurs? I get the same message with different urdf and xacro files.
%figure
![staticbaseerror.png](https://cdn.discordapp.com/attachments/565154703139405824/940585289829716028/staticbaseerror.png)
%end

##### Benjamin Hug [Moderator] 02/08/2022 12:37:57
Hi, did you updated `urdf2webots` from `webots_ros2/webots_ros2_importer/webots_ros2_importer/urdf2webots` ?

If it is the case it is normal that the `staticbase` argument is not recognised and by `git checkout 1.0.13` it should work fine.

##### Flo\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_\_ 02/08/2022 14:01:39
Thank you for the hint, i had the ros1 based urdf2proto package installed as well which probably was executed (i thought due to the ros2 run command the other one would be executed anyways), after uninstalling the other package it worked!

##### Craig 02/08/2022 14:52:11
Is it possible to have two joints (hinge and slider) both connect between the same two solid nodes? I'm trying to simulate a hydraulic arm connected at the elbow with a hinge but actuated with a slider

##### DrakerDG [Moderator] 02/08/2022 14:54:59
Yes, you need make a coupled motor, in your case slide motors, I think so.



Take a look in this example: coupled\_motor.wbt



[https://www.cyberbotics.com/doc/guide/samples-devices#coupled\_motor-wbt](https://www.cyberbotics.com/doc/guide/samples-devices#coupled_motor-wbt)


Thisone

##### Craig 02/08/2022 15:02:36
Thanks, I'll take a look at it!


I'm not sure who to report it to, but the GitHub link is broken, the URL for it should be ...coupled\_motors... but reads as ...coupled\_motor... (singular)

##### DrakerDG [Moderator] 02/08/2022 15:20:43
You are right, it is a broken link, it don't have the letter "s" in the end


Thanks! you can open the sample world from webots too
%figure
![2022-02-08.png](https://cdn.discordapp.com/attachments/565154703139405824/940628728659202089/2022-02-08.png)
%end
%figure
![2022-02-08_1.png](https://cdn.discordapp.com/attachments/565154703139405824/940628728889892914/2022-02-08_1.png)
%end

##### DrVoodoo [Moderator] 02/09/2022 10:07:19
How do the controllers actually communicate with webots, some sort of network socket? If so, what's the port?

##### josh101 02/09/2022 13:39:23
I don't suppose anyone has any experience using NAO in WeBots?

##### B̴͌͘r̸̛̐ö̴́̎t̵̤̋ 02/09/2022 14:37:49
Hi guys, I have a quick question: Do you know why my entire simulation restarts when i call `supervisor.step(1)`?

(For context: I'm calling it like this:

```py
if self.supervisor.step(self.timestep) == -1):
  exit()

```  i have checked that it's not just running  `exit()` and even when i call it in a `print` it just restarts)

##### DDaniel [Cyberbotics] 02/09/2022 15:39:58
The step returns `-1` when webots terminates the controller,  don't you intend to check for `!=` instead?

##### B̴͌͘r̸̛̐ö̴́̎t̵̤̋ 02/09/2022 15:42:08
Well the problem is that it doesn't return anything, not even -1, because before it can return anything, everything just restarts

##### DDaniel [Cyberbotics] 02/09/2022 15:42:22
can you share more of the controller?

##### B̴͌͘r̸̛̐ö̴́̎t̵̤̋ 02/09/2022 15:44:17
It's part of the deepbots project for reinforcement learning in webots:

```py
def step(self, action):
        """
        Default step implementation that contains a Webots step conditional for
        terminating properly.

        :param action: The agent's action
        :return: tuple, (observation, reward, is_done, info)
        """
        if self.supervisor.step(self.timestep) == -1:
            exit()

        self.apply_action(action)
        
        return (
            self.get_observations(),
            self.get_reward(action),
            self.is_done(),
            self.get_info(),
        )
```


```py
env = TheController()
while not solved and episodeCount < episodeLimit:
  observation = env.reset()
  env.episodeScore = 0
  for step in range(200):
    # In training mode the agent samples from the probability distribution, naturally implementing exploration
    selectedAction, actionProb = agent.work(observation, type_="selectAction")
    # Step the supervisor to get the current selectedAction's reward, the new observation and whether we reached 
    # the done condition
    newObservation, reward, done, info = env.step([selectedAction])

``` and this is where it is called

##### moebius 02/10/2022 03:08:57
This the .wbo file and the other one is the .stl file, created by the same program. Why is the coordinate system for these two rotated 90 degree about the x axis CW? The worldinfo coordinate system is is ENU, i tried it with the other systems it looks exactly the same. Is there something I'm missing? Thanks!
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/941168834100994078/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/941168834352656404/unknown.png)
%end

##### DrakerDG [Moderator] 02/10/2022 03:36:07
Since webots version 2022, the reference of the Y and Z axes changed, now the Y axis is horizontal and the Z axis is vertical.

What version you are use?

##### moebius 02/10/2022 05:24:41
I'm using 2021B

##### DrakerDG [Moderator] 02/10/2022 05:27:07
This is the reason why, before the last version, the axes were crossed


In 2022a, the Z axis is the vertical

##### moebius 02/10/2022 05:44:18
but if im using 2021b this shouldn't be an issue right

##### DrakerDG [Moderator] 02/10/2022 05:45:47
Not at all, It is a different reference only


You can make your 3D model and you will rotate it, and all will be fine

##### moebius 02/10/2022 05:48:40
so i don't need to align the coordinate frames? I do want to use a compass and gps however, would the mismatch not affect it?

##### DrakerDG [Moderator] 02/10/2022 05:52:49
If you make your project and open it in the same version, you will not any problem.  Only you will need make some additional steps if your open your project in the new webots version

##### moebius 02/10/2022 06:05:39
oh okay. thank you so much!

##### DrakerDG [Moderator] 02/10/2022 06:05:59
You welcome!

##### moebius 02/10/2022 06:14:14
oh another thing, I am not making this node it in webots, a different program is giving the wbo and stl output which i am then importing into webots, so using the compass with this rotated coordinate frame would not cause issues right?

##### DrakerDG [Moderator] 02/10/2022 06:20:30
Not, but thy it, if you will have some issue, you will can share with as, your type of issues

##### moebius 02/10/2022 06:38:13
oh okay, thank you!

##### ~E 02/11/2022 15:23:56
I've programmed a bot with matlab before, but now I'm trying to use python. Can anyone lend a hand or send me in the right direction for finding out how to make this accelerometer work? I can't figure out if it's my code or my bounding for the accelerometer that's messing up my attempt at printing the data to the console.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/941716189556654090/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/941716189892214814/unknown.png)
%end

##### Rico Schillings[Sweaty] [Moderator] 02/11/2022 15:58:17
Dont know if this is the problem, but you are missing () in line 36 at "getValues" (should be getValues())..

##### DrakerDG [Moderator] 02/12/2022 04:45:22
You need initialize the accelerometer and enable this first:



\# Initialize and enable accelerometer

accel = robot.getDevice('accelerometer')

accel.enable(time\_step)


`@Rico Schillings[Sweaty]`  is right, to get the values you need write this:



acceldata = accel.getValues()


\#acceldata is  a vector with x y z values


You can obtain the unique value using trigonometry:



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/941919151805566996/unknown.png)
%end


[https://www.cyberbotics.com/doc/reference/accelerometer?tab-language=python](https://www.cyberbotics.com/doc/reference/accelerometer?tab-language=python)

##### Dharani Saravanan 02/12/2022 14:35:39
Hi team, it looks like position sensor data is not right for the highlighted robot arm position. Please help me in getting the right sensor data.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/942066425986695198/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/942066426292875324/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/942066426599051264/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/942066426888474634/unknown.png)
%end

##### DrakerDG [Moderator] 02/12/2022 15:31:36
Hi! Can you share all your controller?

##### Dharani Saravanan 02/12/2022 15:51:57

> **Attachment**: [python\_arm\_controller.py](https://cdn.discordapp.com/attachments/565154703139405824/942085631088164924/python_arm_controller.py)

##### DrakerDG [Moderator] 02/12/2022 15:52:27
Ok, let me check it


I did try to run your controller but it result is an error, I don't know why.  I don't have a lot of knowledge in python. However I did try get the position sensors values using c code, and I don't have problems
> **Attachment**: [URE\_example\_1.mp4](https://cdn.discordapp.com/attachments/565154703139405824/942100318395699230/URE_example_1.mp4)


Maybe I need your world to test your controller.  Can you try obtain the position sensors without your rad2deg function?

##### Dharani Saravanan 02/12/2022 16:58:00
I tried that too. But I got the same results.


This is the world file I am working with
> **Attachment**: [UR5-kitchen-world.wbt](https://cdn.discordapp.com/attachments/565154703139405824/942103842064060426/UR5-kitchen-world.wbt)

##### DrakerDG [Moderator] 02/12/2022 17:24:23
Ok, let me try too


You made this in webots 2021b, right?

##### Dharani Saravanan 02/12/2022 17:27:59
yes

##### DrakerDG [Moderator] 02/12/2022 17:28:11
Ok

##### 안정수 02/12/2022 18:37:02
How can I visualize a Pointcloud in Rviz using Lidar from webots? Tried using the controller as ros / python , but I don't know how.

##### DrakerDG [Moderator] 02/12/2022 19:02:06
Hi, your code works
%figure
![2022-02-12_1.png](https://cdn.discordapp.com/attachments/565154703139405824/942133481973088346/2022-02-12_1.png)
%end



%figure
![2022-02-12_2.png](https://cdn.discordapp.com/attachments/565154703139405824/942133815709691904/2022-02-12_2.png)
%end



> **Attachment**: [ConvertedVideoOutput\_5.mp4](https://cdn.discordapp.com/attachments/565154703139405824/942135151759093780/ConvertedVideoOutput_5.mp4)


I did convert to 2022a version and add a table, only



> **Attachment**: [ConvertedVideoOutput\_11.mp4](https://cdn.discordapp.com/attachments/565154703139405824/942139800255889408/ConvertedVideoOutput_11.mp4)

##### Dharani Saravanan 02/13/2022 13:21:57
yes changing to version 2022a fixed the issue. Thanks😀

##### waynemartis 02/14/2022 07:36:56
Can I get the internal model and parameters of the mavic2pro?

##### DrakerDG [Moderator] 02/14/2022 14:55:07
I understand that what you need is additional information to what can be seen in the structure of the PROTO



[https://cyberbotics.com/doc/guide/mavic-2-pro](https://cyberbotics.com/doc/guide/mavic-2-pro)

##### Chuong 02/14/2022 21:28:05
I have had a tough time designing robot. The tasks sound like simple, but I am a beginner. Can you guys help me with moving robot "rectangle", "circle", and "waypoints" ?

##### 안정수 02/15/2022 03:50:24
Sorry for repeating the question. Is there a way to publish received LidarPoint data to ROS using getPointCloud(data\_type=buffer) function? The data could not be analyzed.

##### Benjamin Hug [Moderator] 02/15/2022 07:34:45
You should be able to ask a Lidar to publish a PointCloud message in ROS by using a service ([https://github.com/cyberbotics/webots/blob/24050b50b9d1b67799fd40d2fbf127e899a9e749/projects/default/controllers/ros/RosLidar.cpp#L25](https://github.com/cyberbotics/webots/blob/24050b50b9d1b67799fd40d2fbf127e899a9e749/projects/default/controllers/ros/RosLidar.cpp#L25)).

##### ~E 02/15/2022 13:00:06
Whenever I start my simulation, my robot drops through the floor and jumps back up through it. After the sim has started, the collision works for the most part, except that when it falls over, the wheels no unbind from the side for a split second. Could this be a bounding problem? Or is my robot starting with a >9.81 negative acceleration or something? I'll post a gif in a sec



%figure
![Bot2.gif](https://cdn.discordapp.com/attachments/565154703139405824/943131170072326244/Bot2.gif)
%end

##### DrakerDG [Moderator] 02/15/2022 13:24:54
Try reducing the value of basicTimeStep from WorldInfo node, for example from 32 to 16

##### ~E 02/15/2022 13:56:07
How can I get the current step for the program using python? I'm trying to use "step = robot.step()"


wait duh I can just add one in the while loop

##### DrakerDG [Moderator] 02/15/2022 14:01:13

%figure
![unknown-1-1-1.png](https://cdn.discordapp.com/attachments/565154703139405824/943144926307307590/unknown-1-1-1.png)
%end


\# get the time step of the current world.

time\_step = int(robot.getBasicTimeStep())

##### wesllo 02/15/2022 14:16:52
Hello, I have a question about the automation of the world change and save. Is there a way to force a world save after the simulation is launched? Indeed the WorldSave function returns an error.

```
The simulation has run!
Saving the .wbt file will store the current world state: the objects position and rotation and other fields may differ from the original file!
Do you want to save this modified world?
```

##### DDaniel [Cyberbotics] 02/15/2022 14:19:17
If you mean manually using the save button, that's just a warning which you can ignore. The warning is there because more often than not you don't want to save an already started simulation, but there might be situations where you might want to. If you mean saving the world from the controller/supervisor, then you can use the appropriate function: [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_world\_save](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_world_save)

##### wesllo 02/15/2022 14:26:11
thanks <@787796043987025941>. 

Yes, I'm talking about the use of the supervisor's wb\_supervisor\_world\_save function, I need to save even after the simulation has been run, but "The boolean return value indicates the success of the save operation." and the result is always wrong due to the simulation being run.

##### DDaniel [Cyberbotics] 02/15/2022 14:37:33
I've tested and I don't have any issue with it, I can save after the simulation has run without problems. Which version are you using? and which programming language?

##### wesllo 02/15/2022 14:49:02
webots R2021b with C++

##### Jan Weber 02/15/2022 15:34:03
Hello,

in a procedural PROTO file I am calculating many field values with a javascript function. These values are used many times with the IS statement. How can I run my function at the beginning of each time step so that all Field values are updated?

Jan

##### Venkat 02/15/2022 16:12:31
I am trying to use extern controller on the robot(s) I have. The robot is mounted on a linear axis which traverses on one axis. The linear axis is modeled as a robot, so it has its own controller. For the linear axis motion , I use a sliderjoint with "LinearMotor" and "PositionSensor" devices. The structure looks like: LinearAxisRobot (LinearAxisController) -> children -> sliderjoint (LinearMotor and PositionSensor) ->  children -> UR10Robot.. I am trying with "extern" controller for both the robots to have sequential control. URRobot controller works fine with extern controller, that the joints move as expected. But linear axis controller isnt workin properly (from extern controller). The error is "AttributeError : Nonetype object has no attribute 'enable' " (for the position sensor). Is there any way to handle this problem.

Thanks,

Venkatachalam

##### DDaniel [Cyberbotics] 02/15/2022 16:15:28
A proto file containing javascript template statements is executed and therefore converted to pure VRML before the world is run. If after the load of this PROTO you change one of its parameters (using for instance a supervisor) and this change triggers a regeneration of the PROTO, this occurs automatically. Can you show an example of what you're trying to do?


As far as I understood you have a Robot within a Robot, right? If so, the controller of each robot can only access its own devices, not those of the other robot

##### Jan Weber 02/15/2022 16:45:49
I wrote a javascript function that changes positions of joints when the translation of the robot changes. What I understand after your answer is that with each change the proto is regenerated. And this takes about 0.5s. So my simulation becomes very slow...

So my approach is not useful. I will continue to do it with a supervisor for now.

Since I have to change many joint positions of the robot with the supervisor (with setSFFloat or setMFFloat), my simulation is also slowed down. Is there a way to set multiple values, e.g. an array with a single set command?

##### DDaniel [Cyberbotics] 02/15/2022 16:49:08
I assume you only need to change the positions once and then you let the motors/controller do the rest right? If so, for this initialization of the joints you can use [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_set\_joint\_position](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_set_joint_position), which is a more proper approach rather than getting the field references and changing them yourself


but it still requires calling this function for each joint you wish to set a position

##### Jan Weber 02/15/2022 17:21:50
My "robot" is the Pedestrian Model. This has no motors. Since I want to simulate many pedestrians (e.g. 100) and each has 14 joints, I have 1400 setSFFloat() calls per time step. I would like to reduce the overhead from the many individual function calls. According to my measurements, setSFVec3f takes a similar amount of time as setSFFloat, so I suspect that the many individual calls have a large overhead.

##### Venkat 02/15/2022 18:37:14
Yes, I understand that and it works when executed with local controller. The problem is to use extern controller.. 1. since there are more robots in the environment, I am unable to create "Robot()" instance for each robot locally (in local controller). So, I created this instance from the external program and pass this as parameter to the class to instantiate... For the UR10robot, this works fine but not for the Sliderjoint.. I am getting the problem when I try to access "Position Sensors" and their values. I had to change the program to "enable" the position sensors from the method where it is used and not in the class (for UR10 robot and Sliderjoint). For UR10 it is working fine and not for Sliderjoint..

##### ~E 02/16/2022 00:45:40
Anyone wanna help me tune a PID? I'm having trouble figuring out if the issues I'm having is from bad programming, or bad simulation.

##### DrakerDG [Moderator] 02/16/2022 00:54:51
Hi! Where you need tune a PID.  Maybe you can be more specific please


I used PID in line follower robot in webots

##### ~E 02/16/2022 01:14:18

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/943314312330612766/unknown.png)
%end


self balancing robot

##### DrakerDG [Moderator] 02/16/2022 01:23:05
OK, it is the same project


What process you use to tune the PID?  Have you one?

##### ~E 02/16/2022 01:25:23
Just trying different numbers

##### DrakerDG [Moderator] 02/16/2022 01:28:52
OK, maybe you need tuning P first, second I and last D.



Exist a Manual method that using this order


Maybe you need applay this method to balancing robot


How to calculate PID values?

##### 안정수 02/16/2022 07:35:53
yes i already tried it Pointcloud2 Topic is being published but cannot be visualized in Rviz.

##### Benjamin Hug [Moderator] 02/16/2022 07:42:29
May I ask you to open an issue on [https://github.com/cyberbotics/webots\_ros/issues](https://github.com/cyberbotics/webots_ros/issues) and to explain more in detail how you try to visualise it in RViz ? It will be easier for us to discuss.

##### Venkat 02/16/2022 08:33:14
Yes, I understand that and it works when executed with local controller.  As in the attached figure, I made the sliderjoint and had a connector as the "endPoint" to the slider joint. To the children of the slider joint is the UR10e robot attached. With this I tried, invoking the controller from "extern" python program. The problem is to use extern controller.. since there are more robots in the environment, I am unable to create "Robot()" instance for each robot locally (in local controller, it threw an error that there could only one robot instance created). So, I created this instance from the external program and pass this as parameter to the class to instantiate... For the UR10robot, this works fine but not for the Sliderjoint.. I got the problem when I try to access "Position Sensors" and their values. I had to change the program to "enable" the position sensors from the method (inside function of the class) where it is used and not in the class (for UR10 robot and Sliderjoint). For UR10 it is working fine and not for Sliderjoint.. But accessing the "device" motor of the slider joint is still throwing a warning that the devices are not found. In the simulation, the robot itself is not moving. 

I would like to know, if we can have a robot with slider joint and write a controller for it. I had this working for local controller.



~~Also I see now that, when I execute the external python program for only the linear axis, I am getting the following warning: "Failed to attach extern robot controller: no available "<extern> robot controller named "UR10e" found." I made the UR10e controller to be "void" and use extern controller only on the linear axis. I am getting this message when I do so.~~ This problem is due to environment variable error. I would like to solve the problem for error on "Only one instance of robot class should be defined".
%figure
![Webots_Robots.jpg](https://cdn.discordapp.com/attachments/565154703139405824/943424773382373436/Webots_Robots.jpg)
%end
%figure
![Webots_Extern.jpg](https://cdn.discordapp.com/attachments/565154703139405824/943424773709516800/Webots_Extern.jpg)
%end


May I know if the AROSYS from RobMoSys is active. I would like to integrate my simulation with model driven robot control and I found this package. [https://github.com/cyberbotics/AROSYS](https://github.com/cyberbotics/AROSYS). It looks interesting for my work and I would like to use it. Kindly advise, Thank you.

##### jbittner 02/16/2022 13:28:35
Simple question, When using MATLAB controllers is there anyway to speed up the loading of the interpreter? It seems odd to have to execute a entirely new MATLAB interpreter for every run of the simulation.

##### 안정수 02/17/2022 01:40:31
Okay, I will. I will write it down as soon as possible.

##### Vikramaditya 02/17/2022 05:22:36
Hello, I am new to webots and I am trying to model the yamor robot and make some changes in it. Can someone help me with this or just guide me to what resources I should check out. I will really appreciate any help and would like to connect with you.

##### DrakerDG [Moderator] 02/17/2022 05:30:12
Maybe this can help you: [https://www.cyberbotics.com/doc/guide/yamor](https://www.cyberbotics.com/doc/guide/yamor)

##### Ian Lau 02/17/2022 11:29:37
Hello, I put the robot in the example "emit and receiver" into the scene I designed, and found that the main process memory of webots keeps growing
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/943831548967399434/unknown.png)
%end



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/943831644404609054/unknown.png)
%end

##### DDaniel [Cyberbotics] 02/17/2022 11:30:26
which version of webots?

##### Ian Lau 02/17/2022 11:30:30
2021a


Because the scene coordinate system in 2022a has all changed, which will affect my scene, so I didn't update to the latest version.

##### DDaniel [Cyberbotics] 02/17/2022 11:32:05
2021b also fixed a number of memory leaks, did you try that instead?

##### Ian Lau 02/17/2022 11:32:48
ok, I'll try it right now


There is also a memory leak in 2021b, about 0.5m per second



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/943834142855032852/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/943834143039561738/unknown.png)
%end


If I send you the whole project,would it help to solve this problem? `@DDaniel`

##### DDaniel [Cyberbotics] 02/17/2022 13:11:59
Yes definitely it would be appreciated, however the fix would affect only the 2022a version, so doesn't solve your issue if you don't intend to upgrade

##### moebius 02/18/2022 00:52:55
Hi i have a program which generates the stl and wbo for a robot in webots, and i have attached the wheels properly using the hingejoint connector, but the gps and compass seem to float outside of the robot body, with surprisingly large translation values (that are not in the wbo file to begin with). How do i make sure that these components stay attached in the location that is defined for them, is there a connector node i can use ? I tried to make sure it inherits 0 0 0 translation from the parent Robot node, but it did not seem to work

##### Ian Lau 02/18/2022 01:11:28
It doesn't matter, I just report the problem, I will commit an issue on github. Thx.

##### DDaniel [Cyberbotics] 02/18/2022 08:55:08
The way you defined the robot is probably wrong. If a GPS is set as a child of the Robot node (i.e added to the `children` field) it will move together with it. Can you share the wbo that defines the robot?

##### amna 02/18/2022 09:13:46
is it possible to add moveable hands on our own designed robot?


moveable hands mean like a greeting robot which greets the person with hands


`@DDaniel` `@DrakerDG`

##### DDaniel [Cyberbotics] 02/18/2022 09:20:11
The nao robot (`nao_room.wbt`), the darwin-op (`motion.wbt`), robotis-op3 (`robotis_op3.wbt`) and maybe a couple more have a wave animation. If you mean actually shaking hands then not out of the box, no. You'd need to program it yourself.

##### amna 02/18/2022 09:22:05
ok I want to add waving hands on my own robot. Using NAO or others makes the simulation very slow. I have around 30 robots in my simulation

##### Ranga Kulathunga 02/18/2022 13:38:32
Hi, can we change the acceleration and deceleration of a Webots vehicle?

##### Rojer Mathew 02/18/2022 14:40:30
Hi has anyone done a take where the robot avoids obstacle and finds a red box and goes towards it, when it has achieved that you pick up the red box and place it on its back and it automatically returns back to its starting position


done a task*


Or something similar to it


I dont mind paying some money for the help

##### Albukerk 02/18/2022 17:19:59
Hey, when I try to add a children node of an LED, the software crashes. How can I solve that?

##### DrakerDG [Moderator] 02/18/2022 17:21:06
Hi! What version of webots you used?

##### Albukerk 02/18/2022 17:21:37
2021a

##### DrakerDG [Moderator] 02/18/2022 17:21:55
Can you try with 2022a version?

##### Albukerk 02/18/2022 17:22:50
is there a way to solve that using the same version?

##### DrakerDG [Moderator] 02/18/2022 17:26:07
OK, you can try adding the point light or spot light in other place of the webots tree, and after you can cut and paste in the children LED

##### Albukerk 02/18/2022 17:27:03
I'll try that one. Thank you

##### DrakerDG [Moderator] 02/18/2022 17:29:38
OK, tell us if it works please

##### moebius 02/18/2022 19:00:54
i did add the gps and compass as children actually.
> **Attachment**: [graph-model.wbo](https://cdn.discordapp.com/attachments/565154703139405824/944307510343524382/graph-model.wbo)

##### Robokashi 02/18/2022 19:45:15
Hi ! I am trying to run Webots on a remote computer (using Windows' Remote Desktop Connection) but Webots fails loading and crashes immediately. 

It worked once (prior adding a 2070 card to the computer). I read : [https://www.cyberbotics.com/doc/guide/general-bugs](https://www.cyberbotics.com/doc/guide/general-bugs). Do you have any tips to make it work ?

##### Chi 02/18/2022 19:47:18
Hi, I am using race\_wheel.py to control vehicle added in webots by Logitech G29. Can I also make SUMO know the controlled vehicle?

##### DDaniel [Cyberbotics] 02/18/2022 20:50:42
Did you try to give a DEF name to your vehicle in the form of : `SUMO_VEHICLE0` ?


Basically what you did was: `Robot { GPS { Solid { Shape {}}}`, however all devices, GPS included, are already solids themselves so you can put the shape directly in the children field of the GPS. The reason it falls off is because for the Robot and Solid nodes you added a Physics node, but you didn't for the GPS. In summary, to fix your issue and simplify your design you need to move the shape into the children field of the GPS and move the Physics node you defined in the Solid to the GPS itself.

##### moebius 02/18/2022 21:15:43
oh so basically have it be Robot { GPS { Shape { physics }}}. Thank you so much !! I'll try this out.I thought i would have to use a transform node to orient it properly.

##### DDaniel [Cyberbotics] 02/18/2022 21:19:43
No, more like: 

```
Robot {
  children [
    GPS {
      translation # use this to orient it
      rotation    # use this to orient it
      children [
        Shape {
          # define here the geometry/appearance of the GPS
        }
      ]
      physics Physics {
        # define here the mass of the GPS
      }
    }
  ]
  ...
}
```

##### Chi 02/18/2022 21:19:50
Thanks so much. It works after I changed the name. I still have the issues like the car controlled by the logitech G29 started to run very slowly after I push the gas pedal. After I improve to gear 2, it stops right away.

##### moebius 02/18/2022 21:20:26
oh right right, the physics in the GPS node. i'll do that. Thanks!!

##### Chi 02/18/2022 21:22:26
What I am trying to do is building up a driving simulator based on the webots and SUMO. Are there any resources about this?  Thanks!

##### maxwell 02/19/2022 03:44:40
Hi,I'm going to edit .wbt files manually(without editor),is there any API/Lib provided to modify this file?Thanks

##### Rithsagea 02/19/2022 05:51:58
is it possible to draw 3d lines in the world using physics plugins?

##### DDaniel [Cyberbotics] 02/19/2022 13:38:19
World files are defined using VRML syntax, which is quite straightforward. If you ever created a PROTO yourself, it's virtually the same thing. All you really need is the official documentation  (like this: [https://cyberbotics.com/doc/reference/camera#camera](https://cyberbotics.com/doc/reference/camera#camera)) to know what are the fields of the nodes and based on the type of field (SFBool, SFFloat, ...) , you'll need to learn what is the correct the syntax. Try opening a simple world with a text editor and it should be fairly simple to learn it.


Not with the Physics plugin directly, but you can send the necessary information from the physics plugin to a supervisor with `dWebotsSend` ([https://www.cyberbotics.com/doc/reference/utility-functions#dwebotssend](https://www.cyberbotics.com/doc/reference/utility-functions#dwebotssend)) and let the supervisor do the drawing. For instance in the sample worlds: `file > open sample worlds > supervisor_draw_trail.wbt` is shown how to draw a path

##### Albukerk 02/19/2022 18:04:17
This method worked


I added a LED near distance sensor but the reading it gives is still the same (same as without LED). Why is that?

##### maxwell 02/21/2022 07:41:16
Thanks~I still have some questions :How the world editor parse .wbt file to generate the world? Is there any lib to finish this progress?For example,turn some VRML text into a c++ object(or the reverse progress). 

Since I want to write a program to generate this file,I'm finding some ways to simplify this progress.(or I have to write the whole code by myself)

##### DDaniel [Cyberbotics] 02/21/2022 10:21:30
I don't think that's the right approach, if you want to create worlds in a programmatic way, the best approach is to use a supervisor to spawn objects into the world, using functions such as: [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_field\_import\_mf\_node\_from\_string](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_field_import_mf_node_from_string). This tutorial goes through this approach: [https://cyberbotics.com/doc/guide/tutorial-8-the-supervisor](https://cyberbotics.com/doc/guide/tutorial-8-the-supervisor)

##### DrakerDG [Moderator] 02/21/2022 14:20:57
Can you share your world to take a look it?

##### Pahanda 02/21/2022 17:02:10
I have a probably stupid question but here we go

If you had to make a game that allows the user to improve their drone flying skills would webots be suitable for that or would I be better of using a traditional gameengine (unity or the likes)?

##### DrakerDG [Moderator] 02/21/2022 18:30:53
It is possible in webots, but I think that it will be better or easier using some game engine

##### Pahanda 02/21/2022 18:48:14
ok, thank you very much for the answer

##### DrakerDG [Moderator] 02/21/2022 18:56:14
`@Pahanda` you welcome 👍

##### SeanLuTW 02/22/2022 06:56:43
Hi, is it possible to write a PROTO that inherited from another self-defined PROTO?

##### Rico Schillings[Sweaty] [Moderator] 02/22/2022 07:05:08
Hey. Yes thats possible. Most examples of webots are based on this structure. Just for demonstration, take a look at [https://github.com/cyberbotics/webots/tree/master/projects/vehicles/protos/abstract](https://github.com/cyberbotics/webots/tree/master/projects/vehicles/protos/abstract). Here you have the `ackermannvehicle.proto` as base. The `car.proto` inherits from it. And if look one directory above (vehicles/Protos) you have further examples of some car manufactories that inherits from `car.proto` (e.g. BMW/Tesla etc). Same structure you find with the provided robot Protos (e.g. Nao) which also inherits from `robot.proto`. The same procedure you can use for self-defined PROTOS

##### SeanLuTW 02/22/2022 07:14:17
Ok, thank you very much!

##### Wikum Jayasanka 02/22/2022 11:29:01
Hi, I am using a slider joint in my robot arm and I'm using position control for it to move my robot arm. However, when the robot is traveling the robot arm keeps sliding automatically even when I have set the position to a standby location. Is there a way to hold the robot arm in a fixed position when it is not being used?   

I'm using Webots 2021b for my robot simulation and I can't seem to figure out what's wrong with my implementation. Thanks in advance.

##### DrakerDG [Moderator] 02/22/2022 11:32:34
Can you share a little video of your issue please?

##### giaco\_mz 02/22/2022 12:28:36
There are samples of batteries discharge/charge problems? Robot that once discharged their batteries comes back to home to charge

##### Rico Schillings[Sweaty] [Moderator] 02/22/2022 12:48:29
May a good entry point would be [https://github.com/cyberbotics/webots/blob/master/projects/samples/devices/worlds/battery.wbt](https://github.com/cyberbotics/webots/blob/master/projects/samples/devices/worlds/battery.wbt) were you can see the usage of the charger component

##### giaco\_mz 02/22/2022 17:10:07
yes , thx


there is a way to show the node tree graphically?


now i can see only  The Scene Tree


what i mean is to see it as diagram


as shown on [https://www.cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot](https://www.cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)


it should be named node chart

##### N4RP 02/23/2022 03:44:38
Hi, I'm new to the software, and  I got a warning to set Shape.castShadows to FALSE. Can anyone send me the instruction to change the settings?

##### DrakerDG [Moderator] 02/23/2022 04:06:05
You need find the shape that have in true this property and put it in FALSE
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/945894260853583922/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/945894261134598164/unknown.png)
%end

##### N4RP 02/23/2022 04:06:35
Ok, thanks

##### DrakerDG [Moderator] 02/23/2022 04:23:21
You welcome!

##### maxwell 02/23/2022 08:54:07
thanks,I'll try it~

##### Isad5566 02/23/2022 14:55:45
Hi all, is it possible to use `Supervisor` to make a paused world (maybe triggered by UI or streaming viewer) resume playing?

##### DrakerDG [Moderator] 02/23/2022 14:56:20
Yes!

##### DDaniel [Cyberbotics] 02/23/2022 14:56:32
you can get/set the mode using `wb_supervisor_simulation_get_mode` [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_simulation\_get\_mode](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_simulation_get_mode)

##### DrakerDG [Moderator] 02/23/2022 14:58:26
Example In Python:



from controller import Supervisor

robot = Supervisor()



\# Stop simulation

robot.simulationSetMode(0)

##### Isad5566 02/23/2022 15:11:22
Thanks replies from`@DDaniel` and `@DrakerDG`. I created a simple world with one robot with `supervisor` set to true and wrote a simple controller.

```
from controller import Supervisor

s = Supervisor()

while s.step(16) != -1:
    if s.simulationGetMode() == Supervisor.SIMULATION_MODE_PAUSE:
        s.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
        s.step(0)
```

I clicked pause button in UI (Ctrl+0) and found that the controller does not take effect 😦

##### DrakerDG [Moderator] 02/23/2022 18:40:44
Some things must be have bounding object and physics: the robot node and every solid of endpoint hinge join.



You can share your world to take a look it.

##### DDaniel [Cyberbotics] 02/25/2022 07:44:17
Your loop doesn't make much sense. If you pause the simulation, the controllers are also paused so the body of your while loop doesn't have the chance of executing. I don't really understand why you need the controller to do this specifically, the use-case of this function usually is the opposite, meaning when the "job" is done, you pause the simulation using the controller so it doesn't go on endlessly (or to switch between real-time and fast mode)

##### kimmcg 02/25/2022 09:36:40
Hi! I'm new here but I've been trying out webots for last few weeks now I am  currently in the process of making a Crazyflie model (small quadcopter).


I noticed that the meshes shape, when I want to use a .dae model, the visual aspects are not being imported with it. Also the orientation seems to be off then compared to an stl file.


Currently I build up the robot in parts in webots, and give each of them materials, but it is a bit tedious and hard to maintain, especially as a big part of the model is just visual with non moving parts.


But I would like to import the fiull body in one go as a dae file, with webots importing the material aspects as well. Would that be possible?

##### DDaniel [Cyberbotics] 02/25/2022 09:41:32
Have you tried using the `ColladaShape` PROTO instead?

##### kimmcg 02/25/2022 09:43:55
Yes I've tried it, but it adds weird rotations to the parts of the model.

##### DDaniel [Cyberbotics] 02/25/2022 09:44:03
Concerning the wrong orientation, I believe the issue was fixed and you might just need to install a nightly build from here: [https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)

##### kimmcg 02/25/2022 09:45:24
Ahh oke! maybe that is the reason why I saw those rotations  on each element in the colladashape proto as well?


I will indeed try out the nightly build, thanks!

##### Benjamin Hug [Moderator] 02/25/2022 09:56:55
The fix is supposed to resolve all those rotations on each element in the ColladaShapes PROTO and in the Mesh node. Moreover the support of the different materials have also been improved for the ColladaShapes PROTO.

##### kimmcg 02/25/2022 10:01:32
Yes I have just tried out the nightly build and that is fixed. Thanks! The only thing a bit off is that the origin is not the same as my blender file, but that is perhaps an export issue on their part, and I can fix that with a solid node first or an transform


cool, thanks for you help! I will try to finalize the model in the next few weeks but then soon Ill be able to contribute it to the webots github

##### DDaniel [Cyberbotics] 02/25/2022 10:15:58
Awesome! These are the guidelines to follow when proposing the addition of a new robot: [https://github.com/cyberbotics/webots/wiki/Adding-a-New-Robot](https://github.com/cyberbotics/webots/wiki/Adding-a-New-Robot)

##### kimmcg 02/25/2022 10:16:21
ah very handy, thanks again!


it might seem that I'll need to wait a bit for a fix here for colladashapes  [https://github.com/cyberbotics/webots/issues/4195](https://github.com/cyberbotics/webots/issues/4195) I also noticed that relative URLs doesnt work, while it does for the meshes node

##### DDaniel [Cyberbotics] 02/25/2022 10:28:34
indeed, I'll get to it as soon as I can, I need it as well

##### Kormit 02/25/2022 14:06:35
Hi, I was wondering if there is a way of programmatically, with a Supervisor robot, limiting the max velocity of a robots wheel motors. The issue i current have with setting the max velocity of the robot is that if the robot controller (not supervisor) sets the motor velocities once at the start of the program, the new max velocities don’t seem to come into effect until the robot controller tries to change them again. Is there any solution for this problem? Thanks.

##### DDaniel [Cyberbotics] 02/25/2022 14:09:43
Which type of control are you doing (position, velocity or torque)? Because some of the motor API functions have different effects depending on the type of control you do (specified here: [https://cyberbotics.com/doc/reference/motor#motor-control-summary](https://cyberbotics.com/doc/reference/motor#motor-control-summary)). But yes in principle you can use a supervisor to retrieve a reference to the `maxVelocity` field of the motor and set whatever value you wish to

##### Kormit 02/25/2022 14:15:54
In general, a robot controller sets its position to infinity and velocity to max at the very start of its controller. If the supervisor sets the robots max velocity to a new value, the new value only comes into effect if the robot controller resets its velocity values again. Is there a way to make this update happen regardless of what the robot controller does?

##### DDaniel [Cyberbotics] 02/25/2022 14:20:07
If you set the position to infinity it means you're controlling the motor in velocity and as the table shows, in velocity control calling `wb_motor_set_velocity` it specifies the desired velocity, not the maximal. The maximal velocity in this case is specified by changing the `maxVelocity` parameter of the motor (which you can do using a supervisor too)

##### Kormit 02/25/2022 14:28:39
From my tests, if you set a velocity higher than the new max before setting the new max by a supervisor, the robot doesn’t seem to slow down.


Could I be setting the max velocity via the supervisor incorrectly?

##### DDaniel [Cyberbotics] 02/25/2022 14:30:24
In theory it should show a warning message if you're requesting the motor to go faster than its max. Can you provide the code? It might be easier to understand the issue

##### Kormit 02/25/2022 14:34:33
It does show the warnings if I am setting the velocities for example every time in a while loop, however when set just the once at the start of a program it doesn’t show them, which I thought was odd. Sorry, at this moment I can’t easily give the code.

##### Crazy Ginger 02/25/2022 16:26:51
`@Darko Lukić` hi in [https://github.com/cyberbotics/epuck\_ros2/blob/master/installation/README.md](https://github.com/cyberbotics/epuck_ros2/blob/master/installation/README.md) you say the recommended way to install ROS2 is with the pi-puck image but I couldn't find a link to it anywhere on the github or webots page

##### Darko Lukić [Moderator] 02/25/2022 16:44:21
Unfortunately, we didn't upload the image. Instead, I suggest to use the Docker method.

##### zhexu 02/26/2022 13:43:03
Hello. When I was learning the speaker and running the "speaker" sample, no sound came out. What I should do now?

##### DrakerDG [Moderator] 02/26/2022 14:21:55
Try close and open webots again

##### zhexu 02/26/2022 14:37:34
I tried and still no sound. Is there a sound switch in Webots?

##### DDaniel [Cyberbotics] 02/26/2022 14:38:23
By default it might be muted, did you unmute/increase the volume?

##### DrakerDG [Moderator] 02/26/2022 14:45:30

%figure
![2022-02-24.png](https://cdn.discordapp.com/attachments/565154703139405824/947142334179594300/2022-02-24.png)
%end

##### zhexu 02/26/2022 14:48:14
Thank you, I unmutated and reload the world, it solved.

##### DrakerDG [Moderator] 02/26/2022 15:04:27
Good!

##### giaco\_mz 02/27/2022 12:17:53
Hi i have this problem...



Example vision.wbt



Error:

vision.cpp:29:10: fatal error: opencv2/core/core.hpp: No such file or directory

   29 | #include <opencv2/core/core.hpp>



OPENCV\_DIR   C:\opencv\x64\vc16



The library location is correct because i have 

C:\opencv\include\opencv2\core\core.hpp  

file located there. So i am missing what i am doing wrong..

##### DrakerDG [Moderator] 02/27/2022 16:40:23
Maybe this document can help you:  [https://cyberbotics.com/doc/guide/using-webots-makefiles?tab-language=ros#adding-an-external-library-ccp](https://cyberbotics.com/doc/guide/using-webots-makefiles?tab-language=ros#adding-an-external-library-ccp)


And this one: [https://github.com/cyberbotics/webots/tree/f6886726c320b86d8ab07cab4d849ca82ca791af/projects/samples/howto/vision/controllers/vision](https://github.com/cyberbotics/webots/tree/f6886726c320b86d8ab07cab4d849ca82ca791af/projects/samples/howto/vision/controllers/vision)

##### Tom\_Wolf 02/28/2022 14:19:14
Hello everyone ! I have a weird error that I can't fix, could I get some help? 😀 



[driver-2] /opt/ros/galactic/lib/webots\_ros2\_driver/driver: symbol lookup error: /opt/ros/galactic/lib/webots\_ros2\_driver/driver: undefined symbol: \_ZN22rosidl\_typesupport\_cpp31get\_message\_type\_support\_handleIN16webots\_ros2\_msgs3msg27WbCameraRecognitionObjects\_ISaIvEEEEEPK29rosidl\_message\_type\_support\_tv

##### Benjamin Hug [Moderator] 02/28/2022 14:41:26
Hi, how did you installed `Webots` and `webots_ros2`?

##### Tom\_Wolf 02/28/2022 14:51:49
Hi i follow those two links


[https://www.cyberbotics.com/doc/guide/tutorial-7-using-ros?version=cyberbotics:R2019a](https://www.cyberbotics.com/doc/guide/tutorial-7-using-ros?version=cyberbotics:R2019a)


[https://cyberbotics.com/doc/guide/installation-procedure](https://cyberbotics.com/doc/guide/installation-procedure)

##### DDaniel [Cyberbotics] 02/28/2022 14:53:26
It seems you're looking at an old page, the correct one should be [https://www.cyberbotics.com/doc/guide/tutorial-9-using-ros](https://www.cyberbotics.com/doc/guide/tutorial-9-using-ros)


I assume things changed since 2019

##### Benjamin Hug [Moderator] 02/28/2022 14:54:29
The error you stated here is for `webots_ros2` but the link you gave is for ROS 1.What version of ROS are you using?

##### Tom\_Wolf 02/28/2022 14:54:51
ROS 2 galactic

##### Benjamin Hug [Moderator] 02/28/2022 14:55:25
Ok and you installed Webots with debian package ?

##### Tom\_Wolf 02/28/2022 14:55:41
Yes !

##### Benjamin Hug [Moderator] 02/28/2022 14:59:56
Could you run `apt-cache policy ros-galactic-webots-ros2` and copy past the output here please?


I guess you also installed `webots_ros2` with `apt install ros-galactic-webots-ros2`?

##### Tom\_Wolf 02/28/2022 15:45:59
I solved the problem, it was an installation error thanks !

##### Robokashi 02/28/2022 15:59:27
Hi ! I added a camera to my Webots simulation. Now I would like to send the camera data through a ROS2 topic, but I see that the Devices/Sensor/Camera page on Github is under "References" which is labeled deprecated.

Is there any up to date documentation on how to get Webots' camera data to ROS2 ? Thanks !

##### Rico Schillings[Sweaty] [Moderator] 02/28/2022 16:02:46
Enable the sensor by its name in the controller/ros node and use e.g cvbridge (depending on programming language). How to use the camera you can find in the documentation (getImage())

##### Robokashi 02/28/2022 16:07:08
This is what I had in mind, thank you for the help, I appreciate 🙂

##### Rico Schillings[Sweaty] [Moderator] 02/28/2022 16:10:28
From my own experiences i can tell that the controller node should be implemented in cpp to ensure you get the fastest runtime. I had implemented my controller node in a first trial in python and had a decreasing simulation time. After porting it to cpp now its running close to realtime

##### Robokashi 02/28/2022 16:11:34
Followup on this error.



 [https://docs.microsoft.com/en-us/answers/questions/185287/remotedesktop-loses-opengl-after-you-reconnect-you.html](https://docs.microsoft.com/en-us/answers/questions/185287/remotedesktop-loses-opengl-after-you-reconnect-you.html)



The accepted answer fixed my issue. I think this solution could be added to the "general-bugs" documentation page.


Thank you for the feedback. I am using C++17 so I should be good 🙂

##### Rico Schillings[Sweaty] [Moderator] 02/28/2022 16:16:36
This reminds me on a similar "Bug" i had last week. I wanted to share my results in webots during a zoom session (videocall), but wasnt able to start it. After zoom was finished, everything worked. Seems that zoom also uses opencv for e.g. background blur and seems to "block" the opencv lib, cause i had some opencv errors in my controller node during zoom..

##### Robokashi 02/28/2022 16:19:22
I can imagine how frustrated you were xD

##### Rico Schillings[Sweaty] [Moderator] 02/28/2022 16:21:14
I felt more awkward instead of frustrated, but for the future i know to record the demonstrations before zoom and just play the video... 😅

##### josh101 02/28/2022 17:17:43
Does anyone know how to get the centre of mass of a robot? I'm using the Nao robot and just wish to find its centre of mass for varying positions but I can't figure it out. Any help is appreciated

##### Baya19 02/28/2022 17:20:35
Right click on the robot and then follow the picture
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/947906142829416498/unknown.png)
%end

##### josh101 02/28/2022 17:40:16
Thank you for the help, is there a way I can print out this centre of mass, so I can get the coordinates of it, or get the CoM of individual limbs. I'm not too familiar with webots and am mainly using it to get the CoM of different posutions, to save me doing the calculations on teh real thing.

##### DDaniel [Cyberbotics] 02/28/2022 17:45:31
You can use a supervisor: [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_center\_of\_mass](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_center_of_mass)


And here's how to use a supervisor: [https://www.cyberbotics.com/doc/guide/supervisor-programming](https://www.cyberbotics.com/doc/guide/supervisor-programming)

##### josh101 02/28/2022 18:07:41
Thank you for the help it's very much appreciated. I tried running the  example in the webots:supervisor, but it's telling me that Def My\_robot isn't in my current world. I presume ive done something stupid but when i google the error it just brings me back to the same page. So im not entirely sure what its telling me is wrong

##### Rico Schillings[Sweaty] [Moderator] 02/28/2022 18:10:16
In your world, you have to define your robot correctly. E.g. In your world file (wbt) you should have declared the robot with DEF My\_robot Nao {...}

##### josh101 02/28/2022 18:20:31
Currently, my world file looks like 





WorldInfo {

}

Viewpoint {

  orientation -0.09459468826266149 0.02820628412754632 0.9951161994903942 2.564725061828473

  position 4.197923228745181 -2.8929555372994225 1.4389110953651982

}

TexturedBackground {

}

TexturedBackgroundLight {

}

RectangleArena {

  rotation 0 1 0 0

  floorSize 5 5

  wallThickness 0.1

}



Nao {

  controller "my\_controller\_pyth"

}



I haven't touched it directly I've only added to it via the add a node button.  Would I put the DEF My\_Robot{Nao} or is it something more complicated.


And thank you for the help

##### Rico Schillings[Sweaty] [Moderator] 02/28/2022 18:23:09
The last entry of your world Nao {...} just needs the prefix as i mentioned, so add "DEF My\_robot" before your Nao entry and it should work

##### josh101 02/28/2022 18:49:39
josh101 — Today at 18:47

Thanks for the help its very much appreciated. My only final question is that I have created a class called Nao which is a subclass of robot, and in such class, I have functions in which I can set the position of all its joints and what not. The only issue now is when i try and combine this im told I can only have one instance of Robot class. 



so what I'm asking is I'm  unsure how I would take 



supervisor = Supervisor()



robot\_node = supervisor.getFromDef("MY\_ROBOT")



CoM = robot\_node.getCenterOfMass()



 print(CoM)



And use it with a Nao class which is a subclass of Robot, once again thank you for the help theres no way i wouldve figured this out on my own

##### Rico Schillings[Sweaty] [Moderator] 02/28/2022 19:50:13
You could try to not really built your Nao class as a inherited one of robot class, since you get all the functionality by the Supervisor object of your code snippet. So just implement a class (nao) and let this Just have a reference/attribute to the supervisor as you already mentioned. In your functions to control the joints eg, you can use either the supervisor or the robot\_node. if you set your robot to Supervisor = True in the world, you have a combined access to All functions of robot + supervisor.



At the moment with your world as pasted above, the supervisor call wont work since there is no object which can be accessed as supervisor (this could be your robot as I mentioned above or a extra dummy robot)

##### josh101 02/28/2022 20:02:25
But if i were to have it so my Nao class doesn't inherit from Robot, currently i move a limb by initiating  using 



self.limb = self.getDevice("limb") 



and then  later moving said limb using 



self.limb.setPosition(angle)





Im not really sure how id do it without inheriting the robot in this case though


And obviously i appreciate the help

##### DDaniel [Cyberbotics] 02/28/2022 20:04:13
A Supervisor is a Robot with extra powers, instead of subclassing robot, you can subclass Supervisor.

##### josh101 02/28/2022 20:58:22
Thank you for all the Help 🙂


My only final concern is, is there a way to know what my coordinates are in reference to? Or can I set a reference point? At the moment I don't have a ground as I want him to set positions without kicking it and flying off, but then currently its really not clear as to what my my coordinates thus mean. Any help is very much appreciated

##### Simon Steinmann [Moderator] 02/28/2022 21:12:38
`@josh101` coordinates you get from the supervisor are generally in reference to the world. I have not followed your whole thread so I dont know the specifics of the question.


what are you trying to achieve?

##### josh101 02/28/2022 21:15:57
I just want to look at how the CoM of my robot changes when I set it to different positions, and I can now read the CoM of my robot. But my coordinates don’t mean much as I can’t figure how what they’re in reference to. When you say they’re in reference to the world, is there a way to get this axis so I can see where it’s coming from. If that makes sense

##### DrakerDG [Moderator] 02/28/2022 21:45:38
The reference is the center of the world x, y, z (0, 0, 0)

## March

##### Baya19 03/01/2022 06:18:21
Hi everyone, did anyone deal with drones before in webots ? and know how to maintain a drone at a fixed high altitude ?

##### Simon Steinmann [Moderator] 03/01/2022 06:20:27
Implement some sort of PID controller. Add accelerometer and GPS sensor to get error values and control the propeller speed based on those

##### Yannnick3 [Cyberbotics] 03/01/2022 09:32:35
You can take a look at the mavic\_2\_pro example. The world and associated controller can be found in  `webots/projects/robots/dji/mavic`.

##### Baya19 03/01/2022 10:43:16
I've tried that. it doesn't work, i think the problem is in the coordinate system. I suffer to find the right PID params for ENU coordinate system

##### Simon Steinmann [Moderator] 03/01/2022 10:45:26
Feel free to post your code for the Implementation. Maybe I can spot some obvious issues

##### Baya19 03/01/2022 12:14:08
thank you


the drone flies but it can't be fiwed at a fixed altitude



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/948191495788314654/unknown.png)
%end

##### Rico Schillings[Sweaty] [Moderator] 03/01/2022 16:33:07
I have the following problem i cant explain.. I'm using the latest docker image, start it with the provided command and add `--stream` since i want to use the Web streaming interface. With this it starts an empty world and i can connect to the streaming server from outside the container. When i mount the local folder with my world/controller files, i can start the simulation inside the container without the streaming server (needs nearly 5secs to load and start the world). But when i add the `--stream ` flag with my world, it hangs and the output of initializing the webserver (showing port) is not coming. 🤷‍♂️ so using container without stream works, but with it wont. Streaming works without loading my world. Its pretty confusing.. Any ideas/hints to check?

##### Simon Steinmann [Moderator] 03/01/2022 22:14:48
hmm perhaps you can share your whole project. It is hard to tell from the code alone. But from what I can see, you only implemented a P controller, but you control it proportional to the error ^3


at least do a PD controller (you can probably leave the Integral error out)


for the D error you can just do `error - error_previous ` .  To get a true derivative, you would have to divide by your timestep, but that is a constant and can be skipped, assuming you never change your timestep. If you might change your timestep in the future, I would advice dividing by it.


`vertical_input = kp * P_error + kd * D_error`


And leave the P\_error linear, dont do a power


and instead of clipping an error, I would rather clip the final input signal to a stable range

##### Ranga Kulathunga 03/02/2022 05:13:20
Hi all! How can see a Webots vehicle in SUMO simulation? I tried as mentioned in the documentation, to change the DEF name to WEBOTS\_VEHICLE0. But it is not worked. Do you guys have any idea about this?

##### joachim honegger [Moderator] 03/02/2022 07:25:56
Hi! You can check one of the example, sumo\_interface\_example, highway\_overtake or highway

##### Baya19 03/02/2022 10:51:54
Thank you so much


Did anyone visualize point cloud in python from lidar sensors ?

##### Missaw 03/02/2022 14:11:43
Hello everyone, I will have regarding access to my controller. I am currently working on macos. But the error tells me that it can't load the "controller" module but I don't understand, why? Thank you in advance for your answers.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/948583387742998538/unknown.png)
%end

##### Baya19 03/02/2022 14:13:38
try to install numpy

##### Missaw 03/02/2022 14:14:53
Yeah that's what I tried to do several times but basically they tell me it's already installed on my machine



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/948584716515307601/unknown.png)
%end


in fact I have a problem with the reading of the controller

##### zev 03/02/2022 14:22:19
This is because python 3.10 is not supported by default, you can add the webots modules python 3.9 in your PYTHONPATH environment variable. Or you can just use python 3.9 instead

##### Missaw 03/02/2022 14:24:21
okey , i will try to test this, thx 🙂


Unfortunately, I still have the same problem 😦



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/948593983943237672/unknown.png)
%end



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/948594135768657960/unknown.png)
%end

##### Yannnick3 [Cyberbotics] 03/02/2022 15:07:57
It seems like the version of python in which numpy is installed is not the same as the one used by Webots. Try to install numpy with the following command `python3.9 -m pip install --upgrade numpy`

##### Robokashi 03/02/2022 16:54:41
Hi, is there an alternative way to specify which robot a controller should target without using the environment variables ([https://cyberbotics.com/doc/guide/running-extern-robot-controllers#single-simulation-and-multiple-extern-robot-controllers](https://cyberbotics.com/doc/guide/running-extern-robot-controllers#single-simulation-and-multiple-extern-robot-controllers)) ?



I would like to be able to specify this directly in the controller. I tried using \_putenv right after the main() but it is ignored.



I am using Windows 10. Thanks !


I would suggest you avoid using pip install xxx syntax. You can't easily be sure which version of python is going to be used.



Try python --version to check if the python version known to your path is the one you want to use, and if it is, run "python -m pip install numpy".


I can see when you ran "pip install numpy" there was a mention of 3.8, whereas you use 3.9 so this might be your problem.

##### goksun 03/02/2022 17:34:11
Hello everyone! i have a problem with webots whenever i try to run a java code. I tried changing the path to jdk, i updated java to the latest version but it still keeps appearing and idk how to solve it. Can somebody please help?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/948634340550668308/unknown.png)
%end


and i also have this problem that whenever i try to open a project other than mine, every object changes its place and whole world gets pretty complicated

##### amna 03/03/2022 01:31:06
How can we keep the drones inside a boundary like if I want drone to do not cross the wall then how can I do that? Please help! `@DrakerDG`

##### Mat198 03/03/2022 02:47:59
You can create walls with solid > collision > shape > box to create an invisible room. You also need to turn on the drone collision. If you don't know how to do that just send me a private message :)

##### Missaw 03/03/2022 08:29:21
I have retry this command , but i have others error ....



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/948859623358742548/unknown.png)
%end

##### KENPACHI 03/03/2022 09:01:47
Hey, I'm working on a snake robot and I'm trying to emit signals from my head module to the rest of the body. but I'm running into this error, which says 

"Error: wb\_emitter\_send(): invalid argument: data = NULL.

TypeError: expected bytes, float found"



code snippet:

while robot.step(TIME\_STEP) != -1:

      for i in range(1,n) : 

        Ac = 1

        lat\_angle = Ac*math.sin(omg*i+w*t)

        dor\_angle = e*Ac*math.sin(omg*i+w*t+off)

        emitter.setChannel(i)

        if i%2 !=0:

          emitter.send(lat\_angle)          

        else:

          emitter.send(dor\_angle)



***********NEED HELP *************

##### Simon Steinmann [Moderator] 03/03/2022 09:03:09
`@KENPACHI` take a look at this sample world and controller. It should show you how to implement it
> **Attachment**: [emitter\_receiver\_test.rar](https://cdn.discordapp.com/attachments/565154703139405824/948868121391923260/emitter_receiver_test.rar)


line 2 and 27 are the important part


that you are missing

##### amna 03/03/2022 09:05:00
How to turn on drone collision?

##### KENPACHI 03/03/2022 09:05:27
Got it! trying it right now!


Working! thanks a lot.

##### Simon Steinmann [Moderator] 03/03/2022 09:18:13
You are welcome!

##### Mat198 03/03/2022 09:50:39
Which drone are you using?

##### amna 03/03/2022 09:51:03
mavic from dji

##### Yannnick3 [Cyberbotics] 03/03/2022 10:19:20
I saw many people having the same issue as you on M1 macs. Is it your case? What I would suggest is to try to use Python 3.8 in Webots and see if the problem persists.

##### Missaw 03/03/2022 10:20:29
yes I have mac M1 😅


I'm going to try version 3.8 now


thx

##### Yannnick3 [Cyberbotics] 03/03/2022 10:22:27
You could try to follow this SO thread for example to fix the issue : [https://stackoverflow.com/questions/65336789/numpy-build-fail-in-m1-big-sur-11-1](https://stackoverflow.com/questions/65336789/numpy-build-fail-in-m1-big-sur-11-1)

##### Missaw 03/03/2022 10:33:00
always the same problems with the version 3.8 ....



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/948890743643209748/unknown.png)
%end

##### tokia 03/03/2022 17:38:50
Is there a 2 finger gripper available in webots compatible for UR5

##### DrakerDG [Moderator] 03/03/2022 19:17:33
I think not, I remember one PROTO with 3 fingers, but it is possible to adapt one or make a new one


Some options to adapt
%figure
![coupled_motors.png](https://cdn.discordapp.com/attachments/565154703139405824/949026719115927552/coupled_motors.png)
%end

##### Mat198 03/03/2022 23:26:58
You are running on windows? If so open the cmd and try -> python -m pip install numpy

##### Simon Steinmann [Moderator] 03/04/2022 00:26:00
[https://github.com/cyberbotics/community-projects](https://github.com/cyberbotics/community-projects) there is the robotiq 2 finger gripper included here

##### tokia 03/04/2022 00:26:32
yessssssssssssssssssss thank you i didnt know this existed

##### Simon Steinmann [Moderator] 03/04/2022 00:27:16
there is also a bunch of robot arms I added, such as the majority of kuka robots

##### tokia 03/04/2022 00:37:34
I modelled and made bounding box groups for all the robocup@work manipulation objects into webots, so I could add them to that once my project is working

##### Simon Steinmann [Moderator] 03/04/2022 00:38:14
that would be great 🙂

##### amna 03/04/2022 10:54:17
How to reduce the voice of the robots? `@DrakerDG`

##### Missaw 03/04/2022 10:57:10
no , I'm into macos 😅

##### giaco\_mz 03/05/2022 06:52:45
How can I save my robotic system with its controllers without the world to load it in other simulation/world?

##### Mat198 03/05/2022 16:11:08
Just save it as a protonode with the .wbo extension. You can just copy and paste the controller in the correct folder or create another controller and paste the code.

##### Ranga Kulathunga 03/06/2022 11:23:29
Thank you very much for your response. I have worked on those but I was unable to see that the vehicle controlled by Webots is present in SUMO GUI interface. Do you have any idea or method to add a Webots vehicle into the SUMO simulation environment?

##### pipppoo 03/06/2022 17:32:37
Hi, I'm trying to follow the first tutorial on the webots website. However, when creating the first controller (python) I get this error:



INFO: epuck\_go\_forward: Starting controller: python3 -u epuck\_go\_forward.py

Traceback (most recent call last):

  File "epuck\_go\_forward.py", line 1, in <module>

    from controller import Robot, Motor

  File "/usr/local/webots/lib/controller/python38/controller.py", line 31, in <module>

    import \_controller

ImportError: /usr/local/webots/lib/controller/python38/\_controller.so: undefined symbol: \_ZN6webots5Robot19internalGetInstanceEv

WARNING: 'epuck\_go\_forward' controller exited with status: 1.



Can somebody help me please?


Update: I found the solution:



export LD\_LIBRARY\_PATH=/usr/local/webots/lib/controller:$LD\_LIBRARY\_PATH

##### chrikatz 03/07/2022 11:19:48
Hi! I am currently working on W2022a and the webots\_ros2\_driver. Similiarly to the tesla\_driver and the mavik\_driver I built up a simulation. Everything is working fine, except that I am not able to evaluate supervisor functions as getPosition() etc although my vehicle has it's supervisor field set to active. Can somebody help me with my problem? Or is there any example available in the webots\_ros2 package which I can reuse to get a supevisory plugin? Thanks in advance!

##### Mat198 03/07/2022 12:37:30
Did you set like thing = supervisor.getFromDef("thing")



Then



Pos =thing.getPosition()?

##### chrikatz 03/07/2022 12:48:02
I am not able to run "from controller import Supervisor" in my driver.py and therefore not able to run the code you recommended. The error is the following:ImportError: /usr/local/webots/lib/controller/python38/\_controller.so: undefined symbol: \_ZNK6webots5Robot7getTypeEv

##### DDaniel [Cyberbotics] 03/07/2022 12:53:40
are you sure your WEBOTS\_HOME environmental variable points to the correct webots folder (did you install multiple versions of webots)?

##### Benjamin Hug [Moderator] 03/07/2022 12:55:38
This has been fixed after the release of Webots R2022a, therfore I invite you to download one of the nightly build ([https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)).

##### moebius 03/07/2022 23:40:42
Hi, when i am using the webots recording feature while running the simulations in headless mode with xvfb-run , the recordings are all blank, is there a way around it? Also when i record it in gui mode, the quality is pretty bad ( the quality parameter cannot seem to be set above 60, it doesn't record otherwise and this is something i saw mentioned in an issue on github as well)

##### Simon Steinmann [Moderator] 03/08/2022 01:19:34
Well it needs to render in order to record a video. So headless will not work. As for the quality, make sure the resolution is high enough. You might also have turned down a lot of the "nice" rendering options, or it is not supported (I am assuming you are using docker containers without gpu acceleration?)

##### moebius 03/08/2022 06:09:40
yes currently running it headless on docker containers without gpu acceleration but will run it on gpus soon.  Is there  any workaround to record it in headless mode, in situations like this?

##### Simon Steinmann [Moderator] 03/08/2022 06:52:09
software render on cpu is crappy always. You really need that gpu acceleration for anything visual. That includes visual sensors as well (lidar, camera etc.)


what do you need to record?


or why?

##### moebius 03/08/2022 07:31:54
so we want to run batches of simulations, and since those will be running on docker containers, we want the ability to record them as well

##### Simon Steinmann [Moderator] 03/08/2022 07:35:13
add cameras in the simulation


that way you can explicitly controll all aspects of it

##### moebius 03/08/2022 17:12:03
okay so if i add cameras, i can record videos in headless mode as well??

##### Simon Steinmann [Moderator] 03/08/2022 18:06:46
Yeah, but you need to do it through the controller. But perhaps wait for an <@&568329906048598039> dev to answer. There might be other solutions

##### Robokashi 03/08/2022 19:49:23
Hi ! I am trying to build MoveIt2 on Windows using the recently provided information (deep thanks to whoever wrote/made that available !).

I am running into an issue when building, as cmake complains I don't have pkgconfig installed. Is it supposed to come with the ROS2 install ?

##### Simon Steinmann [Moderator] 03/08/2022 20:34:34
You have to install all dependencies


I think I ran into the exact issue as well. Just Google the specific error / missing package

##### moebius 03/08/2022 20:44:22
I want the wheels to move with the front half of the car, as i set the rotation of the servo in the middle part. This is the current .wbo. Should i change the bounding box, for the endpoint to also include the names of the solids ? or do they need to be children nodes, and if so, should the HingeJoint and Tire solids be under a subassembly?? (i don't know how to do that)

```
DEF pivotcar Robot {
                children [
                    DEF hinge0_servo_servo_h HingeJoint {
                                jointParameters HingeJointParameters {
                                                position 0
                                                anchor 0.    0.017 0.021
                                                axis -7.96e-04 -1.00e+00  0.00e+00
                                            }
                                device [
                                        RotationalMotor {  
                                        name "hinge0_servo_servo_h"
                                        controlPID 0.01 0 0
                                        maxVelocity 25
                                        }
                                    ]
                                endPoint DEF car1_sheathsplit1_split Solid {
                        children [ 
                            DEF car1_sheathsplit1_split Shape 

                    {   appearance PBRAppearance { baseColor 0 0 0 roughness 1.0 metalness 0.0 name "generic_material"}
                        geometry IndexedFaceSet { the geometry}
                            ] 
                        contactMaterial "default"  
                        boundingObject USE car1_sheathsplit1_split 
                        physics Physics {
                        density 1 
                        mass 0.3}

                        }
```
%figure
![Screenshot_from_2022-03-06_19-50-42.png](https://cdn.discordapp.com/attachments/565154703139405824/950856526376087593/Screenshot_from_2022-03-06_19-50-42.png)
%end
%figure
![Screenshot_from_2022-03-06_19-50-36.png](https://cdn.discordapp.com/attachments/565154703139405824/950856526589988934/Screenshot_from_2022-03-06_19-50-36.png)
%end


will do, thank you!

##### Olivier Michel [Cyberbotics] 03/09/2022 07:12:30
Instead of recording movies, I would recommend you to record 3D animations: it works in headless mode and provides a better way to examine the result of a simulation (as you can zoom and change the viewpoint).

##### moebius 03/09/2022 07:37:28
Oh alright, I'll look into this thank you!

##### Baya19 03/09/2022 10:42:08
Hi guys, what software did u use to see point cloud from a lidar sensor ? ROS,cloudcomapre ... ?

##### Benjamin Hug [Moderator] 03/09/2022 10:56:04
You can see them in Webots by hitting `Ctrl+F8` or by going in `View`>`Optional Rendering`>`Show Lidar Point Cloud`.

##### Baya19 03/09/2022 11:48:59
I need the pictures of point cloud

##### Ranga Kulathunga 03/09/2022 11:51:13
Hi all! I have worked on sumo\_interface, highway, highway\_overtake examples that are connected to SUMO  but I was unable to see that the vehicle controlled by Webots is present in SUMO GUI interface. Do you have any idea or method to add a Webots vehicle into the SUMO simulation environment?

##### moebius 03/09/2022 18:49:15
how do i do this via the python API? I can't seem to find this in the documentation


ok i found it, but the output is really bad, i can't see anything
%figure
![Screenshot_from_2022-03-09_11-52-28.png](https://cdn.discordapp.com/attachments/565154703139405824/951206012642267166/Screenshot_from_2022-03-09_11-52-28.png)
%end

##### BluAs\_a\_Berry 03/10/2022 00:23:17
Hello, I have a pretty simple question with the installation; whenever I try to open the Webots app, it just crashes, without explanation. No error sign or anything, just crashes. Does anybody know how to fix this?

##### Mat198 03/10/2022 00:27:00
Do you have minimum requirements? Witch SO are you using?

##### BluAs\_a\_Berry 03/10/2022 00:41:23

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/951278560876822558/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/951278561149485096/unknown.png)
%end

##### Mat198 03/10/2022 00:43:54
Do you have a graphic card? I use webots on Win10 and it's fine


Type dxdiag in the windows menu to see

##### BluAs\_a\_Berry 03/10/2022 00:50:41
I'm using Windows 10 and my graphics card is NVIDIA Quadro K1200

##### Mat198 03/10/2022 01:01:39
Sure. Hardware definitely isn't the problem


Did you try to reinstall and reset your computer?


Are you installing the last version?

##### Robokashi 03/10/2022 04:02:46
Are you accessing the computer remotely?

##### giaco\_mz 03/10/2022 09:08:28
there is a way to see the variable inside the simualtion in a different window?or the only way is to print it on console?


*variables


there will be possible otherwise to link,in some way, Spyder or other IDE to Webots?. So if Webots doesn't have this possibility hard-code implemented there is some work-around to do to overcome this problem?

##### Rico Schillings[Sweaty] [Moderator] 03/10/2022 20:26:10
May the robot Window is something you are looking for. Rightclick on the robot in left World view, Show robot Window. Here you can find several default informations but you can also add Individual coded views and infos to it

##### Naxi 03/10/2022 21:01:08
Hi! I'm trying to spawn a node with a supervisor controller, using a .wbo file ,both are in the same folder. But when I run the controller, it only looks for the node file within the WEBOTS\_HOME path, and doesn't take into account the relative path to the controller. I'm working on a catkin workspace on an external C++ controller, and so far I've been able to access other nodes in the scene tree without issues

##### Simon Steinmann [Moderator] 03/10/2022 21:01:58
you have to get a handle to the current working directory

##### Naxi 03/10/2022 21:17:39
That did it, thanks!

##### KENPACHI 03/10/2022 22:33:17
I'm trying to track the center of mass of my bot, what would be the best way to do that?

##### DDaniel [Cyberbotics] 03/10/2022 22:36:02
You can use a supervisor to retrieve that information using [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_center\_of\_mass](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_center_of_mass)

##### KENPACHI 03/10/2022 22:42:33
I'm using webots just for a week now, I'm not sure how I use that code to track my bot.

##### Simon Steinmann [Moderator] 03/10/2022 22:44:04
what language are you using?

##### KENPACHI 03/10/2022 22:44:11
python

##### Simon Steinmann [Moderator] 03/10/2022 22:44:42
okay, so in webots make sure you set ïs SUpervisor" to true

##### KENPACHI 03/10/2022 22:46:14
did that

##### Simon Steinmann [Moderator] 03/10/2022 22:46:20
then in your controller, instead of 

`from controller import Robot`

you do

`from controller import Supervisor`

and initialize it as `robot = Supervisor()`


all robot commands work the same, but you have access to all supervisor functions on top of that


Now you need a handle to the node you want. If you want a handle to the robot itself you can do:

`robot_node = robot.getSelf()`


then to get the center of mass you do

`center_of_mass = robot_node.getCenterOfMass()`


this should be a vector [x, y, z] of the center of mass in world coordinates

##### KENPACHI 03/10/2022 22:50:18
I'll give that a try


Thank you

##### Simon Steinmann [Moderator] 03/10/2022 22:50:32
let us know how it goes

##### KENPACHI 03/10/2022 22:52:20
I am getting the coordinates now!


my bot is a snake at the moment, and my head is the supervisor


so are these coordinates the center of the whole bot or just the head?

##### Simon Steinmann [Moderator] 03/10/2022 22:53:49
it should be the combined center of mass of everything that is inside your node, so your robot with all its chilldren


but better to verify

##### KENPACHI 03/10/2022 22:55:32
ohhh, I have them as separate modules. module 1 is the supervisor
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/951614314870083604/unknown.png)
%end


so I think I'll be getting the coordinates of the first module.

##### Simon Steinmann [Moderator] 03/10/2022 22:56:08
they are independent from each other? each with its own controller?

##### KENPACHI 03/10/2022 22:56:58
just one controller, with the supervisor emitting the motor rotation values

##### Simon Steinmann [Moderator] 03/10/2022 22:58:07
but the other modules need a controller to receive them right?

##### KENPACHI 03/10/2022 22:58:16
yes!

##### Simon Steinmann [Moderator] 03/10/2022 22:58:25
so every one of them has its own controller

##### KENPACHI 03/10/2022 22:58:42
correct

##### Simon Steinmann [Moderator] 03/10/2022 22:58:58
yeah, then this does not work


I'm guessing you want to dynamically link and seperate modules during the simulation?

##### KENPACHI 03/10/2022 23:01:03
they are all linked to eachother with a motor, I just want to track the center of mass of the bot so maybe I could train it to optimize it's gait parameters.


would I be able to pick a module as it's center and track that

##### Simon Steinmann [Moderator] 03/10/2022 23:01:46
Is there a reason the have to be defined as individual robots and not a single robot?

##### KENPACHI 03/10/2022 23:02:57
I'd be controlling each point motor individually


based on a sine wave

##### Simon Steinmann [Moderator] 03/10/2022 23:03:22
they can all be inside the same robot for that

##### KENPACHI 03/10/2022 23:03:31
oh

##### Simon Steinmann [Moderator] 03/10/2022 23:03:47
the only reason to have it as separate robots would be swarm robots


that can dynamically link up

##### KENPACHI 03/10/2022 23:04:05
it's not a swarm

##### Simon Steinmann [Moderator] 03/10/2022 23:04:15
but if the robot in its configuration is fixed, just add them as children

##### KENPACHI 03/10/2022 23:04:43
how would the controller change in that case?

##### Simon Steinmann [Moderator] 03/10/2022 23:05:12
you just need one controller and you initiallize all its motors and address them


is it rotational motors?

##### KENPACHI 03/10/2022 23:05:49
oh alright, I'll check it out

##### Simon Steinmann [Moderator] 03/10/2022 23:06:02
make sure the motors have unique names

##### KENPACHI 03/10/2022 23:06:17
but at the moment if I need to track the center module, is there any way?

##### Simon Steinmann [Moderator] 03/10/2022 23:07:08
manually by getting all the center of masses and calculating the combined one


Just put it all in the same robot. you will save yourself tons of hassle

##### KENPACHI 03/10/2022 23:08:09
haha, will try


thank you very much.

##### Simon Steinmann [Moderator] 03/10/2022 23:09:35
`from controller import Supervisor, Node



supervisor = Supervisor()

timeStep = int(supervisor.getBasicTimeStep())



\# --------------------------------------------------------------------

\# Initialize the arm motors and sensors. This is a generic code block

\# and works with any robotic arm.

n = supervisor.getNumberOfDevices()

motors = []

sensors = []

for i in range(n):

    device = supervisor.getDeviceByIndex(i)

    print(device.getName(), '   - NodeType:', device.getNodeType())

    # if device is a rotational motor (uncomment line above to get a list of all robot devices)

    if device.getNodeType() == Node.\_\_dict\_\_['ROTATIONAL\_MOTOR']:

        motors.append(device)

        sensor = device.getPositionSensor()

        try:

            sensor.getName()

            sensors.append(sensor)

            sensor.enable(timeStep)

        except Exception as e:

            print('Rotational Motor: ' + device.getName() +

                  ' has no Position Sensor')

\# --------------------------------------------------------------------`



You can use this generic code to initialize all rotational motors and put them into a list

##### giaco\_mz 03/11/2022 05:46:14
Thanks :) i find also that is possible to link visual studio and pycharm as IDE. But i need to understand better the debugging on it and what you can check .

##### pipppoo 03/11/2022 06:44:52
Hi, is there any way to improve the simulated lidar point cloud (see jumps, especially at larger distances)? Resolution is already = -1.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/951732422335135864/unknown.png)
%end

##### swadhin20 03/11/2022 07:23:34
Hello everyone, I am new to this server as well as webots. I am looking for an answer to this question: is it possible to create a custom world directly from a python script. For instance, is it possible to create a random polygon shaped boundary wall using python? or add random number of robots and set its properties through the python script?

##### Swadhin 03/11/2022 07:29:11
If so, can you please redirect me to some related example projects.

##### DDaniel [Cyberbotics] 03/11/2022 07:44:50
Hi, yes you can use a supervisor to spawn objects into the world, this tutorial goes through it: [https://cyberbotics.com/doc/guide/tutorial-8-the-supervisor](https://cyberbotics.com/doc/guide/tutorial-8-the-supervisor)

As for the polygons, if they are pre-defined objects you can save them as .wbo files and spawn them into the world in the same manner as explained in the tutorial, or alternatively spawn them from a string that describes their shape

##### Swadhin 03/11/2022 07:46:14
Great! Thank you I will check this out. 😀

##### giaco\_mz 03/11/2022 07:57:38
I see the default ones, but how can i add individual coded views and infos to Robot Window?


in particular what i missing is the receiver and emitter infos, but also know other variables infos there (user defined variable) would be great.

##### Rico Schillings[Sweaty] [Moderator] 03/11/2022 08:04:36
a nice introduction for custom windows is here. I meant there also exist a minimal tutorial how to get started with custom windows but i cant find it now..



[https://www.cyberbotics.com/doc/reference/robot-window-plugin](https://www.cyberbotics.com/doc/reference/robot-window-plugin)

##### giaco\_mz 03/11/2022 08:05:46
the link is broken, can you check it please?


It shows...

Webots Reference Manual R2022a

404: Not Found

##### Rico Schillings[Sweaty] [Moderator] 03/11/2022 08:07:25
Should work now

##### giaco\_mz 03/11/2022 08:08:28
Yes thx :). Could anyone please show to me piece of code where he/she shows the emitter/receiver infos inside robot window ?

##### Naxi 03/11/2022 13:35:48
Hi all, I've been searching through the documentation but I didn't find anything about this.. Is it possible to set USE/DEF keywords (besides the get\_from\_def method) within a supervisor to modify nodes?

##### kimmcg 03/11/2022 13:47:29
I've been working on a c-based pid controller for a small quadcopter, and I wrapped that with SWIG to a python  module for students to use in the python based webots controller

 The problem is, is that when I use the c based controller, the drone flies fine, but when I try the python controller with the same 

 wrapped c controller, I get the following error in the console: 

`WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.` 

The bounding boxes are already simple shapes (just a single cilinder), and I only have 4 joints, and I tried reducing the basic time step to 16 already. Also I get regular values for all four motors from the SWIG wrapped controller.

Hope that anybody could give me some more pointers to look at.

##### sHiiNe 03/11/2022 23:41:41
Hi all, we're trying to revitalise some code we've been left for a ROS2/e-puck platform project on webots - it depends on the `E-puck_enu.proto` file which appears to have been deleted from the `webots_ros` repository in commit `54005fe5fbf0ae4d9bb12479076ee9ba3687f664` - the commit message implies the functionality is now elsewhere. If this is so, how can we adapt the code to work with the latest version of webots/the `webots_ros2` tool?

##### mouselet2017 03/12/2022 09:11:38
Is there any way to get the device from supervisor? I haven't found any way so far, if it works I hope someone can show me a demo. Thanks!!!😭 😭

##### Swadhin 03/12/2022 09:46:07
Can anyone please tell how to use the USE "DFE name" with boundingObject in supervisor code?

##### DDaniel [Cyberbotics] 03/12/2022 09:47:04
The same way you do for a Robot, Supervisor is just a Robot with additional powers


what do you mean? What are you trying to do?


Since 2022a everything was converted to FLU/ENU by default, so you should be able to use the normal E-puck.proto

##### Swadhin 03/12/2022 09:48:03
I want to reuse the shape DFE in solids boundingObject in my supervisor script


Using the importMFNodeString

##### DDaniel [Cyberbotics] 03/12/2022 09:49:11
You're importing a shape object from string?

##### Swadhin 03/12/2022 09:50:08
Yea, there I want to specify the boundingObject as the "USE shapeobject" which was previously specified

##### swadhin20 03/12/2022 09:50:28
children\_field.importMFNodeFromString(-1, 'DEF wall Solid {name    "wall", translation     0 0 0, children    DEF wall\_shape Shape {appearance  PBRAppearance { } , geometry   Box {size  0.01 1 0.2}}}, boundingObject   USE')


something like this

##### DDaniel [Cyberbotics] 03/12/2022 09:51:17
should be `boundingObject USE wall_shape`

##### swadhin20 03/12/2022 09:51:20
in boundingObject I want to use the wall\_shape


I tried this, its not working]


It is showing Null

##### DDaniel [Cyberbotics] 03/12/2022 09:52:31
can you copy the actual code? Might be missing a closing bracket after `wall_shape`

##### swadhin20 03/12/2022 09:52:56
from controller import Supervisor

import numpy as np



TIME\_STEP = 32



supervisor = Supervisor()  # create Supervisor instance



root\_node = supervisor.getRoot()

children\_field = root\_node.getField('children')



children\_field.importMFNodeFromString(-1, 'DEF BB-8 BB-8 {name    "BB-8",controller      "void", translation     0 0 0}')

children\_field.importMFNodeFromString(-1, 'DEF wall Solid {name    "wall", translation     0 0 0, children    DEF wall\_shape Shape {appearance  PBRAppearance { } , geometry   Box {size  0.01 1 0.2}}}, boundingObject   USE wall\_shape')

\# wall = supervisor.getFromDef('wall')

\# wall\_child\_field = wall.getField('children')

\# wall\_child\_field.importMFNodeFromString(-1, 'DEF wall\_shape Shape { }')

\# wall\_shape = supervisor.getFromDef('wall\_shape')

\# wall\_shape\_appearance = wall\_shape.getField('appearance')

\# wall\_shape\_geo = wall\_shape.getField('geometry')

\# wall\_shape\_appearance.importMFNodeFromString(-1, 'PBRAppearance { }')

\# wall\_shape\_geo.importMFNodeFromString(-1, 'Box {size    0.01 1 0.5}')

wall\_bounding\_field = wall.getField('boundingObject')

wall\_bounding\_field.importMFNodeFromString(-1, 'USE wall\_shape')

\# [CODE PLACEHOLDER 1]



i = 0



bb8\_node = supervisor.getFromDef('BB-8')

print(children\_field)

while supervisor.step(TIME\_STEP) != -1:

    # [CODE PLACEHOLDER 2]

    if i<=5:

        children\_field.importMFNodeFromString(-1, 'DEF BB-8'+str(i) + ' BB-8 {name    "BB-8",controller      "void", translation     '+ str(5*np.random.random())+' '+ str(5*np.random.random())+' '+ str(5*np.random.random())+' '+'}')

    if i == 0:

        translation\_field = bb8\_node.getField('translation')

        new\_value = [2.5, 0, 0]

        translation\_field.setSFVec3f(new\_value)

    if i == 10:

        bb8\_node.remove()

    if i == 20:

        children\_field.importMFNodeFromString(-1, 'Nao { }')





    i += 1


you are right...


I think I ammissing a bracket


Yes, it works!

##### Ranga Kulathunga 03/13/2022 06:02:32
Hi all, how position and orientation of a Webots vehicle can be updated in SUMO GUI?

##### Kormit 03/14/2022 10:13:42
Hi, I'm running Webots on Arch linux and recently (probably because of a package update), robot windows no longer work (I cannot see them, the panel is just the ui color). Could someone help me out with what could potentially be the problem?

##### Baya19 03/14/2022 10:24:54
Hi guys how can i make webots faster ? i'm runnning a simulation with lidar and drones and the world is becoming too slow

##### DDaniel [Cyberbotics] 03/14/2022 10:25:37
are you printing a lot of information to the console?

##### Baya19 03/14/2022 10:25:48
Yes


I'm exporting info to  a file

##### DDaniel [Cyberbotics] 03/14/2022 10:28:34
to file should be fine, but if you print a lot of info to the webots console it slows down noticeably. You can start webots with: `--stdout --stderr` to print to the terminal instead

##### Baya19 03/14/2022 11:11:51
I stopped printing but it still slow

##### kimmcg 03/14/2022 11:54:56
Just an reminder here for my issue last friday, i think it was a bit lost in the list. I had a bit too much printing  problem too but that didn't solve my issue either.  Wonder if it is maybe better to put this as an github issue? Seems to be quite weird that I can't control the quadcopter from python with the exact same controller

##### DDaniel [Cyberbotics] 03/14/2022 12:43:51
Yes feel free to open an issue about it on github, if you can provide a minimal example that showcases the issue even better.

I didn't understand why you're not using directly the webot's Python API though, it already uses SWIG to wrap the C API.

##### kimmcg 03/14/2022 12:46:01
ah Ii think you are misunderstanding me in that bit. I wrote a custom controller in C, that is just an PID controller, but I didn't write that again fully in python. So it is only the rotational motor velocities that I'm computing with that swig wrapped c controller, but I still use the webots python api to set those motors to those velocities


It is a bit convoluted, I know.. but the course that I'm writing this for don't have experience with C.


I could rewrite the controller fully in python just to check it out if the SWIG conversion did something wrong, but I hope that I could do the same for the actual c-based controllers directly from the firmware, so that is sort of software in the loop.

##### DDaniel [Cyberbotics] 03/14/2022 12:51:44
ok I see, then yes I suggest you open an issue on github. If you can provide a minimal example we are able to reproduce we can look into it, it's a bit hard to say what's wrong otherwise

##### kimmcg 03/14/2022 12:59:55
I'll give it a try to make an minimal example, but not sure if I can make it more minimal than it is already. I'll post an issue about it.

##### thomas pesquet 03/14/2022 13:47:07
[Motion from Controller] Hello, I'd like to make a motion of waves and of an oil slicks upon a sea (cube of fluid). Does someone know where I can find more informations about the motion API (Python) ?

What kind of motion file can be used ?

##### kimmcg 03/14/2022 16:13:46
You know what... it seems that I have fixed the problem.... just the simple mistake of not turning the motors in the right direction... so it was not supposed to go anywhere in the first place. Still a weird issue but I'm unblocked now!

##### Naxi 03/16/2022 13:06:19
Hi, I'm managing different robot models from many packages and I want to have a supervisor to spawn them all, is it possible to import a node from a proto file which is not in the webots library nor in the /protos directory of the supervisor project?

##### Simon Steinmann [Moderator] 03/16/2022 15:48:50
In the settings you can add an extra directory

##### Naxi 03/16/2022 16:43:19
WIll check that, thanks!

##### [Optimum Pride] Wintery Melony 03/17/2022 00:58:28
does anyone know how to fix this error?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/953819579476484176/unknown.png)
%end

##### Simon Steinmann [Moderator] 03/17/2022 02:23:24
your file seems corrupt, seems like you did not format it properly or have a syntax error somewhere

##### [Optimum Pride] Wintery Melony 03/17/2022 02:23:51
i just installed it and its my first time opening it

##### Simon Steinmann [Moderator] 03/17/2022 02:24:56
then i have no idea

##### [Optimum Pride] Wintery Melony 03/17/2022 02:26:29
im going to try reinstalling it


i changed the install location to a folder without a space and it works fine i guess

##### Ale 03/17/2022 06:08:16
Hi, I want to use Webots to generate Training Data for a neural network.

Is there any way to get the current timestamp of the simulation time (in python) or the elapsed time since the beginning of the simulation?

Since I dont get a 100% consistent speed its kinda tricky how many values I get per simulated Second.

As basicTimeStep I chose 8 and as Time\_Step for the controller I chose 64

##### Simon Steinmann [Moderator] 03/17/2022 06:26:19
Check the api documentation for robot(). The function you are looking for is in there

##### DDaniel [Cyberbotics] 03/17/2022 06:31:11
You can get the elapsed time with `robot.getTime()`

##### Ale 03/17/2022 07:01:45
Thanks, I will look into it later :)

##### Folafoyeg 03/17/2022 08:37:04
Hi all, I am very new to webots and also just joined this discord this morning. I have an assignment to create a webot project, does anyone know where I can get solved projects with codes to look into? Thanks for your help.

##### DrakerDG [Moderator] 03/17/2022 08:40:12
Hi! Webots have a lot of samples, that you can view how it do it
%figure
![2022-03-17.png](https://cdn.discordapp.com/attachments/565154703139405824/953935774515097640/2022-03-17.png)
%end

##### Folafoyeg 03/17/2022 08:47:24
Thanks a lot `@DrakerDG`

##### DrakerDG [Moderator] 03/17/2022 08:47:47
You welcome

##### thomas pesquet 03/17/2022 08:51:22
Have you ever done waves upon the sea in PROTO file ? I don't think it's possible

##### S4JJ4D 03/17/2022 13:36:40
I'm trying a to use a PROTO  inside another PROTO but Webots is not able to parse the parent PROTO when I attempt to import it into the world.

.proto files are minimal and very simple and I can't figure out why things go wrong. files are attached. Can anyone please help me?
> **Attachment**: [ChildNode.proto](https://cdn.discordapp.com/attachments/565154703139405824/954010382584922143/ChildNode.proto)
> **Attachment**: [ParentNode.proto](https://cdn.discordapp.com/attachments/565154703139405824/954010382748504084/ParentNode.proto)
%figure
![WebotsErrorLog.png](https://cdn.discordapp.com/attachments/565154703139405824/954010383193079908/WebotsErrorLog.png)
%end

##### DDaniel [Cyberbotics] 03/17/2022 13:40:09
works fine for me, which version of Webots are you using?

##### S4JJ4D 03/17/2022 13:40:39
R2022a

##### DDaniel [Cyberbotics] 03/17/2022 13:41:10
normal release or a nightly build?

##### S4JJ4D 03/17/2022 13:44:12
I followed the instructions given in:

[https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt)

##### DDaniel [Cyberbotics] 03/17/2022 13:49:23
I can't reproduce it, works fine for me and I don't see any issues with the protos themselves. Does adding the ChildNode proto work by itself?

##### S4JJ4D 03/17/2022 13:49:48
yes


Something strange is happening. I changed the name "ChildNode" to "Child" and "ParentNode" to "Parent" and now I get no errors.

##### DDaniel [Cyberbotics] 03/17/2022 13:54:37
weird indeed 🤨


and if you rename it back as it was now, does it work?

##### S4JJ4D 03/17/2022 13:56:29
No it doesn't

##### DDaniel [Cyberbotics] 03/17/2022 14:00:52
in the protos folder, if you display the hidden files (CTRL + h) you should find `.ParentNode.cache` and `.ChildNode.cache`, could you try to delete them?


the cache might be corrupted, it's the only possibility I can see

##### S4JJ4D 03/17/2022 14:45:41
Thank you so much.

Errors were gone after I removed the cache files.

##### Folafoyeg 03/17/2022 16:12:09
Pls oo, who can help me with fruit sorting robotic simulation?

##### bobetos 03/17/2022 16:21:15
hello i need to ask a very important question. For some reason webots\_ros2 package doesnt work for me. ROS2 is perfectly installed and so is webots 2022a yet in the colcon build phase it leaves 11 packages aborted and doesnt work for ros2\_webots\_core and importer. I somehow made it work by bouncing around different versions of webots but the the example builds dont all work. For example ros2 launch webots\_ros2\_tesla robot\_launch.py works yet ros2 launch webots\_ros2\_tiago robot\_launch.py doesnt. Same with epuck and turtlebot. Please any help?


also webots\_ros2 says failing in github page i dont know if this is why.

##### DDaniel [Cyberbotics] 03/17/2022 16:24:11
did you follow these instructions? [https://github.com/cyberbotics/webots\_ros2/wiki/Build-and-Install](https://github.com/cyberbotics/webots_ros2/wiki/Build-and-Install)

##### bobetos 03/17/2022 16:25:39
i have followed everything excactly as it says. I am quite tech savvy and have performed many installations in various programms/packages/programming languages. I completed the procedure excactly as it says for ROS2, webots\_ros2 and webots

##### Dorteel 03/17/2022 17:44:58
Hi there! I'm trying to convert my urdf into proto using urdf2webots, but I keep getting an error message. Any suggestions? I ran out of ideas on how to fix it..



`(base) user<@!591756845388005378>:~/Documents$ python3 -m urdf2webots.importer --input=locobot.urdf

Traceback (most recent call last):

  File "/usr/lib/python3.8/runpy.py", line 194, in \_run\_module\_as\_main

    return \_run\_code(code, main\_globals, None,

  File "/usr/lib/python3.8/runpy.py", line 87, in \_run\_code

    exec(code, run\_globals)

  File "/home/user/Repos/urdf2webots/urdf2webots/importer.py", line 337, in <module>

    convertUrdfFile(options.input, options.output, options.robotName, options.normal, options.boxCollision, options.disableMeshOptimization,

  File "/home/user/Repos/urdf2webots/urdf2webots/importer.py", line 91, in convertUrdfFile

    return convertUrdfContent(urdfContent, output, robotName, normal, boxCollision,

  File "/home/user/Repos/urdf2webots/urdf2webots/importer.py", line 190, in convertUrdfContent

    domFile = minidom.parseString(input)

  File "/usr/lib/python3.8/xml/dom/minidom.py", line 1969, in parseString

    return expatbuilder.parseString(string)

  File "/usr/lib/python3.8/xml/dom/expatbuilder.py", line 925, in parseString

    return builder.parseString(string)

  File "/usr/lib/python3.8/xml/dom/expatbuilder.py", line 223, in parseString

    parser.Parse(string, True)

xml.parsers.expat.ExpatError: syntax error: line 1, column 0`

##### Simon Steinmann [Moderator] 03/17/2022 17:58:12
Can you post your urdf file, the last line of the error message suggests, that there is a Syntax error on line 1

##### Dorteel 03/17/2022 17:59:34
sure! thanks for the quick reply!
> **Attachment**: [locobot.urdf](https://cdn.discordapp.com/attachments/565154703139405824/954076547424329798/locobot.urdf)


I converted it from this xacro file, if that helps
> **Attachment**: [mobile\_wx250s.urdf.xacro](https://cdn.discordapp.com/attachments/565154703139405824/954076767751127081/mobile_wx250s.urdf.xacro)


ah I just opened it, saw the conversion didn't work 😅


I managed to create a proper urdf file, but the textures directory that gets created ends up being empty, and when I import the proto it has no body
> **Attachment**: [model.urdf](https://cdn.discordapp.com/attachments/565154703139405824/954083662448721920/model.urdf)

##### AlexandrosNic 03/17/2022 18:37:12
Hello everyone. Did anyone managed to successfully run Webots+ROS2 through WSL2 and Docker (with a Dockerfile)? If so, any tutorial suggestion for it? Since currently I seem to go through issues with the XServer (display), when trying to deploy it, using the official procedure [https://cyberbotics.com/doc/guide/installation-procedure#run-webots-in-docker-with-gui](https://cyberbotics.com/doc/guide/installation-procedure#run-webots-in-docker-with-gui)

##### Simon Steinmann [Moderator] 03/17/2022 23:13:06
If you have no textures in the urdf, then the texture folder created will be empty. As for the "no body" You would have to be more specific. Perhaps post your proto file


it might be due to the absolute links of the mesh files. Urdf is usually structured with ROS packages. Or at least relative paths


However absolute paths should work. I did implement mesh path handling fixes a while back

##### Çağrı Kaymak 03/18/2022 08:00:42
Hi all. When I click Robotis-Op2 to open robot window in Webots, I take error like this: Error: libzip.so.4: cannot open shared object file: No such file or directory (dynamic library)

Error: robot window initialization failed

Error: Cannot load the "/usr/local/webots/projects/robots/robotis/darwin-op/plugins/robot\_windows/robotis-op2\_window/librobotis-op2\_window.so" robot window library.

How can i solve this problem? Thanks in advance.

By the way, my os is Ubuntu 18.04 and webots version is r2020b rev1.

##### Simon Steinmann [Moderator] 03/18/2022 08:02:08
how did you install?

##### Çağrı Kaymak 03/18/2022 08:02:22
Via .deb

##### Simon Steinmann [Moderator] 03/18/2022 08:03:01
using the correct one? there is a separate version for 18.04 I believe

##### Çağrı Kaymak 03/18/2022 08:03:29
Yes, this is for 18.04

##### Simon Steinmann [Moderator] 03/18/2022 08:03:42
then I dont know

##### Çağrı Kaymak 03/18/2022 08:04:11
Ok, thank you

##### sHiiNe 03/18/2022 14:57:36
are `.proto` files compliant with protocol buffer syntax, or is that a naming coincidence?

##### DDaniel [Cyberbotics] 03/18/2022 15:26:29
"proto" is defined in the VRML97 norm. This norm defines the syntax used in worlds and proto

##### kazeko55 03/18/2022 18:56:59
hello everyone, I have a proto file translated from urdf and I open it in webots. When I want to drive differentially, I send cmd\_vel data, but the robot does not move forward even though the wheels are turning. I think the problem is with the physics plugin because for example, when I lift the robot a little bit and release it, it doesn't fall to the ground. How can I solve this problem?



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/954457282941444176/unknown.png)
%end

##### cnbarcelo 03/19/2022 05:26:57
Hi!

Is there any way of programmatically spawn robots defined in PROTOS into the simulation on runtime?

##### Simon Steinmann [Moderator] 03/19/2022 05:28:55
[https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_field\_import\_mf\_node](https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_import_mf_node) try this

##### cnbarcelo 03/19/2022 05:32:35
Thanks for the quick reply.

I'll give it a try, tho reading the code I saw it expects the input to be `.wbo/.wrl`

##### Simon Steinmann [Moderator] 03/19/2022 05:38:56
try the proto... not sure if LUA scripting works with that though.

##### DDaniel [Cyberbotics] 03/19/2022 07:53:09
There's a similar import function that imports from string. So if you defined your robot as a proto (ex: MyProto) you can spawn it using the string: `'MyProto{}'`

##### S4JJ4D 03/21/2022 08:42:53
Hey. Is there any speedSensor for hinge joints in Webots?

##### DDaniel [Cyberbotics] 03/21/2022 10:33:44
No, only a position sensor, but you can derivate it to get an estimate of the velocity ( `(current_position - previous_position) / timestep`), assuming the timestep is in seconds

##### S4JJ4D 03/21/2022 10:35:28
thank you


I attached a GyroSensor to the solid endpoint of the hinge to get angular velocity along the joint axis

##### cnbarcelo 03/23/2022 14:02:42
Hi!

I'm trying to contribute to Webots, but I'm getting an unexpected segmentation fault (segmentation fault is always unexpected haha). Is it possible to run Webots with GDB? I'm getting errors related to xcb when trying it, so I wanna be sure it's not something I'm doing wrong.

##### DDaniel [Cyberbotics] 03/23/2022 14:05:21
Yes, you can compile webots in debug mode (`make -j12 debug`) and then use GDB. Also make sure the `WEBOTS_HOME` variable is set correctly. More information about the development environment and how to set it up is available here: [https://github.com/cyberbotics/webots/wiki](https://github.com/cyberbotics/webots/wiki)

##### cnbarcelo 03/23/2022 14:29:13
Yeah, that's what I'm doing, tho GDB says cannot load xcb (even though it was found).

Might be an issue with my setup then, will work on that.

Thanks!

##### zev 03/23/2022 20:05:14
I had the same issue with xcb, turn out that webots was using a different version of QT than what it is supposed to. It was because I set the LD\_LIBRARY\_PATH to include another path

##### MaudesMeow 03/23/2022 23:01:32
Hello all! New here.



Question: I'm trying to build a roach like robot within webots using the e-puck bot. I've successfully been able to make it avoid light using the built in light sensors. I've activated two children nodes on the ground sensor that are IR distance sensors, and I want them to be able to detect "food" and its "scent", so when it smells the scent, it knows food is nearby


I hope I'm explaining that well, any input would be appreciated, and thank you in advance for your time!

##### 张飞尘 03/24/2022 07:30:56
vscode


hi！can i connect with vscode？

##### DDaniel [Cyberbotics] 03/24/2022 08:57:12
One way of handling the food could be to color the ground with a specific color, and based on the ground sensor data it reads decide whether it's food or not. In the sample projects (`file > open sample worlds`) there's an example of something similar called `battery.wbt`, in which the robot's battery recharges when over the recharge zones.



As for the scent, there's nothing in Webots to do it out of the box, but you can approximate it with a supervisor. Basically every timestep the supervisor reads the position of the robot and the position of the food, and based on the distance between the two it can send the robot a message about the intensity of the scent it would perceive (the closer it is, the higher the value for example)

##### AlexandrosNic 03/24/2022 09:10:47
hey, is there a workaround to set the fields of protos programmatically? (they are normally read-only)

##### DDaniel [Cyberbotics] 03/24/2022 09:12:16
Internal fields of a PROTO cannot be changed, but exposed parameters can be changed programmatically the same way as normal ones are. So you might need to expose the parameter first.

##### bocchio 03/24/2022 13:18:20
Hi there!

I'm automating some stuff in a supervisor node. Right now I want to implement a call that's part of the UI

I don't exactly know what to do to the children field to replace it with its default value (I mean, I guess I could delete all the nodes and insert the default value that's in the proto, but there must be a better way).

```
obstacles_children_field = robot.getFromDef("OBSTACLES").getField("children")
```

How would you proceed?



Thanks in advance!
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/956542485038006282/unknown.png)
%end

##### Hawnzah 03/25/2022 08:31:26
Hi guys, I'd like to put aruco marker on top of a robot (surveyor for example). Is that possible? and how'd I do that?

##### áçè 03/25/2022 10:10:38
hi guys  i m using webots 2020b version , in this i need to have boundingobject of my wheelchair to be footrest but i can't increase the boundingobject area  , so what can i do ?  plz help

##### DDaniel [Cyberbotics] 03/25/2022 11:16:27
The Surveyor (and most robots) have an extensionSlot to which you can add whatever you want. In your case, you can add the `VisualArmature` node, which you can place/resize as you wish on the robot. In the `textureUrl` you put an image of the marker itself (an example of this exists in the `nao_challenge.wbt` world available under `file > open sample worlds`).

##### Hawnzah 03/25/2022 12:35:30
it works great, tyvm!


is there a way to get video from a specific camera? what im doing rn is pressing 'Start video recording of the current simulation' and then resizing the camera to full screen

i mean it works, but i'd like to get rid of the pink borders


i guess i have to write my controller?

##### AlexandrosNic 03/25/2022 15:47:37
Hey, did anyone created a proto of an OnRobot VGC10 Gripper ?

##### moebius 03/26/2022 00:26:20
`math.atan2(d_z, d_x)` is there something wrong in this approach?

##### S4JJ4D 03/26/2022 05:49:14
second function makes more sense to me


and I'm not sure the direction of travel effects the bearing values returned by the compass, as long as the orientation of the robot remains the same

##### áçè 03/26/2022 05:53:56
by using custom controller on my wheelchair i m able to rotate the wheels but its not moving forward ....plz help

##### moebius 03/26/2022 08:07:40
yep thats what made sense to me as well, i could not figure out why the compass page has the formula it does for the bearing angle for the ENU configuration. The only other issue i have is using the GPS coordinates to get correct the target bearing angle, i think i am doing something wrong there as well

##### S4JJ4D 03/26/2022 08:32:18
I don't understand it either. Webots folks might be able to explain it


I think the method you're using to get the target bearing is fine as long as the relative displacement vector is expressed in the correct coordinate frame.

##### Franky 03/26/2022 11:02:13
Hi,



I have attached a rotational motor to a propeller. Was just wondering if there is any sort of default values the motor takes for inertia for torque to rotational acceleration calculation? Also if i give the fast and slow helix solids a physics node, will the mass and inertia be taken into the motors torque to rotational acceleration calculations?

##### Süeda Şen 03/26/2022 11:08:41
Hello to all of you. I'm new to Webots. What I want to ask is, I can program the movements and positions of the robots, but can I program the position of any object? For example, when I put a fluid object or any other object on the ground, how can I write its positions in the script? Thanks in advance.

##### S4JJ4D 03/26/2022 11:27:35
You can use a Supervisor to directly assign positions to your objects in Webots world. You should mark one your robots in the model as a Supervisor by setting its supervisor attribute to True. Later in the controller script, you are able to directly specify position for any object in the world.


refer to Webots user guide tutorials, There's a separate section on how to use a Supervisor

##### Süeda Şen 03/26/2022 12:12:06
Thank you so much 🙏 I'll try right away


Okay I'm gonna check thanks for your  attention  🙏


Yes, the Supervisor feature you mentioned exists for objects of the robot type, but is there a feature where I can assign a position with a controller that allows me to specify a specific position for any object, just like the Supervisor in robots? For example, let's take an office chair object, with which property can we assign a specific position from the controller?
%figure
![D__deneme_worlds_trial.wbt_deneme_-_Webots_R2022a_26.03.2022_15_17_32_2.png](https://cdn.discordapp.com/attachments/565154703139405824/957252590649888839/D__deneme_worlds_trial.wbt_deneme_-_Webots_R2022a_26.03.2022_15_17_32_2.png)
%end

##### DDaniel [Cyberbotics] 03/26/2022 12:56:21
Supervisor can move any object, in the case of your chair you'd do so by changing its translation field with a Supervisor. There's a tutorial about supervisors here [https://www.cyberbotics.com/doc/guide/tutorial-8-the-supervisor](https://www.cyberbotics.com/doc/guide/tutorial-8-the-supervisor), checkout the "Moving Objects Using a Supervisor" section

##### áçè 03/26/2022 16:12:45
plz help

##### S4JJ4D 03/26/2022 16:19:12
You need to explain the problem in more detail and also more clearly

##### MaudesMeow 03/26/2022 16:38:36
Amazing, thank you `@DDaniel` ! I successfully implemented the charger node and my bot reads it as "food" when it charges. Again, I really appreciate it. Following up on the scent part, I enabled a gps node on my epuck, but I I was having troubles accessing the values via the controller, is there a better way to implement or what is the best option for obtaining the position of robot and the food from the supervisor/controller script? 



thank you again and I appreciate your time

##### DDaniel [Cyberbotics] 03/26/2022 16:45:48
You can use the supervisor's `getPosition` method [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_position](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_position)

For how to do so you can check this part of the supervisor tutorial: [https://www.cyberbotics.com/doc/guide/tutorial-8-the-supervisor?tab-language=python#acquire-measurements-using-a-supervisor](https://www.cyberbotics.com/doc/guide/tutorial-8-the-supervisor?tab-language=python#acquire-measurements-using-a-supervisor)

Basically you need to get a node reference of the robot (which you can do by giving it a DEF name and then calling `getFromDef`), then simply call `robot_node.getPosition()`

##### Darragh 03/26/2022 17:10:06
Hi there, I was just wondering if there is a function to empty a receivers queue? I'm using emitters and receivers to change the states of my bots and when I get to the second iteration my receiver function is activated right away even when my emitting function hasn't been used in this iteration yet.

Any help would be great, thanks.

##### DDaniel [Cyberbotics] 03/26/2022 17:11:55
you can call `wb_receiver_next_packet` on a loop until `wb_receiver_get_queue_length` is zero [https://www.cyberbotics.com/doc/reference/receiver#wb\_receiver\_next\_packet](https://www.cyberbotics.com/doc/reference/receiver#wb_receiver_next_packet)

##### Süeda Şen 03/26/2022 17:34:56
Appreciate your help🙏 , I'm gonna check this link 👍

##### áçè 03/26/2022 17:40:50
i have  a wheelchair solid work model which i imported to webot , there i gave the objectbound as the four wheels of wheelchair , so that it doesn't slip through floor , after that i wrote controller using c++ for giving/moving wheelchair but when i ran my controller the wheels of wheelchair only rotate at it's own axis instead of moving the wheelchair forward

##### DDaniel [Cyberbotics] 03/26/2022 17:41:28
do you have a physics node defined in all intermediary solids?

##### áçè 03/26/2022 17:44:03
do i have to give physics node to all wheelchair children  part (wheelchair is parent and seat , backrest , etc are children )


as i m new to webots so i don't know much about it

##### DDaniel [Cyberbotics] 03/26/2022 17:47:02
all solids from the robot node to the wheels need to have physics

##### áçè 03/26/2022 17:47:51
ok thanx so much ...


let me try it


i m not getting proper result

##### DrakerDG [Moderator] 03/26/2022 17:56:25
Can you share your project to take a look it?

##### áçè 03/26/2022 17:57:19
can we have google meet now as of tomorrow i have to show my work

##### VRsE 03/26/2022 18:12:24
Would anyone know why my MultiSensorS21   enable function  could possibly crash from  in C++ ?, (it all builds fine and  its Camera object definition works with the world model, similarly defined ordinary camera works fine) Thanks for any tips, Sandy

##### DDaniel [Cyberbotics] 03/26/2022 18:23:12
could be a bug, can you share the project?


doesn't crash for me

##### VRsE 03/26/2022 18:31:17
thanks, Daniel, debugging I could do myself... but do not have access to the source code of the enable function of the S21 camera, which is part of the world model through a proto, how would you do it?


as I said other cameras, even the meta camera show the overlays fine


it is just the meta range finder which does not work with enable

##### DDaniel [Cyberbotics] 03/26/2022 18:34:22
if it's the range finder that has issues, then try installing the latest nightly build from here: [https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)

I believe there was a bug with that, but should not crash

##### VRsE 03/26/2022 18:35:19
thanks ! ... will try this new built

##### DDaniel [Cyberbotics] 03/26/2022 18:36:16
if that doesn't solve it, I'll need to have a look at exactly what you do in your world/controller and whats in it, it's hard to say whats wrong with just this info if I'm unable to reproduce the problem

##### VRsE 03/26/2022 18:47:51
thanks, sending the world file and controller in a sec


actually, it stopped crashing but does not show any images in the meta range finder  overlay... camera is enabled on line 983 in firstUR10eControl.cpp... the 3 other cameras are working fine

##### American Patriot 03/26/2022 21:51:44
hey i got a question (also i am new here, hiiii)


how do i import the python package for use with webots via external ide? im using visual studio code to edit my controller but it cant find the controller package or anything


yo


anyone online?

##### Simon Steinmann [Moderator] 03/26/2022 22:35:58
`@American Patriot` you have to set the correct environment variables


[https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers) refer to this documentation

##### American Patriot 03/26/2022 22:50:58
what if i set those vars and nothing new happens

##### Simon Steinmann [Moderator] 03/26/2022 22:51:17
restart your program

##### American Patriot 03/26/2022 22:51:18
do i need to reboot or can i just sign out and back in?


i did. signed in/out

##### Simon Steinmann [Moderator] 03/26/2022 22:51:37
or the console you launch the python file with

##### American Patriot 03/26/2022 22:51:50
i did


and slight correction. im editing the file with vs code but i am not running it from there


i just need a handle to the webots framework. so i can do things like if (hardware is not type(controller.motor)) or something like that

##### Simon Steinmann [Moderator] 03/26/2022 22:54:33
you import the Controller package at the top of your file?

##### American Patriot 03/26/2022 22:54:40
i try


nothing works

##### Simon Steinmann [Moderator] 03/26/2022 22:54:55
show me the file and error message you get

##### American Patriot 03/26/2022 22:55:18

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/957412460485296128/unknown.png)
%end


its more a edit time error


i mean it aint even an error. but i cant get intellisense to register. so i dont really know where to pull the motor python class type from

##### Simon Steinmann [Moderator] 03/26/2022 22:57:57
do you initialize the Robot() class?

##### American Patriot 03/26/2022 22:58:04
yes

##### Simon Steinmann [Moderator] 03/26/2022 22:58:04
does the controller work inside webots?

##### American Patriot 03/26/2022 22:58:09
yes


it just imports that file

##### Simon Steinmann [Moderator] 03/26/2022 22:58:26
show me all the environment variables you set

##### American Patriot 03/26/2022 22:58:49
and i use robot =
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/957413344472617061/unknown.png)
%end


oh h/o



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/957413478547718184/unknown.png)
%end



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/957413536664010782/unknown.png)
%end

##### Simon Steinmann [Moderator] 03/26/2022 23:00:50
you have to set the  PYTHONPATH  variable

##### American Patriot 03/26/2022 23:01:01
oh


does webots have its own copy or no

##### Simon Steinmann [Moderator] 03/26/2022 23:01:16
also, are you sure your WEBOTS\_HOME is correct not being the default      C:\Program Files\Webots

##### American Patriot 03/26/2022 23:01:23
yes

##### Simon Steinmann [Moderator] 03/26/2022 23:01:27
okay

##### American Patriot 03/26/2022 23:01:30
i installed it differently i guess


theres no webots folder in the default directory


i traced my windows desktop shortcut back to the exe location

##### Simon Steinmann [Moderator] 03/26/2022 23:02:16
well if the folder exists where you specified, it should be correct


but dont forget row 5 and 6 in the table of the documentation

##### American Patriot 03/26/2022 23:03:07
just got them


hey wait


im an idiot lol


i forgot that the autocomplete thing is a separate thing and its a component part of vs code


i added a search path leading to the controller and wallah!

##### VRsE 03/27/2022 09:50:58
Hi ! I am new to Webots and was looking into RosCamera.hpp. It contains the osEnable function but does not declare a service to enable cameras. Do I need to add that code myself? Asking as this is the most basic you would need to do with the camera first - enable it. Can anyone  shed some light on this?

##### Ranga Kulathunga 03/28/2022 05:26:16
Hey all, can we change the acceleration of a Webots vehicle at each time step in the contoller code?

##### aysegul ucar 03/28/2022 11:23:34

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/957963156163022918/unknown.png)
%end


Hi, It is waiting  to transfer to the real robot. No answer no error I am using ubuntu20 and 2022a webots do you have any idea?


Do you have an answer. Previously many thanks

##### giaco\_mz 03/28/2022 14:33:42
Hi i want create similar robot window. Could you please suggest to me some references to do it?

##### MaudesMeow 03/28/2022 20:16:29
hello, would anyone have any idea why my light sensors wouldn't be picking up from the Spotlight? 

the val: and sens: are the light sensors values!
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/958097266898513930/unknown.png)
%end


update to my last question ^ it appears my distance sensors are doing the same thing. They're registering a value, but not responding to anything the controller is advising them to do

##### JoeyBadass 03/29/2022 18:46:08
Guys a serious question, do anyone know how to get that pole on the back of the tractor?[https://www.youtube.com/watch?v=IH16eGxCmbM](https://www.youtube.com/watch?v=IH16eGxCmbM)

##### moebius 03/29/2022 20:06:37
yes it is working now


I am controlling a 2 wheeled robot, and I am setting identical velocities on both the wheels, yet it it ends up moving in a curved trajectory and not in a straight line, is there any other setup i need to do for the wheels to work properly? i am doing the initial motor setup by setting position to infinity

##### Robokashi 03/29/2022 20:11:10
Treat this as a feature ! This is often/always the real life behavior hahah

##### S4JJ4D 03/29/2022 20:35:49
hard to tell without a model


make sure all dummy wheels are actually dummy


not locked or motorized


also, check the symmetry

##### DrakerDG [Moderator] 03/30/2022 02:55:38
If it possible, share your world to check it out and find issues, please


Add the pole is easy, maybe the difficult is the spraying effect.

##### JoeyBadass 03/30/2022 08:52:37
Hello I’m new to webots, that’s why i asked. There isn’t much video online so I’m stuck

##### DrakerDG [Moderator] 03/30/2022 09:00:08
Maybe it be using a display to make the spraying effect like a drawing

##### DDaniel [Cyberbotics] 03/30/2022 09:09:01
I assume it's using the pen node to draw the lines, you can look up an example of it under `file > open sample worlds > pen.wbt`

##### DrakerDG [Moderator] 03/30/2022 09:11:10
In this case, maybe it is using more than one pen, right?

##### DDaniel [Cyberbotics] 03/30/2022 09:36:55
Yes, I assume so, a series of pens along the horizontal bar

##### DrakerDG [Moderator] 03/30/2022 09:38:03
Interesting effect, thanks

##### JoeyBadass 03/30/2022 10:38:36
Thanks, when I get home I will try it out and ask if I run into any problems  `@DrakerDG` `@DDaniel`

##### moebius 03/30/2022 18:06:37
this is the world file and the controller.
> **Attachment**: [debug1.zip](https://cdn.discordapp.com/attachments/565154703139405824/958789364547219456/debug1.zip)

##### DrakerDG [Moderator] 03/30/2022 19:09:58
OK, let me check it out


I made clone (blue) and I did simplify the bounding object, include the wheels, and adding a balljoing in the rear part to simulate the sliding effect. Your controller is a same. Check it out
> **Attachment**: [SmallMap.mp4](https://cdn.discordapp.com/attachments/565154703139405824/958827281021796353/SmallMap.mp4)
%figure
![SmallMap.png](https://cdn.discordapp.com/attachments/565154703139405824/958827281625804860/SmallMap.png)
%end



> **Attachment**: [debug1.zip](https://cdn.discordapp.com/attachments/565154703139405824/958827319957528586/debug1.zip)

##### moebius 03/30/2022 23:08:00
I was able to open the wbt file, but i can only see blank. can you not see this world? I could zoom into your blue robot and see that however. Why did you add the ball joint there, what purpose is it serving here?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/958865205817925652/unknown.png)
%end

##### Rekanice 03/30/2022 23:48:48
Beginner question: Is it possible to simulate/implement visual slam methods in webot? The slam module in webot seems to touch on lidar-based methods. I was wondering if using a camera is an option.

##### DrakerDG [Moderator] 03/31/2022 00:23:12
What version Webots you use?



I understand that the back slides, so I put a ball join to simulate that behavior

##### moebius 03/31/2022 00:24:12
2021B. Back slides? No no it's a simple 2 wheeled robot

##### DrakerDG [Moderator] 03/31/2022 00:26:19
The back part of your robot have contact with the floor, the simulation of webots calculate the friction effect


The friction has a braking effect in the robot, adding a ball join, this effect is avoided

##### moebius 03/31/2022 00:31:12
oh right got it. you added it to the tail. But in the video, both of them seemed to move in a straight line, mine rotated to face the target however before doing so


are the weird orientations because of the axis changes in 2022 version then?

##### DrakerDG [Moderator] 03/31/2022 00:33:04
Yes

##### moebius 03/31/2022 00:36:38
were you able to see that curving behavior then in the straight line motion? or do you might know why that might be happenening

##### DrakerDG [Moderator] 03/31/2022 01:18:24
In the beginning yes, but not always happen it

##### moebius 03/31/2022 04:52:14
that is very strange because simply setting the same  speedvalue for both wheels is giving that behavior

##### DrakerDG [Moderator] 03/31/2022 05:43:51
Maybe is the strange effect for the tail friction, this is my theory

##### StefDev 03/31/2022 07:16:00
Hey guys! I just wanted to thank you for open sourcing and helping me using Webots 🙂 

I chose Webots in my bachelor thesis for the simulation part of a ROS2 robot and now the company i work with and my university ([http://hshl.de](http://hshl.de)) are building on webots (they even changed there study plan to incorporate webots and robotic topics).

##### cnbarcelo 03/31/2022 14:57:10
Hi!

I was wondering if there's any way of creating an Ortographic camera on Webots (and take a picture of it)?


I've looked at the code, and I think the camera mode is forced to be always perspective, is that right?

##### Olivier Michel [Cyberbotics] 03/31/2022 15:01:12
Yes.

##### cnbarcelo 03/31/2022 15:08:13
I see.

Do you think would be valuable to have the possibility of switching modes/exposing projection matrix?

It would be useful to extract information from the environment more than used in a robot, so might be seen as a tool more than a sensor.

##### Olivier Michel [Cyberbotics] 03/31/2022 15:13:34
Sensor models in Webots should correspond to actual sensors. So, I don't it's a good idea to mix such a tool with a sensor model. I would rather implement a supervisor API function allowing to take a snapshot with an optional orthographic mode.

##### cnbarcelo 03/31/2022 15:15:05
That indeed feels like the right way to go. I'll create an issue and see if I can contribute with a proposal PR

##### Olivier Michel [Cyberbotics] 03/31/2022 15:17:12
You may want to try the following idea : modify the Viewpoint parameters from a supervisor and take a snapshot. Not sure it will fulfill all your requirements though.

##### cnbarcelo 03/31/2022 15:19:34
Basically, what I want is a cut at a sensor level of the whole environment, so I wanna be able to play with the clipping planes too. I'm not sure if that will work tho, but will run some experiments and if it fells useful, I'll share the results

##### Olivier Michel [Cyberbotics] 03/31/2022 15:19:44
You may need to add a couple of supervisor API functions to switch to orthographic mode and change the viewport resolution.

##### JoeyBadass 03/31/2022 17:47:32
is there a way to make the human sit down?



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/959146990158118912/unknown.png)
%end

##### barryman44 03/31/2022 19:46:01
I want to start using ROS with moveIT in webots for a robot arm. Does somebody have a good tutorial/guide for this?

##### Pilsley 03/31/2022 20:14:23
Hi guys, I am busy with making my own conveyer belt in webots. It's work very good, but I got a problem with the animatedGeometry function. I would like place a bounding box on the fins of the conveyer belt, but I don't know how. In the picture below you can see what is happening now. Does anyone has a solution? Thank you in advance!
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/959183903011254293/unknown.png)
%end

##### moebius 03/31/2022 20:19:48
maybe. The robot also keeps bouncing around, without settling when it encounters an obstacle. I set the coloumbfriction to 100 from 8 it still kept happening. changing the bounc velocity also did not seem to do much
> **Attachment**: [pprbotdebug2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/959185267099598979/pprbotdebug2.mp4)

##### JoeyBadass 03/31/2022 23:25:16
Guys i need serious help with this as i am trying to make something similar, on the sample world ( universal robot) how do they program the robot to pick up something and place it on the basket?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/959231940496588861/unknown.png)
%end

##### DrakerDG [Moderator] 03/31/2022 23:48:40
Every robot have a IR sensor in the grip, when detect an object, close the grip and make the movement to put it in the basket

##### JoeyBadass 03/31/2022 23:50:12
oh i already knew that, but i am talking about the code. I followed everything they did but mine isnt working



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/959238643959091301/unknown.png)
%end

##### DrakerDG [Moderator] 03/31/2022 23:52:09
Can you share your example, including your world and controller?

##### JoeyBadass 03/31/2022 23:53:35
These are it
> **Attachment**: [Official\_world\_Design.wbt](https://cdn.discordapp.com/attachments/565154703139405824/959239068812722176/Official_world_Design.wbt)
> **Attachment**: [ure\_controller.c](https://cdn.discordapp.com/attachments/565154703139405824/959239069014032444/ure_controller.c)

##### DrakerDG [Moderator] 03/31/2022 23:54:02
OK, I will check it

##### JoeyBadass 03/31/2022 23:54:16
Its just basically the same thing as the sample, but i dont understand why it isnt working or do i need to specify positions etc


Thanks

## April

##### DrakerDG [Moderator] 04/01/2022 00:24:39
You need to change some parameters of the IR sensor of the 3 robots. The current orientation of each IR sensor is incorrect.



You need change basicTimeStep to 8 too.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/959246885338636308/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/959246885703520337/unknown.png)
%end

##### JoeyBadass 04/01/2022 00:27:56
Ohh alright, so i should just change the distance sensor rotation for all of them to the current one you have shown on the right?

##### DrakerDG [Moderator] 04/01/2022 00:31:24
And you need move every plastic crate and the conveyor belt little bit near to the UR5e and UR10e arms
> **Attachment**: [Official\_world\_Design\_2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/959248583511654460/Official_world_Design_2.mp4)


Yes


look at this
> **Attachment**: [Official\_world\_Design\_3.mp4](https://cdn.discordapp.com/attachments/565154703139405824/959249743609991178/Official_world_Design_3.mp4)

##### JoeyBadass 04/01/2022 00:37:19
oh wow thanks, let me give the it a try and i will update you in a sec


`@DrakerDG` it gave me this error
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/959251424213082153/unknown.png)
%end

##### DrakerDG [Moderator] 04/01/2022 00:50:13
It is a warning.



 I'm trying to understand why your world is so slow, compared to the example. It's currently slow, but it works

##### JoeyBadass 04/01/2022 00:53:01

> **Attachment**: [8mb.video-M84-ZFDvEFyO.mp4](https://cdn.discordapp.com/attachments/565154703139405824/959254024396345405/8mb.video-M84-ZFDvEFyO.mp4)


i even decreased the step speed to 7 but still the robot didnt move and that error came up

##### DrakerDG [Moderator] 04/01/2022 00:57:43
Active Show DistanceSensor Rays please,  to see the correct position of every sensor
%figure
![2022-03-31.png](https://cdn.discordapp.com/attachments/565154703139405824/959255203150966784/2022-03-31.png)
%end
%figure
![Official_world_Design.png](https://cdn.discordapp.com/attachments/565154703139405824/959255204065329272/Official_world_Design.png)
%end

##### JoeyBadass 04/01/2022 01:02:57
Tried nothing is working



> **Attachment**: [8mb.video-QKI-EB88wqjx.mp4](https://cdn.discordapp.com/attachments/565154703139405824/959257248750788678/8mb.video-QKI-EB88wqjx.mp4)


Or would you want to hop on voice channel and i can share my screen?

##### DrakerDG [Moderator] 04/01/2022 01:10:15
Did you change translation and rotation parameters in all robots?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/959258359788371998/unknown.png)
%end

##### JoeyBadass 04/01/2022 01:15:09
Ohh no, i forgot the translation part


It works now but there is a problem with the other arm. it flips the can instead of dropping it down. Should i adjust the position of the cans and the arm itself



> **Attachment**: [8mb.video-MEY-W9vrYNTA.mp4](https://cdn.discordapp.com/attachments/565154703139405824/959262059688767488/8mb.video-MEY-W9vrYNTA.mp4)

##### DrakerDG [Moderator] 04/01/2022 01:26:58
Yes

##### JoeyBadass 04/01/2022 01:52:08
`@DrakerDG` Thank you very much Sir or Mrs for you help👍

##### DrakerDG [Moderator] 04/01/2022 01:52:41
Hahahahahaha, Sir

##### JoeyBadass 04/01/2022 01:52:50
`@DrakerDG` Also how did you know what translation and rotation to put or was it a good guess?

##### DrakerDG [Moderator] 04/01/2022 01:52:55
You welcome

##### JoeyBadass 04/01/2022 01:53:05
Oh alright, Sir indeed😂

##### DrakerDG [Moderator] 04/01/2022 02:22:46
Wrong position
%figure
![Official_world_Design_1.png](https://cdn.discordapp.com/attachments/565154703139405824/959276609507065897/Official_world_Design_1.png)
%end


Right position
%figure
![Official_world_Design_2.png](https://cdn.discordapp.com/attachments/565154703139405824/959276783004426250/Official_world_Design_2.png)
%end

##### JoeyBadass 04/01/2022 02:29:56
OHHh😂 😂

##### DrakerDG [Moderator] 04/01/2022 02:58:16
You can eliminate the TexturedBackgroundLight node to improve your simulation

##### JoeyBadass 04/01/2022 03:22:42
Would Darken it or would it be just fine?


`@DrakerDG` it doesnt look nice when turned off, I will leave it on. Sometimes we just got to sacrifice something😂

##### DrakerDG [Moderator] 04/01/2022 03:34:37
Check the example and try understand how to compensate the light effect without TextureBackgroundLight

##### JoeyBadass 04/01/2022 08:48:04
Ohh cool, i will check that out

##### American Patriot 04/03/2022 19:49:39
Does cyberbotix limited pay royalty for the use of real life robots from different companies in their simulation software??

##### moebius 04/04/2022 05:59:31
hi, if the robot is tumbling around, after toppling over while going over a hill/colliding with an object, what are the best physics parameters to change? I added a little bit of linear damping , and that seemed to help, but in general what other parameters should i change, if the general behavior seems unstable?

##### DrakerDG [Moderator] 04/04/2022 06:07:42
Can you share your world to check it out?

##### S4JJ4D 04/04/2022 09:19:35
you can add suspension springs to your vehicle. stiff springs improve stability


the geometric structure of the robot is also important. for example, lowering the center of mass of the system improves lateral stability.


or for example, if your robot is "wide", it is harder to topple it


adding inertia helps too, if that's a viable option

##### nelsondmmg 04/04/2022 13:49:23
Hello, I'm trying to use a controller with a vehicle but the execution keeps freezing at the first this->step(). I initialized all the sensors in the vehicle and there is only one robot in the simulation. Which could be the reason for this behavior? Thanks

##### S4JJ4D 04/04/2022 13:53:46
hard to tell without a model, but notice that when the simulation is paused, robot->step() does not return.

##### DDaniel [Cyberbotics] 04/04/2022 13:54:27
if it freezes typically it means you have an infinite loop, therefore it never moves to the next simulation step

##### nelsondmmg 04/04/2022 14:01:22
The simulation is not paused. Infinite loop where? Even if I use step at the beginning of the simulation without any sensors the simulation blocks itself. Am I forgetting some function to be called before the execution?

##### DDaniel [Cyberbotics] 04/04/2022 14:02:18
do you have a while loop somewhere? can you show the code?

##### nelsondmmg 04/04/2022 14:05:48
This is the function executed to initialize the controller. When I stop the execution at debug the execution is inside the step function
> **Attachment**: [initFunction.txt](https://cdn.discordapp.com/attachments/565154703139405824/960540697419718716/initFunction.txt)

##### Franky 04/04/2022 15:10:07
Hi, anyone know if there is any way to add a position sensor to a propeller? I've noticed the documentation says position sensors should be on joints and can't find a way to add to a propeller.


I need the angular velocity of a propeller for a control algorithm. Think i've narrowed it down to adding a gyro to the propellers slowHelix and fastHelix nodes. Only question I've got is will the gyro for slowHelix show the angular velocity when the propeller is in fastHelix, and vise-versa?

##### Pilsley 04/04/2022 19:56:42
Hi guys, I am busy with making my own conveyer belt in webots. It's work very good, but I got a problem with the animatedGeometry function. I would like place a bounding box on the fins of the conveyer belt, but I don't know how. In the picture below you can see what is happening now. Does anyone has a solution? Thank you in advance!



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/960629019768918097/unknown.png)
%end

##### moebius 04/05/2022 00:11:55
I have added the .wbt file and the accompanying video, if you could take a look, thanks.
> **Attachment**: [bot\_hill\_physics.mp4](https://cdn.discordapp.com/attachments/565154703139405824/960693232495366174/bot_hill_physics.mp4)
> **Attachment**: [physics\_debug.zip](https://cdn.discordapp.com/attachments/565154703139405824/960693232986128445/physics_debug.zip)


I think i will try to model the mass and damping value properly for now, rather than adding suspension springs, since that is not a part of the actual vehicle


thanks


It is initially running at faster than real time, and then i slow it down to real time

##### DrakerDG [Moderator] 04/05/2022 03:24:39
I didn't change anything, but it don't make the same thing like your video. Maybe you need try using webots R2022a
> **Attachment**: [target\_world.mp4](https://cdn.discordapp.com/attachments/565154703139405824/960741732419506226/target_world.mp4)



> **Attachment**: [target\_world\_1.mp4](https://cdn.discordapp.com/attachments/565154703139405824/960742534206865428/target_world_1.mp4)

##### nelsondmmg 04/05/2022 09:10:10
Hi, I'm having some problems executing controllers for vehicles. Each time that the step function is executed, the simulation freezes. When I stop the execution at debug level (since I'm executing the controller as extern) the instruction executed is always the step function. why this is happening? I try to execute the city sample world and the controller functions perfectly, both the autonomous\_vehicle and my custom controller. Am I forgetting something that needs to be done to not block the simulation at each step?
> **Attachment**: [main.cpp](https://cdn.discordapp.com/attachments/565154703139405824/960828689203990570/main.cpp)
> **Attachment**: [new\_dilemma\_onlyAV.wbt](https://cdn.discordapp.com/attachments/565154703139405824/960828689388564510/new_dilemma_onlyAV.wbt)

##### cnbarcelo 04/05/2022 12:49:35
Hi! I saw there's a PR to connect controllers from sibling docker containers, but also a piece of code in the codebase mentioning a Docker-type controller that is not documented. Connecting Webots with a controller running in a different container is already possible, or is that exactly what the PR addresses?

##### DDaniel [Cyberbotics] 04/05/2022 13:26:01
works fine for me, which version of Webots are you using? Are you sure it isn't the other controller (TestingAgentsLib") that isn't stalling the progress of the simulation? Try removing this controller, does the simulation progress?

##### Olivier Michel [Cyberbotics] 04/05/2022 13:27:31
This is not yet possible, we are working on it. But it will soon be possible (the PR may be complete later this week or so).


FYI, the PR at [https://github.com/cyberbotics/webots/pull/4344](https://github.com/cyberbotics/webots/pull/4344) currently contains a working setup to run Webots in a docker container and a sample robot controller in another docker controller. The only current limitation is that it doesn't yet support camera, lidar and range-finder devices, because these devices require shared memory communication.

##### Naxi 04/05/2022 16:41:38
Hi all, I'm having an issue when spawning robots from a proto file in the /protos folder. I'm copying proto files from another package into the simulation directory in runtime and then importing a node with "MyRobot {}" with a supervisor, but I'm getting an "Unknown PROTO" error. 

Do the proto files have to be inside the /protos folder before the initialization of the supervisor?

##### moebius 04/05/2022 16:54:30
oh wait but you don't seem to have the hill. I want to fix the physics when it tries to go up the hill

##### DrakerDG [Moderator] 04/06/2022 02:25:55
The example did you share don't have any hill. Sorry

##### Redéco 04/06/2022 10:13:52
Hi everyone, I have launched webots with --log-performance but I don't find any documentation on the signifcation of the parameters. 

Could you please tell me what does

stepsCount / loading / prePhysics / physics / postPhysics / mainRendering 

mean ?


Just to be clear, I'm talking about this
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/961271632444411904/unknown.png)
%end

##### moebius 04/06/2022 19:29:10
oh okay :/ thanks


maybe the version mismatch is causing that problem

##### DrakerDG [Moderator] 04/06/2022 19:29:55
It is possible

##### Folafoyeg 04/06/2022 20:21:47
Hello all! Pls which moveable robot with a gripper is ready available on webots to use?

##### DDaniel [Cyberbotics] 04/06/2022 21:40:39
Here's a list of all included robots: [https://www.cyberbotics.com/doc/guide/robots](https://www.cyberbotics.com/doc/guide/robots)

Youbot fits the bill, you can search for it in the samples under `file > open sample world`


Those metrics tell you how much time you've spent loading the world, executing the physics step, generating the 3D rendering etc (what's exactly behind each is complicated and mostly for developers). I think a better question is, what are you looking for?

##### Spy Guy 04/06/2022 23:21:14
I'm trying to use Python with OpenCV using the Webots camera. When I run my code with my timestep loop like this:
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/961405252672848042/unknown.png)
%end


It crashes with no useful error message:
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/961405459326197770/unknown.png)
%end


Never mind, I was able to use getImage() instead, not sure why that wasn't working though

##### aj2001 04/07/2022 02:15:24
hi, i'm trying to make a robot for my engineering class and i'm having trouble with hinge joints. they always fall to gravity even if i explicitly apply force, torque, and/or velocity to them. i've also tried changing around the physics values, locking parts, etc.

sometimes i get it to move but it just bounces up and down, probably because it can't fully lift, or it lifts too much and spins out of control. any help would be appreciated.

here's my webots file
> **Attachment**: [simulation.wbt](https://cdn.discordapp.com/attachments/565154703139405824/961449085276155944/simulation.wbt)


the tutorials my professor gave us mentioned nothing about this so i would really appreciate any help

##### Redéco 04/07/2022 04:52:50
Thank you, I mainly wanted to be sure if the time loding the world was "global" and not per solid or something like that

##### moebius 04/07/2022 05:20:03
how do i view more verbose webots error and warning message (basically change the log level it prints to the console), but via command line, not any gui action

##### DrakerDG [Moderator] 04/07/2022 06:04:47
Hello! I see some improvements you can make to the structure of your robot. My advice is to use bounding objects made with simple shapes like cylinders and boxes. Avoid using the original mesh of your 3D design. Additionally, your robot is of considerable dimensions, so you will need enough touch to move each part of your robot. Take a look at this example whose boundign object is made of some cylinders and boxes only.
%figure
![Line_Follower_Robot_V1.png](https://cdn.discordapp.com/attachments/565154703139405824/961506810152501268/Line_Follower_Robot_V1.png)
%end

##### Kuroson 04/07/2022 07:21:04
Hello newbie at pretty much everything here.

I made a 170mm diameter circle with 4mm height in Fusion360. I tried exporting this design into an STL file to be imported into Webots via the "Import 3D Model". After importing the STL file, the object appears massive.



Is there a workaround for this?

##### DrakerDG [Moderator] 04/07/2022 07:23:03
You can change the scale of your solid. for example from 1 to to 0.01

##### Kuroson 04/07/2022 07:23:24
So theres no way to import the shape with correct scale?

##### DDaniel [Cyberbotics] 04/07/2022 07:24:00
when exporting it, make sure the units are in meters

##### Folafoyeg 04/07/2022 07:25:06
Thanks a lot Daniel.

##### Kuroson 04/07/2022 07:27:58
Ah yes! Thanks so much, setting a unit on the STL file solved this issue.

##### moebius 04/07/2022 15:54:53
the simulation simply stops sometimes, even though the controller is still running, and the simulation time is advancing. Any particular reason this might be happening?

##### aj2001 04/08/2022 03:36:40
ok so i'm still having issues with gravity. i've taken drake's advice and simplified my bounding objects to primitives but my motors still don't want to work


i got one to work by making the damping value really big until gravity didn't affect it and then using a big forec multiplier


the other one though just wants to swing around, if i do the damping trick then my robot destroys itself


it does work with gravity off though, i tried turning physics off for it and then it wouldn't move at all even if i applied force to it


i want the bucket on the end of the arms to rotate with a rotational motor
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/961832588048236554/unknown.png)
%end


the entire robot works with gravity disabled. i can just disable it but i need it enabled as this is a group project and we need to test it going over bumpy surfaces

##### DrakerDG [Moderator] 04/08/2022 04:45:59
Hi!, I understand that the every cylinder like a wheel can turn to left and right or not?

##### aj2001 04/08/2022 04:48:57
there's a second motor above each wheel that rotates the wheels around the Z axis

##### DrakerDG [Moderator] 04/08/2022 04:50:15
Do you know the approximate mass of your robot in kg?

##### aj2001 04/08/2022 04:51:44
the robot is a theoretical robot we had to come up with. it doesn't have a real mass. i was using the default masses that came from the density values before but i set all the part masses to 1kg because the robot kept sinking into the floor


it is 12kg

##### DrakerDG [Moderator] 04/08/2022 04:52:58
Ok

##### aj2001 04/08/2022 04:54:06
here is my issue
> **Attachment**: [Screencast\_from\_04-08-2022\_125342\_AM.webm](https://cdn.discordapp.com/attachments/565154703139405824/961851410247475260/Screencast_from_04-08-2022_125342_AM.webm)

##### DrakerDG [Moderator] 04/08/2022 04:58:27
Yes, I see it.

I making a clone to test the structure tree
> **Attachment**: [simulation\_V2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/961852504528801812/simulation_V2.mp4)

##### aj2001 04/08/2022 04:59:00
okay

##### DrakerDG [Moderator] 04/08/2022 04:59:16
Give me some minutes

##### aj2001 04/08/2022 05:11:24
here is my updated design if you need it, i changed the wheels so the steering mechanism moves the wheels
> **Attachment**: [simulation.wbt](https://cdn.discordapp.com/attachments/565154703139405824/961855762676723743/simulation.wbt)

##### DrakerDG [Moderator] 04/08/2022 07:10:24
Only need a little adjust in the mass for some parts and will work well
> **Attachment**: [simulation\_V2\_1.mp4](https://cdn.discordapp.com/attachments/565154703139405824/961885710883905636/simulation_V2_1.mp4)

##### aj2001 04/08/2022 07:21:29
awesome, thanks for the help

##### DrakerDG [Moderator] 04/08/2022 07:23:28
I found in your configuration some bounding object and physics that they not necessary


Direction and displacement test
> **Attachment**: [simulation\_V2\_2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/961891494208348180/simulation_V2_2.mp4)

##### aj2001 04/08/2022 07:41:49
what exactly did you do the mass values?

##### DrakerDG [Moderator] 04/08/2022 07:49:01
Arm and bucket test
> **Attachment**: [simulation\_V2\_3.mp4](https://cdn.discordapp.com/attachments/565154703139405824/961895427706335252/simulation_V2_3.mp4)


All together
> **Attachment**: [simulation\_V2\_4.mp4](https://cdn.discordapp.com/attachments/565154703139405824/961898859133542430/simulation_V2_4.mp4)


Body: 10 kg

Steering: 1 kg x 2

Wheel: 3 kg x 2

Arm: 1 kg

Bucket: 1 kg



Total: 20 kg  at this moment


If the mass is defined in kg, it is necessary to set the density value to -1 and vice versa
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/961910510952001576/unknown.png)
%end

##### aj2001 04/08/2022 08:53:42
i got the arm working correctly but when i edit the bucket's physics it drags the arm down with it


the endpoints have physics modules but the child solids do not


or the bucket falls off


i followed this, my bucket joint looks like joint < bucket solid < shape, and the bucket solid has physics


the part masses are the same as you listed

##### DrakerDG [Moderator] 04/08/2022 09:26:11

> **Attachment**: [simulation\_V2.zip](https://cdn.discordapp.com/attachments/565154703139405824/961919882662920212/simulation_V2.zip)



> **Attachment**: [simulation\_V2\_5.mp4](https://cdn.discordapp.com/attachments/565154703139405824/961920465805398016/simulation_V2_5.mp4)

##### aj2001 04/08/2022 09:35:18
thank you

##### áçè 04/08/2022 09:53:25
HELLO CAN SOMEBODY PLZ TELL ME HOW TO GIVE GEAR MOTION FOR PLANETARY GEAR AS SHOWN BELOW



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/961926753348108318/unknown.png)
%end


HOW TO CONNECT THEM


SO THAT MOVING SUN GEAR WILL MOVE PLANET GEAR


ANYONE ?


ALSO IN MY WEBOT THERE IS NO GEAR NODE PRESENT

##### DDaniel [Cyberbotics] 04/08/2022 10:37:17
Please stop with the caps. And yes, there's no gears like that in webots but you can model the behavior using coupled motors [https://cyberbotics.com/doc/reference/motor#coupled-motors](https://cyberbotics.com/doc/reference/motor#coupled-motors)

##### áçè 04/08/2022 10:43:17
the thing i m not seeing gear node in my webot
%figure
![Screenshot_2022-04-08_161251.png](https://cdn.discordapp.com/attachments/565154703139405824/961939283181207572/Screenshot_2022-04-08_161251.png)
%end


as u can see after garden there is no gear


<@787796043987025941> ?

##### DDaniel [Cyberbotics] 04/08/2022 11:46:31
It's available only in R2022a, but I don't think it will work for what you're trying to do

##### áçè 04/08/2022 11:55:07
ok thank you sir

##### moebius 04/09/2022 00:58:20
is there any way to put a torsional spring in hingejoint? It looks like the suspension spring field only allows for linear springs

##### MMS 04/09/2022 10:44:47
gps: The process crashed some time after starting successfully.

WARNING: 'gps' controller crashed.



i m getting this

##### áçè 04/10/2022 03:31:22
i have a question in gear train mechanism how should i move my passive gear by my active gear motion .... is it done in controller ?


plz help


anyone?

##### DrakerDG [Moderator] 04/10/2022 07:43:35
The name of the coupled motors must use the following syntax:



* Using double colon between the main name and specific name

* All coupled motors must be use the same main name too



Active motor name (The motor that it call from code): 

motor\_X::active



Pasive motor(s) name(s) (Don't need call it (they) from code):

motor\_X::pasive1

motor\_X::pasive2

.

.

motor\_X::pasiveN

##### áçè 04/10/2022 07:46:23
Thanx sir

##### DrakerDG [Moderator] 04/10/2022 08:01:14
The multiplier motor must change sign between gears to obtain opposite turns.


If use gears of different diameter, the value of the multiplier must be the necessary to obtain the relation speed needed



> **Attachment**: [Planetary\_gears\_1.mp4](https://cdn.discordapp.com/attachments/565154703139405824/962624743519977492/Planetary_gears_1.mp4)


This is basic example:
> **Attachment**: [Planetary\_gears\_sample.zip](https://cdn.discordapp.com/attachments/565154703139405824/962625472091553792/Planetary_gears_sample.zip)



> **Attachment**: [Planetary\_gears\_and\_wheels.mp4](https://cdn.discordapp.com/attachments/565154703139405824/962626896703680562/Planetary_gears_and_wheels.mp4)


this a sample adding wheels, only
> **Attachment**: [Planetary\_gears\_and\_wheels\_V1.zip](https://cdn.discordapp.com/attachments/565154703139405824/962627043256852550/Planetary_gears_and_wheels_V1.zip)

##### Franky 04/11/2022 14:48:38
Anyone know how to set the inertia of a propeller? Is it fast and slow helix solids or are those solids only for graphical representation?

##### DrakerDG [Moderator] 04/12/2022 01:40:20
The solid of fastHelix and slowHelix define the graphical representation according to angular velocity of motors, only I think, but I not sure

##### vuwij 04/13/2022 01:34:47
Is there any we you can run webots without the gui?


for exmaple --no-rendering just disables the rendering without disabling the gui completely

##### DrakerDG [Moderator] 04/13/2022 01:45:36
Try setting in preferences options and check this document: [https://www.cyberbotics.com/doc/guide/preferences#opengl](https://www.cyberbotics.com/doc/guide/preferences#opengl)
%figure
![2022-04-12_1.png](https://cdn.discordapp.com/attachments/565154703139405824/963615913863708732/2022-04-12_1.png)
%end

##### vuwij 04/13/2022 01:48:02
not sure which setting here you mean disable ambient occulusion completely? I want to use webots in a CI setting where there is no GUI needed

##### DrakerDG [Moderator] 04/13/2022 01:49:42
Ok

##### vuwij 04/13/2022 02:01:33
world using the web simulation streaming server help?

[https://cyberbotics.com/doc/guide/web-simulation#streaming-server](https://cyberbotics.com/doc/guide/web-simulation#streaming-server)

##### aysegul ucar 04/13/2022 17:19:16
Do you have ant idea? İfconfig or another solutşon?

##### AlexandrosNic 04/14/2022 08:29:12
Is there any way to change the default joint angles of a robot when it is spawning in webots? (without having to do it through script)

##### DDaniel [Cyberbotics] 04/14/2022 08:33:52
In theory you can create the PROTO such that it exposes the joint angles as parameters. That way based on which parameters you provide, it will load in a specific configuration. In practice however most existing PROTO do not expose them. What you can still do is use the supervisor's `wb_supervisor_node_set_joint_position` ([https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_set\_joint\_position](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_set_joint_position))  function to set an arbitrary position from your controller as it starts

##### AlexandrosNic 04/14/2022 11:30:10
Another question. The way I create custom proto files currently is by:

1. using the "Import 3D model" and importing the custom stl file I want to use,

2. Saving the world, open the wbt file and identify the part I want to use

3. Copy paste the part in a new file which I will save as .proto (by adding the corresponding proto definition)

4. Save it in the proto folder.



However when I do this (and added Physics), the objects even though they are rendered correctly, they seem to act like crazy, flying around in the scene. Does anyone know what causes this problem?

##### DDaniel [Cyberbotics] 04/14/2022 11:40:14
The importer translates what it decodes from the file in a very naive way, so often it ends up adding a lot of intermediary solids for the purpose of adding a rotation or translation. So if you go down that route, either change these intermediary solids to Transform nodes or you might need to ensure that ALL of them have a Physics node (having a mix of physical and non-physical stuff usually causes what you describe)


Alternatively you can disable the "import as solid" option and just do the opposite of converting the transforms to solids of the nodes you care about (and adding the physics of course)

##### AlexandrosNic 04/14/2022 11:52:33
changing them to transform worked. All of them having Physics node didn't. Thank you Daniel!

##### giaco\_mz 04/14/2022 17:11:28
everytime i open a simulation it starts , There is a way to avoid it? I want start it manually.

##### DDaniel [Cyberbotics] 04/14/2022 17:11:44
you can disable it in the preferences

##### giaco\_mz 04/15/2022 09:03:32
<@787796043987025941> thanks

##### TheLobbit 04/15/2022 20:51:54
I am trying to have a bot scoop up a sphere, but there is a "ghost" of the arms that is pushing against the sphere, not allowing it to go into the robot. Is there a way to make the "ghost" disappear?(image below)



%figure
![Screenshot_2022-04-15_164941.jpg](https://cdn.discordapp.com/attachments/565154703139405824/964629218459811840/Screenshot_2022-04-15_164941.jpg)
%end

##### DrakerDG [Moderator] 04/15/2022 21:27:32
Hi! You need put the bounding object (ghost) in the bonding object of the solid in the hinge join where you have the black shape that your robot move
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/964638128369442826/unknown.png)
%end


My advice is don't use the original mesh to take a bounding object, it's better use simple shapes like boxes, cylinders, spheres, etc.

##### TheLobbit 04/15/2022 21:31:10
ty

##### DrakerDG [Moderator] 04/15/2022 21:46:26
You welcome!

##### Oamlair 04/16/2022 09:33:05
is there a method I can use for python which returns all objects in the world, and a method I can use for a robot which returns its rotation values

##### PrinceVickster06 04/17/2022 22:30:11
Hi there I need help with coding on how to make another robot detect another robot


In python


Do you have any solutions for that

##### Mat198 04/17/2022 23:03:13
Detect in which way? There are several ways to do that. Explain better and I'm sure there is a exemple you can copy :)

##### jmshin 04/19/2022 03:29:40
Hi everybody! I have a question.

Is there any solution convert 2d map to webots world?

Because i'm using ros now, want to gmapping real world and then make it to webots world.

##### goch [Moderator] 04/19/2022 06:38:00
hey. Does the webots api provide events? I'm looking for a way to detect if a new object was added and if the simulation was reset.

##### Olivier Michel [Cyberbotics] 04/19/2022 06:41:41
What do you mean by 2D map? An image file? And how would you like to convert it, like adding walls in a world file corresponding to black pixels in the image?


No, there is not such API. But if these reset and object adding are done from a supervisor process, it can also notify the robot controllers using an Emitter / Receiver system.

##### goch [Moderator] 04/19/2022 06:49:27
I guess a Robot Window for a Custom UI that allows to reset and add Objects via supervisor is the way then. Thank You.

##### jmshin 04/19/2022 06:51:49
From .png or .pgm file to .wbt convert. and any way is okay 1 black pixel match to 1 box.

##### Olivier Michel [Cyberbotics] 04/19/2022 06:52:40
There is no such tool, but it should be fairly easy to develop a python script doing this.

##### sEngBots 04/19/2022 09:04:14
I have loaded a world model that I built 4 month sago on the Webots version on Ubuntu 18.04  into a new 2022a Webots and the order of coordinates for all object sizes have changed in this new version of Webots on Ubuntu 20.04. Do I have to manually change the object sizes for all of my 100 objects in my world model? Looked at Preferences settings... is there a setting where the order of coordinates can be fixed for all parts of a  .wbt definition ?

##### Benjamin Délèze [Cyberbotics] 04/19/2022 09:14:15
Hi, you can follow this guide: [https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a](https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a)

You will find a script that can help you and some instructions

##### sEngBots 04/19/2022 09:27:12
Thanks ! Looks like I will go with the manual change of coordinates. No major problems with that.


Just to comment on my experience with rotating objects manually in the new Webots 2022a: I find it more difficult to rotate around the main axis than in the previous version, I do not see why this change had to be made,  could you please clarify? I am making a total mess now when I  touch an object with the intention to rotate it - very time consuming to correct it.

##### MysticalFire 04/19/2022 10:35:00
Hi all, can someone please help explain why my simulation runs differently after resetting the world? I haven't touched anything (save button won't let me save anything) between runs, although I do notice a warning at the bottom saying *"WARNING: Robot > DEF LEVER\_MECHANISM Transform > HingeJoint > Solid: As 'physics' is set to NULL, collisions will have no effect"*. A method to fix this would also be appreciated. Thanks!
> **Attachment**: [C\_\_Users\_Joel\_UNSW\_z5418566\_OneDrive\_-\_UNSW\_Documents\_SD\_Project\_Webots\_Files\_worlds\_test.wbt\_Webots\_Files\_-\_Webots\_R2022a\_2022-04-19\_20-20-16\_2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/965923463674085406/C__Users_Joel_UNSW_z5418566_OneDrive_-_UNSW_Documents_SD_Project_Webots_Files_worlds_test.wbt_Webots_Files_-_Webots_R2022a_2022-04-19_20-20-16_2.mp4)

##### DrakerDG [Moderator] 04/19/2022 11:27:59
You need add physic node in the solid of the end point of the hinge join

##### MysticalFire 04/19/2022 11:28:29
There's another error that pops up when I do though. Something about the mass being too light.


Then the robot just disappears when the simulation is run


*"WARNING: Robot > DEF LEVER\_MECHANISM Transform > HingeJoint > Solid: Webots has detected that this solid is light and oblong according to its inertia matrix. This belongs in the physics edge cases, and can imply weird physical results. Increasing the weight of the object or reducing its eccentricity are recommended."*

##### DrakerDG [Moderator] 04/19/2022 11:32:15
You need save your world before you run and reset it

##### MysticalFire 04/19/2022 11:33:36
ss before and after it's run (saved and reloaded the world)
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/965938211413172234/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/965938211744526416/unknown.png)
%end

##### DrakerDG [Moderator] 04/19/2022 11:36:14
your robot fell

##### AlexandrosNic 04/19/2022 11:36:53
Webots is recommended for Windows 8 and 10, however it still runs flawlessly after upgrading to Windows 11. Are there any reported issues with Webots on Windows 11?

##### MysticalFire 04/19/2022 11:37:46
How would I make it not fall then?


Increasing the weight in the hingejoint's solid makes the whole robot do a jump at the start.

##### DrakerDG [Moderator] 04/19/2022 11:49:12
Try reducing the basicTimeStep in the WorldInfo node

##### MysticalFire 04/19/2022 11:57:08
Hmm. The lever is no longer moving though.

##### DrakerDG [Moderator] 04/19/2022 12:14:33
What is the weight of your lever now?

##### MysticalFire 04/19/2022 12:17:26
1


Anything less than 1 and it returns this warning

##### DrakerDG [Moderator] 04/19/2022 12:19:51
1 kg of mass?

##### MysticalFire 04/19/2022 12:21:02
Yep

##### DrakerDG [Moderator] 04/19/2022 12:21:35
With density in - 1, right?

##### MysticalFire 04/19/2022 12:23:30
Yes

##### DrakerDG [Moderator] 04/19/2022 12:24:31
What are the dimensions of your lever?

##### MysticalFire 04/19/2022 12:25:47
x=0.01, y=0.01, z=0.055


It's a box with those dimensions ^

##### DrakerDG [Moderator] 04/19/2022 12:29:41
Try increasing the torque of the motor of the hinge joint

##### MysticalFire 04/19/2022 12:34:53
Robot's gone again... This time into space 😭
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/965953635643179028/unknown.png)
%end

##### DrakerDG [Moderator] 04/19/2022 12:36:25
If is possible, share your world to check it

##### MysticalFire 04/19/2022 12:37:47
Would this work?
> **Attachment**: [test.wbt](https://cdn.discordapp.com/attachments/565154703139405824/965954367310164048/test.wbt)

##### DrakerDG [Moderator] 04/19/2022 14:20:13
Your robot does not fall when running the simulator as the ball does, it is necessary to complete the configuration by adding physics nodes in each solid that has a bounding object, such as the chassis, the wheels, and the lever. By the way, the exact same robot does not appear to me, wheels do not appear for example
> **Attachment**: [test.mp4](https://cdn.discordapp.com/attachments/565154703139405824/965980143422177340/test.mp4)

##### MysticalFire 04/19/2022 14:24:01
Ah, that's probably because I used a PROTO for the wheel (and appearance of some of the parts), which webots couldn't pick up on your end.

##### DrakerDG [Moderator] 04/19/2022 14:25:03
I will try to fix it

##### MysticalFire 04/19/2022 14:25:46
Here's are the protos used if it's any help
> **Attachment**: [YellowWheel.proto](https://cdn.discordapp.com/attachments/565154703139405824/965981539898912888/YellowWheel.proto)



> **Attachment**: [WoodAppearance.proto](https://cdn.discordapp.com/attachments/565154703139405824/965981615106953287/WoodAppearance.proto)


Ok I've gone in and added physics nodes to all solids with bounding objects. The robot still doesn't drop to the ground, and the wheels are now moving in a strange way. Not sure where I went wrong. (Also converted the wheels into solid nodes)
> **Attachment**: [test.wbt](https://cdn.discordapp.com/attachments/565154703139405824/965987266713501706/test.wbt)

##### DrakerDG [Moderator] 04/19/2022 14:49:23
You need have a correct structure, I working in it

##### Naxi 04/19/2022 15:53:53
You can check map2gazebo package, generate a mesh, create a solid with its shape geometry to be mesh, and pass the url to the generated file

[https://github.com/shilohc/map2gazebo](https://github.com/shilohc/map2gazebo)

##### vuwij 04/19/2022 16:00:02
Not the best question but is it possible for a world to have multiple supervisors? I want to have a seperate file other than the main supervisor to get some information about the world


answered question myself, yes you can

##### DrakerDG [Moderator] 04/19/2022 17:37:40
Check this version, please
> **Attachment**: [test01.wbt](https://cdn.discordapp.com/attachments/565154703139405824/966029828555956304/test01.wbt)
> **Attachment**: [test01.mp4](https://cdn.discordapp.com/attachments/565154703139405824/966029828916650034/test01.mp4)
> **Attachment**: [test01\_1.mp4](https://cdn.discordapp.com/attachments/565154703139405824/966029829759713361/test01_1.mp4)
%figure
![test01.png](https://cdn.discordapp.com/attachments/565154703139405824/966029830351097876/test01.png)
%end

##### Reyk 04/19/2022 23:17:10
Hey, i'm currently struggling with the build in feature of differential wheels. As far as i understood: two hingejoints located on the same axle and anchors on this axle are automatically interpreted as differential wheel. This happens by default and leads to the problem that both wheels always turn with the same velocity. (As long as the velocity is set and not the position) For my project its essential to turn every wheel on its own even if all conditions for the differential wheel are fulfilled.

Now, my question is if there is a way to disable this feature for specific robots or worlds.

If the solution is already included in your documentation i'm sorry to bother you here. I did take a look in there and found nothing

##### MysticalFire 04/20/2022 02:30:56
Thanks so much for the help! Everything's staying in one piece when moving now and the motor hits the ball correctly, but the whole robot is still flying into the air whenever I run the lever's motor in directions one after the other (it only flies once the command to reverse the motor's direction is executed though). Here are some videos showing those cases.
> **Attachment**: [test01.mp4](https://cdn.discordapp.com/attachments/565154703139405824/966164034556682300/test01.mp4)
> **Attachment**: [test02.mp4](https://cdn.discordapp.com/attachments/565154703139405824/966164034971902002/test02.mp4)
> **Attachment**: [test03.mp4](https://cdn.discordapp.com/attachments/565154703139405824/966164035538128926/test03.mp4)


Would it have anything to do with the code's `set_position` function executing `set_position(INFINITY)`?

##### DrakerDG [Moderator] 04/20/2022 02:43:18
You don't need this command for short movements

##### MysticalFire 04/20/2022 02:43:38
What would I use then?


Just set velocity?

##### DrakerDG [Moderator] 04/20/2022 02:44:35
Just you need set position in some angle. set\_postition(angle)


Like an arm or somthing like that


The steering, arm and bucket, are moved only with set\_position command.  You only need know two angles in radians for every one:



[https://discordapp.com/channels/565154702715518986/565154703139405824/961920467231453184](https://discordapp.com/channels/565154702715518986/565154703139405824/961920467231453184)

##### MysticalFire 04/20/2022 02:57:25
Ah I see.

##### áçè 04/20/2022 04:47:28
in this i need the planetary wheels to rotate on its own axis while coming down what should i do  , and also i want to make my wheelchair to move in reverse direction what should i make changes in my controller
> **Attachment**: [empty\_2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/966198395393949757/empty_2.mp4)


\#include <stdio.h>

\#include <webots/robot.h>

\#include <webots/motor.h>



int timestep = 0;

int left\_speed = 2;

int right\_speed = 2;  

int turn = 0;



// Motors

WbDeviceTag left\_motor, right\_motor;



int main(int argc, char **argv) {

  wb\_robot\_init();



  // Initalize time step

  timestep = wb\_robot\_get\_basic\_time\_step();



  // Intialize motors

  left\_motor = wb\_robot\_get\_device("motor\_BL::left");

  right\_motor = wb\_robot\_get\_device("motor\_BR::right");

  wb\_motor\_set\_position(left\_motor, INFINITY);

  wb\_motor\_set\_position(right\_motor,INFINITY);

  wb\_motor\_set\_velocity(left\_motor, left\_speed);

  wb\_motor\_set\_velocity(right\_motor, right\_speed);



  // main loop

  while (wb\_robot\_step(timestep) != -1) {

  

    // if (turn > 300) {

      // left\_speed = -left\_speed;

      // right\_speed = -right\_speed;

      wb\_motor\_set\_velocity(left\_motor, left\_speed);

      wb\_motor\_set\_velocity(right\_motor, right\_speed);

      // turn = 0;

    // }

    // turn += 1;

    

  };



  // wb\_robot\_cleanup();

  // return 0;

}


anybody??


Does anybody can help  me out

##### AlexandrosNic 04/20/2022 07:27:13
Even though I know it is not recommended to run Webots without GPU, is there anyone that found/created a dockerfile for Webots without (nvidia) GPU support?

##### Rico Schillings[Sweaty] [Moderator] 04/20/2022 07:33:47
There is no specific configuration required to run without GPU. For example if you install webots in a VM, it runs per default without GPU.

##### AlexandrosNic 04/20/2022 07:36:16
But I cannot install it using the official dockerfile right? since it also installs many dependencies for nvidia and cuda

##### Rico Schillings[Sweaty] [Moderator] 04/20/2022 07:41:40
You can easily use a base docker-image (e.g. ubuntu/ros or what you want) and install webots in the container. Its the same like doing it on your native system.

##### AlexandrosNic 04/20/2022 07:52:05
oh yeah right! thank you `@Rico Schillings[Sweaty]`

##### Reyk 04/20/2022 12:04:28
Can someone help me with this pls?

##### vuwij 04/20/2022 16:13:49
is there any way to change the default camera image displayed size?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/966371118212603905/unknown.png)
%end

##### Rico Schillings[Sweaty] [Moderator] 04/20/2022 16:27:25
The displayed size correspondent with the resolution of your camera (i mean..),so for example if you use a camera resolution of 100x100 then the displayed size would be a small square. By double clicking you also can open it in a separate view and not as overlay in the world view

##### vuwij 04/20/2022 18:16:47
Can I start of the simulation with the camera in a different view?

##### moebius 04/20/2022 19:10:56
I upgraded to 2022a, and i am using the compass get bearing formula given in the website. For the first photo where you can see the coord system of the bot, the bearing angle it gives is 90 degrees. In the second photo you can see the global coord system. In the third photo the compass angle it gives is 0.14 degrees. 

The compass coord system is aligned to the robot coord system, how is the value of the north vector so off like this? In the first photo, should it not be 0 degrees, since the x-axis of the robot points the same way as the global x axis
%figure
![Screenshot_from_2022-04-20_12-04-14.png](https://cdn.discordapp.com/attachments/565154703139405824/966415689059532860/Screenshot_from_2022-04-20_12-04-14.png)
%end
%figure
![Screenshot_from_2022-04-20_12-07-41.png](https://cdn.discordapp.com/attachments/565154703139405824/966415689432842370/Screenshot_from_2022-04-20_12-07-41.png)
%end
%figure
![Screenshot_from_2022-04-20_12-09-01.png](https://cdn.discordapp.com/attachments/565154703139405824/966415689831317504/Screenshot_from_2022-04-20_12-09-01.png)
%end


Coordinate system is ENU, using this exact function given on the website 

```
double get_bearing_in_degrees() {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}
```

##### Rico Schillings[Sweaty] [Moderator] 04/20/2022 19:17:11
I'm not really sure since i quite rarely use the camera overlay (i use ros2 and often have rqt/rviz opened..) but it could be possible that its saved automaticly in the ui config when closing webots to reload your settings the next time. Would be in the same procedure where things like optional renderers are stored/reloaded (there it also stores if you want the camera overlay active). Just give it a try


Youre 100% sure that the compass is correct rotated in your proto (so without relative rotation to your root robot)? Sounds like the compass is rotated.



Alternatively to ensure there is nothing wrong with the calculation of the compass bearing, you could just add a single compass to the world (May need a wrapper to add the sensor) and check it again. If this brings correct values, you know that you have a rotation in your proto between robot and compass..



Upgrading to 2022a often brings some rotation "bugs" caused by the changed coord system :)

##### moebius 04/20/2022 19:36:08
pretty sure
%figure
![Screenshot_from_2022-04-20_12-35-27.png](https://cdn.discordapp.com/attachments/565154703139405824/966422035184091167/Screenshot_from_2022-04-20_12-35-27.png)
%end


also another weird thing is is if i use the north[1] instead, that gives a 0 to 360 range of values when i rotate it, that using north[2] as given in the function doesn't

##### Rico Schillings[Sweaty] [Moderator] 04/20/2022 19:47:31
Ah than May i guess the provided code in the documentation is not updated. 0+2 was north+east (x+z) in the past cause it was NUE (Y up). Now in 2022a it is ENU, so it would be `atan2(north[1],north[0])`. Could you check the values with this (i have no option to check it at the moment..)?



This would also explain why it is rotated by 90°, since you switched xy in the calculation (0,1 instead of 1,0).

##### moebius 04/20/2022 20:12:23
yes that seems to work. This specifically:

```
double get_bearing_in_degrees() {
  const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[1], north[0]);
  double bearing = (rad) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}
```


thank you!

##### Rico Schillings[Sweaty] [Moderator] 04/20/2022 20:15:32
No problem :) Than we just need to report this "issue" in the documentation. I've already found the commit where the description text was updated to ENU but the code wasnt updated..

##### Guest\_AMET 04/20/2022 20:26:23
Can Webots cross-compile to ARM?

##### Yashraj 04/20/2022 20:51:51
guys , every time I try to open the sample sumo example 

WARNING: SumoInterface (PROTO) > TrafficLightBigPole (PROTO) > TrafficLightHorizontal (PROTO): 'name' field value should be unique: 'horizontal traffic light' already used by a sibling Solid node.

WARNING: SumoInterface (PROTO) > TrafficLightBigPole (PROTO) > TrafficLightHorizontal (PROTO): 'name' field value should be unique: 'horizontal traffic light' already used by a sibling Solid node.

INFO: sumo\_supervisor: Starting controller: python.exe -u sumo\_supervisor.py --no-netconvert --max-vehicles=100 --port=8873 --seed=1 --step=200 --radius=-1 --maximum-lateral-speed=2.5 --maximum-angular-speed=3 --lane-change-delay=3

Traceback (most recent call last):

  File "sumo\_supervisor.py", line 17, in <module>

    from SumoSupervisor import SumoSupervisor

  File "D:\Pillai college\4th Year\Final Year Project\Webots\projects\default\controllers\sumo\_supervisor\SumoSupervisor.py", line 17, in <module>

    from controller import Sumo\_Supervisor, Node

  File "D:\Pillai college\4th Year\Final Year Project\Webots\lib\controller\python37\controller.py", line 31, in <module>

    import \_controller

ImportError: DLL load failed: The specified module could not be found.

WARNING: 'sumo\_supervisor' controller exited with status: 1.this comes
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/966441090616021122/unknown.png)
%end


Can anyone help , I have Final year project submission in 10 days .

##### ~E 04/20/2022 21:15:42
Has anyone here successfully imported a solidworks model into WB and got their bot running? I'm having a lot of trouble importing parts, and am not sure if there is a good tutorial or documentation of the ***right*** way to do it

##### ian.lau 04/21/2022 00:51:27
Can anyone help, I can't launch webots anymore. I have reinstalled again, but it can't help.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/966501388785950760/unknown.png)
%end


btw, the windows version is 11

##### Rico Schillings[Sweaty] [Moderator] 04/21/2022 05:14:14
Looks like it cant find the libraries of webots for the controller. Ensure you have also configured the environment variables like documented here: [https://cyberbotics.com/doc/guide/running-extern-robot-controllers#environment-variables](https://cyberbotics.com/doc/guide/running-extern-robot-controllers#environment-variables)



In your case since your using python it needs the PYTHONPATH variable (and its required webots\_home)

##### ~E 04/21/2022 12:30:32
How can I make an invisible bounding box? Can I use a solid>shape and make that shape invisible somehow?

##### barryman44 04/21/2022 12:36:19
You can create a bounding box with only a shape, so without a solid. Now you will have an invisible bounding box.

##### ~E 04/21/2022 12:39:05

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/966679470314504272/unknown.png)
%end


Oh! I think I've got it now


`@barryman44` I have the bounding box now, but I'm not sure how to rotate it so that it fits my wheel...


Is there a good sample world where I can see how this can be done?

##### DrakerDG [Moderator] 04/21/2022 12:47:58
You need add transform node in the bounding object and then you need add your shape in the children of transform

##### ~E 04/21/2022 12:59:30
Thanks, I got it now. I was putting the bounding box too far down the tree, and couldn't translate it independently. For future reference:

Hinge joint

endpoint (Solid-STL\_BINARY) <-Set boundingobject as shape

Solid <-Rotate this second solid to fit the bounding object you set above
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/966684605040169001/unknown.png)
%end


My tree ended up looking like this:
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/966684861505101834/unknown.png)
%end


so my issue now is that I have a bunch of bounding boxes that aren't connected to each other...


`@barryman44` any reccomendations?


Wtf I closed webots and reopened and now everything is connected lol


How can I change the coefficient of friction of different parts? Do I need to make new physics plugins or something?


I want the white kickstand to have low friction and the wheels to have high friction with the ground
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/966715976307572796/unknown.png)
%end

##### Dr.SilentMetal 04/21/2022 15:49:03

%figure
![Error.png](https://cdn.discordapp.com/attachments/565154703139405824/966727274500071444/Error.png)
%end


I need help on how to resolve this issue

##### Rico Schillings[Sweaty] [Moderator] 04/21/2022 15:59:07
As the error tries to tell, it cant find the external python library "numpy". Try installing it like `pip install numpy`or check the internet how to install numpy on windows

##### Dr.SilentMetal 04/21/2022 16:00:53
Thanks so much for helping but I've done that several time before asking for help. I installed numpy into the directory it states in the error but error persist

##### Rico Schillings[Sweaty] [Moderator] 04/21/2022 16:06:12
You should install it globally to your python Interpreter/Environment you are using. Webots will use the global python env (if not specified another one). May you can list all python libs and check if numpy is listed (default this works with `pip list` in a terminal, but i'm from Linux and not sure if these commands are os-indepent)

##### Dr.SilentMetal 04/21/2022 16:09:59
Numpy is listed but I can't get rid of the error

##### Rico Schillings[Sweaty] [Moderator] 04/21/2022 16:11:26
Do you have several python versions installed (e.g. 3.5/3.7/3.9)?

##### Dr.SilentMetal 04/21/2022 16:13:28
Yes

##### Rico Schillings[Sweaty] [Moderator] 04/21/2022 16:17:30
Then may check if the version of your default python env (where you just checked `pip list`) is the same like you have defined in the environment variable PYTHONPATH to link the specific webots python library. For example, if you have default 3.7 then the environment variable should also link to that library, otherwise it could be that you have numpy installed in your 3.7 python but webots tries to use 3.9 where its not?

##### Dr.SilentMetal 04/21/2022 16:21:06
Thanks so much `@Rico Schillings[Sweaty]`. The webots directory is set to the location I checked using `pip list`


I'm uninstalling all other version of python, leaving only the one that will be utilised, if that will help

##### Rico Schillings[Sweaty] [Moderator] 04/21/2022 16:24:04
Alright.. I slightly remember to had a similiar problem, but i dont know again hoe i've fixed it. And at the moment i've no further ideas to check for fixing it. May another one here has more knowledge around this error.. 😕

##### Olivier Michel [Cyberbotics] 04/21/2022 16:26:03
When you type `python --version` in the console, do you get the same version as the one specified in the Webots Preferences? You can easily check it by writing in your python controller the following lines:

```python
import sys
print(sys.version)
```

##### Dr.SilentMetal 04/21/2022 16:27:17
Yes I do get same version


Fixed it. Thanks `@Rico Schillings[Sweaty]` for prompt response and `@Olivier Michel` for coming through

##### Olivier Michel [Cyberbotics] 04/21/2022 16:35:31
How did you fix it?

##### Dr.SilentMetal 04/21/2022 16:37:35
Uninstall all other python version, leaving only the version used in the project and check if numpy is installed using `pip list`, it revealed the numpy version previously displayed was from the highest version installed. As such I installed numpy to the directory and simulation worked

##### goch [Moderator] 04/22/2022 16:46:03
Hey. Is it possible to access a Motor that is attatched to  a different Controller from a Supervisor?  I Have 2 Controllers. Controller A is a Supervisor and needs to call setPosition() of the LinearMotor that is attatched to Controller B.

##### AL058\_Ashutosh 04/24/2022 04:00:14
Hello, I have been trying to take user input, but the simulation does not stop to take user input and instead executes the next command. What do I need to do to resolve this issue?

##### snackei 04/24/2022 05:23:41
Does anyone know why camera.getRecognitionObjects() returns 



"[<controller.CameraRecognitionObject; proxy of <Swig Object of type 'webots::CameraRecognitionObject *' at 0x00000207BB194AB0> >, <controller.CameraRecognitionObject; proxy of <Swig Object of type 'webots::CameraRecognitionObject *' at 0x00000207BB1949C0> >, <controller.CameraRecognitionObject; proxy of <Swig Object of type 'webots::CameraRecognitionObject *' at 0x00000207BB194A80> >]" 



and calling .get\_position() on these objects crashes my controller?

Python btw

##### bedarbis 04/24/2022 20:47:28
Hello, how to control KUKA YouBot's second arm?

##### klammerc 04/25/2022 03:06:33
Any experience with importing 3D models here? I am trying to import some parts for a UAV but my axes always seem to be in crazy spots after I import the STL file. Likely am just missing something I assume. The cap I imported is below the ground and the axes are shown as such
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/967984935975673906/unknown.png)
%end

##### MistyMoon 04/25/2022 08:09:30
Hi all, I am doing SLAM research in Webots and ROS2. Using webots\_ros2\_driver Interface. I need up to 200Hz frequency output in IMU. But the default is 30Hz, where do I need to change it?



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/968061469893726238/unknown.png)
%end

##### DDaniel [Cyberbotics] 04/25/2022 08:19:14
Every controller only has access to the devices of its own robot, however you can use emitter/receivers to communicate between robots and instruct it to set the necessary position that way


It's not really meant for it, but there's ways around it depending on what you're trying to do. If you wish to control the robot using the keyboard then you can rely on the Keyboard node (there's several robot examples of this, for instance `file > open sample world > fabtino`). If instead you need user input so that multiple similar robots behave differently, then you should provide them a different controllerArgs parameter

##### MistyMoon 04/25/2022 08:26:06
I changed the updateRate in the urdf file in resource and it doesn't seem to work.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/968065356792987679/unknown.png)
%end

##### DDaniel [Cyberbotics] 04/25/2022 08:26:51
Which version of Webots are you using? Also the correct method is `getPosition`, not `get_position`


The piece doesn't appear to be centered which may or may not be you want (either you export the pieces centered at (0,0,0) and then position them on the Webots side using Transform nodes or you export them already translated, usually depends whats their purpose and if there's joints of other stuff in between them). From the picture I suspect however the issue is the scaling, which I think might be exported wrongly (unless the size of the object seems reasonable to you). When exporting you have to make sure to export in meters.


if you're talking about the provided demo (`file > open sample world > youbot`) then you need to press shift+arrows


you indeed need to set the updateRate to 200 in the urdf file, but you should keep in mind that by design the controller cannot bet faster than the simulation itself so you need to adapt the `basicTimeStep` of the world file accordingly as well

##### MistyMoon 04/25/2022 08:55:17
Thanks for the suggestion, I'll test it


It does work! To achieve a higher updateRate though, it seems to require a high configured computer to run the simulator. I'll try using a server tonight. Thanks!

##### jinu061353 04/25/2022 10:33:41
I want to run pytorch on webots. Is it possible?If possible, is it possible to run python in an anaconda based virtual environment?

##### AL058\_Ashutosh 04/25/2022 15:10:20
Thank you, will try this.

##### vuwij 04/25/2022 16:19:44
I have a question, when Im creating a webots supervisor and I want to connect to webots, in what way does it connect to webots? If the simulator and the robot are in seperate docker containers, how can I connect it to the simulator? Is there an environmental variable such as webots\_port webots\_ip etc?

WEBOTS\_ROBOT\_NAME, I basically want to run the simulator and the robot code in different computers or containers

##### Olivier Michel [Cyberbotics] 04/25/2022 16:32:18
This is not yet possible, but we have an open PR addressing this: [https://github.com/cyberbotics/webots/pull/4344](https://github.com/cyberbotics/webots/pull/4344)

##### vuwij 04/25/2022 16:33:52
at the moment how does it communicate with the simulation?

##### Olivier Michel [Cyberbotics] 04/25/2022 16:34:49
Through anonymous pipes and shared memory segments.

##### Joshua S 04/25/2022 16:48:19
Did you happen to find how to include openCV? I am also getting the same error

##### moebius 04/25/2022 17:45:28
i have a wheeled robot, that's trying to cross some speed bumps. I'm observing some weird physics, such as rocking back and forth upon colliding with the bumps, doing wheelies on the rear wheels, and falling in slow motion. I've attached the world files, could someone tell me, what changes i can make that can model the physics more realistic? thanks!
> **Attachment**: [nwheeled\_bumps.zip](https://cdn.discordapp.com/attachments/565154703139405824/968206121481424896/nwheeled_bumps.zip)

##### giaco\_mz 04/25/2022 18:55:40
No I wasn’t able to do that.

##### moebius 04/25/2022 20:11:48
Also sometimes, the robot freezes, while the controller is still running (i can see prints in the console), what is usually the reason for that?

##### shin. 04/26/2022 04:03:39
Hello!

i was trying to use extern as controller 

I used visual studio 22

when compiling this appears:

Error: The specified module could not be found.

 (dynamic library)

Error: F:/Webots/projects/robots/gctronic/e-puck/plugins/remote\_controls/e-puck\_bluetooth/e-puck\_bluetooth.dll remote control library initialisation failed

Error: Cannot load the "F:/Webots/projects/robots/gctronic/e-puck/plugins/remote\_controls/e-puck\_bluetooth/e-puck\_bluetooth.dll" remote control library.

##### snackei 04/26/2022 04:03:45
from controller import CameraRecognitionObject



class CameraRecognitionObject:

    def get\_id(self):

    def get\_position(self):

    def get\_orientation(self):

    def get\_size(self):

    def get\_position\_on\_image(self):

    def get\_size\_on\_image(self):

    def get\_number\_of\_colors(self):

    def get\_colors(self):

    def get\_model(self):



is this from cyberotics inaccurate then? and switching the version to 2021a worked, thanks for the tip

##### DDaniel [Cyberbotics] 04/26/2022 06:08:01
No it's correct, I thought you were talking about a different getPosition API

##### Ludane 04/26/2022 07:20:56
Hi guys, I'm trying to use a supervisor that import another robot in the simulation, my question is, is it possible to only use the controller of the supervisor to control the motors of the  robot I just import ? Right now I'm trying to get access to the motors by importing the class Motor, and then I define the motors as Motor("motor\_name") but he doesn't find the devices.

##### Mat198 04/26/2022 07:55:19
You give an argument to specify the robot controller while you import it. Or just import the robot and set the controller later with the supervisor. I suggest to import everything while the simulation is stopped to avoid strange behaviours or crashs.



It doesn't find the motor because they are not in the supervisor robot.


Look at these tutorials :



[https://cyberbotics.com/doc/guide/tutorial-8-the-supervisor?tab-language=c](https://cyberbotics.com/doc/guide/tutorial-8-the-supervisor?tab-language=c)



[https://cyberbotics.com/doc/reference/supervisor](https://cyberbotics.com/doc/reference/supervisor)

##### MistyMoon 04/26/2022 09:49:53
Is there any documentation on optimising webots? I'm finding the performance of the basicTimeStep parameter to be unsatisfactory when I tweak it.

##### Olivier Michel [Cyberbotics] 04/26/2022 13:48:08
See [https://www.cyberbotics.com/doc/guide/speed-performance](https://www.cyberbotics.com/doc/guide/speed-performance)

##### MistyMoon 04/27/2022 09:59:08
Thanks for the link, by testing on three consecutive computers I have switched to in the last few days. When `BasicTimeStep` adjusted to 6 and webots open alone, the `/imu` topic provided by `webots_ros2_driver::Ros2IMU` is 160-170Hz. But as soon as I open the LIDAR topic, the `/imu` rate drops to less than 50Hz. It is hard to satisfy my subsequent work and it seems to be irrelevant to the computer performance.  Do I need to raise an issue on GitHub, describing the problem in detail and attaching test results?

##### Olivier Michel [Cyberbotics] 04/27/2022 10:02:19
Opening an issue about it won't help. Instead, you can try to optimize the code in webots and/or webots\_ros2 and open a PR if you are getting improved results.

##### ~E 04/27/2022 13:56:45
Is there a way to change the Y axis scale on these graphs?



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/968873344608903238/unknown.png)
%end

##### giaco\_mz 04/27/2022 15:46:13
There is a way to reorder manually the tree? 

There are shortcut to expand or compact the tree?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/968900890817855518/unknown.png)
%end

##### DrakerDG [Moderator] 04/27/2022 15:50:43
Yes, you can add group nodes and copy and paste in children of every group

##### giaco\_mz 04/27/2022 17:56:05
Thanks, is more like a work around but i will be fine with that 🙂

How can i simulate the wave of sonar with rays?

The problem is that object between rays are not detected as this yellow robot.

I can increase the number of rays but isn't really like simulate wave.
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/968933570066411551/unknown.png)
%end

##### AlexandrosNic 04/28/2022 12:33:20
I have Webots on Windows 11 and installed WSL2. When I run "webots" from within wsl2 terminal I get the "command not found" error. PATH and WEBOTS\_HOME env variables of Windows have been set correctly. Any idea on this?


Ok I have the solution:

export PATH="/mnt/c/Program Files/Webots/msys64/mingw64/bin:$PATH"

and then "webotsw.exe" (I don't understand why the extra "w" but this is the executable"

##### jmshin 04/29/2022 06:11:23
Hi, I made map\_to\_webotsworld, convert black pixel to solid box using python script. In this case, it shows good performance because there are fewer black pixels. If there are many black pixels, it takes a long time to convert and the simulation is also stuttering. I want to convert the box into a polygon or defined custom object. Do you have any recommendations?
%figure
![Screenshot_from_2022-04-29_14-26-53.png](https://cdn.discordapp.com/attachments/565154703139405824/969481002751762462/Screenshot_from_2022-04-29_14-26-53.png)
%end

##### AlexandrosNic 04/29/2022 12:40:01
Coming back to this, I would like to open Webots that is locally installed in Windows, through WSL2 through ROS2. So writing in a terminal webotsw.exe is not an option since ROS2 checks for binaries in the WEBOTS\_HOME variable. Any idea for this?

##### goch [Moderator] 04/29/2022 19:18:15
Hey does someone know why the html page of the robot window doesnt seem to be rendered? Opened the Sample World for the Simple and Custom Robot Window World but neither of the worlds seem to render the page. Using Webots R2022a from the Arch Linux Package. Is there a way to debug this problem?
> **Attachment**: [webots\_robot\_window\_example\_bug.mp4](https://cdn.discordapp.com/attachments/565154703139405824/969679026400669816/webots_robot_window_example_bug.mp4)

##### Naxi 04/29/2022 21:00:54
Hi! I'm trying to access an imported PROTO node fields with a supervisor using getFromDef(), but since the node is dynamically spawned in the wbt the supervisor doesn't find it. Is there any way to access a node I spawned in the same controller? I'm guessing that the supervisor looks for the nodes in the wbt and since this node wasn't there in the beginning, it doesn't find it

##### DDaniel [Cyberbotics] 04/30/2022 15:50:12
It shouldn't matter, fields from spawned nodes can be accessed just the same. Are you trying to access a node defined within the PROTO or the PROTO itself? If within, you must use getFromProtoDef

##### Naxi 04/30/2022 19:59:02
Strange, getFromDef is returning None, and using getFromProtoDef raises an `AttributeError: 'Supervisor' object has no attribute 'getFromProtoDef'`


Oh, I see, so, I was trying to access a node within the PROTO (its boundingObject) as a Field , my bad. Thanks for the answer!

## May

##### AL058\_Ashutosh 05/01/2022 06:10:44
What is the science/math behind the recognition node of camera?

How does it identify the objects?

##### jinu061353 05/01/2022 10:21:12
I want to run pytorch on webots. Is it possible?If possible, is it possible to run python in an anaconda based virtual environment?

##### Stefania Pedrazzi [Cyberbotics] 05/02/2022 06:54:53
The camera recognition functionality provides ground truth data. it doesn't use any detection algorithm but directly the information of the objects position in the scene. 

As described in the documentation ([https://www.cyberbotics.com/doc/reference/recognition](https://www.cyberbotics.com/doc/reference/recognition)) in order to specify which objects will be detected you have to set the `Solid.recognitionColors` field.


Yes, you can use pytorch with Webots. it just needs to be installed in the python version used by Webots.

Here you can find the instructions to use anaconda instead of the default system python version:

[https://cyberbotics.com/doc/guide/using-python#use-an-alternative-python-version](https://cyberbotics.com/doc/guide/using-python#use-an-alternative-python-version)

##### bookatron 05/02/2022 18:41:39
Hi all, I'm trying to record a trace of an action (pushing an object across a table) and I'd like to sample the object position every timestep. 



I've tried just dumping the object location to a file in the main supervisor loop (supervisor.step(settings.TIME\_STEP) ~= -1). The problem is that this loop hangs when the robot is moving, and so I'm not getting samples within that period.



I've also tried the supervisor.animationStartRecording() method, but it doesn't look like it's including info on the moving object in the scene. Any way to control what goes into this file?



Any other recommendations? Thanks in advance!

##### áçè 05/04/2022 07:33:13
i want to make my planetary cluster wheels to rotate on stairs so for this should i give individual motor to the wheels or have braking mechanism on wheels so that it does not slip on stair (in controller ) or what to do ?
> **Attachment**: [empty\_2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/971313537215520778/empty_2.mp4)


if somebody can give any suggestions it would be appreciated , thank you...

##### Wanting 05/04/2022 11:46:56
Hello, I have a problem when simulating the epuck with the kinematic mode as True. With the kinematic mode set as true, the collision detection is not working and the epuck will cross walls. While as kinematic mode set as false, the wheels will detach from the robot. Do you have any idea what could be wrong?


And  I am converting a project from webots 2021a to 2022a. The code works well with kinematic ==True in webots 2021a, but started to have same problem as mentioned above with 2021b.


Many thank!!

##### DDaniel [Cyberbotics] 05/04/2022 12:10:16
On 2022a I'm unable to reproduce your issue (with kinematic = true). Are you using a custom e-puck/arena or the included one?
> **Attachment**: [e-puck.mp4](https://cdn.discordapp.com/attachments/565154703139405824/971383257658699816/e-puck.mp4)

##### Wanting 05/04/2022 12:13:00
Hi, I am using the included one for both e-puck and arena

##### DDaniel [Cyberbotics] 05/04/2022 12:13:56
odd, could you send the world/project? (dm or mail)

##### Wanting 05/04/2022 12:15:18
Sure, Thank you! Could you give me the email address?

##### DDaniel [Cyberbotics] 05/04/2022 12:15:42
support@cyberbotics.com

##### Wanting 05/04/2022 12:15:49
Thanks a lot


I have sent 😉 Thanks a lot for your time.

##### Wanting Jin 05/04/2022 13:09:22
I also recreate the world but it still not working, it is possible for you to send this world to me?

##### DDaniel [Cyberbotics] 05/04/2022 13:14:22
it's just the sample world: `file > open sample world > robots > gtronic > e-puck` and setting the kinematic to TRUE before running it.

But I do see the issue in the world you provided, still no clue what makes it go crazy though

##### Wanting Jin 05/04/2022 13:15:35
Thanks a lot. I will have a look at the sample world


In the sample world for epuck this is the obstacle avoidance behavior in the controller. If I replace the controller with our controller (no obstacle avoidance), it will cross the wall unfortunately

##### DDaniel [Cyberbotics] 05/04/2022 13:44:10
Yes, something did change between 2021a and 2022a it seems. In 2021a if you have an e-puck with a controller that just goes straight (in kinematic mode), it will stop at the wall whereas in 2022a it doesn't, still investigating why

##### Wanting 05/04/2022 14:57:44
Okay thanks. Actually this behavior was already broken with 2021b

##### DDaniel [Cyberbotics] 05/04/2022 14:58:56
Yes, I think I found it: [https://github.com/cyberbotics/webots/pull/4509](https://github.com/cyberbotics/webots/pull/4509)

##### Wanting 05/04/2022 15:06:23
Okay. Thanks!! I will have a look


So a follow-up question for this. If we set the kinematic to false, is it normal that the wheel will detach from the robot after some time?

##### DDaniel [Cyberbotics] 05/04/2022 15:31:03
No 🙂 in the same world/controller you mean? I haven't seen wheels detaching in it nor in the sample world so far


but if physics are involved, you need to also reset the physics of the robots in-between iterations otherwise the inertia will carry through

##### Wanting 05/04/2022 15:32:53
Yes, if you set the kinematic to false and run the controller with the fast speed for several minutes (it will happen after several iterations)


Okay. I see. Maybe it's true we need to reset the physics

##### DDaniel [Cyberbotics] 05/04/2022 15:34:15
I see now, it takes some time to show. Yes, that's likely the culprit, you're resetting the positions of the robots but the angular velocities aren't

##### Wanting 05/04/2022 15:38:57
Okay I will have a look. Thank you a lot

##### Simon Steinmann [Moderator] 05/04/2022 16:23:57
Hi, is there a way to make the lines of the optional camera frustrum longer?


nvm, figured it out. increasing  the far field does the job

##### AlexandrosNic 05/04/2022 21:59:53
I'm wondering whether the webots docker image has to be updated because of this error:

```
GPG error: https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64  InRelease: The following signatures couldn't be verified because the public key is not available: NO_PUBKEY A4B469963BF863CC
#11 30.69 E: The repository 'https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64  InRelease' is not signed.
```



[https://forums.developer.nvidia.com/t/notice-cuda-linux-repository-key-rotation/212772](https://forums.developer.nvidia.com/t/notice-cuda-linux-repository-key-rotation/212772)

##### Stefania Pedrazzi [Cyberbotics] 05/05/2022 06:31:25
The animation recorded files contains the information of the objects translation and rotation changes. However, it is probably easier to write them manually in a file from the robot controller instead of having to parse the animation file.

I don't understand what issue you have with writing the object location to a file from the supervisor loop. If the TIME\_STEP you use is too big and you are not getting enough samples, then you should consider reducing it.

##### Olivier Michel [Cyberbotics] 05/05/2022 06:42:27
I am not sure how to fix our docker images, as we don't explicitly download from the NVIDIA repositories. Maybe updating the base image from version 11.0 to version 11.4.2 here: [https://github.com/cyberbotics/webots-docker/blob/master/Dockerfile#L1](https://github.com/cyberbotics/webots-docker/blob/master/Dockerfile#L1) (as newer versions are available from [https://registry.hub.docker.com/r/nvidia/cudagl](https://registry.hub.docker.com/r/nvidia/cudagl)). Could you try this and let us know if that fixes the problem?

##### AlexandrosNic 05/05/2022 09:08:22
Nope. Unfortunately neither this worked, nor (this)[[https://github.com/NVIDIA/nvidia-docker/issues/1631#issuecomment-1112828208](https://github.com/NVIDIA/nvidia-docker/issues/1631#issuecomment-1112828208)]. I will let you know once I find a solution though

##### Olivier Michel [Cyberbotics] 05/05/2022 09:18:13
I believe I found a solution: [https://github.com/cyberbotics/webots-docker/pull/15](https://github.com/cyberbotics/webots-docker/pull/15). Can you please review it and let me know what do you think about it?

##### AlexandrosNic 05/05/2022 10:22:51
Indeed the fix works. Thank you Olivier!

##### Olivier Michel [Cyberbotics] 05/05/2022 10:24:19
Can you please approve the PR?

##### AlexandrosNic 05/05/2022 10:30:45
Did it. Let me know if I can do anything more 🙂

##### 黑色親衛隊 05/05/2022 10:42:10
Hello! I originally opened the program with version 2019 on the robot simulator and it runs without any problem. However, when I run it on version 2022, I get the error "controller exited with status: 1. " What is the meaning of this error and how can I solve it?

##### Peeyusha 05/05/2022 13:11:34
Hi, was wondering if someone could help me in figuring this out. Whenever I'm trying to run the controller code for Python for the "e-puck" this is what I'm getting. I've installed Version 3.9 as mentioned in the pre requisites and added the path in tools>preferences.
%figure
![Capture.JPG](https://cdn.discordapp.com/attachments/565154703139405824/971761073852141568/Capture.JPG)
%end

##### bookatron 05/05/2022 22:06:49
Hi Stefania, that's good to know about the animation files. I believe that I've solved my problem, and it was on my end. Thanks for the reply!

##### AlexandrosNic 05/06/2022 13:20:59
hey, I know this question has been asked multiple times but I cannot get it running, when I run my custom package. The error is:

```
ImportError: usr/local/webots/lib/controller/python38/_controller.so: undefined symbol: _ZN6webots5Robot19internalGetInstanceEv
```

I installed ROS2 and Webots following the installation guide (everything runs in a docker container), and I exported the envs as noted:

```
echo $WEBOTS_HOME
/usr/local/webots
echo $LD_LIBRARY_PATH
/opt/ros/foxy/opt/yaml_cpp_vendor/lib:/opt/ros/foxy/opt/rviz_ogre_vendor/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:/opt/ros/foxy/lib:/usr/local/webots/lib/controller
echo $PYTHONPATH
/home/cocobots/cocobots_ws/install/cocobots_simu/lib/python3.8/site-packages:/opt/ros/foxy/lib/python3.8/site-packages:/usr/local/webots/lib/controller/python38
```

probably there is no error with my envs, since other examples such as these, are running:

```
ros2 launch webots_ros2_epuck robot_launch.py
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```



UPDATE:

Fixed it from here [https://githubhot.com/repo/cyberbotics/webots\_ros2/issues/420](https://githubhot.com/repo/cyberbotics/webots_ros2/issues/420)

Basically I replaced the 'Import Supervisor' with this:

```
import os
if os.getenv('ROS_DISTRO') is not None:
    from webots_ros2_driver_webots.controller import Supervisor
else:
    from controller import Supervisor
```

##### Naxi 05/06/2022 14:58:27
Hi all! Can a normal robot obtain information about its children nodes (translation, rotation, etc?)

##### maranello1088 05/06/2022 19:07:29
Hi everyone. 



This question isn't entirely technical, but, how can I create a Factory world with spillages and dirt?

##### goch [Moderator] 05/08/2022 20:34:59
I think it is not possible. The Robot class acts like a real robot. If you want to know where you are you need sensors for that. The Supervisor class was designed to get some extra functionality that you could not get in the real world. 





The documentation explains this here ->  [https://cyberbotics.com/doc/guide/supervisor-programming](https://cyberbotics.com/doc/guide/supervisor-programming)



"One important thing to keep in mind is that the Supervisor API corresponds to functionalities that are usually not available on real robots; it rather corresponds to a human intervention on the experimental setup. Hence, the wb\_robot\_* vs. wb\_supervisor\_* distinction is intentional and aims at reminding the user that Supervisor API functions may not be easily transposed to real robots."


Depends on what you need from the factory. Do you need the physical properties of the dirt and spillage or do you just want the Look of a dirty factory?


did you solve the issue? if not open a command line and check if your path variable is set correctly



echo %PATH%

and 

echo %PYTHONPATH%



also test if your commandline detects the python binaries 



python --version

##### AlexandrosNic 05/09/2022 12:04:16
Hey, I installed webots through Docker in a machine with (a supported) Nvidia driver, however, when I first start webots after I built docker, it says that Webots currently uses a not supported (Intel? through Mesa) driver which makes the performance very low. Why isn't it running with Nvidia?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/973193688027136030/unknown.png)
%end

##### DDaniel [Cyberbotics] 05/09/2022 12:33:26
did you install nvidia-docker2 and run the docker with `--gpus=all` ?


More details are available here: [https://www.cyberbotics.com/doc/guide/installation-procedure?version=develop#run-webots-in-docker-with-gui](https://www.cyberbotics.com/doc/guide/installation-procedure?version=develop#run-webots-in-docker-with-gui), in the `With GPU Acceleration` sub-section

##### AlexandrosNic 05/09/2022 14:33:27
I did in the Linux OS (WSL2 in particular). Should it be inside the docker?

##### AxlSyr 05/09/2022 16:19:11
Hi Webots community. I was wondering if anyone can share with me any basic code of a Nao robot moving without Motion files and with or without a Supervisor.

##### moebius 05/09/2022 23:12:59
How do i model collisions and rolling over obstacles more accurately, for a wheeled mobile robot that i am generating? Should i tweak the ERP and CFM values? or is changing the damping and/or inertia sufficient?

##### Peeyusha 05/10/2022 01:34:26
Thanks! Working now.

##### AlexandrosNic 05/10/2022 10:36:33
Still didn't manage to fix. any idea?

##### DDaniel [Cyberbotics] 05/10/2022 12:26:14
I'm not familiar with WSL2 but it should be done on the host side. Before that, you should ensure your docker setup is functional and that gpu is detected as well. Running `sudo docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi` should work, if it doesn't then your issue is elsewhere. More details are available here: [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

##### maranello1088 05/10/2022 12:35:38
It would be the physical properties. 



However, right now, I just simply downloaded a wbt file. 



And here is what I am getting as errors.



%figure
![20220510_133558.jpg](https://cdn.discordapp.com/attachments/565154703139405824/973564056470429746/20220510_133558.jpg)
%end

##### DDaniel [Cyberbotics] 05/10/2022 12:39:12
You downloaded it from github? That won't work unless you also download the assets. Instead you should `file > open sample world > search factory`. After doing the modifications you want, you can `file > save world as`


As for your other question, you should define a contact material for your wheel (parameter `contactMaterial`, calling it for ex: "wheel material")  and one for the solid that represents the spillage (calling it for ex: "oil material") and then you can add a `ContactProperties`  node under `WorldInfo > ContactProperties` between these two materials and configure the friction/... accordingly [https://www.cyberbotics.com/doc/reference/contactproperties](https://www.cyberbotics.com/doc/reference/contactproperties)

##### maranello1088 05/10/2022 12:48:46
Thanks Dan, I've been able to save world as. 



Now I'll try define contact materials.

##### AlexandrosNic 05/10/2022 17:23:58
The docker you provided indeed works, but still webots is not able to detect nvidia.

Btw the installation of nvidia docker for WSL is here: [https://docs.nvidia.com/cuda/wsl-user-guide/index.html](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)

##### Olivier Michel [Cyberbotics] 05/11/2022 06:16:04
That seems to work for CUDA, but not OpenGL, see "Applications relying on OpenGL will not work.".

##### AlexandrosNic 05/11/2022 06:47:33
Oh, I didn't notice before. Then I suppose I should give up the hope of using Webots in a WSL-docker  yet

##### Olivier Michel [Cyberbotics] 05/11/2022 06:48:55
I think so.

##### thedutchassasin 05/11/2022 13:12:39
hey if a give a robot a boundry box and physics is it stil supposed to fall trough the floor?

##### DrakerDG [Moderator] 05/11/2022 13:20:16
Should not

##### thedutchassasin 05/11/2022 13:21:43
wel it still does on multiple of my bots


does anyone know how to fix this?

##### DrakerDG [Moderator] 05/11/2022 13:40:06
Did you try reduce the basicTimeStep of the WorldInfo node?

##### AlexandrosNic 05/11/2022 15:04:30
an irrelevant and maybe noob question: How can webots dockerfile successfully run on mac, when it uses commands such as apt-get that are normally not supported by mac? ([https://github.com/cyberbotics/webots-docker/blob/master/Dockerfile](https://github.com/cyberbotics/webots-docker/blob/master/Dockerfile))

##### OS 05/12/2022 01:06:57
Hello!

I have a problem when i try to run an external python controller on Mac OS, I've read all instructions on Webots User Guide but when i try to start the  controller manually from the terminal this shows me this error.

```
Segmentation fault: 11
```

Thank you in advance!

##### Olivier Michel [Cyberbotics] 05/12/2022 05:57:09
Can you run your controller from inside Webots, e.g., not as an extern controller?

##### Rico Schillings[Sweaty] [Moderator] 05/12/2022 06:09:59
The commands like `apt-get ` etc are used inside the dockerfile to build the container. Its OS independent, your Mac just starts the final built container.


Is WSL2 in the meantime supporting nvidia/GPU support officially? When I tried it last year, I needed to subscribe the Windows dev programm and build it from source to get GPU support in it.. 🙄

##### AlexandrosNic 05/12/2022 07:35:42
I just noticed that the webots dockerfile is built on "nvidia/cudagl:11.4.2-devel-ubuntu20.04" which itself is built on "ubuntu20.04", so now it makes sense


yes is kind of new development, but either if you have Windows 10 and use Windows Insider program, either if you update to Windows 11, then you get GPU support through WSL2, which is great, since I can run my (windows-installed) webots through WSL2, with the (windows-installed) gpu drivers

##### OS 05/12/2022 10:34:41
Actually it works pretty well.


Could anyone help me please?

##### Olivier Michel [Cyberbotics] 05/13/2022 06:22:41
Are you sure you set the `DYLIB_LIBRARY_PATH` and `WEBOTS_HOME` environment variables to point to the correct version of Webots? Have you several versions of Webots installed on your machine? Do the other controllers (the ones provided with Webots) work as extern controllers or is it only yours that crashes?

##### OS 05/13/2022 12:17:04
Yes, I've set those variables as i have only one version of Webots. As there's no error in importing from python Controller file in Vs Code. The ones provided with Webots gave me the same error when i try to run it.
%figure
![Screen_Shot_2022-05-13_at_2.15.55_PM.png](https://cdn.discordapp.com/attachments/565154703139405824/974646460178137088/Screen_Shot_2022-05-13_at_2.15.55_PM.png)
%end

##### Olivier Michel [Cyberbotics] 05/13/2022 12:23:37
And did you try to run a non-python (e.g., binary) controller as an extern controller? The crash you get might be a mismatch between the python version needed by the Webots python library and the one you are actually using...

##### Chi 05/15/2022 00:51:45
Hi, I would like to add new functions to the  race\_wheel controller like getting GPS device information from the vehicle, which file should I edit?



%figure
![SUMOWebots.png](https://cdn.discordapp.com/attachments/565154703139405824/975199895076102184/SUMOWebots.png)
%end

##### Tom\_Wolf 05/16/2022 14:49:58
Hi everyone, i am installing ros2 and webots on my new debian computer and when I build the webots\_ros2 packages I get this error 

The following packages/stacks could not have their rosdep keys resolved to system dependencies 

Webots\_ros2\_epuck: No definition for os debian


I might need your help please

##### Tomobomo 05/16/2022 23:15:36
Hi everyone, I would like to change my webots python version from 3.6.9 to 3.8.10.  When I do tools->preferences, I see that the python used is the built-in (python) snap. I am not sure where this is or how to change it. Any suggestions

##### Stefania Pedrazzi [Cyberbotics] 05/17/2022 06:26:46
Hi, if it is not the case you first have to install python3.8, then simply change the command in the Webots preferences from "python" to "python.3.8".

Here is the documentation:

[https://www.cyberbotics.com/doc/guide/preferences](https://www.cyberbotics.com/doc/guide/preferences)

##### robotnik 05/17/2022 10:08:38
Hi everyone, I'm having problem changing the velocity for a linear motor with an external python controller. The controller is connected with ROS and is supposed to change the value of velocity with an action call, but the value stays the same.


Some idea of what could be happening, I have tried many change like putting step inside the ROS Action

##### DDaniel [Cyberbotics] 05/17/2022 10:30:13
Are you controlling the motor in position or velocity? If you want to control the motor in velocity, you need to set the position `/set_position` to INFINITY beforehand

##### robotnik 05/17/2022 10:33:11
Already did that, I will try to restructure the code with classes to see if something changes.

##### DDaniel [Cyberbotics] 05/17/2022 10:37:34
This is an example of a controller that controls an e-puck robot using ROS: [https://github.com/cyberbotics/webots\_ros/blob/master/src/e\_puck\_line.cpp](https://github.com/cyberbotics/webots_ros/blob/master/src/e_puck_line.cpp), doesn't have linear motors but it's the same principle

##### AlexandrosNic 05/17/2022 13:49:32
Guys it would be very helpful if someone with working Webots on WSL, could give some hints. For example,

1. Shall I also install webots inside WSL? or just leave the Windows native Webots only?

2. What are the environment variables? Shall I use /usr/local/webots or /mnt/c/Program Files/Webots for WEBOTS\_HOME etc

##### Kugelkopf 05/17/2022 21:06:14
Hello  everyone,

I would have a question regarding a bug or something weird in WSL 2 and Webots.

I have a Windows 10 21H2 and I'm using VcXserv to use GUI apps in my WSL 2 instance. Everything seems to be ok as I can run different GUI app,s Xcalc, Gedit, Gazeboo and also Webots. I would like to use Webots , but when I click in the middle of the screen where there is an object, the screen goes black. So I cannot click on the objects in the given webots scene or the whole screen goes black.



Does anybody has any idea of this issue? Or how this could be solved? Is it present on win 11, I can only use win 10 for now.

I also use GPU acceleration, here is my ~/.bashrch 



         source /opt/ros/noetic/setup.bash

         export DISPLAY=$(ip route list default | awk '{print $3}'):0

         export LIBGL\_ALWAYS\_INDIRECT=0

If I use  export LIBGL\_ALWAYS\_SOFTWARE=1, the GPU acceleration is turned off, weird thing is the screen does not go black at that time. but the GPU usage remains at around 0-2%.



BUt When I try to Utilize the GPU, the issue happens.



Another weird thing is that glxgears shows a lot higher FPS with CPU only setting than when I use GPU acceleration as well:

around 730 FPS with only CPU

around 150 FPS with GPU acceeleration.



Shouldnt be this the other way around?

##### moebius 05/17/2022 22:40:01
Does the bounding box have to be an existing Shape in the scene tree or can we draw simple bounding boxes around existing shapes?

##### Kugelkopf 05/18/2022 05:48:41
If your question is in regards to mine. Just create a New scene, add an arena field then a Wood box. Click on them with your mouse and the screen should go black in the scene view.



You can however move around in the scene via the  mouse if you are not clicking on any objects,  just the empty space.


Also im interested if this only happens on win 10 21H1 with VcXsrv or this also happens on Win 11 latest build. (I dont have Access to win 11 at the moment)

##### DDaniel [Cyberbotics] 05/18/2022 06:07:54
No, it can be a new geometry/shape (a single primitive or multiple ones if you insert a Group/Transform beforehand) or an existing one (using DEF/USE mechanism)

##### Olivier Michel [Cyberbotics] 05/18/2022 06:09:32
It's not recommended to install Webots in WSL as you won't be able to get 3D hardware acceleration. You should install the Windows native Webots only. The `WEBOTS_HOME` environment variable doesn't need to be set unless you want to run extern controllers.

##### SeanLuTW 05/18/2022 10:26:05
Hi, I am using `IndexedFaceSet` to define my Solid, and I found that there is a `normalPerVertex` field. I am confused why a vertex has a normal？

##### DDaniel [Cyberbotics] 05/18/2022 12:58:20
There's two ways of defining the normals for `IndexedFaceSet`. If `normalPerVertex` is TRUE then `normalCoordIndex/normalIndex` is used and has to be specified for each vertex. If it's FALSE, the normals are defined per-face

##### moebius 05/18/2022 22:13:33
how does a group/transform differ from a solid (or shape), and when would you use multiple bounding boxes?

##### AlexandrosNic 05/19/2022 10:41:54
Thank you. I uninstalled webots from WSL and only have it on Windows side. Now when I type "webots", is not opening (command not found). But it should open since when I type the name of Windows apps (like notepad, or notepad.exe) they open. the only way to make open it from the windows side is to type "/mnt/c/Program\ Files/Webots/msys64/mingw64/bin/webotsw.exe". This is not a solution because I want to open it through ROS. Do you know how I can workaround this problem? (maybe this is more of a wsl question)

##### Olivier Michel [Cyberbotics] 05/19/2022 10:46:34
You should probably add the path to the Webots binary in your `PATH` environment variable.

##### AlexandrosNic 05/19/2022 12:20:19
This was not working because the "webots" (windows) shortcut is not working for linux. The solution I found is to create a symlink with: 

```
sudo ln -s /mnt/c/Program\ Files/Webots/msys64/mingw64/bin/webotsw.exe /usr/local/bin/webots
```

(the first part is the directory of the executable, the second part is where to store the symlink) and then add the '/usr/local/bin/webots'  in the PATH variable. You can store it in any directory you want but then make sure to add this directory in path

##### DrVoodoo [Moderator] 05/20/2022 08:17:09
Is there a known issue with point clouds and ROS in webots 2022a? I can't see anything in the issues.


I'm running webots 2022a, with ROS noetic and the included VelodynePuck proto on a Car


Using the provided ros\_automobile as the controller.


I can enable the sensor and the pointcloud using the ros service calls and the sensor turns on fine


range\_image looks correct and I get a sensor\_msgs/PointCloud2 topic with messages being published


*BUT* nothing can interpret the data in those messages


not rviz, rosshow, or if run a custom node and try and convert from the sensor\_msg to a pcl::PointCloud


the custom node is the most promising as at least it gives an error message rather than silently failing to do anything. the pcl::fromROSMsg function call errors with `Failed to find match for field 'x'.` for x, y, and z.


ok so looks like in the RosLidar.cpp file the `fields` and `point_step` fields just aren't getting set

##### goch [Moderator] 05/20/2022 09:13:07
Does someone know how to get the radius of  a sphere in an shape Node?  

The Documentation says, that geometry is of Type SFNode but  calling getSFNode does not work ->[https://cyberbotics.com/doc/reference/shape?tab-language=python](https://cyberbotics.com/doc/reference/shape?tab-language=python)

  



This is what I'm curently trying to do. 

`robot.getSelf().getField("children").getMFNode(0).getSFNode('geometry').getField('radius')`  



Calling this throws Attribut error Node has no attibute getSFNode()



Am I interpreting the Docs wrong?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/977136883035422740/unknown.png)
%end

##### DDaniel [Cyberbotics] 05/20/2022 09:25:01
You need to retrieve the field prior to retrieving the node, same thing for the radius:

`robot.getSelf().getField("children").getMFNode(0).getField('geometry').getSFNode().getField('radius').getSFFloat()`

##### goch [Moderator] 05/20/2022 09:30:34
Ups. Somehow I missed, that getSFNode doesn't take a parameter. Now it makes sense. Thank you.

##### DrVoodoo [Moderator] 05/20/2022 09:36:50
Ok my issue was an issue in the webots code but have fixed it and done a pull request

##### Winner 05/20/2022 23:06:20
Hi, I am pretty new to webot. Currently trying to construct a robotic cell with illustration of human and robot interactions. I am wondering if webot can support change of transparency of the robot or solid's shape node during simulation runs? Thanks a lot !

##### Joshua S 05/21/2022 14:09:37
I have a robot arm with three joints attached to a robot. I have added physics in each arm section of the robot however webots can't seem to detect when the robot arm intersects the robots body. Does anyone have any advice on how to fix this?



The for the robot are structured something like this

- robot

   - children 

      - arm 1 hinge joint

         - arm 2 hinge joint

            - arm 3 hinge joint

   - robot body (colored red)



The picture below shows the robot not detecting collisions with itself
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/977573886302515240/unknown.png)
%end

##### DDaniel [Cyberbotics] 05/21/2022 14:34:23
The robot doesn't collide with itself unless you enable the selfCollision field


In theory yes, you can use a supervisor to programmatically change the transparency of the appearance node in your shape. There's a tutorial that explains how to use a supervisor to access/change fields


[https://www.cyberbotics.com/doc/guide/tutorial-8-the-supervisor](https://www.cyberbotics.com/doc/guide/tutorial-8-the-supervisor)

##### Winner 05/21/2022 18:49:11
Thanks I will check it out

##### Zezo99 05/22/2022 21:34:12
I had this problem after installing webots, I tried several solutions on YT, but all didn't work, so I am not sure if this may be a specific issue to me, or I need to change the version?
%figure
![IMG-20220522-WA0112.jpg](https://cdn.discordapp.com/attachments/565154703139405824/978048158246518814/IMG-20220522-WA0112.jpg)
%end

##### Olivier Michel [Cyberbotics] 05/23/2022 07:54:49
Which version are you using?

##### Zezo99 05/23/2022 14:09:39
R2022a

##### Rico Schillings[Sweaty] [Moderator] 05/23/2022 19:15:15
Are there any known issues with r2022a when upgrading to Ubuntu 22.04?

##### moebius 05/23/2022 21:01:51
sometimes the simulation seems to stop, even though the controller is working and printing out stuff, why would that happen?

##### Stefania Pedrazzi [Cyberbotics] 05/24/2022 06:32:35
Yes, there are a couple of issues using the R2022a Debian or tarball package on Ubuntu 22.04: one preventing to download the textures due to mismatching OpenSSL library and the other is about running sumo for traffic generation.

These issues are fixed on the R2022b nightly build otherwise you can install Webots using the snap package.

##### Rico Schillings[Sweaty] [Moderator] 05/24/2022 06:34:33
Alright, thanks for this information. Then i will give it a try :)

##### Su\_zone 05/24/2022 08:45:12
Wenn I install webots\_ros2, I meet some problems, Failed

<<< webots\_ros2 driver [1.14s, exited with code 11

##### TherealRebecca 05/24/2022 14:29:36
is it possible to add a proto as a child ?


forexample a basenode can have children. Can I add a proto as it's child?

##### Olivier Michel [Cyberbotics] 05/24/2022 16:05:41
Yes.

##### Rithsagea 05/24/2022 18:28:02
Is there a way for physics plugins to access devices on robots?

##### DDaniel [Cyberbotics] 05/25/2022 06:28:39
Only in a somewhat limited way (i.e through physics engine calls to specific devices). The more straightforward approach is to communicate/exchange the necessary information between the physics plugin and a supervisor using the dWebotsSend/dWebotsReceive methods and do these manipulations on the controller side. This example [https://cyberbotics.com/doc/guide/samples-howto#physics-wbt](https://cyberbotics.com/doc/guide/samples-howto#physics-wbt) (or `file > open sample world  > samples > how to > physics`) shows how to exchange sensor data between the two

##### pipppoo 05/25/2022 12:39:31
Does anybody know what might be the reason for this "discretised" point cloud? Seems to be related to this issue ([https://github.com/cyberbotics/webots/issues/3594](https://github.com/cyberbotics/webots/issues/3594)).
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/979000767312723968/unknown.png)
%end

##### Chi 05/25/2022 14:34:41
Does anyone meet the issue when running the vehicle and sumointerface, the voice of the vehicle engine is breaking up? I am using a laptop with RTX 3070 and AMD Ryzen 9 5900H. I am not sure that the issue of my hardware capacity or the webots issues.

##### AlexandrosNic 05/25/2022 15:40:22
hey, anyone tried to run webots in a docker container in Windows (not WSL) with GPU? Does it work?

##### moebius 05/25/2022 18:15:47
i have run it in a docker container. Have not figured out how to run it with GPU. I ran it headless with xvfb and disabled rendering


I have a wbt file that shows that behavior, i have no idea why the robot just stops moving, even though the controller keeps printing stuff. Could someone tell me why that might be happening. The geometry is being generated, hence it is a little weird, but i am using simple bounding boxes wherever I can.
> **Attachment**: [pivotcar\_flat\_locking.zip](https://cdn.discordapp.com/attachments/565154703139405824/979085939345346590/pivotcar_flat_locking.zip)

##### Rithsagea 05/25/2022 21:20:25
thank you!

##### moebius 05/26/2022 16:29:37
Could someone pls help me with this

##### áçè 05/27/2022 05:07:56
how to shift the axis in center plz help?
%figure
![Screenshot_2022-05-27_103509.png](https://cdn.discordapp.com/attachments/565154703139405824/979611897601073252/Screenshot_2022-05-27_103509.png)
%end



%figure
![Screenshot_2022-05-27_103407.png](https://cdn.discordapp.com/attachments/565154703139405824/979611977833938964/Screenshot_2022-05-27_103407.png)
%end

##### SeanLuTW 05/27/2022 07:26:56
Hello, I am using a SCARA robot arm PROTO converted by a URDF file and using a simple controller to set the joint angle. Everything works find if I just set link 3 (spline link part), but if I adjust the other part, the weird behavior happened: the spline separated from the arm (as right figure). What may cause the problem and how can I fix it? Thanks in advance!
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/979646875793752094/unknown.png)
%end


I used the generic robot window to control the joint angle
> **Attachment**: [arm.mp4](https://cdn.discordapp.com/attachments/565154703139405824/979648913759956992/arm.mp4)

##### Su\_zone 05/27/2022 11:08:02
Has anyone installed tiago++ in webots\_ros2? I just see the video , ros2 launch webots\_ros2\_tiago tiago.launch.py.  But I can't find tiago.launch.py document.

##### Mars\_J 05/27/2022 14:24:26
I have a newbie question guys. Can webots simulate winds?

##### Rithsagea 05/27/2022 19:28:42
whenever i try to send the character `|` through an emitter, webots seems to inexplicably freeze and crash. Is there any way to mitigate this, or should i just another character?

##### áçè 05/29/2022 03:59:53
any one have any suggestions?


plz help me out

##### Winner 05/29/2022 16:05:13
Hi, I have that issue before. If you import your model from a CAD like stl, you need to redefine its orientation on your CAD software

##### Chi 05/29/2022 20:15:46
I am wondering how could I change the driving view into the following vehicles in webots? Also, I found the following figure on the websit. Is that from webots?



%figure
![Screenshot_20210204_163925_com.google.android.youtube.jpg](https://cdn.discordapp.com/attachments/565154703139405824/980565240590766121/Screenshot_20210204_163925_com.google.android.youtube.jpg)
%end

##### Stefania Pedrazzi [Cyberbotics] 05/30/2022 06:42:39
Yes, this image is representing a Webots automobile simulation setup.

To get an inboard camera view you have to:

1) open the context menu (right-click on the car in the scene tree)

2) select the option: "Follow object" > "Mounted Shot"

3) move the viewpoint to the desired view inside the car


I just checked and added a `|` character to the hello message in the `emitter_receiver.wbt` and Python `example.wbt` sample simulations distributed with Webots and everything is working fine for me.

I would suggest you to try as well with the default simulations to make sure that the issue is not in your controller program. Then, you should provide more information and at least specify the OS, Webots version, and the programming language.

##### AlexandrosNic 05/30/2022 09:46:32
I also had this problem on Win10, but not on Win11. Did you find any solution to it?

##### Kugelkopf 05/30/2022 10:30:32
Yes, its a driver issue. I reported on the WSLg github repository, then i closed the ticket.



We made the same setup for a collague(win 10, x-launch and webots) everything is working OK, on his laptop. On mine it's crashing or blacking out.

I have 2 or 3 years older GPU and CPU , my collague has newer hardwares.



I think its better to change for win 11 as this whole wsl GUI experience was intended for win11. I understand its not possible for some people (company policy, or hardware is not supported for win11), but until that there is only this workaround which works kinda unstable.

##### áçè 05/30/2022 16:58:20
i tried that but didn't got the result

##### AlexandrosNic 05/31/2022 09:07:38
Win11 is also not the best solution since there is no support for OpenGL, so impossible to use Webots with GPU

##### Olivier Michel [Cyberbotics] 05/31/2022 09:09:04
Why do you need to run Webots in WSL? Can't you run it on native Windows?

##### Kugelkopf 05/31/2022 09:17:47
Actually microsoft ships support for a certain extent. This is stated on microsofts site and also benchmarked by nvidia with some software. So when running programs inside wsl with GUI enabled, the GPU is clearly working. 

If you use a VM to run a Linux and use webots inside the VM , then there is no workload on your GPU

If you do the same with WSL 2 , you can see that the GPU is working. 

Of course running webots on a native linux will produce better performance,  but wsl 2 with the right settings gives you significant performance boost compared to a conventional VM.

Win 11 is easier to setup than win 10 and has better support for running linux GUI apps inside wsl

##### AlexandrosNic 05/31/2022 09:20:59
Only to couple it with Ubuntu's ROS2 through WSL (ROS is more stable in Ubuntu)

##### AndrewP 05/31/2022 23:30:35
I'm trying to run


[http://wiki.ros.org/webots\_ros/Tutorials/Sample%20Simulations#Simulation\_Pionneer\_3\_AT](http://wiki.ros.org/webots_ros/Tutorials/Sample%20Simulations#Simulation_Pionneer_3_AT)


however, the error happens which saying:

REQUIRED process [webots-1] has died!

process has died [pid 371545, exit code 1, cmd /home/**/**/devel/lib/webots\_ros/webots\_launcher.py


What else am I missing here?


For the first example, it has an such additional error: 

Could not find parameter robot\_description on parameter server

## June


Solved it. It was webots\_home environment issue

##### Winner 06/02/2022 00:16:02
Sorry have been too busy. I used solidworks to define a new coordinate system and save the step for the new coordinate system then repoen it and save it as stl so it has the new coordinate system in webot

##### Kugelkopf 06/02/2022 12:48:46
Hi, how can i set relative path in .proto files which is in a ros2 package?


So lets say my launch file would respect an environment variable which is in the .proto file


url "$my\_exported\_path/resource/mymesh.stl"


It doesnt work


Or can some show me how would you define a relative path in a .proto file, which is not a web url?

##### áçè 06/03/2022 10:07:40
can someone plz tell me what the diff between physics(base node ) and use gear physics ?



%figure
![Screenshot_2022-06-03_153759.png](https://cdn.discordapp.com/attachments/565154703139405824/982224152503144468/Screenshot_2022-06-03_153759.png)
%end

##### DDaniel [Cyberbotics] 06/03/2022 12:53:55
if it's show under the "USE" category, it means it's something defined by a "DEF gear" somewhere else, likely something you defined yourself

##### Emerson Maki 06/03/2022 15:46:24
Using the UR10 arms that are found within the demo, I am struggling to find their true 0 position. Is a joint's position updated and then saved such that on world creation, wherever it was previously is now the new zero point?

##### Sergey Semendyaev 06/03/2022 17:16:28
Hi! We've used Webots LoLa Controller ([https://github.com/Bembelbots/WebotsLoLaController](https://github.com/Bembelbots/WebotsLoLaController)) for RoboCup SPL field. We did as it mentioned in instruction - connected to UNIX socket via command: nc -U /tmp/robocup . In Webots we have message: Lola Client Connected. But right after that in Webots time stops. Why time stops? How can we solve this problem?

##### AndrewP 06/03/2022 18:42:53
I installed webot but the path snap/webots is empty. Where should I install them?


It's for defining Webot\_Home path

##### Winner 06/03/2022 22:20:53
Hi, It might be the reason that your URDF is not defined correctly and the axis position for the joint is not correct

##### Olivier Michel [Cyberbotics] 06/04/2022 07:20:30
You should probably open an issue at [https://github.com/Bembelbots/WebotsLoLaController/issues](https://github.com/Bembelbots/WebotsLoLaController/issues)

##### áçè 06/04/2022 21:02:37
i m facing this problem what should i do to avoid this?
> **Attachment**: [empty\_3.mp4](https://cdn.discordapp.com/attachments/565154703139405824/982751252217757707/empty_3.mp4)


anyone???

##### Eolo 06/05/2022 02:01:37
hello


my camera overlay


is gone


how can i get it back


someone stole it


nvm


it came back to me


my precius

##### Rico Schillings[Sweaty] [Moderator] 06/05/2022 06:37:35
Hey guys. Did someone already create a proto to simulate a stereo camera like Intels realsense or stereolabs zed?

##### Winner 06/05/2022 16:57:56
I don't quite know what's wrong but have you define the contact properties and bonding object correctly? Most of my problem with physics are because of that.

##### Endogan 06/06/2022 07:20:48
Hi Guys, Im glad I found this discord. Im currently working on a project where webots shall simulate a self-designed robot and the actual controller is running in ROS. Now the tricky part: The webots simulator is running on macOS and the ROS part is running on a VM on the same Machine. Do you have an Idea how to connect the webots with the ROS1 environment? Thanks in advance!

##### áçè 06/06/2022 07:59:07
yes i did that but still it is collapsing

##### Endogan 06/06/2022 08:18:06
but how did u get the ROS communication running on macOS ?

##### áçè 06/06/2022 12:11:20
does anyone know how i can i achieve the wheelchair to climb stair .... i  m stuck here ...thanx in advance
> **Attachment**: [empty\_4.mp4](https://cdn.discordapp.com/attachments/565154703139405824/983342324832690226/empty_4.mp4)


in this the cluster should rotate when climbing stairs

##### VRsE 06/06/2022 15:35:39
I am programming a UR10e robot arm through ROS, each time I run Webots, my robot gets a different random name that is included in names of services and topics. How can I fix the name of my robot arm for the purposes of ROS communications, so that the services and topics have a constant address?

##### moebius 06/06/2022 23:50:48
i am using fs90r motor in these robots we are building, and i added the toque value of the same to the torque field in `getattr(self,motor).setAvailableTorque(0.15)` as so, but in the simulation it rotates very slowly, no matter what speed i input ( i am using velocity control for the motors). This is not comparable to real life performance. What could be the reason?

##### Olivier Michel [Cyberbotics] 06/07/2022 06:14:32
Did you check the motor parameters, like `maxVelocity`, `maxTorque`, etc. ?

##### moebius 06/07/2022 06:15:04
it's the default values, i did not change those

##### Olivier Michel [Cyberbotics] 06/07/2022 06:21:08
Did you try to increase them?

##### Chrimo 06/07/2022 17:56:43
Hello Webots specialist, is there a detailed description of ROS2 integration available ? I have the following use case: Using Webots simulation at OSX (latest 12.4) and ROS2 nodes at Jetson Nano or RPI. Problem: there is no ROS2 available for OSX 12.4. How can I make Webots available for ROS2 in the network ? Is there any chance to use OSX 12.4 with Webots without installing or compiling ROS2 at unsupported OSX ? TIA Chrimo


IS there any kind of bridge/gateway or network API to the ROS2 environment available ?


background: all my ROS2 environments are graphically headless 😉


ROS2 at OSX is a nightmare 😭

##### moebius 06/07/2022 21:20:06
the max velocity is already set at 25 and i incresed the maxtorque, but I am setting the available torque to 0.15 Nm already, and that has the problem as i described

##### Danial 06/08/2022 16:36:36
Hello everyone, I'm struggling a bit with the Web simulation. I was able to create a session and add multiple simulation servers, however when I try to connect using the quick start example ([https://cyberbotics.com/doc/guide/web-simulation#quick-start](https://cyberbotics.com/doc/guide/web-simulation#quick-start)) the connection cant be established. 

I have found the reason for this problem; the session tries to connect to a port, which is not a simulation sever (the port is always 1 higher than the highest simulation server port). But unfortunately I cant figure out why it does that. Can someone here help me?

##### Sunni 06/10/2022 09:11:35
Hi, I'm trying to import a VRML file into Webots, but the only files that the import 3D function supports are .obj, .stl, and .dae files. I'm using the R2022a version. Is there a reason why I can't import other file types into Webots?



%figure
![2.png](https://cdn.discordapp.com/attachments/565154703139405824/984747223902478376/2.png)
%end
%figure
![3.png](https://cdn.discordapp.com/attachments/565154703139405824/984747224263168040/3.png)
%end

##### Mars\_J 06/10/2022 13:23:42
Hello guys! Can robots in webots communicate with each other? Like knowing the position of each other. Any help would be great!

##### Benjamin Délèze [Cyberbotics] 06/10/2022 13:37:35
Hi,

A robot that is a Supervisor ([https://cyberbotics.com/doc/reference/supervisor](https://cyberbotics.com/doc/reference/supervisor)) can retrieve other information about other robots.

Otherwise robots can communicate with each other through emitter ([https://cyberbotics.com/doc/reference/emitter](https://cyberbotics.com/doc/reference/emitter)) and receiver ([https://cyberbotics.com/doc/reference/receiver](https://cyberbotics.com/doc/reference/receiver))

##### daz.quintal 06/12/2022 00:25:58
Hi, can someone please tell me if robot Nao can recognize objects? I read that the function wb\_robot\_has\_recognition tells you if it has recognition or not, and for the case of Nao the value it gives is 0, but is there any way to activate recognition on this robot?

##### sEngBots 06/12/2022 09:00:20
[https://discord.gg/ZhtzaRgC](https://discord.gg/ZhtzaRgC)

##### 갈대같은새키ㅇ 06/12/2022 10:12:36
hello. I want to run a webbot program on the cloud Widnow Server-16 OS. But as soon as I run it, the program crashes. Is there any way to make it work?

##### Stefania Pedrazzi [Cyberbotics] 06/13/2022 06:31:47
A camera can recognize objects only if it has a non-empty Recognition object. The Nao default cameras don't have it. But you can modify the Nao PROTO model or add an additional custom camera for example in the `Nao.headSlot` field to enable the camera recognition functionality for the Nao robot.

[https://www.cyberbotics.com/doc/reference/recognition?tab-language=python](https://www.cyberbotics.com/doc/reference/recognition?tab-language=python)


Import of VRML files is no longer supported in Webots.

You should convert your VRML file into one of the supported formats using a 3D graphics editor (Blender, AutoCAD, etc.).

##### mfra 06/14/2022 12:54:12
Hi, I'm developing a custom plugin for communication between ROS2 and Webots. I'm using the webots\_ros2\_driver package for this purpose. Within the plugin I want to trigger a callback with a rate of 100Hz. I created a ROS2 wall timer  and set the use\_sim\_time parameter to true. Unfortunately, the timer is called only every 32ms instead of 10ms. Also, the /clock topic is published every 32ms. Does someone know the reason for this?

##### Olivier Michel [Cyberbotics] 06/14/2022 12:57:41
Did you check the `WorldInfo.basicTimeStep` value of your simulation world file? Setting it to 10 might help.

##### Din96\_Boy 06/15/2022 01:40:29
Hello can anyone help me to answer this question? , I posted it on stackoverflow but i didnt get answers . Thank you [https://stackoverflow.com/questions/72594148/inverse-kinematics-in-python](https://stackoverflow.com/questions/72594148/inverse-kinematics-in-python)

##### áçè 06/15/2022 02:50:55
hi i m writing my code in  cpp but i m getting this error =="initializer-string for 'char [1]' is too long [-fpermissive] "  continuously tough i changed the number


?

##### mfra 06/15/2022 06:07:59
Thank you, this solved my problem! I also had to use rclcpp::create\_timer instead of node->create\_wall\_timer, such that the timer respects the simulated time

##### pluto 06/15/2022 10:45:14
Hi, on the [doc/reference/supervisor]([https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_node\_get\_self](https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_node_get_self)) page, the Supervisor class is defined in Python. Is there anywhere I can find the code for all the methods in the class please? Such as `getSelf`, `getFromDef`, etc. Thanks 🙂

##### Olivier Michel [Cyberbotics] 06/15/2022 13:31:20
The Python interface is basically a wrapper of the C interface. You will find the code in there: [https://github.com/cyberbotics/webots/blob/master/src/controller/c/supervisor.c#L1838](https://github.com/cyberbotics/webots/blob/master/src/controller/c/supervisor.c#L1838). The Python wrapper is generated by SWIG here: [https://github.com/cyberbotics/webots/tree/master/src/controller/python](https://github.com/cyberbotics/webots/tree/master/src/controller/python) from the C++ interface (which inherits from the C code). The new Python API will be simpler with some native Python code calling directly the C API, see [https://github.com/Justin-Fisher/new\_python\_api\_for\_webots](https://github.com/Justin-Fisher/new_python_api_for_webots)

##### pluto 06/15/2022 14:10:27
Excellent, thank you very much.

##### kimmcg 06/16/2022 12:35:13
I just tried the nightlybuild of both 2022a and b on Windows 11 but I get the `WARNING: Windmill (PROTO) > Shape > PBRAppearance > ImageTexture: Cannot download '[https://raw.githubusercontent.com/cyberbotics/webots/2016f6204d81af62f0ad631af2efc2413e28fe6a/projects/objects/buildings/protos/textures/windmill_roughness.jpg](https://raw.githubusercontent.com/cyberbotics/webots/2016f6204d81af62f0ad631af2efc2413e28fe6a/projects/objects/buildings/protos/textures/windmill_roughness.jpg)', error code: 5: Operation canceled` error. This does not happen with 2022a stable version.



I want to try out the nightly build because I want to try out if the collada import works (the new one is CadShape?) and installing from source is a bit tedious on windows...


Anyway, hope somebody has some tips to deal with this? Perhaps I should download these textures locally somewhere.


I saw this happening in the DJI example btw

##### áçè 06/16/2022 12:38:53
i m trying to use distance sensor in front as u can see in the video but while running into an obstacle it is not turning to green and also i need it to turn to right side but not happening can someone know what's the problem ....
> **Attachment**: [wheelchiar-14-6-22\_1.mp4](https://cdn.discordapp.com/attachments/565154703139405824/986973136278929438/wheelchiar-14-6-22_1.mp4)


can someone plz verify what's wrong with my code as sensor is not getting calibrated
%figure
![Screenshot_2022-06-16_181003.png](https://cdn.discordapp.com/attachments/565154703139405824/986974816932352090/Screenshot_2022-06-16_181003.png)
%end


thanx in advance

##### DDaniel [Cyberbotics] 06/16/2022 12:52:57
I'm encountering a similar issue, but it should happen exclusively on R2022b nightlies, not R2022a. You had the issue in both? Also, does reloading the world right after still shows the same error?

##### kimmcg 06/16/2022 13:04:32
It was actualy interesting, with 2022b it showed issues with the DJI world but not with a world with only the blocked floor proto. With 2022a nightly had an issue with the standard blocked floor proto (but got to check if dji also had an issue)


let me reinstall that one again.


so yes, the 2022a nightly build had both an issue with the standard blocked floor proto and the dji world


For me no longer blocking as I will finish my PR based on 2022b nightly with a simple environment, but if you want I can make an GH issue on the repo for the DJI world if this is handier for you?


reloading the world doesn't fix it btw

##### DDaniel [Cyberbotics] 06/16/2022 13:46:11
It seems the windows nightly of R2022a is broken, we are investigating. For you specifically, yes you need R2022b anyway. One way of bypassing the dji issues is to download `assets-R2022b.zip` (from the same nightly) and extract it as explained here: [https://cyberbotics.com/doc/guide/installation-procedure?version=develop&tab-os=windows#asset-cache-download](https://cyberbotics.com/doc/guide/installation-procedure?version=develop&tab-os=windows#asset-cache-download)

##### kimmcg 06/16/2022 13:48:23
Ah yes sounds good. I won't be developing for the DJI world anyway, but it's just my goto try out world and then when I noticed it:) But I'm glad that at least the floor proto works again in R2022b!

##### áçè 06/16/2022 13:51:49
<@787796043987025941> 

Sir can u have a look over my case

##### DDaniel [Cyberbotics] 06/16/2022 14:11:39
did you define the `boundingObject` field of the walls?

##### áçè 06/16/2022 14:12:06
No sir


I will do that


Thanx sir

##### Din96\_Boy 06/17/2022 08:24:01
I'm learning Webots and in this sample robot I can graphically see the Gyro sensor readings , but for the rotational motors it is showing as Nan. Can anyone know how to display the motor velocities in the graphs? Thank you
> **Attachment**: [2022-06-17\_13-45-24-1.mp4](https://cdn.discordapp.com/attachments/565154703139405824/987271385523032084/2022-06-17_13-45-24-1.mp4)

##### Soma 06/17/2022 16:18:04
cant find ROS under languages. any help?

##### Winner 06/17/2022 17:40:14
Hi, I am trying to import a multi robot onto the robotic cell in webot using the a existing working urdf with URDF2Webot importer. However, the multi robot system (proto file) also show up with parts in wrong orientations. Is URDF2Webot supports multi robot systems or is there some other way I can solve this? Thanks a lot!

##### DepressedCoder 06/18/2022 06:32:13
I am unable to run the Matlab example code on webots. Both Matlab and WeBots versions are R2022a. I have followed the instructions exactly as in website. I am able to open Matlab from the terminal window. Please help
%figure
![Screenshot_2022-06-18_at_11.59.38_AM.png](https://cdn.discordapp.com/attachments/565154703139405824/987605640056819742/Screenshot_2022-06-18_at_11.59.38_AM.png)
%end
%figure
![Screenshot_2022-06-18_at_12.00.01_PM.png](https://cdn.discordapp.com/attachments/565154703139405824/987605640404951070/Screenshot_2022-06-18_at_12.00.01_PM.png)
%end


I am using MacOS

##### Olivier Michel [Cyberbotics] 06/18/2022 06:37:39
Please try the nightly build of version R2022b of Webots. We fixed the support for MATLAB in it.

##### DepressedCoder 06/18/2022 06:38:06
Ok . Let me try Thank you `@Olivier Michel`  for quick response


It worked Thanks `@Olivier Michel`

##### áçè 06/18/2022 13:54:39
can anyone tell me why my sensor is still green instead of red  ,  and also when it comes in contact with any object it is still green
%figure
![Screenshot_2022-06-18_192326.png](https://cdn.discordapp.com/attachments/565154703139405824/987716980096241694/Screenshot_2022-06-18_192326.png)
%end

##### DrakerDG [Moderator] 06/18/2022 15:42:05
When the line of sensor ir green, the sensor has detected something

##### áçè 06/18/2022 15:59:48
But sir when it comes in contact with box it down not change color

##### Rico Schillings[Sweaty] [Moderator] 06/18/2022 17:13:32
I'm Not sure but i mean youre lookup table of the sensor is wrong calibrated.. Firstly the rows should be in increasing order (so second row with zeros is invalid) and secondly ive just tested it with this Single row you are using (3 1000 0) and its also Not working for me.



Take a Look here how to define a correct lookup table [https://cyberbotics.com/doc/reference/distancesensor#lookup-table](https://cyberbotics.com/doc/reference/distancesensor#lookup-table)

##### áçè 06/18/2022 17:57:41
sir i tried to change that also but still i got no solution

##### Rico Schillings[Sweaty] [Moderator] 06/18/2022 17:59:10

%figure
![IMG_20220618_191602.jpg](https://cdn.discordapp.com/attachments/565154703139405824/987778514189156382/IMG_20220618_191602.jpg)
%end


Here you have an example of my definition of a ultrasonic sensor as distance sensor (simulating the hcsr04 from arduino). With this table i have no problems (as in todo noise is missing ;))


And youre really sure that you have enabled the boundingobjects of your area border where your wheelchair is driving to?

##### hoi 06/18/2022 19:31:05
im quite new to webots, is this how i'd translate the force\_control example to python?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/987801648497852446/unknown.png)
%end

##### paing 06/18/2022 19:36:15
Hi , I'm totally new to Webots and I just want to use it to introduce my high school students about Robotics. But I saw this discussion on github ([https://github.com/cyberbotics/webots/issues/4112](https://github.com/cyberbotics/webots/issues/4112)) , saying that Webots doesn't support natively for M1 pro chip. This discussion was from January , 2022 , so anything changed for now? or Webots had a plan to support M1 pro chip in the future?

##### DepressedCoder 06/19/2022 05:27:56
R2022b has support for Apple silicon . It will release by July I think


`@paing`

##### Olivier Michel [Cyberbotics] 06/19/2022 06:39:01
I confirm. Meanwhile you can download the latest nightly builds of Webots R2022b which already support natively the Mac M1 chip.

##### Din96\_Boy 06/19/2022 16:59:09
hello anyone know how to solve this one / thank you

##### Mars\_J 06/20/2022 16:05:14
Guys, how can I add a battery to read on the mavic 2 pro? Thanks in advance!



Edit:

Can I also add environmental wind in webots?

##### DrVoodoo [Moderator] 06/22/2022 15:54:23
This is a new one, upgrading to 2022a and the vehicle we have been using has started doing this
> **Attachment**: [bouncy2.mp4](https://cdn.discordapp.com/attachments/565154703139405824/989196663832862771/bouncy2.mp4)


Wheels shouldn't really have any suspension at all


but im not convinced it is a suspension issue, it's almost as though the axles are offset on the wheels?


anyone got any ideas?


right...... so it only happens if you use an IndexedFaceSet as the geometry on it.

##### Toprak 06/23/2022 08:24:47
Hello can anyone help me about my webots project?


I should do my signature with e-puck robot  with python


and i should use coordination system


but i don't know how i can use

##### TherealRebecca 06/23/2022 14:10:48
Hello,



I am using rectangleArena and I want to know if it is possible to get the size of the arena in my robot controller.



I am tryingto create a 2d map of the world. and dont want to hardcode the size of the  arena. I want to be able to use the controller in different sizes of arenas

##### hoi 06/24/2022 00:43:19
where can i find a list python packages for webots?


also, i'm trying to simulate a multispring system of many bodies in a row, i've been toying with the force control example, would i just use a connector to join two of them together?

##### adabas 06/24/2022 07:46:24
Hi all,

any advice why the function getValue() of the build-inlight sensors with the e-puck return only NaNs?

They are enabled with the sampling period of the robot. I am using Webots R2022a.

Thanks

##### Florian 06/24/2022 08:22:07
Hi there,

does someone knows how I am able to add material to a solid? I had added a mesh as solid to webots, but my robot is just falling through it.



Thanks!

##### DDaniel [Cyberbotics] 06/24/2022 09:38:58
You can use a supervisor to read the size field of the RectangleArena object. This tutorial explains how: [https://cyberbotics.com/doc/guide/tutorial-8-the-supervisor](https://cyberbotics.com/doc/guide/tutorial-8-the-supervisor)


You need to specify the boundingObject of the solid, and possibly the physics. The material field is used to define contact properties

##### Florian 06/24/2022 09:44:44
Thanks!

##### DDaniel [Cyberbotics] 06/24/2022 10:24:57
I just checked and it seems to be working, can you show your controller?

##### adabas 06/24/2022 12:46:07
Hi Daniel, thanks for your reply. While cleaning up my code in order to show you my controller. I realized that my getValue() call of the sensor was still in the setup phase, without checking for the sim step to be != -1. 

It does now work for me. I really appreciate your effort checking it and i apologize for the stupid mistake.

##### Buttowski 06/25/2022 14:55:51
Hello guys , how do I use SUMO on a Mac with apple silicon ?

##### Olivier Michel [Cyberbotics] 06/25/2022 16:07:03
You should try the latest nightly build of Webots R2022b which runs on M1 Apple Silicon.

##### viorel\_gheorghe 06/25/2022 16:09:11
`@Olivier Michel` threre is more info regarding custom\_robot\_window\_simple.wbt ?


The info cen be seen in Webots if the html page is opened, but not by external browser

##### Olivier Michel [Cyberbotics] 06/25/2022 17:17:09
Which version of Webots are you using?

##### viorel\_gheorghe 06/25/2022 18:49:46
soory for delay


2022a on Windows

##### Buttowski 06/25/2022 18:51:14
`@Olivier Michel` okay thanks

##### viorel\_gheorghe 06/25/2022 18:51:18
R2022a

##### Olivier Michel [Cyberbotics] 06/25/2022 19:03:44
In Webots R2022b (nightly build), the robot windows open in the web browser.

##### viorel\_gheorghe 06/25/2022 19:03:59
oki, I'll make an udate


this means get it from github


wow! Right of the box!


thanks!

##### Mars\_J 06/26/2022 03:25:25
Guys can you help me with reading the robot's battery value in python?

##### Shubhu 06/26/2022 03:27:31
U first need fetch and publish the battery voltage via a voltage indicator on ros topic and then write a subscriber

##### Arindam 06/27/2022 07:20:38
Hello everyone, can anybody tell me how to make custom sensor nodes in Webots using python? Similar to the base nodes (GPS, Accelerometer etc).

##### áçè 06/27/2022 09:23:41
how should i get the torque generated by motor in this graph ...?
> **Attachment**: [ice\_video\_20220627-145244.mp4](https://cdn.discordapp.com/attachments/565154703139405824/990910281238605894/ice_video_20220627-145244.mp4)


how to read motor torque using c code ?

##### Din96\_Boy 06/27/2022 11:27:35
i also has the same issue. Any one know why the motor reading are showing as "Nan"? , I also check this with a webots provided robot examples and for those the readings are shown


Hello everyone! , Anyone can share some tutorial on how to import robots that are created on external softwares like solidworks to webots? , Thank you in advance

##### cnbarcelo 06/27/2022 12:25:46
Hi! Is in the roadmap creating a connector for Omniverse as some others simulators are doing? Maybe setting the bases and opening for collaboration or so?

##### Stefania Pedrazzi [Cyberbotics] 06/27/2022 13:27:20
Here is the documentation to read the torque of a motor.

[https://www.cyberbotics.com/doc/reference/motor#wb\_motor\_get\_torque\_feedback](https://www.cyberbotics.com/doc/reference/motor#wb_motor_get_torque_feedback)

1) first enable the torque feedback using `wb_motor_enable_torque_feedback`

2) get torque value in your simulation loop using `wb_motor_get_torque_feedback`


As described in the documentation I just posted before, the torque feedback is only available in physics-based simulation. You should check this is the case for your simulation.

##### áçè 06/29/2022 08:35:30
i tried this but i still not getting graph...   1.


1. enable torque in main function


2. and get torque in while loop ?


is this the correct way


plz confirm

##### Stefania Pedrazzi [Cyberbotics] 06/29/2022 08:36:24
You can just get the current sensor measurement. If you want the graph then you have to draw it yourself based on the measurements.


Yes, it is the correct procedure. Additionally you can write all the torque values to a file and then draw the graphs.

##### áçè 06/29/2022 08:38:03
i want this graph to work
%figure
![Screenshot_2022-06-29_140711.png](https://cdn.discordapp.com/attachments/565154703139405824/991623573431128104/Screenshot_2022-06-29_140711.png)
%end


\#include <stdio.h>

\#include <webots/robot.h>

\#include <webots/motor.h>

\#include <webots/brake.h>

\#include<webots/distance\_sensor.h>



\#include <math.h>

\#include <stdlib.h>

\#include <string.h>



int timestep = 0;



int left\_speed = 1;

int right\_speed =1;





int main(int argc, char **argv) {

  wb\_robot\_init();

  

   // Initalize time step

  timestep = wb\_robot\_get\_basic\_time\_step();



 

  // Intialize motors

  WbDeviceTag left\_motor = wb\_robot\_get\_device("motor\_BL::left");

  WbDeviceTag right\_motor = wb\_robot\_get\_device("motor\_BR::right");

  wb\_motor\_set\_position(left\_motor, INFINITY);

  wb\_motor\_set\_position(right\_motor,INFINITY);

  wb\_motor\_set\_velocity(left\_motor, left\_speed);

  wb\_motor\_set\_velocity(right\_motor,right\_speed);

  wb\_motor\_enable\_torque\_feedback(left\_motor , 1000);

  

   while (wb\_robot\_step(timestep) != -1) {

  

      // //Main Motors

      wb\_motor\_set\_velocity(left\_motor, left\_speed);

      wb\_motor\_set\_velocity(right\_motor, right\_speed);

   wb\_motor\_get\_torque\_feedback(left\_motor);

    

   };



  wb\_robot\_cleanup();

  return 0;

}
> **Attachment**: [message.txt](https://cdn.discordapp.com/attachments/565154703139405824/991627726219919392/message.txt)


this is my code ... plz confrm if there is any  mistake?

##### Stefania Pedrazzi [Cyberbotics] 06/29/2022 08:57:52
Ok sorry, I maybe misunderstood what you want.

You don't need to read the torque values in your controller  to enable the graph.

If you enable the torque feedback, it should work out of the box.

Did you try to enable it from the robot window directly by checking the graph check box?

Do you get any warning in the Webots console?

##### Din96\_Boy 06/29/2022 09:01:16
I also tried the method given by `@Stefania Pedrazzi` and was able to get the torque values of the motors. But not for the graphs ,its shown in the command line output in Webots. I also checked the sample projects given in Webots. In most cases in those projects as well in the Robot window the graphs are shown as Nan for rotational Motor.

##### Stefania Pedrazzi [Cyberbotics] 06/29/2022 09:10:12
The graph shows the motor position (linear) or angle (rotational). If you control the motor in velocity the graph is disabled by default because the position is set to INFINITY.

##### Din96\_Boy 06/29/2022 09:11:59
Is there any way we can show the motor velocities in the graph?

##### Stefania Pedrazzi [Cyberbotics] 06/29/2022 09:12:34
Yes, you can modify the default robot window code and create your own and change the values that are displayed

##### áçè 06/29/2022 09:18:29
as u can see in video the value of motor does not change



> **Attachment**: [ice\_video\_20220629-144453\_edit\_01.mp4](https://cdn.discordapp.com/attachments/565154703139405824/991633948004331560/ice_video_20220629-144453_edit_01.mp4)


`@Stefania Pedrazzi` ?

##### Din96\_Boy 06/29/2022 09:25:50
`@áçè` this is the reason why the graph doesnt show any readingas


this one


in your code FOR HERE   wb\_motor\_set\_position(left\_motor, INFINITY); try to give a value insted of giving INFINITY  , it works for me


do for both motors

##### áçè 06/29/2022 10:06:47
`@Din96_Boy`  `@Stefania Pedrazzi`  i m getting the line in graph but not able to see the fluctuations in torque as the vehicle move on unknow terrain
%figure
![Screenshot_2022-06-29_153506.png](https://cdn.discordapp.com/attachments/565154703139405824/991645902097879100/Screenshot_2022-06-29_153506.png)
%end

##### Dnumgis 06/30/2022 07:43:15
Hi, We're converting our files to work with R2022a, and we got everything working correctly again after using the scripts and quite a lot of manual work. But everything looks a bit off. I think the normal vectors of the shape files didn't get converted by the script. Any ideas how to fix that?

## July

##### Olivier Michel [Cyberbotics] 07/01/2022 06:31:28
Do you really need to fix that?

##### Dnumgis 07/01/2022 07:16:45
Not really, but it looks weird and confusing. And I figured if there were a simple way I would prefer to fix it as well


A screenshot to explain. Notice how the rightmost pillar (which has it's own shape file) now has the light and dark sides completely different from the others, and how the orange robot on top has illuminated sides that doesn't match at all how the shadows are cast
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/992344097312493658/unknown.png)
%end

##### DDaniel [Cyberbotics] 07/01/2022 08:22:20
How is the shape defined? It's a Mesh?

##### Dnumgis 07/01/2022 08:25:36
It's a IndexedFaceSet with 

```
    coord Coordinate {
      point [
```

and 

```
    normal Normal {
      vector [
```

and

```
    coordIndex [
```

defined


And the update script only updated the coord part, so the normal vectors are probably all wrong


we might have some calibration boards that doesn't show up right in the cameras that cause some actual problems as well

##### Guido 07/01/2022 13:12:55
Just checking, has anyone successfully compiled Webots in an arm64 environment?

##### Eske 07/01/2022 16:57:20
I am attempting to, but I have not been successful yet.

##### hoi 07/02/2022 05:58:32
if i were to make a compound spring system, would i start with the force\_control example as a base, then just duplicate it, remove one endpoint from one, and reinsert? if so, how do i manage the rotation?

##### DepressedCoder 07/04/2022 07:36:48
I am getting error while trying to compile webots from source . My system configuration is ( Ubuntu 20.04 ARM)
%figure
![20220704_130539.jpg](https://cdn.discordapp.com/attachments/565154703139405824/993420096947105812/20220704_130539.jpg)
%end
%figure
![20220704_130552.jpg](https://cdn.discordapp.com/attachments/565154703139405824/993420097236500631/20220704_130552.jpg)
%end


Tried not working

##### Olivier Michel [Cyberbotics] 07/04/2022 08:15:32
Try "make clean" and then "make release" again.

##### DepressedCoder 07/04/2022 09:01:59
Still same error `@Olivier Michel`

##### Guido 07/04/2022 12:33:08
I got the same thing too


(of course we are talking about an ARM64 environment)

##### Olivier Michel [Cyberbotics] 07/04/2022 12:34:34
Sorry, but I never tried to compile Webots on Linux ARM64, so I cannot help much on this.

##### ClBaze 07/04/2022 15:38:38
Hi, I'm trying to build an old version of Webots from sources but it seems that [https://cyberbotics.com/files/repository/dependencies/linux64/release/webots-qt-5.15.0-linux64-release.tar.bz2](https://cyberbotics.com/files/repository/dependencies/linux64/release/webots-qt-5.15.0-linux64-release.tar.bz2) is no more available. Where can I find it ?

##### Olivier Michel [Cyberbotics] 07/04/2022 15:45:58
Unfortunately we don't keep archives of this... You will have to rebuild it from the instructions here: [https://github.com/cyberbotics/webots/wiki/Qt-compilation](https://github.com/cyberbotics/webots/wiki/Qt-compilation)


Alternatively, you may try to use [https://cyberbotics.com/files/repository/dependencies/linux64/release/webots-qt-5.15.2-linux64-release.tar.bz2](https://cyberbotics.com/files/repository/dependencies/linux64/release/webots-qt-5.15.2-linux64-release.tar.bz2) which should work the same as version 5.15.0.

##### ClBaze 07/04/2022 15:48:26
`@Olivier Michel` Ok thank you !

##### Darth Jon 07/04/2022 20:01:33
Hello guys

I need to model a Coveyor Belt that has a closed loop geometry, I'm trying to understand how the ConveyorBelt.proto works. Then I came through these %< and %<= tags... it seems to work like an operator and a tag sometimes... can anyone help on this?

##### DDaniel [Cyberbotics] 07/04/2022 20:14:20
They are JavaScript scripting statements and allow to define the proto in a procedural way: [https://cyberbotics.com/doc/reference/javascript-procedural-proto](https://cyberbotics.com/doc/reference/javascript-procedural-proto)

##### Darth Jon 07/04/2022 20:16:44
Yeah, I've read the proto documentation


I just thought it'd save me time if I used a simple coveyor proto as a template for my own. I'm just having a hard time understanding this specific proto as it seems to use a different kind of coding... I've found that %>% is an operator used to simplify the code, but in the ConveyorBelt.proto file it seems not to be the same operator...

##### DDaniel [Cyberbotics] 07/04/2022 20:28:05
It's not an operator, anything between `%< ... >%` is pure JavaScript, anything between  `%<= ... >%` is JavaScript but evaluated (so `%<= abc >%` will be replaced by the value of abc)  and anything else is VRML

##### Darth Jon 07/04/2022 20:39:16
Tyvm


I'm not used with javascript insertions on VRML


Thank you for answering

##### ClBaze 07/05/2022 10:04:13
Hello `@Olivier Michel`, I'm still trying to build my old version of Webots, could you tell me where to find (or how to rebuild) [https://www.cyberbotics.com/files/repository/dependencies/linux64/release/ros\_kinetic\_ubuntu16.04.tar.gz](https://www.cyberbotics.com/files/repository/dependencies/linux64/release/ros_kinetic_ubuntu16.04.tar.gz)

##### Olivier Michel [Cyberbotics] 07/05/2022 10:14:47
That's very old. Unfortunately, I cannot help on this one... But why do you need to build such an old version?

##### ClBaze 07/05/2022 11:31:29
I wanted to run an old simulation from two years ago. But I think I have a docker image somewhere

##### simror 07/07/2022 08:35:17
Hi, I'm trying to model a conveyor as an endpoint of a slider joint, where the slider joint is meant to lift the conveyor up. However the required physics field of a Track proto and related solid nodes is causing a lot of problems for my simulation (cant get the slider joint to behave properly when i place solid nodes with defined physics field on top of the conveyor track). Is it possible to model a functional conveyor solely kinematic? Any suggestion on how to resolve my issue?

##### vuwij 07/08/2022 02:07:02
Hi, i am wondering if there is a automatic way to start video recording or HTML recording when webots starts in a batch mode


I think my question is solved it is the movieStartRecording in the supervisor node

##### SeanLuTW 07/08/2022 03:26:28
Hi, how can I reconstruct depth of each pixel from PNG image saved by `RangeFinder`? From document, it used 8UC3 to encode the depth data instead of the commonly used 16UC1 format

##### mbana 07/08/2022 12:34:25
Hello, everyone..


My team and I are working on dji tello drone project. I didn't find the drone in the simulation...how can I go about this

##### Din96\_Boy 07/09/2022 17:20:42
Hello , does anyone know how to hide and unhide objects in webots?

##### áçè 07/11/2022 05:50:58
hello everyone , does anyone know how to have "vehicle path planning"  using mathlab and webots ?

##### Kirlin 07/11/2022 23:25:26
Hello, I'm from robot's behavior area on my team and we were trying to develop a simulator thats independent of another areas, one in which all movements would be performed by translations and rotations through a supervisor. However, while we are running our custom controller, which receives movement requests via ros to transform into rotations and translations, the webots stops stepping out of nowhere, without issuing any error. Could anyone help me with this issue?


We've tried debugging in several ways, but it doesn't seem to be an error specifically in our code, maybe in its interaction with webots, but there is no error message.


We've read about gdb for more accurate debugging, but we couldn't find a way to run our custom controller launch with it.

##### Winner 07/12/2022 01:39:48
Hi, I am pretty new to webots. Is there anyway I can use controller or codes to save my current world file into a new file? I am currently using Linux and controlling webots via ROS. Thanks you very much!

##### Addy 07/12/2022 01:56:04
Hi folks is there any **online video course for webots** I can enroll/buy?



I only see courses for ROS2: [https://www.theconstructsim.com/robotigniteacademy\_learnros/ros-courses-library/](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/)



but Can't find webots <:frogcry:978088886129537024>

##### Stefania Pedrazzi [Cyberbotics] 07/12/2022 06:00:44
Here is the documentation of the Supervisor API function to save the world:

[https://cyberbotics.com/doc/reference/supervisor?version=master&tab-language=ros#wb\_supervisor\_world\_save](https://cyberbotics.com/doc/reference/supervisor?version=master&tab-language=ros#wb_supervisor_world_save)

##### paing 07/12/2022 06:00:44
Latest Webots 2022b nightly build works well for Apple M1 chip. Thanks Webots team for the update.



One of my friends tested to control webots from Scratch Programming using Robot Operating System (ROS). I knew it worked well. But using ROS as middleware can cause troubles for the high school kids who started learning Robotics. I just want to know - are there any alternative ways to use/learn Webots from Block Programming (like Scratch or Snap) ? 

(I tried to find the answers on the Internet but I didn't find any possible workflow for this. That's why I posted here. Sorry for my dump question)

##### Benjamin Délèze [Cyberbotics] 07/12/2022 06:05:02
You can use the `wb_supervisor_node_set_visibility` function if you want to hide the node from a specific device/viewpoint ([https://www.cyberbotics.com/doc/reference/supervisor?tab-language=c#wb\_supervisor\_node\_set\_visibility](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=c#wb_supervisor_node_set_visibility))

Otherwise, you can use a `supervisor`to move the node far away to hide it or to change its transparency by modifying its appearance ([https://www.cyberbotics.com/doc/reference/supervisor](https://www.cyberbotics.com/doc/reference/supervisor))

##### Stefania Pedrazzi [Cyberbotics] 07/12/2022 06:08:07
No, the `Track` node works only physics-based simulation and not in kinematic mode.

Unfortunately you will need to properly adjust the physics of all your objects in order to make it work correctly and be robust.


If you want to debug your custom C/C++ controller with gdb, then you should execute as an "extern controller" that is started manually from a terminal instead of letting Webots automatically start it.

Here is the documentation describing how to use "extern controllers":

[https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers)

Then, from the terminal you can easily start the controller with gdb.


Thank you for the feedback on the Apple M1 chip!



Webots doesn't currently support any block programming language out of the box.

The best available solution at the moment is to use Python that is probably the "easiest" programming language instead of using ROS.



We developed some custom web-based versions for kids robotics classes in the past using block programming. If you are interested in such a customized solution we could provide you an offer for it.

##### H-Man19 07/12/2022 11:36:25
Hello


My webots keeps throwing up this error when I try to run


INFO: epuck\_go\_forward: Starting controller: /Library/Frameworks/Python.framework/Versions/3.10/bin/python3.10 -u epuck\_go\_forward.py

Traceback (most recent call last):

  File "/Users/harrylongbottom/Documents/Webots/my\_first\_simulation.wbt/controllers/epuck\_go\_forward/epuck\_go\_forward.py", line 1, in <module>

    from controller import Robot, Motor

ImportError: cannot import name 'Robot' from 'controller' (/Library/Frameworks/Python.framework/Versions/3.10/lib/python3.10/site-packages/controller/\_\_init\_\_.py)

WARNING: 'epuck\_go\_forward' controller exited with status: 1.


I feel like it is something to do with my python library path

##### SeanLuTW 07/12/2022 13:14:29
I have tested the saved image. I set the `minRange` and `maxRange` of `rangeFinder` to 0.2 and 10 respectively and resolution 640*480 (other with default values). Then put two cubes in front of the camera, one for 1 meter and 2 for the other. I saved the depth image and using OpenCV to read it. The values are 18 (for 1-meter-far cube), 44 (for 2-meter-far cube) and 249 (distance more than 10 meter).  I wonder why the maximum value is not 255?

##### Naxi 07/12/2022 18:00:14
Hi everyone, I'm having issues when adding js script in my proto, it's like it doesn't recognize the header comment indicating javascript as the template language, and it looks for lua template statement opening. I'm getting this error

`error: Unexpected template statement opening. Expected='{', Received='<'.`


Do the protos have to comply with something in particular to be able to use js?

##### DDaniel [Cyberbotics] 07/12/2022 18:01:14
Can you post the proto?

##### Naxi 07/12/2022 18:01:16
The header in my proto is

```#VRML_SIM R2022a utf8
\# template language: javascript
```

##### DDaniel [Cyberbotics] 07/12/2022 18:02:15
That's correct, what about the rest?

##### Naxi 07/12/2022 18:11:32
```#VRML_SIM R2022a utf8
# template language: javascript

PROTO MyRobot [
    field SFVec3f       translation         0 0 0
    field SFRotation    rotation            0 1 0 0
    field SFBool       synchronization     TRUE                
    field SFBool       static              FALSE               # Disables physics to use the model in edit mode
]
{
  Robot {
    translation IS translation
    rotation IS rotation
    children[
      DEF BASE Shape {
        appearance PBRAppearance {
          baseColor 0 1 0
          roughness 1
          metalness 0
        }
        geometry Mesh {
          url [
            "meshes/base.stl"
          ]
        }
      }
    ]
    boundingObject Shape {
      geometry Mesh {
        url [
          "meshes/base.STL"
        ]
      }
    }
    %< if (!fields.static.value) { >%
      physics Physics {
        density -1
        mass 600
        centerOfMass [
          0.7 0 0.37
        ]
        inertiaMatrix [
          597.4929817231915 1387.5717914243987 868.0371961666204
          3.968686067800202e-08 -597.5280842684364 2.9335525566202445e-08
        ]
      }
    %< } >%
    controller "<extern>"
    supervisor TRUE
  }
}
```

##### DDaniel [Cyberbotics] 07/12/2022 18:16:22
works for me, the cache is likely corrupted. In the same folder of the proto, look for a file called `.MyRobot.cache` and delete it


it's a hidden file

##### Naxi 07/12/2022 18:17:06
Yep, tried that too, weird. Well, at least the proto is syntax is correct


I'll look for that then, thanks


Indeed, copying the contents of the file to a new proto seems to solve the issue. Although there aren't any cached files in the original folder, I wnder what might be


For some reason, if the file extension is PROTO, it doesn't recognise js, but with lower case .proto it does

##### Addy 07/13/2022 02:47:27
noob question



I tried to convert the MotorBikeSimple proto base node.



However I can't see the controller field, how to add a controller field to this model?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/996608772128657549/unknown.png)
%end

##### owuraku-zenas 07/13/2022 02:58:58
Hello, I'm new to ROS and webots. I want to know if the ROS controller in webots works for any robot

##### DrakerDG [Moderator] 07/13/2022 03:07:42
Hi! You need a robot node to have a controller field.  In the children filed of your robot node you can put your motorbikesimple

##### Addy 07/13/2022 03:08:20
I see, thank you very much <:blobpinkkirby:978067996381679666>

##### Adithya 07/13/2022 05:39:19
I'm new to ROS

I have a Rosbag file and 3 topics in that bag file


For the first topic which is of type sensor\_msgs/Image

It has different fields


One of the fields is data


Data according to the documentation should be an array


But in the rosbag file it's a vector


It is supposed to be a 2 channel array


How do I extract only the first channel?


All this in C++

##### Mars\_J 07/13/2022 11:59:22
Hello guys, we have a project that makes an aerial mesh with quadcopters. How do I go about creating text messages to relay with the mesh. Do I need to create a robot window for the robot to send a text message. I already know how to move the quadcopters, create a formation and relay string messages.

##### Winner 07/13/2022 22:01:53
Hi. I am currently facing issue with robotic assemblies project with webots and there multiple physics nodes has multiple connectors. I am wondering if connector can be specified to connect to a specific reference connector nodes? Thank you very much!

##### Addy 07/14/2022 02:51:54
Hello!



Is there an auto-complete feature in Text-Editor when using Webots Python API? just like Webots C API

##### Din96\_Boy 07/14/2022 03:09:42
Hello , Can anyone help to solve this issue? , When I start the simulation for my robot the caster wheels of my robot are not connected to the robot body ? Does anyone know how to resolve this issue? , I have recorded an video explaining my issue Here : [https://streamable.com/v4mvyw](https://streamable.com/v4mvyw)

##### DrakerDG [Moderator] 07/14/2022 03:21:48
Yes, I can help you.



You need make some changes in the children of your tree structure, from your robot in children field, to firts hinge join, from solid children field of your first hinge join to second hinge join. From solid children field of your second hinge join to wheel.



If it is possible, you will can share your world and I will make the corrections

##### Stefania Pedrazzi [Cyberbotics] 07/14/2022 05:58:46
Yes, the Webots default ROS controller is generic and works with any robot.


No, the Webots build-it text editor has a limited auto-complete functionality for Python just including the API classes.


Not sure if I understood correctly the question.

The Connector node has a `model` field that you can set so that two Connector nodes can connect only if their model strings are identical.

[https://www.cyberbotics.com/doc/reference/connector#field-summary](https://www.cyberbotics.com/doc/reference/connector#field-summary)

##### Addy 07/14/2022 06:31:33
thanks! I used PyCharm and followed the tutorial. And autocomplete works <a:catjump:978067996956307466>

##### owuraku-zenas 07/14/2022 08:30:25
Oh okay. Right. I added a Lidar sensor to my robot but I couldn't access the lidar topic.(It's a NASA Sojourner Rover with the lidar). I can't tell what I have left out or whether I added the Lidar wrongly
%figure
![webots.png](https://cdn.discordapp.com/attachments/565154703139405824/997057469580841071/webots.png)
%end
%figure
![topics.png](https://cdn.discordapp.com/attachments/565154703139405824/997057469920583730/topics.png)
%end

##### Stefania Pedrazzi [Cyberbotics] 07/14/2022 08:38:17
Did you save and reload the world after adding the lidar?

Did you already try the examples [http://wiki.ros.org/webots\_ros/Tutorials/Sample%20Simulations](http://wiki.ros.org/webots_ros/Tutorials/Sample%20Simulations) ?

I would suggest you to start from a default example to debug why it doesn't work.

##### owuraku-zenas 07/14/2022 08:41:50
I saved and reloaded after the lidar.


I tried the examples and they are working fine. All sensors appear as topics



%figure
![pioneer3at.png](https://cdn.discordapp.com/attachments/565154703139405824/997061628963070002/pioneer3at.png)
%end
%figure
![p_topics.png](https://cdn.discordapp.com/attachments/565154703139405824/997061629223112864/p_topics.png)
%end

##### SeanLuTW 07/14/2022 09:06:42
Hi, I am using `Keyboard` to detect pressed key when the simulation is running. From the document, it mentioned that it can detect up to 7 simultaneous keys pressed, and the notes show how to detect `Ctrl+B`. I wonder is it possible to detect simultaneous pressed of multiple alphabet keys, says `A+B`?

##### row 07/14/2022 13:14:28
I have generated a sequence of control inputs over some time horizon from a motion planning algorithm. What would be the best way to pass into Webots and let the robot execute the controls at specific time instances?

##### DrakerDG [Moderator] 07/14/2022 13:55:13
Maybe you can using a variable like counter and combine with some ifs

##### row 07/14/2022 14:14:39
Thank you. I have figured out something. I set up my motion planner to generate controls with 64ms interval between each control updates. The interval matches my robot's time step. And then I stored those values into a list and use a counter to incrementally update my controls. To ensure the control is updated at the specific instance, I added t += timestep/1000.0 in my while loop. Everything runs perfectly.


I think the tricky part is ensure the robot control timestep and sim timestep are set  up correctly. The sim timestep is always less or equal to the control timestep. In my case, I have sim timestep = 32 and control timestep = 64. Then, the planner needs to spit out control sequence with 64ms in between.

##### Yasmine Masmoudi 07/14/2022 14:20:49
Heyy, where can i find the preferences window in webots please?


where can i find this:
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/997146302997610529/unknown.png)
%end

##### Benjamin Délèze [Cyberbotics] 07/14/2022 14:24:15
On Windows and Linux it is in `tools -> preferences...`, on MacOS I think it is `Webots->preferences`

##### Yasmine Masmoudi 07/14/2022 14:25:42
Thank you.


is it possible to import a trained model ( jupyter notebook) in my controller code?

##### row 07/14/2022 15:25:06
By trained model, do you mean a machine learning model?


I was reading something this morning and I encounter this: [https://cyberbotics.com/doc/guide/using-numerical-optimization-methods](https://cyberbotics.com/doc/guide/using-numerical-optimization-methods), which might be relevant to your case. You probably want to load your model in a separate supervisor controller. I don't think jupyter notebook is directly supported, so you need to convert it into .py file.

##### Yasmine Masmoudi 07/14/2022 19:20:29
I see.  found this [https://github.com/RobInLabUJI/WebotsLab](https://github.com/RobInLabUJI/WebotsLab)


I'll try both methods


thank you

##### row 07/14/2022 19:21:05
Cool. Good luck!

##### Darth Jon 07/14/2022 23:14:12
Hello everyone, I'm trying to model a circular coveyor belt, I succeed to manipulate the borders in the ConveyorBelt.proto file to create the border of a "L" shaped coveyor belt. Then I tried to replace the Box in the "geometry DEF BELT-B0 Box" with a IndexedFaceSet but it's not working at all, the entire node disappears from the simulation... 



Did any of you have try anything like this? Do you have any suggestion or advice?

##### DepressedCoder 07/16/2022 17:00:57
I am unable to set a color to the display (Line no 55). I am using Webots 2022a. Is there anything particular I forgot. I added a display node as a child of my robot  and referenced it in the control program. The width of the display is printed correctly.
%figure
![Screenshot_2022-07-16_at_10.28.54_PM.png](https://cdn.discordapp.com/attachments/565154703139405824/997910724775006279/Screenshot_2022-07-16_at_10.28.54_PM.png)
%end

##### Alex0u0 07/16/2022 19:00:09
Hi everyone maybe a noob question but is there a way to pass a value extracted from an html window plugin into the controller of a robot?, i wanted to send float numbers from an html window to the controller but maybe there is no way of doing it, thank you for answering in advance

##### madbat09 07/17/2022 16:50:51
Hello guys I am having some trouble finding the answers in the documentation thats why I'm writing here:

1. Is there a possibility to get the speed/velocity of a solid without it being a robot if so how?

2. I have a ball on a slope im trying to get the velocity at a given x position. I found that the Accelerometer has a lookup table for all the values captured. How can I combine those values with the current position of the ball and write them out ?

##### Stefania Pedrazzi [Cyberbotics] 07/18/2022 06:06:50
Yes, you can send data from the robot window to its controller.

Here is the documentation of the robot window: [https://www.cyberbotics.com/doc/guide/controller-plugin#robot-window](https://www.cyberbotics.com/doc/guide/controller-plugin#robot-window)

it also links to an example showing how to do it.


You can get the velocity of a Solid using the Supervisor API function `wb_supervisor_get_velocity`:

[https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_velocity](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_velocity)

For retrieving the position of a Solid you could use this Supervisor API function: `wb_supervisor_node_get_position`

[https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_position](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_position)


The code seems correct. Note that it is the instruction `display->drawRectangle()` at line 59 that actually draws the outline of a rectangle into the display image. If you want a filled rectangle you should use `display->fillRectangle()`.

##### DepressedCoder 07/18/2022 06:18:52
Got it. Thank you

##### Florian 07/18/2022 06:57:21
Has someone experience how to create a boundingObject for an uneven mesh?

##### Sunni 07/18/2022 09:13:59
Hi, I'm trying to integrate a Qt GUI as a robot window controller plugin using the nex firebird6 robot as a starting point. However, there was no call to wb\_robot\_window\_init() and wb\_robot\_window\_step(), and there was no insertion of wb\_robot\_wwi\_receive\_text() and wb\_robot\_wwi\_send\_text() in the example's controller program, so I was wondering if I should put that in for the robot window to communicate with the controller. Also, this may be a separate problem, but after trying to run my simulation of Webots, I am currently getting the Error: robot window invalid library name. Error: Cannot load the "" robot window library. How should I proceed from here? I am using Webots 2021a. Thank you so much!

##### Stefania Pedrazzi [Cyberbotics] 07/18/2022 09:20:05
Qt robot windows has been deprecated since Webots R2020a.

It is strongly suggested to use HTML robot windows instead:

[https://www.cyberbotics.com/doc/guide/controller-plugin?version=R2021a#robot-window](https://www.cyberbotics.com/doc/guide/controller-plugin?version=R2021a#robot-window)

##### Florian 07/18/2022 09:23:17
Already got it!

##### Sunni 07/18/2022 09:29:56
Got it, I will try it. Is it possible to insert the running simulation itself inside the HTML robot window, next to some actuator commands etc? Perhaps something like this, but displayed in real time as the simulation runs, so I can sort of run it on the separate window and, if possible, embed it in another application? [https://www.cyberbotics.com/wwi/R2019a/webots.js](https://www.cyberbotics.com/wwi/R2019a/webots.js)



Thank you!

##### Stefania Pedrazzi [Cyberbotics] 07/18/2022 09:33:12
Theoretically it should be possible but we never tried it.

##### Sunni 07/18/2022 09:35:24
I'm looking at the robots/thymio/thymio2.wbt sample simulation, and there seems to be no Makefile for the robot window. Is that recommended?

##### Stefania Pedrazzi [Cyberbotics] 07/18/2022 09:39:55
It depends on the project. Often it is possible to write all the logic in Javascript directly.

But it is still possible to integrate C code. You can find an example in `robots/gctronic/plugins/robot_windows/e-puck/`

##### Sunni 07/18/2022 09:52:43
Thank you! Apologies for all the questions, but I'm having a little trouble understanding the Javascript code. May you explain what the argument for something like document.getElementbyId("prox.horizontal.0").innerHTML = values[0] means (in the thymio robot window js file)?


Is there documentation that translates robot windows wwi related functions (ex. wb\_robot\_wwi\_receive\_text(),  wbu\_default\_robot\_window\_configure()) from C to C++?

##### madbat09 07/18/2022 10:36:13
Hey Stefania thanks for the quick answer. I am now trying following 

1. getting the node by def to then track the position by 

```
from controller import Supervisor as sp
defName = "solid"
node = sp.getFromDef(defName)
```

 but I get following error msg:   

```
node = sp.getFromDef(defName)
TypeError: getFromDef() missing 1 required positional argument: 'name'
```

##### Stefania Pedrazzi [Cyberbotics] 07/18/2022 11:21:26
`getFromDef` is not a static method but a class method and you have to call it from a Supervisor class instance.

Please read the documentation to learn how to write controller programs:

[https://www.cyberbotics.com/doc/guide/controller-programming?tab-language=python](https://www.cyberbotics.com/doc/guide/controller-programming?tab-language=python)

##### madbat09 07/18/2022 11:22:53
Thanks alot for the tipp

##### Stefania Pedrazzi [Cyberbotics] 07/18/2022 13:25:47
The robot window code can only be programmed in C (or javascript) but not in C++.

The documentation for the functions to communicate with robot window from a controller are documented here:

[https://www.cyberbotics.com/doc/reference/robot?tab-language=c++#wb\_robot\_wwi\_receive](https://www.cyberbotics.com/doc/reference/robot?tab-language=c++#wb_robot_wwi_receive)

##### Berryman 07/18/2022 17:56:48
Hey everyone!! 👋 



How do I turn 2 cubes into a 3rd cube in Webots?



I'm trying to simulate an assembly factory. In which, several robots gather the necessary parts (objects A & B) and drop them off on the assembly table. Once A & B are both on the assembly table, I want an event to trigger which converts A & B into a third object C. This event can either be automatic or triggered via keyboard input. Objects A & B should disappear and object C should spawn at a specific coordinate.



I currently have ground & arm robots which transports objects A & B to the assembly table, but that's all I have right now. The ground robot is remotely controlled (Webot's Khepera 1 TCP/IP model), so timing will always be different.



My main question: How do I remove objects A & B and spawn object C based on an event during simulation?



A side question: How can the assembly table automatically detect when both objects A & B are on the table?

##### Yasmine Masmoudi 07/18/2022 21:43:55
Hello, I'm trying to generate a sumo network file, I faced a problem at this level, please can you help me?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/998706713723150366/unknown.png)
%end



> **Attachment**: [exporter.py](https://cdn.discordapp.com/attachments/565154703139405824/998706830584844329/exporter.py)

##### Kirlin 07/18/2022 22:37:03
Hello, I am trying to compile the robotis darwin-op2 sample with ROS, so we could test our robot's behavior on that simulation, can anyone help me ?

The idea is to compile the Darwin-op2 simulation with ROS, so we can pass our behaviour's decision via ROS to the controller.

##### Sunni 07/19/2022 01:43:13
Hi, is there a corresponding C++ function to wbu\_default\_robot\_window\_configure()?

##### Stefania Pedrazzi [Cyberbotics] 07/19/2022 06:06:19
As described by the error message, you need to install `lxml` module for the python version you use in Webots.

Here are the instructions to install the dependencies usually needed when using SUMO:

[https://www.cyberbotics.com/doc/automobile/openstreetmap-importer#windows](https://www.cyberbotics.com/doc/automobile/openstreetmap-importer#windows)


You should add an additional `Robot` node with the `supervisor` field set to TRUE that monitors the position of objects A and B and adds object C when needed.

You can find an example how to use the Supervisor functionalities in this example:

[https://www.cyberbotics.com/doc/guide/samples-devices#supervisor-wbt](https://www.cyberbotics.com/doc/guide/samples-devices#supervisor-wbt)

##### Sunni 07/19/2022 07:25:17
Hi, is it possible to use extern "C" to wrap headers such as #include <webots/plugins/robot\_window/default.h> and functions such as char *wbu\_default\_robot\_window\_configure() in a C++ controller code?


Hi, I'm trying to understand the search algorithm to convert the window and the remoteControl to an existing path as explained in the Reference Manual, but I'm unsure where to put this path. Currently, when I select my custom robot window from the window field, I get the error "Error: robot window invalid library name. Error: cannot load the "" robot window library." Thank you!


Is it possible to determine the contact point of a collision without using the physics plugin?

##### Stefania Pedrazzi [Cyberbotics] 07/20/2022 06:09:52
The robot window should be located in a subfolder of your projects folder named `plugins/robot_windows/<robot window name>`:

[https://www.cyberbotics.com/doc/reference/robot-window-plugin](https://www.cyberbotics.com/doc/reference/robot-window-plugin)


You can retrieve the contact points also from the Supervisor API:

[https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_contact\_points](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_contact_points)

##### Sunni 07/20/2022 06:12:22
Thank you!


Yes, I've checked and my robot window folder is in the correct position with the correct folder names


Hi, in the webots example showing how to use wb\_supervisor\_node\_get\_contact\_points(), the code called the wb\_supervisor\_node\_get\_number\_of\_contact\_points(arg) function, but it is not listed in the Reference Manual's Supervisor functions. Are there other functions related to getting contact points via Supervisor that aren't in the reference manual?

##### Stefania Pedrazzi [Cyberbotics] 07/20/2022 07:58:18
All the available functions are documented. `wb_supervisor_node_get_number_of_contact_points` is deprecated and thus not documented and it is not used in any of the Webots default example since R2021b.

If you use an old version of Webots you should refer to the corresponding documentation (open it from the Help > Reference Manual menu item in Webots).

##### Sunni 07/20/2022 08:00:54
Got it, thank you.

##### Berryman 07/20/2022 15:14:54
This worked perfectly. Thank you so much!!

##### Din96\_Boy 07/21/2022 10:11:57
Hello can anyone help with one of the issues with my robot. When my robot moving in the obstacle path some parts of the robot are going through each other. Can any one suggest what can I do to prevent this issue? I have added an video explaining my issue here : [https://streamable.com/3ef1e8](https://streamable.com/3ef1e8)

##### kano donn 07/21/2022 19:20:55
What are all of the hidden values inside a world file for a robot? Is there a way to turn them off?

##### Rico Schillings[Sweaty] [Moderator] 07/21/2022 19:30:31
They should only appear if you save the World after the Simulation already has been started. So basicly safe your world before hitting the play button and these hidden values should not be in the file.

##### kano donn 07/21/2022 19:32:51
`@Rico Schillings[Sweaty]` as feedback, it would be great to be able to play an animation, see the result, and then saves are just for initial state and not save those hidden values.

##### Rico Schillings[Sweaty] [Moderator] 07/21/2022 19:38:38
You also can save the current state after running the simulation and delete the hidden values afterwards in the world file. Then you have a "modified" world (e.g. other robot pose or view angle) but without these.



But what they exactly do or meant for someone else maybe can answer here

##### kano donn 07/21/2022 19:39:48
Sounds good. I have been doing just that, but it's a time consuming process. I'm thinking of adding a Python script at closing of a webota to do that cleanup

##### DepressedCoder 07/22/2022 06:48:33
I am following the tutorial [https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Webots.html](https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Webots.html) for connecting webots to ROS2. I made the robot\_description file and added device tags corresponding to GPS,IneritalSensor and Lidar. On starting the simulation The topic related to InertialSensor is not  there while the ones corresponding to GPS and Lidar are there. Please help
%figure
![Screenshot_2022-07-22_121535.png](https://cdn.discordapp.com/attachments/565154703139405824/999930939494903828/Screenshot_2022-07-22_121535.png)
%end
%figure
![Screenshot_2022-07-22_121558.png](https://cdn.discordapp.com/attachments/565154703139405824/999930939809472562/Screenshot_2022-07-22_121558.png)
%end


I was able to get the values directly from the robot driver file I wrote. It helped. But out of curiosity why is the topic for inertial sensor not being there

##### Rico Schillings[Sweaty] [Moderator] 07/22/2022 06:58:03
Did you checked some typo error in the reference "inertunit" of your sensor?

##### DepressedCoder 07/22/2022 07:01:35
I have checked. They are the same. Is it because of  the samplingPeriod  not being provided ?
%figure
![Screenshot_2022-07-22_122913.png](https://cdn.discordapp.com/attachments/565154703139405824/999934216328454174/Screenshot_2022-07-22_122913.png)
%end
%figure
![Screenshot_2022-07-22_122947.png](https://cdn.discordapp.com/attachments/565154703139405824/999934216601096212/Screenshot_2022-07-22_122947.png)
%end

##### Mars\_J 07/22/2022 08:49:33
Hello guys! How do I get a robot's rotation for physics?

##### DepressedCoder 07/22/2022 08:53:54
You can use an InertialUnit sensor I guess

##### madbat09 07/24/2022 14:18:45
Hello guys I'm trying to build a pendulum out of a sphere and an cylinder. My Goal is to have an anchor point to let it swing from and read the degree of movement. However im not quite sure how to build the pendulum in webots can someone push me into the right direction ?

##### Stefania Pedrazzi [Cyberbotics] 07/25/2022 06:22:38
Hi, I would suggest you to look at the `position_sensor.wbt` example:

[https://www.cyberbotics.com/doc/guide/samples-devices#position\_sensor-wbt](https://www.cyberbotics.com/doc/guide/samples-devices#position_sensor-wbt)

##### madbat09 07/25/2022 08:02:25
Hi thanks for the suggestion my problem is more of actually building the pendulum which is made out of 2 basic shapes (sphere and a long thin cylinder) and anchoring it on the top of the "rope" for it to swing freely with the gravitation. 

Do I make a solid with a shape of the ball and a solid with the cylinder and put them into an other solid?

##### Stefania Pedrazzi [Cyberbotics] 07/25/2022 08:06:16
No need of different Solid nodes. Both geometries can be added in the same Solid if they are fixed and don't move w.r.t each other.

##### Berryman 07/25/2022 18:01:21
Hello! I'm trying to follow the Web Simulation Quick Start ([https://cyberbotics.com/doc/guide/web-simulation](https://cyberbotics.com/doc/guide/web-simulation)) but I'm getting this error in the browser.



Does anyone have any suggestions?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/1001187417186910338/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/1001187417543413770/unknown.png)
%end



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/1001188761905922148/unknown.png)
%end

##### Mars\_J 07/26/2022 03:12:55
Hello guys, how do I increase the user applied force in increments on an object every step from the physics plugin?

##### Benjamin Délèze [Cyberbotics] 07/26/2022 05:53:57
What is your version of Webots? The develop branch?

##### Yasmine Masmoudi 07/26/2022 09:06:45

> **Attachment**: [myMap\_net.wbt](https://cdn.discordapp.com/attachments/565154703139405824/1001415266732888124/myMap_net.wbt)


Hello, how can i make cars move on the roads and take the exact path as the circulation? thanks

##### Berryman 07/26/2022 14:30:11
Webots version: R2022a. Branch: I have no local repo for Webots. I use Ubuntu 20.04.4 LTS.

Do I need to reinstall or do something else?



Thank you so much for your help!!

##### Benjamin Délèze [Cyberbotics] 07/26/2022 15:02:57
It seems that there is indeed a problem, I will investigate. Can you open an issue on Github ([https://github.com/cyberbotics/webots/issues](https://github.com/cyberbotics/webots/issues)) such that it will be easier to keep track of things?

##### Berryman 07/26/2022 15:30:14
Yes - [https://github.com/cyberbotics/webots/issues/4962](https://github.com/cyberbotics/webots/issues/4962)

Thank you for helping!!!


Hello! Another Webots in the cloud question.



My end goal - Have a fully interactive Webots simulation that responds to external HTTP requests. The requests will be from different computers on the same local network. For example, I have a custom robot arm sequence for the Niryo Ned model which begins when `1` is pressed on the keyboard. I want to send an HTTP request to trigger the sequence instead of pressing `1`. 



I also have an Express server and React client set up for this project already. I'm using long polling to create a persistent connection between the server & client. The external HTTP request is sent to the Express server, and the React client automatically updates due to long polling.



I was thinking of embedding my Webots simulation in my React client, theoretically meaning I can trigger a Webots process when the event happens. I've been looking into Webots Web Server to do this ([https://cyberbotics.com/doc/guide/web-server?version=master](https://cyberbotics.com/doc/guide/web-server?version=master)). I also noticed Webots.cloud ([https://cyberbotics.com/doc/guide/webots-cloud?version=master](https://cyberbotics.com/doc/guide/webots-cloud?version=master))



It would look like this [https://webots.cloud/run?version=R2022b&url=https://github.com/ThomasOliverKimble/Theia\_Cloud/blob/master/thymio/worlds/thymio2.wbt](https://webots.cloud/run?version=R2022b&url=https://github.com/ThomasOliverKimble/Theia_Cloud/blob/master/thymio/worlds/thymio2.wbt) or this [https://webots.cloud/run?version=R2022b&url=https://github.com/ThomasOliverKimble/orobot/blob/main/worlds/OroBOT.wbt](https://webots.cloud/run?version=R2022b&url=https://github.com/ThomasOliverKimble/orobot/blob/main/worlds/OroBOT.wbt) but instead of GUI buttons/sliders controlling behavior, it would be handled in the background



--- 



Can [https://webots.cloud/simulation](https://webots.cloud/simulation) help me do this? Or do I need to follow the Webots Web Server instructions?

Any feedback that you have is extremely helpful. Thank you so much in advance!

##### Olivier Michel [Cyberbotics] 07/28/2022 06:46:17
I believe you can pretty easily achieve this with [https://webots.cloud/simulation](https://webots.cloud/simulation) by having no robot window for your robot and having simply your robot controller open a long pool connection to communicate with your web service. From there, your web service could read sensor information from the robot and send motor commands or high level commands to the robot. I guess you will find some python library that can simplify the setup of this connection. You can check this repo for example of webots.cloud simulations: [https://github.com/cyberbotics/webots-cloud-simulation-examples](https://github.com/cyberbotics/webots-cloud-simulation-examples)

##### áçè 07/28/2022 13:45:59
can somebody plz tell me why it is acting like this
> **Attachment**: [empty\_5.mp4](https://cdn.discordapp.com/attachments/565154703139405824/1002210316756725841/empty_5.mp4)


how to see a 2d motion of robot in graph  or chart?

##### Guido 07/30/2022 14:09:19
Hi!

I have an elevation grid in my world and I want to modify its height field at runtime

The field itself starts with no elements and ideally I'd want around 1 million elements

I tried using `wb_supervisor_field_insert_mf_float` but it's just too slow (even inserting 1000 elements takes more than 10 seconds)

Is there a way to insert many elements at once?

##### áçè 07/31/2022 06:18:58
what to do for seat as it is hanging?
> **Attachment**: [Lift\_mechanism\_1.mp4](https://cdn.discordapp.com/attachments/565154703139405824/1003184980819709982/Lift_mechanism_1.mp4)


plz help

##### H-Man 07/31/2022 20:15:36
Hi, I’m new to Webots



I’m currently programming my controller in python code. Why is it that you always start it with `from controller, import Robot`. What exactly is this controller module and where is it located? There’s not too much info I can find on this in the tutorials or the Webots User Guide.

##### Rico Schillings[Sweaty] [Moderator] 07/31/2022 20:24:17
Its located in your webots Installation path, should be in 'lib/python'. This library provides all the functions to interact with webots and with your described code line you import the robot class from the controller library of webots to interact with your robot in the world

##### H-Man 07/31/2022 21:33:55
Thanks for the clarification.



Unfortunately, my robot class is unable to be imported from my controller library for some reason. When I try to run code, it throws up…



%figure
![Screenshot_2022-07-31_at_22.23.21.png](https://cdn.discordapp.com/attachments/565154703139405824/1003415274873176266/Screenshot_2022-07-31_at_22.23.21.png)
%end


This is my controller folder…



%figure
![Screenshot_2022-07-31_at_22.28.24.png](https://cdn.discordapp.com/attachments/565154703139405824/1003415349628239922/Screenshot_2022-07-31_at_22.28.24.png)
%end


My python path is `/Library/Frameworks/Python.framework/Versions/3.10/bin/python3`.



Any thoughts on how to fix this?

##### Yasmine Masmoudi 07/31/2022 21:36:46
Hello guys, can someone send me an example of a map of a 3 floor-building just to do a test? thanks


Btw is it possible to design a 3 floor-building in webots?

##### kano donn 07/31/2022 21:58:11
You will need to place your controller file inside the controller folder of the website world. 



I have never been able to get a controller script located anywhere else to work.


You should be able to make a solid from a mesh shape that comes from another program where you created the building.


It looks like you are missing something to hold the seat


I am surprised it takes that long. Would you mind sharing how you are calling that function?

##### Guido 07/31/2022 22:09:16
I realized it takes that long because it traverses the hole thing if I use `-1` as the index

Using `0` makes it a bit faster, but it is still painfully slow for what I want to do (which is nothing out of this world, just a dynamic elevation grid).

I'm calling it like

`wb_supervisor_field_insert_mf_float(elevation_grid_height_field, 0, 0);`


`@kano donn`

##### kano donn 07/31/2022 22:14:27
The elevation field is changing in simulation?

##### Guido 07/31/2022 22:14:47
yep, the idea is to add dynamic obstacles

##### kano donn 07/31/2022 22:19:49
Huh .... And normal shapes don't work?

##### Guido 07/31/2022 22:27:48
Apart from the obvious losing the ability to generate more complex reliefs problem, I really want to be able to generate holes, and Webots doesn't provide any way to substract a shape from another one.

##### kano donn 07/31/2022 22:29:47
Have you tried mesh shapes? We have been able to throw a lot of very complex stuff at it without issue

##### Guido 07/31/2022 22:31:47
That sounds interesting, but how would you craft a hole in the ground? I'm not quite getting it

##### kano donn 07/31/2022 22:34:20
I would make a few different tiles of the ground where each tile contains the feature you want to test. They can be quite large or small as long as they are common sizes and so they tile well. You should be able to dynamically render the ground where you want and toggle movement and visibility to make it dynamic.

##### Guido 07/31/2022 22:39:44
Thanks for the idea!

## August

##### Sunni 08/01/2022 06:59:05
Hi, I've been getting the error "[javascript] ReferenceError: Can't find variable: toggleStopCheckbox" when I run my simulation with a custom robot window that should stop the motor when a checkbox is checked, and I'm not sure what the error here is specifically. 



This is the window's javascript code:



function toggleStopCheckbox(obj) {

  if (obj.checked) {

    obj.parentNode.classList.add('checked');

    obj.parentNode.lastChild.innerHTML = 'Start Motors';

    window.robotWindow.send('stop motors');

    log('Stop motors.');

  } else {

    obj.parentNode.classList.remove('checked');

    obj.parentNode.lastChild.innerHTML = 'Stop Motors';

    window.robotWindow.send('release motors');

    log('Release motors.');

  }

}



This is the code in my controller to receive the message from js:



const char *message = (robot->wwiReceiveText()).c\_str();

    

    if(message){

      cout << "Message is " << message << endl;

      if(strcmp(message, "stop motors") == 0){

        cout << "Received 'stop motors' message from JavaScript" << endl;

        stop\_motors = true;

      }else if(strcmp(message, "release motors") == 0){

        printf("Received 'release motors' message from JavaScript\n");

        stop\_motors = false;

      }else{

        cout << "Unknown message: " << message << endl;

      }

    }else{

      cout << "Message received is invalid!" << endl;

    }



Thank you in advance!

##### H-Man 08/01/2022 09:16:53
Sure



To clarify, the controller folder of the website world is the folder I screenshot and sent on my previous message?

Also, where should I be able to find the controller file mentioned?

##### Din96\_Boy 08/01/2022 11:00:07
I'm trying to create a spring motion in Webots; What I'm trying to achieve is to visualize an spring elongation. Is this possible to simulate in Webots? , If so can anyone share some references or any reading material? Thank You!.
%figure
![Animated-mass-spring.gif](https://cdn.discordapp.com/attachments/565154703139405824/1003618125931106394/Animated-mass-spring.gif)
%end

##### Yasmine Masmoudi 08/01/2022 15:31:23
Can you explain more please?

##### AndiHB 08/02/2022 11:07:28
Hi 😃

I am currently trying to simulate driving a car (the standard Proto BMW X5) with the <setThrottle()> command.

Unfortunately, the vehicle does not move.

If I run the setCruisingSpeed everything is working fine.

Is there any additional setup or configuration that i missed? I could not find information on that on ([https://cyberbotics.com/doc/automobile/driver-library?tab-language=python](https://cyberbotics.com/doc/automobile/driver-library?tab-language=python))



Thanks in advance 😃 



(Webots version 2022a)

##### DDaniel [Cyberbotics] 08/02/2022 12:49:13
did you set a gear? `driver->setGear(1);`

##### AndiHB 08/02/2022 12:51:22
Yes, setgear worked,

Checked by calling getGear() which always returned the gear i set

##### DDaniel [Cyberbotics] 08/02/2022 12:53:23
so it moves now?

##### AndiHB 08/02/2022 12:56:13
No, despite of having the right Gear the vehicle does not move.



I even checked if the brakes are on, but those are correclty set to zero 🤔


Sry for the misscommunication, My basd

##### DDaniel [Cyberbotics] 08/02/2022 13:06:56
this is a minimal python controller for the bmw, it works for me:

```
from vehicle import Driver

driver = Driver()

driver.setThrottle(1.0)
driver.setGear(1)

while driver.step() != -1:
    pass
```

##### AndiHB 08/02/2022 13:28:01
Thanks, that worked! I somehow forgott the step call in the while loop. 😅 



I tested the it with the direct controller implementation, because I want to use ros2 to drive the vehicle.

There i have the same problem, that setting the Cruise-speed is working, but setting the throttle is not.



Do you by any chance have an idea what the problem could be there? the step-function is implemented as in the ros2 tutorial for a webots simulation 🤔  (code below)



```python
import rclpy
from joystick_interface.msg import VehicleInput # custom msg

class MinimalDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        rclpy.init(args=None)
        self.__node = rclpy.create_node('minimal_driver')
        self.__node.create_subscription(VehicleInput, 'cmd_vehicle', self.__cmd_callback, 1)

    def __cmd_callback(self, msg):
        self.throttle = msg.throttle
        self.steering_angle = msg.steering_angle
        
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__robot.setGear(1)
        self.__robot.setThrottle(self.throttle)
        self.__robot.setSteeringAngle(self.steering_angle)
```

##### PeecaPoo 08/02/2022 17:08:40
Hello Cyperbotics webmasters!

I hope you are well.



I have an issue with connecting between Webots software and Matlab. as the software keeps showing this warning

WARNING: Unable to find the 'matlab' executable in the current PATH.





I have attached a screenshot to help realize what I am talking about.





I hope you can help me ASAP.





Glowing Regards
%figure
![Screenshot_2022-08-02_184405.png](https://cdn.discordapp.com/attachments/565154703139405824/1004073263518986321/Screenshot_2022-08-02_184405.png)
%end

##### DepressedCoder 08/02/2022 17:35:39
The issue i guess is solved in R2022b . Use the latest nightly build of webots

##### HMan 08/02/2022 17:37:53
Hello



Can someone clarify what is meant by this? I’m a bit of a newbie at Webots.



Thanks

##### PeecaPoo 08/02/2022 17:43:47
I have Webots R2022a, I see people on YouTube using much older versions and it's working fine for them.

##### Winner 08/05/2022 02:56:32
Hi, Hope you guys are doing well. I am currently facing some issues with webots. It is possible to form dynamic deformation on the webots? Thanks very much!

##### Olivier Michel [Cyberbotics] 08/05/2022 12:09:04
I don't think so.

##### NickMetalhead 08/05/2022 18:03:53
Hi guys. I have a problem with simulationReset(). I have a simple world using a modified Altino robot. I have added a camera and I also made it supervisor by modifing the .proto file.


I use a simple controller that commands a constant speed and angle. However, after I reset the simulation with simulationReset(), the angle of the wheels change, but the robot stays still. The controller is :

```
from vehicle import Driver

driver = Driver()

i = 0
while driver.step() != -1:
    i += 1
    if i % 100 == 0:
        driver.simulationReset()
        driver.simulationResetPhysics()
    driver.setCruisingSpeed(1.0)
    driver.setSteeringAngle(-1.0)

```



> **Attachment**: [video.mp4](https://cdn.discordapp.com/attachments/565154703139405824/1005175054516093038/video.mp4)

##### PeecaPoo 08/06/2022 15:05:27
Hello @everyone 

I have been assigned to a project in college and I really stuck.

I hope you guys can help me out.



The project is simulating a navigation robot in Webots with VFH algorithm.



I only found one video on YouTube but no commentary or explanations there

here is the video : [https://www.youtube.com/watch?v=T7njx41sGpo](https://www.youtube.com/watch?v=T7njx41sGpo)



maybe one of you have an idea or have the project done already, who know.



thank you

##### Mete 08/07/2022 21:43:34
Hello @everyone


================================================================================REQUIRED process [webots-2] has died!

process has died [pid 26861, exit code 1, cmd /home/gokmen/webots\_ws/devel/lib/webots\_ros/webots\_launcher.py --world=/home/gokmen/webots\_ws/src/webots\_ros/worlds/keyboard\_teleop.wbt --mode=realtime --no-gui=false, --stream=false \_\_name:=webots \_\_log:=/home/gokmen/.ros/log/50037a78-1696-11ed-b1c3-1b0919343372/webots-2.log].

log file: /home/gokmen/.ros/log/50037a78-1696-11ed-b1c3-1b0919343372/webots-2*.log

Initiating shutdown!

================================================================================


I am getting this error while I was trying to test webots\_ros


My ros= ros noetic


ubuntu versiyon= 20.04

##### kano donn 08/07/2022 22:07:03
`@Mete` there are logs referenced in that error report. What do they report in the last line?

##### Mete 08/07/2022 22:08:02
SUMMARY

========



PARAMETERS

 * /rosdistro: noetic

 * /rosversion: 1.15.14



NODES

  /

    keyboard\_teleop (webots\_ros/keyboard\_teleop)

    webots (webots\_ros/webots\_launcher.py)



auto-starting new master

process[master]: started with pid [26848]

ROS\_MASTER\_URI=*[http://192.168.1.105:11311](http://192.168.1.105:11311)*



setting /run\_id to 50037a78-1696-11ed-b1c3-1b0919343372

process[rosout-1]: started with pid [26858]

started core service [/rosout]

process[webots-2]: started with pid [26861]

process[keyboard\_teleop-3]: started with pid [26863]

WEBOTS\_HOME environment variable not defined.

================================================================================REQUIRED process [webots-2] has died!

process has died [pid 26861, exit code 1, cmd /home/gokmen/webots\_ws/devel/lib/webots\_ros/webots\_launcher.py --world=/home/gokmen/webots\_ws/src/webots\_ros/worlds/keyboard\_teleop.wbt --mode=realtime --no-gui=false, --stream=false \_\_name:=webots \_\_log:=/home/gokmen/.ros/log/50037a78-1696-11ed-b1c3-1b0919343372/webots-2.log].

log file: /home/gokmen/.ros/log/50037a78-1696-11ed-b1c3-1b0919343372/webots-2*.log

Initiating shutdown!

================================================================================

[keyboard\_teleop-3] killing on exit

[webots-2] killing on exit

[rosout-1] killing on exit

[master] killing on exit

shutting down processing monitor...

... shutting down processing monitor complete

done

##### kano donn 08/07/2022 22:09:21
Before those equal line divider is your issue. WEBOTS\_HOME env variable is not defined

##### Mete 08/07/2022 22:12:56
how can I solve this problem


process[webots-1]: started with pid [39582]

process[complete\_test-2]: started with pid [39583]

Traceback (most recent call last):

  File "/opt/ros/noetic/lib/webots\_ros/webots\_launcher.py", line 49, in <module>

    subprocess.call(command)

  File "/usr/lib/python3.8/subprocess.py", line 340, in call

    with Popen(*popenargs, **kwargs) as p:

  File "/usr/lib/python3.8/subprocess.py", line 858, in \_\_init\_\_

    self.\_execute\_child(args, executable, preexec\_fn, close\_fds,

  File "/usr/lib/python3.8/subprocess.py", line 1704, in \_execute\_child

    raise child\_exception\_type(errno\_num, err\_msg, err\_filename)

PermissionError: [Errno 13] Permission denied: '/home/gokmen/snap/webots'


I am getting this error now

##### Ardy Seto P 08/08/2022 02:22:08
hi all, I face a problem, line appear in camera image of mavic 2 pro, anyone know how to fix it?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/1006024484412739746/unknown.png)
%end

##### Mete 08/09/2022 08:55:40
I solved it


can I run a map if it made in older versions?

##### JanSch99 08/09/2022 09:03:24
Hello,

I'm having some trouble with joint limits on the second part of a Hinge2Joint. I've reduced the problem to the following test world. Running the simulation shows that after the endpoint moves too far to the right (as seen in the video), both objects start moving unrealistically and uncontrollably, as if two constraints were fighting each other. I've already done lots of experiments (different limits, positions, damping, ...), but cannot get it to behave properly. The other axis (forwards/backwards as seen in the video) works properly, including the limits (omitted for brevity in the code below).

Hope someone can help, thanks in advance!

```
#VRML_SIM R2022a utf8
WorldInfo {
  defaultDamping Damping {}
}
Viewpoint {
  orientation 0.16 0.83 -0.52 0.74
  position -1.37 0.63 1.24
}
TexturedBackground {}
TexturedBackgroundLight {}
RectangleArena {}
Solid {
  translation 0 0 0.015
  children [
    DEF base_shape Shape {
      appearance BrushedAluminium {}
      geometry Box {
        size 0.7 0.45 0.03
      }
    }
    Transform {
      translation 0 0 0.5
      children [
        Hinge2Joint {
          jointParameters HingeJointParameters {
            axis 0 1 0
          }
          jointParameters2 JointParameters {
            axis 1 0 0
            minStop -0.1
            maxStop 0.1
          }
          endPoint Solid {
            translation 0 -0.02 0.2
            children [
              DEF endpoint_shape Shape {
                appearance BrushedAluminium {}
                geometry Box {
                  size 0.1 0.1 0.1
                }
              }
            ]
            boundingObject USE endpoint_shape
            physics Physics {
              density -1
              mass 1
            }
          }
        }
      ]
    }
  ]
  boundingObject USE base_shape
  physics Physics {
    density -1
    mass 10
  }
}
```



> **Attachment**: [hinge2\_test.mp4](https://cdn.discordapp.com/attachments/565154703139405824/1006487949166268426/hinge2_test.mp4)

##### Hangry 08/09/2022 16:16:55
Hi,

My Webots keeps throwing up that I don’t have a ‘controller module’ installed. Not sure what it’s talking about.



Any ideas?

##### goch [Moderator] 08/11/2022 11:59:10
Depending on how you run your controllers you need to export some Variables e.g WEBOTS\_HOME and add it to your PYTHONPATH variable See all variables that are used here -> [https://cyberbotics.com/doc/guide/running-extern-robot-controllers](https://cyberbotics.com/doc/guide/running-extern-robot-controllers)

##### starscream 08/12/2022 10:53:22
Hello all,

I want to make a grid based map using the map file ie using the location of the walls in the map file. Can anyone guide me in the right direction for how to approach this.

Thanks and regards

##### Olivier Michel [Cyberbotics] 08/12/2022 11:49:32
I would write a Python script to convert your map file into a webots `.wbt` world file.

##### Vishwa\_bandara\_Bogahawaththa 08/12/2022 16:25:32
Hi, I am a beginner to webots . I wanted to assign an image as a front texture of my "AdertisingBoard" node, but I also need to zip my project directory file and send it to someone to see my simulation with that same image as a texture of the adertising board. I selected that image form a folder in my PC. How can simply I do this  ? could you please give me some instructions

##### EmpressDzhee 08/15/2022 02:04:04
Hi! 👋 I created a custom PROTO named **MCTurtle.proto**



**It works **whenever I tried to import it into the **Scene Tree**



But whenever I try to spawn it via **importMFNodeFromString**, there is an error

**ERROR: Skipped unknown 'MCTurtle' node or PROTO.**



MCTurtle.proto is in the **protos folder **of the Project.



Attached are the **controller and proto file**
> **Attachment**: [turtle\_spawner.py](https://cdn.discordapp.com/attachments/565154703139405824/1008556655488806932/turtle_spawner.py)
> **Attachment**: [MCTurtle.proto](https://cdn.discordapp.com/attachments/565154703139405824/1008556655841116293/MCTurtle.proto)

##### Din96\_Boy 08/15/2022 05:52:01
Hello , Can we change the physics engine in Webots from ODE to a different physics engine? , If can what is the procedure to follow?

##### Olivier Michel [Cyberbotics] 08/15/2022 06:17:23
That's a long and difficult task: you have to fork Webots and modify its source code to use another physics engine by replacing the calls to the ODE API with calls to your other physics engine.


It should work... Not sure what is the problem... Could you try to copy a simple proto, like the Chair.proto from the Webots distribution into your local project and change only one detail to assess this one is loaded instead of the one from the Webots distribution. If that works, you should modify this proto step-by-step until it becomes your MCTurtle.proto (e.g., rename it, modify its contents, etc.).

##### EmpressDzhee 08/15/2022 06:21:14
okay got it thanks <a:catchunkroll:978067995731578960>

##### Din96\_Boy 08/15/2022 11:02:55
[https://youtu.be/c4kRe90JaP8](https://youtu.be/c4kRe90JaP8) How can we make an simulation like this in Webots? , Any ideas how to get the physics for the ropes? This video was posted by Webots YouTube channel back in 2011

##### Olivier Michel [Cyberbotics] 08/15/2022 11:46:23
You are looking for this: [https://cyberbotics.com/doc/guide/samples-howto#rope-wbt](https://cyberbotics.com/doc/guide/samples-howto#rope-wbt)

##### Mete 08/15/2022 18:03:46
hello I want to use ROS with webots.


when I started to copy those 

export ROS\_DISTRO=noetic  # or ROS\_DISTRO=melodic, etc.

cd ${WEBOTS\_HOME}/projects/default/controllers/ros

make


I came across this error

Makefile:107: usr/local/webots/resources/Makefile.include: No such file or directory

make: *** No rule to make target 'usr/local/webots/resources/Makefile.include'.  Stop.


and I tried it with sudo


I get this error

Makefile:23: /resources/Makefile.os.include: No such file or directory

make: *** No rule to make target '/resources/Makefile.os.include'.  Stop.


my ros verison is noetic


I setup the webots from deb file

##### DepressedCoder 08/15/2022 19:12:38
I am following the tutorial on [https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots.html](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots.html) for Webots ROS Integration. How should I proceed if I want to  have multiple robots in my simulation, each with its own driver code? Please help

##### Olivier Michel [Cyberbotics] 08/16/2022 06:23:59
It looks like your `WEBOTS_HOME` environment variable is not properly defined. It should be set to `/usr/local/webots` and not `usr/local/webots` (notice the first `/`).


You should create a new robot in the world file (or duplicate the existing one) and change the launcher to start two controllers, one for each robot.

##### Mete 08/16/2022 20:41:44
Makefile:23: /resources/Makefile.os.include: No such file or directory

make: *** No rule to make target '/resources/Makefile.os.include'.  Stop.


I am getting this error now


I use sudo make


when I used make, I got th,s error

cp: cannot remove 'include/webots\_ros/RecognitionObject.h': Permission denied

make: *** No rule to make target 'ROS\_HEADERS', needed by 'RosPen.cpp'.  Stop.

##### mobenrabah 08/17/2022 09:40:09
Hello everyone ! I'm a control engineering student and I'm working on the robust control of drones (Quadrotors). For this I want to replace the default controller of mavic 2 pro with mine so I need to understand this one and I wonder if you have any documents about this controller. Another question, I can't know the unity of the command signals (rotors velocities).

