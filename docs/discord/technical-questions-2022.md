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

##### Osama Bin Laden 01/07/2022 10:43:37
Hello all I am facing an issue when loading code into vs code

##### Luftwaffel [Moderator] 01/09/2022 00:18:11
what issue?

##### Rizalfauz 01/09/2022 04:03:30
Hello everyone, i want to make a plunger like solenoid, any idea to make it?  because using a linear motor does not have much effect on the ball



> **Attachment**: [Tak\_berjudul\_5\_540p.mp4](https://cdn.discordapp.com/attachments/565154703139405824/929586918663024650/Tak_berjudul_5_540p.mp4)

##### Osama Bin Laden 01/09/2022 04:56:40
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

##### [Red Dragons] Mat198 01/09/2022 21:42:23
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
Thanks `@[Red Dragons] Mat198` . In my case, doing `armChain = self.getFromDef("ARM")` returns a Node which does not have `links` so I'd need to investigate further. (Is it "OK" to nest a Robot under a Robot so that it shares the same thread as the cameras)?

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

##### Luftwaffel [Moderator] 01/11/2022 00:59:28
you could convert the robot to basenodes (right click on it). Then Turn the Robot basenode into a transform. That should do the trick


This code-block shows you all devices, and this specifically puts all motors and position sensors in a list. But it prints ALL devices, so you can add other sensors.



> **Attachment**: [webots\_init\_devices.py](https://cdn.discordapp.com/attachments/565154703139405824/930266974196498472/webots_init_devices.py)


If you want to use an inverse kinematics controller I made (and which works very well), check out this repository. I have not checked it recently with the UR10e, but i did in the past and it worked flawlessly


[https://github.com/Simon-Steinmann/webots\_pyikfast\_tutorial](https://github.com/Simon-Steinmann/webots_pyikfast_tutorial)


I uploaded a solver for the ur10e I had lying around, but I have not tested building the module and testing it

##### shpigi 01/11/2022 01:15:31
I really appreciate it `@Luftwaffel` . Going to give all of it a try


so far, this code returns the two cams (`st1_cam` and `st2_cam`) but not the robot (converted from proto). I'm gueesing this is because it's a node, not a device and the getNumberOfDevices / getDeviceByIndex doesn't drill into the Nodes.

##### Luftwaffel [Moderator] 01/11/2022 01:26:13
did you turn the robot node of the ur10e into a solid?


perhaps you can post another screenshot of your scene tree

##### shpigi 01/11/2022 01:27:31
I converted to Base Node and now this is my tree:
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/930271674924941382/unknown.png)
%end

##### Luftwaffel [Moderator] 01/11/2022 01:28:45
create a new solid next to the 2 cameras. Then cut - copy all the ur10e's children to the solids children


perhaps also put the translation and rotation to the same values


then delete the "DEF ARM Robot" node


The issue is that you have nested Robot nodes. A robot is a node that starts a controller and has access to its own devices.


so you should only have your highest level robot node, if you want to control everything with one controller

##### shpigi 01/11/2022 01:37:22
`@Luftwaffel` , you're a genius! I went into a text editor, replaced the `DEF ARM Robot` with `Solid` and it worked!

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

##### Luftwaffel [Moderator] 01/11/2022 01:38:00
😎  Awesome! that is a much smarter way to do it


If you want to get IK to work properly, I highly suggest you use my repository. I spent half a year with that stuff, and it is by far the best solution that is out there and compatible without complex other frameworks

##### shpigi 01/11/2022 01:40:31
Sounds great. Will do. I'm aiming at some reinforcement learning work and controlling the end-effector in Cartesian space (instead of doing actions in joint space) is very promising IMO

##### Luftwaffel [Moderator] 01/11/2022 01:41:16
then you kind of HAVE to use my repo. That was exactly what I was doing. I added support for cartesian velocity control without error drift


and it is very fast, in the dozens to hundreds of microseconds per solution instead of milliseconds


but dont expect too much from RL with robotic arms. Spend about 2 years with it

##### shpigi 01/11/2022 01:42:36
I'll be sure to let you know how it goes

##### Luftwaffel [Moderator] 01/11/2022 01:43:02
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

##### Benjamin Hug [Cyberbotics] 01/12/2022 12:57:54
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

##### [Red Dragons] Mat198 01/16/2022 13:27:23
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

##### [Red Dragons] Mat198 01/16/2022 13:48:28
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

##### Benjamin Hug [Cyberbotics] 01/17/2022 10:00:15
I will need to see your launch file in order to help you, hard to say without any code. You might check some of our tutorials if you begin with Webots ([https://github.com/cyberbotics/webots\_ros2/wiki/Tutorials](https://github.com/cyberbotics/webots_ros2/wiki/Tutorials)). Finally you can check this launch file ([https://github.com/cyberbotics/webots\_ros2/blob/master/webots\_ros2\_mavic/launch/robot\_launch.py](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_mavic/launch/robot_launch.py)), which is very simple.

##### Tom\_Wolf 01/17/2022 10:09:35
Hi ! there is my launch file
%figure
![Capture_decran_2022-01-17_a_11.09.00.png](https://cdn.discordapp.com/attachments/565154703139405824/932577384295698492/Capture_decran_2022-01-17_a_11.09.00.png)
%end

##### Benjamin Hug [Cyberbotics] 01/17/2022 10:17:49
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

##### [Red Dragons] Mat198 01/20/2022 11:45:21
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

##### [Red Dragons] Mat198 01/20/2022 17:26:25
How I do that?

##### Benjamin Hug [Cyberbotics] 01/20/2022 19:37:42
Click on `New issue` here [https://github.com/cyberbotics/webots/issues](https://github.com/cyberbotics/webots/issues), thank you!

##### [Red Dragons] Mat198 01/20/2022 19:41:14
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

##### Benjamin Hug [Cyberbotics] 01/21/2022 14:53:56
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

##### Benjamin Hug [Cyberbotics] 01/21/2022 16:35:29
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

##### Yosi Frost 01/22/2022 20:32:01
Is their a way to run unit test for Python controllers in webots?

##### Max\_K 01/24/2022 12:37:32
ok 😅 .. Do you have an idea for a workaround? I want to detect if the feet of the spot are on the ground. When the spot move/sits the feet turns red, so maybe i could just read out the boundingbox?
> **Attachment**: [2022-01-24\_13-25-55.mp4](https://cdn.discordapp.com/attachments/565154703139405824/935151332325724220/2022-01-24_13-25-55.mp4)

##### Benjamin Hug [Cyberbotics] 01/24/2022 12:50:20
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

##### Benjamin Hug [Cyberbotics] 01/25/2022 09:45:08
Just take example on the mavic plugin here ([https://github.com/cyberbotics/webots\_ros2/blob/master/webots\_ros2\_mavic/webots\_ros2\_mavic/mavic\_driver.py#L40](https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_mavic/webots_ros2_mavic/mavic_driver.py#L40)). (Edit: check the line 40, this is the way to get the instance of the robot)

##### Max\_K 01/25/2022 12:06:44
Ok, i have an instance of my robot, but i cant execute supervisor functions. I want to use getFromDef to access a boundingObject Group that i could use to detect the collision.

I have the same error as here: [https://github.com/cyberbotics/webots/issues/4134](https://github.com/cyberbotics/webots/issues/4134)



I exported the env vars, but the error remains

##### Benjamin Hug [Cyberbotics] 01/25/2022 13:08:13
Please open an issue on Github so we can look at this together more in depth.

##### Yosi Frost 01/25/2022 14:38:43
I had a similar issue


I was using an older version of webots. when I upgraded, everything worked

##### Max\_K 01/25/2022 14:46:07
[https://github.com/cyberbotics/webots/issues/4151](https://github.com/cyberbotics/webots/issues/4151) 

I opened an issue.

##### moebius 01/26/2022 03:47:32
if a .wbo file i am importing into webots has the coordinate system with the x axis pointing up and the arena has the coordinate system with the y axis pointing up (a +90 deg rotation about the z axis basically) , is there anyway I can change the coordinate systems of either of them so they match? I need to use a compass and the values are obviously not usable at the moment. Or should I do some transformation in the controller itself to use the compass values? would really appreciate the help, i am a bit stuck here.

##### Rico Schillings[Sweaty] 01/26/2022 09:57:47
I'm trying to rebuild a RC-car in webots which we are using for several university projects to test self-driving algorithms. Its a Audi q2 scaled with 1:8 (see picture). I am using the Car.proto to allow working with the webots driver library, but when i down-scaled everything with the real dimensions of our car, I have the following problem: Using 4x4 is working but it seems that the car is "slipping" over the front wheels as soon as I steer into a curve (like the friction of the front wheels is to low and the rear wheels are pushing the car ahead instead of steering correctly..). Since our real car has only a rear motor, I would like to use the "propulsion" type of car.proto. But this is not working for such small-scaled cars like I want. With propulsion the front tires are blocking/fixed and do not rotate. 



I also tested the example proto of the saeon altino car. This is by default a 4x4 type, but when I change it to propulsion, its front wheels are also blocking. 



Could it be that there is a kind of a bug with the car.proto which only work for (nearly) realistic dimensions but not when working with small-scaled cars?
%figure
![audi_model.jpeg](https://cdn.discordapp.com/attachments/565154703139405824/935835909113413642/audi_model.jpeg)
%end

##### DDaniel [Cyberbotics] 01/26/2022 09:59:38
Are you using the `scale` parameter to scale it down? This scaling should only be used on geometries, not at the robot level

##### Rico Schillings[Sweaty] 01/26/2022 10:02:58
No I just use `scale` on my created mesh/stl files. I have measured all dimensions from the real car and modified the values in the car.proto (e.g. `trackFront = 0.2` or `wheelbase=0.36`) and also created a new wheel.proto with measured values

