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

