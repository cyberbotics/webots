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

##### (｡•́⌒•̀｡) 01/07/2022 10:43:37
Hello all I am facing an issue when loading code into vs code

##### Luftwaffel [Moderator] 01/09/2022 00:18:11
what issue?

##### Rizalfauz 01/09/2022 04:03:30
Hello everyone, i want to make a plunger like solenoid, any idea to make it?  because using a linear motor does not have much effect on the ball



> **Attachment**: [Tak\_berjudul\_5\_540p.mp4](https://cdn.discordapp.com/attachments/565154703139405824/929586918663024650/Tak_berjudul_5_540p.mp4)

##### (｡•́⌒•̀｡) 01/09/2022 04:56:40
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

##### Benjamin Hug [Cyberbotics] 01/20/2022 19:37:42
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

##### joachim honegger [Cyberbotics] 02/04/2022 13:49:13
You can have a look to this code, it notably purposes to move the mavic to specific coordinates: [https://github.com/cyberbotics/webots-projects/blob/master/projects/forest\_firefighters/controllers/autonomous\_mavic/autonomous\_mavic.py](https://github.com/cyberbotics/webots-projects/blob/master/projects/forest_firefighters/controllers/autonomous_mavic/autonomous_mavic.py)

##### Benjamin Hug [Cyberbotics] 02/04/2022 14:27:22
Do you mean that you stated `--robot-description` in the controllerArgs of the robot in Webots, and that your `robot_state_publisher` node does not publish although you started it ?

##### 안정수 02/04/2022 16:18:49
Yes, this option is used in the wbt file and TF is not generated properly when robot\_state\_publisher is executed.

##### Benjamin Hug [Cyberbotics] 02/04/2022 16:24:19
Can you see if robot\_state\_publisher receive an urdf as parameter?


In case it doesn't receive the urdf, could you open on issue on [https://github.com/cyberbotics/webots\_ros/issues](https://github.com/cyberbotics/webots_ros/issues) and, if possible, upload your world so I can take a look?

##### 안정수 02/04/2022 16:28:16
It is not receiving your urdf. Would it be okay if I upload it later?

##### Benjamin Hug [Cyberbotics] 02/04/2022 16:28:31
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

##### Benjamin Hug [Cyberbotics] 02/06/2022 10:20:30
Perfect I will investigate that in the next days.

##### mclzc 02/06/2022 15:56:05
Thank you very much.


Is it possible to force a body to have a velocity in certain direction and then read the torque values of attached motors? 

I have a mechanism and want to know the input torque profile that would be needed to obtain certain motion in the other end of the mechanism.


My Webots was crashing; it was stuck thinking that it needed to run void.exe as a controller, even though in my world there is no supervisor==True nor controller "something" defined anymore.

It is related to my usage of SolidReference and "<static environment>", which I tried using because removing the physics node wasn't working anymore to fix a body for some reason (that's a different story).

Removing the SolidReference fixes the issue, but I still wonder why was it crashing in that way.
> **Attachment**: [lifting\_platform.zip](https://cdn.discordapp.com/attachments/565154703139405824/940123406760837160/lifting_platform.zip)

##### Benjamin Hug [Cyberbotics] 02/07/2022 10:00:27
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

##### Benjamin Hug [Cyberbotics] 02/08/2022 12:37:57
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

##### Benjamin Hug [Cyberbotics] 02/15/2022 07:34:45
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

##### Benjamin Hug [Cyberbotics] 02/16/2022 07:42:29
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

##### Benjamin Hug [Cyberbotics] 02/25/2022 09:56:55
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

##### Benjamin Hug [Cyberbotics] 02/28/2022 14:41:26
Hi, how did you installed `Webots` and `webots_ros2`?

##### Tom\_Wolf 02/28/2022 14:51:49
Hi i follow those two links


[https://www.cyberbotics.com/doc/guide/tutorial-7-using-ros?version=cyberbotics:R2019a](https://www.cyberbotics.com/doc/guide/tutorial-7-using-ros?version=cyberbotics:R2019a)


[https://cyberbotics.com/doc/guide/installation-procedure](https://cyberbotics.com/doc/guide/installation-procedure)

##### DDaniel [Cyberbotics] 02/28/2022 14:53:26
It seems you're looking at an old page, the correct one should be [https://www.cyberbotics.com/doc/guide/tutorial-9-using-ros](https://www.cyberbotics.com/doc/guide/tutorial-9-using-ros)


I assume things changed since 2019

##### Benjamin Hug [Cyberbotics] 02/28/2022 14:54:29
The error you stated here is for `webots_ros2` but the link you gave is for ROS 1.What version of ROS are you using?

##### Tom\_Wolf 02/28/2022 14:54:51
ROS 2 galactic

##### Benjamin Hug [Cyberbotics] 02/28/2022 14:55:25
Ok and you installed Webots with debian package ?

##### Tom\_Wolf 02/28/2022 14:55:41
Yes !

##### Benjamin Hug [Cyberbotics] 02/28/2022 14:59:56
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

##### Luftwaffel [Moderator] 02/28/2022 21:12:38
`@josh101` coordinates you get from the supervisor are generally in reference to the world. I have not followed your whole thread so I dont know the specifics of the question.


what are you trying to achieve?

##### josh101 02/28/2022 21:15:57
I just want to look at how the CoM of my robot changes when I set it to different positions, and I can now read the CoM of my robot. But my coordinates don’t mean much as I can’t figure how what they’re in reference to. When you say they’re in reference to the world, is there a way to get this axis so I can see where it’s coming from. If that makes sense

##### DrakerDG [Moderator] 02/28/2022 21:45:38
The reference is the center of the world x, y, z (0, 0, 0)

## March

##### Baya19 03/01/2022 06:18:21
Hi everyone, did anyone deal with drones before in webots ? and know how to maintain a drone at a fixed high altitude ?

##### Luftwaffel [Moderator] 03/01/2022 06:20:27
Implement some sort of PID controller. Add accelerometer and GPS sensor to get error values and control the propeller speed based on those

##### Yannnick3 [Cyberbotics] 03/01/2022 09:32:35
You can take a look at the mavic\_2\_pro example. The world and associated controller can be found in  `webots/projects/robots/dji/mavic`.

##### Baya19 03/01/2022 10:43:16
I've tried that. it doesn't work, i think the problem is in the coordinate system. I suffer to find the right PID params for ENU coordinate system

##### Luftwaffel [Moderator] 03/01/2022 10:45:26
Feel free to post your code for the Implementation. Maybe I can spot some obvious issues

##### Baya19 03/01/2022 12:14:08
thank you


the drone flies but it can't be fiwed at a fixed altitude



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/948191495788314654/unknown.png)
%end

##### Rico Schillings[Sweaty] [Moderator] 03/01/2022 16:33:07
I have the following problem i cant explain.. I'm using the latest docker image, start it with the provided command and add `--stream` since i want to use the Web streaming interface. With this it starts an empty world and i can connect to the streaming server from outside the container. When i mount the local folder with my world/controller files, i can start the simulation inside the container without the streaming server (needs nearly 5secs to load and start the world). But when i add the `--stream ` flag with my world, it hangs and the output of initializing the webserver (showing port) is not coming. 🤷‍♂️ so using container without stream works, but with it wont. Streaming works without loading my world. Its pretty confusing.. Any ideas/hints to check?

##### Luftwaffel [Moderator] 03/01/2022 22:14:48
hmm perhaps you can share your whole project. It is hard to tell from the code alone. But from what I can see, you only implemented a P controller, but you control it proportional to the error ^3


at least do a PD controller (you can probably leave the Integral error out)


for the D error you can just do `error - error_previous ` .  To get a true derivative, you would have to divide by your timestep, but that is a constant and can be skipped, assuming you never change your timestep. If you might change your timestep in the future, I would advice dividing by it.


`vertical_input = kp * P_error + kd * D_error`


And leave the P\_error linear, dont do a power


and instead of clipping an error, I would rather clip the final input signal to a stable range

##### Ranga Kulathunga 03/02/2022 05:13:20
Hi all! How can see a Webots vehicle in SUMO simulation? I tried as mentioned in the documentation, to change the DEF name to WEBOTS\_VEHICLE0. But it is not worked. Do you guys have any idea about this?

##### joachim honegger [Cyberbotics] 03/02/2022 07:25:56
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

##### emrys 03/02/2022 17:34:11
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

##### Luftwaffel [Moderator] 03/03/2022 09:03:09
`@KENPACHI` take a look at this sample world and controller. It should show you how to implement it
> **Attachment**: [emitter\_receiver\_test.rar](https://cdn.discordapp.com/attachments/565154703139405824/948868121391923260/emitter_receiver_test.rar)


line 2 and 27 are the important part


that you are missing

##### amna 03/03/2022 09:05:00
How to turn on drone collision?

##### KENPACHI 03/03/2022 09:05:27
Got it! trying it right now!


Working! thanks a lot.

##### Luftwaffel [Moderator] 03/03/2022 09:18:13
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

##### Luftwaffel [Moderator] 03/04/2022 00:26:00
[https://github.com/cyberbotics/community-projects](https://github.com/cyberbotics/community-projects) there is the robotiq 2 finger gripper included here

##### tokia 03/04/2022 00:26:32
yessssssssssssssssssss thank you i didnt know this existed

##### Luftwaffel [Moderator] 03/04/2022 00:27:16
there is also a bunch of robot arms I added, such as the majority of kuka robots

##### tokia 03/04/2022 00:37:34
I modelled and made bounding box groups for all the robocup@work manipulation objects into webots, so I could add them to that once my project is working

##### Luftwaffel [Moderator] 03/04/2022 00:38:14
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

##### Benjamin Hug [Cyberbotics] 03/07/2022 12:55:38
This has been fixed after the release of Webots R2022a, therfore I invite you to download one of the nightly build ([https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)).

##### moebius 03/07/2022 23:40:42
Hi, when i am using the webots recording feature while running the simulations in headless mode with xvfb-run , the recordings are all blank, is there a way around it? Also when i record it in gui mode, the quality is pretty bad ( the quality parameter cannot seem to be set above 60, it doesn't record otherwise and this is something i saw mentioned in an issue on github as well)

##### Luftwaffel [Moderator] 03/08/2022 01:19:34
Well it needs to render in order to record a video. So headless will not work. As for the quality, make sure the resolution is high enough. You might also have turned down a lot of the "nice" rendering options, or it is not supported (I am assuming you are using docker containers without gpu acceleration?)

##### moebius 03/08/2022 06:09:40
yes currently running it headless on docker containers without gpu acceleration but will run it on gpus soon.  Is there  any workaround to record it in headless mode, in situations like this?

##### Luftwaffel [Moderator] 03/08/2022 06:52:09
software render on cpu is crappy always. You really need that gpu acceleration for anything visual. That includes visual sensors as well (lidar, camera etc.)


what do you need to record?


or why?

##### moebius 03/08/2022 07:31:54
so we want to run batches of simulations, and since those will be running on docker containers, we want the ability to record them as well

##### Luftwaffel [Moderator] 03/08/2022 07:35:13
add cameras in the simulation


that way you can explicitly controll all aspects of it

##### moebius 03/08/2022 17:12:03
okay so if i add cameras, i can record videos in headless mode as well??

##### Luftwaffel [Moderator] 03/08/2022 18:06:46
Yeah, but you need to do it through the controller. But perhaps wait for an <@&568329906048598039> dev to answer. There might be other solutions

##### Robokashi 03/08/2022 19:49:23
Hi ! I am trying to build MoveIt2 on Windows using the recently provided information (deep thanks to whoever wrote/made that available !).

I am running into an issue when building, as cmake complains I don't have pkgconfig installed. Is it supposed to come with the ROS2 install ?

##### Luftwaffel [Moderator] 03/08/2022 20:34:34
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

##### Benjamin Hug [Cyberbotics] 03/09/2022 10:56:04
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

##### alex06228 03/10/2022 00:23:17
Hello, I have a pretty simple question with the installation; whenever I try to open the Webots app, it just crashes, without explanation. No error sign or anything, just crashes. Does anybody know how to fix this?

##### Mat198 03/10/2022 00:27:00
Do you have minimum requirements? Witch SO are you using?

##### alex06228 03/10/2022 00:41:23

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/951278560876822558/unknown.png)
%end
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/951278561149485096/unknown.png)
%end

##### Mat198 03/10/2022 00:43:54
Do you have a graphic card? I use webots on Win10 and it's fine


Type dxdiag in the windows menu to see

##### alex06228 03/10/2022 00:50:41
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

##### Luftwaffel [Moderator] 03/10/2022 21:01:58
you have to get a handle to the current working directory

##### Naxi 03/10/2022 21:17:39
That did it, thanks!

##### KENPACHI 03/10/2022 22:33:17
I'm trying to track the center of mass of my bot, what would be the best way to do that?

##### DDaniel [Cyberbotics] 03/10/2022 22:36:02
You can use a supervisor to retrieve that information using [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_center\_of\_mass](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_center_of_mass)

##### KENPACHI 03/10/2022 22:42:33
I'm using webots just for a week now, I'm not sure how I use that code to track my bot.

##### Luftwaffel [Moderator] 03/10/2022 22:44:04
what language are you using?

##### KENPACHI 03/10/2022 22:44:11
python

##### Luftwaffel [Moderator] 03/10/2022 22:44:42
okay, so in webots make sure you set ïs SUpervisor" to true

##### KENPACHI 03/10/2022 22:46:14
did that

##### Luftwaffel [Moderator] 03/10/2022 22:46:20
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

##### Luftwaffel [Moderator] 03/10/2022 22:50:32
let us know how it goes

##### KENPACHI 03/10/2022 22:52:20
I am getting the coordinates now!


my bot is a snake at the moment, and my head is the supervisor


so are these coordinates the center of the whole bot or just the head?

##### Luftwaffel [Moderator] 03/10/2022 22:53:49
it should be the combined center of mass of everything that is inside your node, so your robot with all its chilldren


but better to verify

##### KENPACHI 03/10/2022 22:55:32
ohhh, I have them as separate modules. module 1 is the supervisor
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/951614314870083604/unknown.png)
%end


so I think I'll be getting the coordinates of the first module.

##### Luftwaffel [Moderator] 03/10/2022 22:56:08
they are independent from each other? each with its own controller?

##### KENPACHI 03/10/2022 22:56:58
just one controller, with the supervisor emitting the motor rotation values

##### Luftwaffel [Moderator] 03/10/2022 22:58:07
but the other modules need a controller to receive them right?

##### KENPACHI 03/10/2022 22:58:16
yes!

##### Luftwaffel [Moderator] 03/10/2022 22:58:25
so every one of them has its own controller

##### KENPACHI 03/10/2022 22:58:42
correct

##### Luftwaffel [Moderator] 03/10/2022 22:58:58
yeah, then this does not work


I'm guessing you want to dynamically link and seperate modules during the simulation?

##### KENPACHI 03/10/2022 23:01:03
they are all linked to eachother with a motor, I just want to track the center of mass of the bot so maybe I could train it to optimize it's gait parameters.


would I be able to pick a module as it's center and track that

##### Luftwaffel [Moderator] 03/10/2022 23:01:46
Is there a reason the have to be defined as individual robots and not a single robot?

##### KENPACHI 03/10/2022 23:02:57
I'd be controlling each point motor individually


based on a sine wave

##### Luftwaffel [Moderator] 03/10/2022 23:03:22
they can all be inside the same robot for that

##### KENPACHI 03/10/2022 23:03:31
oh

##### Luftwaffel [Moderator] 03/10/2022 23:03:47
the only reason to have it as separate robots would be swarm robots


that can dynamically link up

##### KENPACHI 03/10/2022 23:04:05
it's not a swarm

##### Luftwaffel [Moderator] 03/10/2022 23:04:15
but if the robot in its configuration is fixed, just add them as children

##### KENPACHI 03/10/2022 23:04:43
how would the controller change in that case?

##### Luftwaffel [Moderator] 03/10/2022 23:05:12
you just need one controller and you initiallize all its motors and address them


is it rotational motors?

##### KENPACHI 03/10/2022 23:05:49
oh alright, I'll check it out

##### Luftwaffel [Moderator] 03/10/2022 23:06:02
make sure the motors have unique names

##### KENPACHI 03/10/2022 23:06:17
but at the moment if I need to track the center module, is there any way?

##### Luftwaffel [Moderator] 03/10/2022 23:07:08
manually by getting all the center of masses and calculating the combined one


Just put it all in the same robot. you will save yourself tons of hassle

##### KENPACHI 03/10/2022 23:08:09
haha, will try


thank you very much.

##### Luftwaffel [Moderator] 03/10/2022 23:09:35
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

##### Luftwaffel [Moderator] 03/16/2022 15:48:50
In the settings you can add an extra directory

##### Naxi 03/16/2022 16:43:19
WIll check that, thanks!

##### [Optimum Pride] Wintery Melony 03/17/2022 00:58:28
does anyone know how to fix this error?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/953819579476484176/unknown.png)
%end

##### Luftwaffel [Moderator] 03/17/2022 02:23:24
your file seems corrupt, seems like you did not format it properly or have a syntax error somewhere

##### [Optimum Pride] Wintery Melony 03/17/2022 02:23:51
i just installed it and its my first time opening it

##### Luftwaffel [Moderator] 03/17/2022 02:24:56
then i have no idea

##### [Optimum Pride] Wintery Melony 03/17/2022 02:26:29
im going to try reinstalling it


i changed the install location to a folder without a space and it works fine i guess

##### Ale 03/17/2022 06:08:16
Hi, I want to use Webots to generate Training Data for a neural network.

Is there any way to get the current timestamp of the simulation time (in python) or the elapsed time since the beginning of the simulation?

Since I dont get a 100% consistent speed its kinda tricky how many values I get per simulated Second.

As basicTimeStep I chose 8 and as Time\_Step for the controller I chose 64

##### Luftwaffel [Moderator] 03/17/2022 06:26:19
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

##### Luftwaffel [Moderator] 03/17/2022 17:58:12
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

##### Luftwaffel [Moderator] 03/17/2022 23:13:06
If you have no textures in the urdf, then the texture folder created will be empty. As for the "no body" You would have to be more specific. Perhaps post your proto file


it might be due to the absolute links of the mesh files. Urdf is usually structured with ROS packages. Or at least relative paths


However absolute paths should work. I did implement mesh path handling fixes a while back

##### Çağrı Kaymak 03/18/2022 08:00:42
Hi all. When I click Robotis-Op2 to open robot window in Webots, I take error like this: Error: libzip.so.4: cannot open shared object file: No such file or directory (dynamic library)

Error: robot window initialization failed

Error: Cannot load the "/usr/local/webots/projects/robots/robotis/darwin-op/plugins/robot\_windows/robotis-op2\_window/librobotis-op2\_window.so" robot window library.

How can i solve this problem? Thanks in advance.

By the way, my os is Ubuntu 18.04 and webots version is r2020b rev1.

##### Luftwaffel [Moderator] 03/18/2022 08:02:08
how did you install?

##### Çağrı Kaymak 03/18/2022 08:02:22
Via .deb

##### Luftwaffel [Moderator] 03/18/2022 08:03:01
using the correct one? there is a separate version for 18.04 I believe

##### Çağrı Kaymak 03/18/2022 08:03:29
Yes, this is for 18.04

##### Luftwaffel [Moderator] 03/18/2022 08:03:42
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

##### Luftwaffel [Moderator] 03/19/2022 05:28:55
[https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_field\_import\_mf\_node](https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_import_mf_node) try this

##### cnbarcelo 03/19/2022 05:32:35
Thanks for the quick reply.

I'll give it a try, tho reading the code I saw it expects the input to be `.wbo/.wrl`

##### Luftwaffel [Moderator] 03/19/2022 05:38:56
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

##### Big Cheeser 03/26/2022 11:02:13
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

##### Loaded god complex 03/26/2022 21:51:44
hey i got a question (also i am new here, hiiii)


how do i import the python package for use with webots via external ide? im using visual studio code to edit my controller but it cant find the controller package or anything


yo


anyone online?

##### Luftwaffel [Moderator] 03/26/2022 22:35:58
`@Loaded god complex` you have to set the correct environment variables


[https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers](https://www.cyberbotics.com/doc/guide/running-extern-robot-controllers) refer to this documentation

##### Loaded god complex 03/26/2022 22:50:58
what if i set those vars and nothing new happens

##### Luftwaffel [Moderator] 03/26/2022 22:51:17
restart your program

##### Loaded god complex 03/26/2022 22:51:18
do i need to reboot or can i just sign out and back in?


i did. signed in/out

##### Luftwaffel [Moderator] 03/26/2022 22:51:37
or the console you launch the python file with

##### Loaded god complex 03/26/2022 22:51:50
i did


and slight correction. im editing the file with vs code but i am not running it from there


i just need a handle to the webots framework. so i can do things like if (hardware is not type(controller.motor)) or something like that

##### Luftwaffel [Moderator] 03/26/2022 22:54:33
you import the Controller package at the top of your file?

##### Loaded god complex 03/26/2022 22:54:40
i try


nothing works

##### Luftwaffel [Moderator] 03/26/2022 22:54:55
show me the file and error message you get

##### Loaded god complex 03/26/2022 22:55:18

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565154703139405824/957412460485296128/unknown.png)
%end


its more a edit time error


i mean it aint even an error. but i cant get intellisense to register. so i dont really know where to pull the motor python class type from

##### Luftwaffel [Moderator] 03/26/2022 22:57:57
do you initialize the Robot() class?

##### Loaded god complex 03/26/2022 22:58:04
yes

##### Luftwaffel [Moderator] 03/26/2022 22:58:04
does the controller work inside webots?

##### Loaded god complex 03/26/2022 22:58:09
yes


it just imports that file

##### Luftwaffel [Moderator] 03/26/2022 22:58:26
show me all the environment variables you set

##### Loaded god complex 03/26/2022 22:58:49
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

##### Luftwaffel [Moderator] 03/26/2022 23:00:50
you have to set the  PYTHONPATH  variable

##### Loaded god complex 03/26/2022 23:01:01
oh


does webots have its own copy or no

##### Luftwaffel [Moderator] 03/26/2022 23:01:16
also, are you sure your WEBOTS\_HOME is correct not being the default      C:\Program Files\Webots

##### Loaded god complex 03/26/2022 23:01:23
yes

##### Luftwaffel [Moderator] 03/26/2022 23:01:27
okay

##### Loaded god complex 03/26/2022 23:01:30
i installed it differently i guess


theres no webots folder in the default directory


i traced my windows desktop shortcut back to the exe location

##### Luftwaffel [Moderator] 03/26/2022 23:02:16
well if the folder exists where you specified, it should be correct


but dont forget row 5 and 6 in the table of the documentation

##### Loaded god complex 03/26/2022 23:03:07
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


Hi, It is waiting  to transfer to the real robot. No answer no error I am using ubuntu20 and 2022a webots


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

