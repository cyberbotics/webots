# Development

This is an archive of the `development` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m).

## 2020

##### David Mansolino [cyberbotics] 05/15/2020 04:57:11
`@Sanket Khadse` the controller of the drone is very simple, it doesn't do any feedback using the GPS position or any inertial unit, it might therefore easily drift.


`@Jesusmd` you want to get the type of distance sensor? If so you should use the ``getType`` function: [https://cyberbotics.com/doc/reference/distancesensor?tab-language=python#wb\_distance\_sensor\_get\_type](https://cyberbotics.com/doc/reference/distancesensor?tab-language=python#wb_distance_sensor_get_type)

##### Sanket Khadse 05/15/2020 03:02:49
Hey, may I know, why does a drone, (let it be DJI Mavic 2 Pro, which is already available into Webots, or a custom made) moves up and down and shifts constantly in one direction even if no such behaviour is defined in the controller code?

##### Jesusmd 05/15/2020 01:49:44
`@David Mansolino` Hi, I am using python, I would like to work with several distance sensors at the same time and comparing their data in console. But instead of obtain the type, I got a number.

##### Luftwaffel 05/14/2020 12:48:46
> `@Luftwaffel` Although working with ROS, we decided to split the simulation projects with minimal dependencies over ROS other than communication. For 3D Math in Cpp, we're using Eigen, and in python either numpy or numpy + transformations.py (which is standalone of tf)

`@Axel M` 

Awesome, thank you so much. That makes things easier

##### Axel M [Premier Service] 05/14/2020 09:11:01
The integration with ROS is super easy too with [http://wiki.ros.org/eigen\_conversions](http://wiki.ros.org/eigen_conversions) 🙂


Regarding your example of getting relative position between two nodes, that can be easily achieved with Eigen in cpp (3x3 Matrix -> Quaternion, quaternion / vector product)


`@Luftwaffel` Although working with ROS, we decided to split the simulation projects with minimal dependencies over ROS other than communication. For 3D Math in Cpp, we're using Eigen, and in python either numpy or numpy + transformations.py (which is standalone of tf)

##### David Mansolino [cyberbotics] 05/14/2020 05:34:23
`@Luftwaffel` instead of relying on ROS, if you are using Python you ca probably use the `transforms3d` python package which allows for example to convert from a rotation matrix to quaternions: [https://matthew-brett.github.io/transforms3d/reference/transforms3d.quaternions.html#transforms3d.quaternions.mat2quat](https://matthew-brett.github.io/transforms3d/reference/transforms3d.quaternions.html#transforms3d.quaternions.mat2quat)

##### Luftwaffel 05/13/2020 18:04:16
`@Sanket Khadse`  perhaps direct .proto file edit can help

##### Sanket Khadse 05/13/2020 17:11:36
`@Luftwaffel` that is what my problem is about. The nodes I have to copy paste one by one are in "hundreds".

##### Luftwaffel 05/13/2020 16:47:12
Perhaps add a 'Group' base node, put all your nodes in, and copy paste that

##### Sanket Khadse 05/13/2020 16:45:26
Oh, thank you for letting me know!
Keep it as a suggestion for the next update. 😄

##### Olivier Michel [cyberbotics] 05/13/2020 16:44:11
Unfortunately, this is not possible.

##### Sanket Khadse 05/13/2020 16:43:42
Hey, could you tell me how to multi-select things in scene-tree? 
I can't find a way to. I imported a VRML97 model into Webots, the model was quite large, and it's difficult to select, cut and paste each 'transform' object into a robot node's children attribute.

##### Luftwaffel 05/13/2020 16:21:05
I'll give it a try


oh lordy 😅

##### Olivier Michel [cyberbotics] 05/13/2020 16:20:18
Yes, if you have an idea to achieve this... Now you know how to open a pull request 😉

##### Luftwaffel 05/13/2020 16:19:08
yes, but it requires runtime.ini edits. Would be nice if people can have it run as an example out of the box

##### Olivier Michel [cyberbotics] 05/13/2020 16:18:13
If you use Webots with ROS, you need to install ROS, and you should get 'tf' for free, isn't it?

##### Luftwaffel 05/13/2020 16:16:44
it is almost a must if working with ROS and robots


how big of a deal would it be to add the 'tf' package to the webots ROS environment?

##### Olivier Michel [cyberbotics] 05/13/2020 16:15:26
Oops...

##### Luftwaffel 05/13/2020 16:15:08
Problem is, the link to their liscense is broken 😅


[https://sscc.nimh.nih.gov/pub/dist/bin/linux\_gcc32/meica.libs/nibabel/quaternions.py](https://sscc.nimh.nih.gov/pub/dist/bin/linux_gcc32/meica.libs/nibabel/quaternions.py)


specifically this:


[https://nipy.org/nibabel/reference/nibabel.quaternions.html](https://nipy.org/nibabel/reference/nibabel.quaternions.html)


The thing is, I want it to be able to run in the native webots environment, so no extra packages. I got it to work using their code:

##### Olivier Michel [cyberbotics] 05/13/2020 16:13:18
OK, so you need to convert this 3x3 rotation matrix to a quaternion then.


Yes, you are right. Sorry, my bad.

##### Luftwaffel 05/13/2020 16:12:12
but that returns the 3x3 matrix

##### Olivier Michel [cyberbotics] 05/13/2020 16:11:40
If you need absolute rotation (in axis-angle notation), use:  [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_orientation](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_orientation)


(gives relative rotation in axis-angle notation)


[https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_field\_get\_sf\_rotation](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_field_get_sf_rotation)

##### Luftwaffel 05/13/2020 16:09:36
how do we get axis angles? I have the 3x3 matrix

##### Olivier Michel [cyberbotics] 05/13/2020 16:09:02
```python
def axis_angle_to_quaternion(axis, theta):
    axis = numpy.array(axis) / numpy.linalg.norm(axis)
    return numpy.append([numpy.cos(theta/2)], numpy.sin(theta/2) * axis)
```


No, you have to get it as an axis-angle representation, but that's super easy to translate into quaternion.

##### Luftwaffel 05/13/2020 16:04:22
Btw, is there a way to get the quaternion orientation directly from webots? That would make things much simpler


Still a bit crude with little error correction, but it works 🙂


<@&568329906048598039> Btw, I'm currently working on a python program, that publishes the PoseStamped of one or more nodes into a rostopic. It also allows to publish the Pose relative to a specific node.

##### David Mansolino [cyberbotics] 05/11/2020 06:43:54
> Is there an quickly way to get the attribute instead of  predefined values as for example in  Keyboard class.?  I would preffer enum as in c or c++

`@Jesusmd` which language are you using?

##### Jesusmd 05/10/2020 06:29:26
Is there an quickly way to get the attribute instead of  predefined values as for example in  Keyboard class.?  I would preffer enum as in c or c++

##### Luftwaffel 04/29/2020 22:31:07
[https://www.andre-gaschler.com/rotationconverter/](https://www.andre-gaschler.com/rotationconverter/)

##### Shubham D 04/29/2020 22:30:02
I am trying to create some solid using transform and shapes.
I am not able to set the objects at desired angle.
How should I set the parameters if I need specific angle.
Or is their any simple way to convert normal vector direction to euler angles

##### David Mansolino [cyberbotics] 04/28/2020 14:09:41
You're welcome

##### Psyka 04/28/2020 14:06:55
it working fine


very nice thank's 🙂

##### David Mansolino [cyberbotics] 04/28/2020 14:04:40
You should have:
> Create\_Webots\_World\wbt\worlds\textures\ground.jpg


##### Psyka 04/28/2020 14:04:40
so I just have to rename it ?


I understand now what you mean

##### David Mansolino [cyberbotics] 04/28/2020 14:04:24
yes indeed

##### Psyka 04/28/2020 14:04:20
ho ok


?


worlds


how should I name that directory ?


it was working fine like last week

##### David Mansolino [cyberbotics] 04/28/2020 14:03:18
It's still problematic 😉

##### Psyka 04/28/2020 14:03:05
it's "Created\_World" and not "Created World" I just writte it wrong

##### David Mansolino [cyberbotics] 04/28/2020 14:01:36
because the names in the project folder should respect a strict convention (you can name the project folder as you want but not the folders inside):
   [https://cyberbotics.com/doc/guide/the-standard-file-hierarchy-of-a-project](https://cyberbotics.com/doc/guide/the-standard-file-hierarchy-of-a-project)

##### Psyka 04/28/2020 14:01:01
is a copy past of my path


Create\_Webots\_World\wbt\Created\_World\textures\ground.jpg


yes

##### David Mansolino [cyberbotics] 04/28/2020 14:00:36
But that's not what you said:
> Create\_Webots\_World\wbt\Created\_World\textures\ground.jpg


##### Psyka 04/28/2020 14:00:35
hooo why that can't it be ?


and all my .wbt are in worlds


exactly

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

##### Psyka 04/28/2020 13:57:47
Create\_Webots\_World\wbt\Created\_World\textures\ground.jpg


there I've got a directory "controllers" for controllers and "Created World" with in it "textures" and then the "ground.jpg"


so all my project are in E:\...\...\Project\wbt\


haaa sorry I'm loosing it


wbt directory


and inside I've got a directory named "textures"


I've got a directory named "Created World" where there is all my .wbt worlds using this

##### David Mansolino [cyberbotics] 04/28/2020 13:53:08
And where is it in your project directory exactly?

##### Psyka 04/28/2020 13:52:57
copy pasted


ground


textures

##### David Mansolino [cyberbotics] 04/28/2020 13:52:11
Is it correctly in a ``textures``  folder?

##### Psyka 04/28/2020 13:51:50
and it's not working


but the file is int the current directory of the current project


WARNING: DEF GROUND Solid > Shape > PBRAppearance > ImageTexture: 'textures/ground.jpg' not found.
A resource file can be defined relatively to the worlds directory of the current project, relatively to the worlds directory of the default project, relatively to its protos directory (if defined in a PROTO), or absolutely.


I've got an other issue :


Hey again,

##### mint 04/26/2020 14:54:05
Thank you Olivier!

##### Olivier Michel [cyberbotics] 04/26/2020 14:44:13
See [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_orientation](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_orientation)


It's a rotation matrix (as in OpenGL).

##### mint 04/26/2020 14:38:09
One question while developing.. what is the convention webot's using for the matrix returned by wb\_supervisor\_node\_get\_orientation function?
is it Euler Angles, Rotation Matrix, or Tait–Bryan angles?

##### David Mansolino [cyberbotics] 04/23/2020 07:38:41
You're welcome

##### fdvalois 04/23/2020 07:38:11
Thank you!

##### David Mansolino [cyberbotics] 04/23/2020 07:37:52
A breaitenberge already compatible with many robot is distributed with Webots, here is the code:  [https://github.com/cyberbotics/webots/tree/master/projects/default/controllers/braitenberg](https://github.com/cyberbotics/webots/tree/master/projects/default/controllers/braitenberg)

##### fdvalois 04/23/2020 07:34:46
You just look braitenberg controller online?

##### David Mansolino [cyberbotics] 04/23/2020 05:44:20
Hi `@imjusta23` you might simply use the braitenberg controller.

##### imjusta23 04/22/2020 20:20:17
Any advice about how to get the epuck moves randomly in this world
%figure
![image0.jpg](https://cdn.discordapp.com/attachments/565155651395780609/702614779767947394/image0.jpg)
%end


##### Dorteel 04/21/2020 09:13:59
Thank you `@Stefania Pedrazzi` ! 🙂

##### Stefania Pedrazzi [cyberbotics] 04/20/2020 06:14:54
`@Dorteel`, no there is no automatic procedure to import Choregraphe built-in motions in Webots. But if you have the motion joints values, then you should be able to reproduce it in Webots.

##### Dorteel 04/18/2020 05:48:34
Hi, I have a question regarding the Nao robot. I saw WeBots used to be able to interface with Choreographe, I was wondering if the motions for the Nao that are built-in in Choreographe can somehow be imported into WeBots?

##### David Mansolino [cyberbotics] 04/16/2020 07:15:29
You're welcome

##### Aan 04/16/2020 07:14:58
thank you 😀

##### David Mansolino [cyberbotics] 04/16/2020 07:13:19
Here is the  fix:
[https://github.com/cyberbotics/webots/pull/1548/files](https://github.com/cyberbotics/webots/pull/1548/files)
It will be available in the next release of Webots. But in the meantime, you might apply it locally to your Webots installation files.


Hi, this is because the recognition color of the soccer ball is not set, let me fix this.

##### Aan 04/16/2020 07:07:35
Hi, I am trying to soccer ball object recognition using camera\_recognition.c in project samples. When I add soccerball, camera does not recognize the ball but it can recognize apple, can, oil barrel. How camera can recognize the soccer ball?

##### David Mansolino [cyberbotics] 04/14/2020 15:13:36
You're welcome

##### İchigogo 04/14/2020 15:09:24
okay thank you so much :)

##### David Mansolino [cyberbotics] 04/14/2020 15:09:13
You can also find some tips to speed up your simulation here: [https://cyberbotics.com/doc/guide/speed-performance](https://cyberbotics.com/doc/guide/speed-performance)


Hi, you may try reducing the OpenGl features to speed up the simulation speed: [https://cyberbotics.com/doc/guide/preferences#opengl](https://cyberbotics.com/doc/guide/preferences#opengl)

##### İchigogo 04/14/2020 15:06:31
hello! sorry for disturbing again. I don't know why but my simulation looks like lagging (I don't know if its correct word to explain but ) it's happens in some sample projects too I tried changing fps and time step but it didn't change too much thing in simulation.is there any other way to solve this ?

##### David Mansolino [cyberbotics] 04/14/2020 05:22:31
Note also that a customizable door is already available in Webots: [https://cyberbotics.com/doc/guide/object-apartment-structure#door](https://cyberbotics.com/doc/guide/object-apartment-structure#door)


Hi `@Sanket Khadse`, the VRML import does indeed import only the visual meshes of the object, you then have to recreate the structure of the object yourself. I would recommend to follow these tutorials to create an objet (1 to 7): [https://cyberbotics.com/doc/guide/tutorials](https://cyberbotics.com/doc/guide/tutorials)

##### Sanket Khadse 04/12/2020 06:30:27
Hey, I tried importing a door hinge as a VRML97 file, which was converted from Solidworks model. 
The problem is, it is considering each part of the hinge, like screws, doors as separate individual. (As in the scene tree, it is showing each part differently with same name - "transform") 
Neither it is considered as other objects in the world's, because while I tried to click on the hinge, it won't click and show it's origin and all. 
What must be the problem here?


Sure,  I will look into them and let you know if it helps! 😊

##### David Mansolino [cyberbotics] 04/09/2020 12:49:19
Did you follow our tutorials? If not I would strongly advice to follow tutorials 1 to 6: [https://cyberbotics.com/doc/reference/supervisor](https://cyberbotics.com/doc/reference/supervisor)


You can either use the Supervisor API to change it's rotation field either mount the radar on a controllable joint.

##### Sanket Khadse 04/09/2020 12:43:17
`@David Mansolino`,  in the first question, after I imported the model into webots,  how should I define the movement of that radar part as needed?

##### David Mansolino [cyberbotics] 04/09/2020 12:38:42
About the second issue, I would use the Supervisor API to move the whole mast: [https://cyberbotics.com/doc/reference/supervisor](https://cyberbotics.com/doc/reference/supervisor)


About the first issue, you mean you don't now how to make the radar moving ?

##### Sanket Khadse 04/09/2020 12:35:55
Hey, I want to use a ship mast in my simulation,  which basically has a tower like structure and a radar (dish),  which continuously moves around in a 90° angle.  
So the problem is,  I dont know how to make the part moving in the manner I need.  
There's another interesting problem,  that the whole mast should move in a sinusoidal manner (basically, the way it moves in the sea). How could these things be done?! 
Thank you!

##### DrVoodoo [Moderator] 04/04/2020 16:51:14
If shape [https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)


Shape or colour/texture?

##### imjusta23 04/04/2020 16:24:36
Hey, I hope you’re doing good!! Where can I find a tutorial to change the appearance of my robot?


> Hi `@imjusta23`, this sample show a very simple light follower: [https://cyberbotics.com/doc/guide/samples-devices#light\_sensor-wbt](https://cyberbotics.com/doc/guide/samples-devices#light_sensor-wbt)

`@David Mansolino`  thanks!!

##### David Mansolino [cyberbotics] 04/01/2020 17:18:23
Hi `@imjusta23`, this sample show a very simple light follower: [https://cyberbotics.com/doc/guide/samples-devices#light\_sensor-wbt](https://cyberbotics.com/doc/guide/samples-devices#light_sensor-wbt)

##### imjusta23 04/01/2020 17:12:59
Hey, does someone has a code for a light follower?

##### David Mansolino [cyberbotics] 03/27/2020 07:49:44
You're welcome.

##### luhang 03/27/2020 07:47:41
Thanks!😁

##### David Mansolino [cyberbotics] 03/27/2020 06:21:02
Hi yes of course, in Webots you can use the regular Webots API to control the simulated robot, you will find some example controllers here: [https://github.com/cyberbotics/webots/tree/master/projects/robots/softbank/nao/controllers](https://github.com/cyberbotics/webots/tree/master/projects/robots/softbank/nao/controllers)

##### luhang 03/27/2020 01:44:51
I didn't find APIs


Can we program and control the NAO robot in webots now?

##### David Mansolino [cyberbotics] 03/25/2020 12:39:30
You're welcome 😉

##### mint 03/25/2020 12:37:37
wow.. Thank you I couldn't find that even after hours of search. You saved a lot of time for me 🙂

##### David Mansolino [cyberbotics] 03/25/2020 12:33:50
In Python I personnally often use the ``transforms3d``module (available on PIP: [https://pypi.org/project/transforms3d)](https://pypi.org/project/transforms3d)) which allows to convert between many 3D representations, you will probably find what you need in the doc: [http://matthew-brett.github.io/transforms3d/](http://matthew-brett.github.io/transforms3d/)

##### mint 03/25/2020 12:32:24
I have been using python 3..

##### David Mansolino [cyberbotics] 03/25/2020 12:31:30
Hi `@mint` unfortunately, Webots API does not provide such function. But I am sure you can find plenty of external libraries that do this. Which language are you using ?

##### mint 03/25/2020 12:29:59
Oh, its setVelocity function at supervisor


Hello, I have been develping with webot and encountered a issue with setVelocity function for a node  in supervisor. The setVelocity function takes the angular velocity in form of rotation aroud respective x y z rotation, but my calculation for angular velocity would give the result in form of axis-angle (normal vector and the magnitude of rotation around it). I'm now troubled with converting w in axis-angle form to the x y z rotation form. Is there any formula, functions or library supported by webot?

##### David Mansolino [cyberbotics] 03/23/2020 08:55:47
You're welcome 😉

##### Jesusmd 03/23/2020 08:50:34
`@David Mansolino`  ho my fault, I see it is points.x or some other proprieties, thanks!


`@David Mansolino`  the loop is correct but for some reason my x,y and z values give 0,  I tried something like this : Lidar\_h\_r = Lidar1.getHorizontalResolution()
    print("the getHorizontalResolution is:", Lidar\_h\_r)
    Layer = Lidar1.getNumberOfLayers()
    Layer\_id = Lidar1.getLayerPointCloud(Layer)
    for i in range(Lidar\_h\_r):
        Point = Layer\_id[i]
        print("point:",Point.z,Point.y,Point.z)

##### David Mansolino [cyberbotics] 03/23/2020 07:37:30
In that case you should be able to make a loop and modify the LidarPoint objects, somthing like (untested):
```Python
for point in lidar.getPointCloud():
    # do something with the point
```

##### Jesusmd 03/23/2020 07:34:28
`@David Mansolino` the global idea is make lidar to grid map


Hi, `@David Mansolino` I using python language. The idea is using the measure of Lidar to make an csv file that contains measures and corresponding angles. so I need to manipulate the data

##### David Mansolino [cyberbotics] 03/23/2020 06:19:27
HI `@Jesusmd` which lanugag are you using?
Why do you want to change the properties of the lidar points? These value should be seen are readonly value as they represent the measure of the sensor.

##### Jesusmd 03/23/2020 04:32:31
Hi, I need to read the LidarPoint properties with the CluodPointEnable, but I just have one list of object pointers, so my questions is how I can set all properties with LidarPoint class imported? I mean set all values of each point into a object made by LidarPoint class.

##### David Mansolino [cyberbotics] 03/18/2020 12:14:40
You're welcome

##### taeyoung 03/18/2020 12:14:23
Ok Thank you

##### David Mansolino [cyberbotics] 03/18/2020 10:56:25
HI `@taeyoung`, this is unfortunately not possible out of the box, however, the solution is to use the Supervisor API, using the supervisor API you can at each step monitor the position of the ball and move the light accordingly.
Here is an example where a Supervisor moves a light at a predefined location: [https://cyberbotics.com/doc/guide/samples-devices#supervisor-wbt](https://cyberbotics.com/doc/guide/samples-devices#supervisor-wbt)

##### taeyoung 03/18/2020 10:15:13
Hi! Can I put the pointlight or spotlight over the ball? I mean make the pointlight keep the height while it follows the ball. When I put the pointlight in the children of the ball with location 0 1 0,  as ball rolls the pointlight goes down and up.

##### David Mansolino [cyberbotics] 03/13/2020 06:31:37
Hello `@rbhuva`, you might use a LightSensor node: [https://www.cyberbotics.com/doc/reference/lightsensor](https://www.cyberbotics.com/doc/reference/lightsensor)

##### rbhuva 03/12/2020 19:12:45
Hello!! Is it possible to measure the exact intensity of the point light source at the floor? I want to know it because I am changing the height of the fixed intense source from the floor!

##### elkelkmuh 03/02/2020 13:20:34
Hi `@David Mansolino`   thank you for answer. I sent  an email again

##### David Mansolino [cyberbotics] 03/02/2020 07:11:56
Hi `@elkelkmuh` thank you I saw it, but haden't time to answer all my emails from the week-end yet, I will answer for sure today.

##### elkelkmuh 03/02/2020 07:11:20
`@David Mansolino`  I sent an email.

##### David Mansolino [cyberbotics] 03/02/2020 06:39:28
`@KyleM` Welcome!
for vehicle and ROS, there is a specific controller making the bridge between Webots and ROS1 it should work out of the box: [https://github.com/cyberbotics/webots/tree/master/projects/vehicles/controllers/ros\_automobile](https://github.com/cyberbotics/webots/tree/master/projects/vehicles/controllers/ros_automobile)
If you have issue recompiling Webots, here is the guide: [https://github.com/cyberbotics/webots/wiki/Linux-installation](https://github.com/cyberbotics/webots/wiki/Linux-installation)
(It should work for Ubuntu 16.04 as I am using it on a daily basis)

##### KyleM 02/28/2020 23:53:37
I had comment most of the child make(s) and just compile the vehicles folder


Was able to solve my own problem by running 'make' on the main /projects folder. It doesn't build cleanly on 16.04 Ubuntu by the way


Is there any documentation or projects I can study to figure how to control my robot my ros?


However...I can't figure out how to compile the ros\_automotive to connect it my some ros nodes


I built a custom Car PROTO and I can drive it around with a python controller using the Driver class


I'd really like to use webots + ros for my autonomous vehicle project


Howdy folks!

##### elkelkmuh 02/28/2020 13:43:16
Ok  thank you

##### David Mansolino [cyberbotics] 02/28/2020 13:42:30
May I ask you to send your request to support@cyberbotics.com .


May I ask you to send your request to support@cyberbotics.com (precising exactly what you need (e.g. real robot only, or both real and simulated, remote-control only or cross-compilation too, which version of the robot, etc.)), I am not able to answer you here directly.

##### elkelkmuh 02/28/2020 13:06:11
`@David Mansolino`  How much is it?

##### David Mansolino [cyberbotics] 02/28/2020 13:03:24
Hi `@elkelkmuh` yes of course we can give support for such kind of things, however, we would need to have access to the real robot with FSR to develop this.

##### elkelkmuh 02/28/2020 12:16:35
Hi `@David Mansolino`   I saw your pricing list. On the event you can give special support. Could you give FSR support for real robot. If you give support. How much is it?

##### rbhuva 02/27/2020 15:36:37
thank you so much...

##### David Mansolino [cyberbotics] 02/27/2020 06:19:17
Beside using a GPS ?

##### rbhuva 02/26/2020 23:06:51
any other solution beside it?

##### David Mansolino [cyberbotics] 02/26/2020 06:45:38
[https://cyberbotics.com/doc/reference/gps](https://cyberbotics.com/doc/reference/gps)

##### rbhuva 02/26/2020 06:45:11
i have attached a GPS at extension of the Khepera-1 can anyone give me the code to store the GPS co-ordinates?

##### David Mansolino [cyberbotics] 02/26/2020 06:42:40
You're welcome

##### rbhuva 02/26/2020 06:26:57
Thank You so much

##### David Mansolino [cyberbotics] 02/26/2020 06:21:10
Hi `@rbhuva` the distance sensors of the khepera-1 are already present in the model: [https://cyberbotics.com/doc/guide/khepera1](https://cyberbotics.com/doc/guide/khepera1)
To use them, I strongly recommend following this tutorial: [https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers](https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers)
It uses the e-puck robot, but it works the exact same way for the khepera robot (excepts that the device names are different)

##### rbhuva 02/26/2020 01:21:11
how can i configure proximity sensor in khepera-1?
and how to add it into code...?
Or how can I avoid collision of robot with wall of circle arena...

##### cnaz 02/25/2020 11:56:42
`@David Mansolino` thx you, I'll look


`@DrVoodoo` I plan to use Q learning

##### David Mansolino [cyberbotics] 02/25/2020 11:45:42
`@cnaz` if you are not familliar with this page, I would suggest to have a look at it: [https://www.cyberbotics.com/doc/guide/using-numerical-optimization-methods](https://www.cyberbotics.com/doc/guide/using-numerical-optimization-methods)

##### DrVoodoo [Moderator] 02/25/2020 11:45:08
Hello `@cnaz` , what approach were you planning on using for reinforcement learning?

##### cnaz 02/25/2020 11:43:03
Hi `@Olivier Michel` I already do the simple python controllers, and it works but with an algorithm of reinforcement learning I don't see where to star and what Ill do. I'll visite robotbenchmark

##### Olivier Michel [cyberbotics] 02/25/2020 08:46:29
All these benchmarks are also included in Webots in the Webots/projects/samples/robotbenchmark folder.


I would recommend you to visit [https://robotbenchmark.net](https://robotbenchmark.net) for a series of Python-based robotics challenges that may involve reinforcement learning.


Hi `@cnaz`, I would recommend you to start with simple Python controllers (see for example Webots/projects/languages/python/worlds/), define a task to be learnt by the robot and use reinforcement learning techniques in your Python controller.

##### cnaz 02/25/2020 08:36:14
I have a project to link a algorithm of rei forcément learning and Webots. My programming learning is python. But the problem is I don't know where to do it. Is someone want to realize the project with me or help, I'll be grateful. Thx you guys (my operating system is Ubuntu 18)

##### David Mansolino [cyberbotics] 02/25/2020 06:36:38
`@rbhuva` you can find an example of robot going in the direction of the light here: [https://cyberbotics.com/doc/guide/samples-devices#light\_sensor-wbt](https://cyberbotics.com/doc/guide/samples-devices#light_sensor-wbt)
The e-puck sensor model already have light sensors so using this robot model should be quite easy to do what you want

##### rbhuva 02/24/2020 16:53:59
I want the behaviour such that my robot go in direction of light and then stop near the light 

if someone can help me with that code I am very thankful...


and i have very less time


because i try a lot but failed in coding


i am ready to use either khepera-1 or epuck


i want code for the bot such that it senses the light and go towards that


okay thank you

##### Olivier Michel [cyberbotics] 02/24/2020 15:21:47
Yes, Cyberbotics can provide you with paid support if needed. You can ask for an offer at sales@cyberbotics.com. Otherwise if you have some quick question, we may answer you here.

##### rbhuva 02/24/2020 15:14:52
can any one help me to achieve this...


In my project, I would be changing the number of light sources considering them as a food source for the robot (treated like animals). The hypothesis of doing so is to observe the behaviour of an individual animal and swarm of the animals when they have different food sources available at different locations. Moreover, it is also assumed that it is possible to simulate the behaviour of the animals towards the food origin of different quality. That is, they might turn to the light(food) source of the higher intensity even though the less intense light source is nearer to them
To carry out these experiments, I will consider the intensity of the sources, number of sources, number of vehicles as my independent variables.

##### Olivier Michel [cyberbotics] 02/21/2020 10:07:30
Yes, that's a very good idea as well.

##### Axel M 02/21/2020 09:51:54
Thats great, in the meantime I was considering modifying urdf2webots in order to generate PROTO files containing only the IndexedFaceSet of the parsed mesh (that's what i've done manually on my model). In the light of the above issue, does that seems still relevant to you ?

##### Olivier Michel [cyberbotics] 02/21/2020 09:47:26
See [https://github.com/cyberbotics/webots/issues/1396](https://github.com/cyberbotics/webots/issues/1396)


Regarding point 2 (large meshes), we are considering implementing support for meshes in Webots in different formats (DAE, OBJ, STL) by introducing a new node called Mesh that would replace the IndexedFaceSet node by referring to a mesh file (DAE, OBJ, STL, etc.) instead of listing a large number of coordinates and coordinate indices. I will open an issue about it to explain our design ideas. You are very welcome to contribute by proposing an implementation.

##### Axel M 02/21/2020 09:28:05
Some issue I ran into when using urdf2webots:
- The resulting model in webots is laying on its right side (-90 deg roll). This seems to be related to axes differences between ROS (Z upward) and webots (Y upward)
- Large meshes are merged into a single PROTO file, which results in a (very) big file not suited to manual modifications in a text editor

I'm eager help on those issues

##### David Mansolino [cyberbotics] 02/14/2020 13:15:00
Perfect !

##### junjihashimoto 02/14/2020 13:01:49
Setup CI is done.


`@David Mansolino` Thx! I'm grad to hear that. I will do it.

##### David Mansolino [cyberbotics] 02/14/2020 08:47:46
`@junjihashimoto` we just had a discussion and you are welcome to transfert your HsWebots-project  🙂


Ok, interesting.

##### junjihashimoto 02/14/2020 08:06:40
I will use github-actions, because it has 8GB memory and long running time.

##### David Mansolino [cyberbotics] 02/14/2020 07:47:27
It would also be very nice if you could include in the README the version of Webots and the supported OS.


Jsut out of curiosity, which CI are you planning to use ?


Adding a CI is a very good idea and sign of quality.
For macOS unfortunately we do not provide any binary-tar-ball, however it should be feasible to install the mac package from the command line.


> Can I move HsWebots-project into cyberbotics? If you have any concerns, please feel free to contact me.

`@junjihashimoto` 
Thanks for this, we will discuss this internally ant let you know.

##### junjihashimoto 02/14/2020 07:36:39
Do you provide binary-tar-ball for macos?


BTW, I want to do CI on both linux and macos. I know linux has  a deb package of webots. So I can install it by apt. How about macos?


Can I move HsWebots-project into cyberbotics? If you have any concerns, please feel free to contact me.


`@David Mansolino` Thank you for inviting me on github.

##### junjihashimoto 02/13/2020 10:07:37
The fix is done. [https://github.com/junjihashimoto/HsWebots/blob/master/src/Webots/Supervisor.hs#L148](https://github.com/junjihashimoto/HsWebots/blob/master/src/Webots/Supervisor.hs#L148)

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

##### junjihashimoto 02/13/2020 09:00:15
Thx! If the field does not exit, It returns NULL. So I should check NULL or not here. [https://github.com/junjihashimoto/HsWebots/blob/master/src/Webots/Supervisor.hs#L144](https://github.com/junjihashimoto/HsWebots/blob/master/src/Webots/Supervisor.hs#L144)

##### David Mansolino [cyberbotics] 02/13/2020 08:55:49
Yes, you can use the supervisor api to get the fields of a node:
[https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_field](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_field)

##### junjihashimoto 02/13/2020 08:54:12
`@David Mansolino` I want to check what fields the nodes have. Does webots have the api finding fields?

##### David Mansolino [cyberbotics] 02/13/2020 08:46:14
You're welcome, thank you for sharing this with us.

##### junjihashimoto 02/13/2020 08:44:39
`@David Mansolino` Thank you for your feedback. I've fixed it.

##### Fabien Rohrer [Moderator] 02/13/2020 08:34:11
That's a great news! You should post it in the <#568354695513374730> chanel for a better visibility.

##### David Mansolino [cyberbotics] 02/13/2020 08:33:01
just for the note, the link to the github project seems wrong (it seems you forgot te replace the 'githubuser') 😉


Hi `@junjihashimoto`, very nice!!!

##### junjihashimoto 02/13/2020 08:30:48
Hello, I'm Junji Hashimoto, Gree Inc. I've developed webots-bindings for haskell.  [https://hackage.haskell.org/package/HsWebots](https://hackage.haskell.org/package/HsWebots)

##### David Mansolino [cyberbotics] 02/10/2020 06:36:30
Hi `@luoyu2014` you can download all the old version fo webots from here: [https://github.com/cyberbotics/webots/releases/tag/R2019a](https://github.com/cyberbotics/webots/releases/tag/R2019a)


Hi `@rbhuva` what do you mean by sources? Source of robots? Of light? Of energy?

##### luoyu2014 02/09/2020 07:44:42
Hi, I want to know where I can download the Webots 2018b version, I need this version. Thank you.

##### rbhuva 02/07/2020 16:51:33
Hi there
I want to make one model in which I want multiple sources and number of bots. And I want to see that how the bots will distribute towards the various sources.
Can anyone please help me how can I achieve that?

##### David Mansolino [cyberbotics] 01/28/2020 10:19:07
`@nitrow` perfect, thank you for the feedback!

##### nitrow 01/28/2020 10:18:03
`@David Mansolino` Great, works a lot better now!

##### David Mansolino [cyberbotics] 01/28/2020 06:56:58
`@nitrow`, just to let you now that this is now fixed in [https://github.com/cyberbotics/webots/pull/1300](https://github.com/cyberbotics/webots/pull/1300) and will be included in the next release (and the nightly builds).


Hi `@nitrow`, thank you for the suggestion, I will check in the datasheet if I find the CoM and fix it if there is some issues.

##### nitrow 01/25/2020 19:18:45
I would suggest moving the CoM a little in front of the turtlebot3 model to counteract it being very unstable when accelerating. I worked a lot better when I did this. I don't know the correct CoM, but I'm sure the real turtlebot is not that prone to doing wheelies! 😄

##### ZoRal 01/23/2020 14:12:18
Thanks

##### David Mansolino [cyberbotics] 01/23/2020 13:49:31
You're welcome

##### Hannah 01/23/2020 13:49:07
Thanks 🙂

##### David Mansolino [cyberbotics] 01/23/2020 13:43:48
Unfortunately it is not.

##### Hannah 01/23/2020 13:42:27
Thanks `@David Mansolino` , I wanted to know if it is already implemented or not.

##### David Mansolino [cyberbotics] 01/23/2020 13:40:09
Hi `@Hannah` not out of the bxox, but I am pretty sure that you can implement your own interface to take these MES data as input of your robot controller.

##### Hannah 01/23/2020 13:38:38
Hello, is there any way to get MES(Manufacturing Execution System) data in webots? I want to simulate a production line, and I need to work with the MES data generated by the machines in the line.

##### Fabien Rohrer [Moderator] 01/23/2020 10:59:35
[https://www.cyberbotics.com/doc/guide/pr2](https://www.cyberbotics.com/doc/guide/pr2)


Yes, here it is:

##### machinekoder 01/23/2020 10:58:35
`@Fabien Rohrer` Thanks, I'll try that. Is the PR2 model in the examples?

##### Fabien Rohrer [Moderator] 01/23/2020 10:50:34
In such case, you can master the grasping force precisely, and that matters 🙂


The most important thing was to control the gripper in force control rather than in position control.


[https://www.youtube.com/watch?v=Lm0FhXAxkXg](https://www.youtube.com/watch?v=Lm0FhXAxkXg)


I had similar issues for the PR2 simulation.


I think you need to actuate the Webots joints of the underactuated joints, yes.

##### machinekoder 01/23/2020 10:14:42
[https://youtu.be/240b1ubUwMA](https://youtu.be/240b1ubUwMA)


Not really. The example grippers seem to have actuated joints. However, the ezgripper has two under-actuated joints, meaning they are not driven by a motor. What's the best way to simulate this behavior. Or shall I just actuate them in the sim? I tried to mimic the real behavior using touch sensors, but my approach wasn't successful.

##### Fabien Rohrer [Moderator] 01/23/2020 10:08:29
Does this answer your question?


Other solutions looks like hacks to me.


This is certainly the best way to implement this.


Generally all our grippers have physics, collision  shapes and joints.


Hi,

##### machinekoder 01/23/2020 10:05:11
I have acquired an EZgripper ([https://sakerobotics.com/)](https://sakerobotics.com/)) for my robot. What's the best way to simulate such an under actuated gripper? I've written a small Python controller, but I can't really make it grasp properly. Any ideas? I don't really need the physics aspect of the grasping itself, just being able to move objects around.

##### David Mansolino [cyberbotics] 01/17/2020 14:29:55
No sorry, Webots doesn't provide such drag and drop interface

##### Angerbot 01/17/2020 14:28:29
Apologies for the continuous questions - lines here have broken down and unable to download just yet. Is there a drag and drop interface for simple movement design? Thanks

##### David Mansolino [cyberbotics] 01/16/2020 06:52:50
You're welcome

##### rbhuva 01/16/2020 06:52:29
Thanks `@David Mansolino`

##### David Mansolino [cyberbotics] 01/16/2020 06:51:59
Using the safe mode might help: [https://cyberbotics.com/doc/guide/starting-webots#safe-mode](https://cyberbotics.com/doc/guide/starting-webots#safe-mode)


`@rbhuva`, what is exactly the error you get?
PLease make sure to update the driver of your GPU.

##### rbhuva 01/15/2020 23:33:21
sorry guys I am asking very silly question but I am stuck
I downloaded and try various versions of the Webot software but after installing it always shows the error of unexpected stop working webots on windows. I need the software please give some solution.

##### David Mansolino [cyberbotics] 01/15/2020 06:54:13
`@rbhuva`, no we are using the Webots simulation environment 😉

##### rbhuva 01/15/2020 02:09:57
anyone has Breve Simulation Environment?

##### David Mansolino [cyberbotics] 01/03/2020 07:32:26
`@Shanaka` instead of using motion file you should control directly the motor individually  (in speed, position or torque/force) using the motor API: [https://cyberbotics.com/doc/reference/motor#motor-functions](https://cyberbotics.com/doc/reference/motor#motor-functions)
You can easily get the name of the individual motors by double-clicking on the robot to open the robot-window.
IF you didn't already did, I strongly recommend to follow our tutorial (which explains how to control the motors individually): [https://cyberbotics.com/doc/guide/tutorials](https://cyberbotics.com/doc/guide/tutorials)

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


## 2019

##### David Mansolino [cyberbotics] 12/10/2019 08:05:30
You're welcome

##### kawaiipotato2023 12/10/2019 08:05:24
Thank you!

##### David Mansolino [cyberbotics] 12/10/2019 08:04:56
Yes of course, you can download the latest version from here: [https://github.com/cyberbotics/webots/releases/tag/R2019b-rev1](https://github.com/cyberbotics/webots/releases/tag/R2019b-rev1)

##### kawaiipotato2023 12/10/2019 08:03:49
This was the only version I found, do you have a link to where I can get the latest version?

##### David Mansolino [cyberbotics] 12/10/2019 08:02:43
Why not using the latest version ?

##### kawaiipotato2023 12/10/2019 08:02:13
Webots 8 2.1.2 for windows

##### David Mansolino [cyberbotics] 12/10/2019 07:59:56
Hi `@kawaiipotato2023`, which version of Webots did you download?

##### kawaiipotato2023 12/10/2019 07:59:17
Hello! I just downloaded webots and when I log in it tells me that I am not licensed to use the program. Does anyone know why this happens or how to resolve it?

##### luoyu 12/09/2019 07:09:40
Thank you very much. The magical password is useful.

##### Fabien Rohrer [Moderator] 12/09/2019 05:03:17
Last week, we simply open the license server (dealing with Webots licenses before R2019) as Webots is open source since one year.


`@luoyu` could you try with the « webots » password? [https://cyberbotics.com/doc/guide/general-faq#can-i-still-use-a-webots-version-before-the-r2019a-release](https://cyberbotics.com/doc/guide/general-faq#can-i-still-use-a-webots-version-before-the-r2019a-release)

##### luoyu 12/09/2019 02:43:09
Hi, I have a problem that wrong license password for my account. Is It caused by the migration on the server？Can I know how long it will take？

##### Fabien Rohrer [Moderator] 12/06/2019 20:08:59
Good job! 👍

##### juanrh 12/06/2019 20:08:22
ah ok, it looks I should use `export WEBOTS_HOME=/var/lib/snapd/snap/webots/current/usr/share/webots` instead


any idea what might be the problem here?


I don't find that .so file in my webots installation. I have [juanrh@juanydell hello\_webots\_py]$ ls $WEBOTS\_HOME/lib
bindtextdomain.so  systemd  udev  x86\_64-linux-gnu


Hi, I'm a newby trying to use webots with Python. I'm on Fedora 30 with webots installed with snap. I did export WEBOTS\_HOME=/var/lib/snapd/snap/webots/current export PYTHONPATH="${WEBOTS\_HOME}/usr/share/webots/lib/python37"
export PYTHONIOENCODING='UTF-8'
export LD\_LIBRARY\_PATH="${LD\_LIBRARY\_PATH}:${WEBOTS\_HOME}/lib" and when I launch `python3` and do `import controller` I get "ImportError: libCppController.so: cannot open shared object file: No such file or directory"

##### David Mansolino [cyberbotics] 12/05/2019 07:21:00
`@iloving`, we are currenlty running some migration on our server. This might take some time, in the meantime you can use the latest version of Webots which doesn't require any internet license: [https://github.com/cyberbotics/webots/releases/tag/R2019b-rev1](https://github.com/cyberbotics/webots/releases/tag/R2019b-rev1)

##### iloving 12/05/2019 01:29:59
Hi, I can not connect the Internet license server, and the site [https://www.cyberbotics.com/](https://www.cyberbotics.com/) is not attached. Anyone have it?

##### Musuyaba 12/03/2019 08:40:33
I see, thank you, i will try it later

##### Stefania Pedrazzi [cyberbotics] 12/03/2019 07:28:46
You can find some examples in the Webots samples library:
- samples/devices/motor.wbt ([https://cyberbotics.com/doc/guide/samples-devices#motor-wbt)](https://cyberbotics.com/doc/guide/samples-devices#motor-wbt))
- samples/devices/motor2.wbt ([https://cyberbotics.com/doc/guide/samples-devices#motor2-wbt)](https://cyberbotics.com/doc/guide/samples-devices#motor2-wbt))
- samples/devices/motor3.wbt ([https://cyberbotics.com/doc/guide/samples-devices#motor3-wbt)](https://cyberbotics.com/doc/guide/samples-devices#motor3-wbt))
- samples/devices/linear\_motor.wbt ([https://cyberbotics.com/doc/guide/samples-devices#linear\_motor-wbt)](https://cyberbotics.com/doc/guide/samples-devices#linear_motor-wbt))


`@Musuyaba` the Servo node has been deprecated since many years now (starting from Webots 7.2). Please use the joint and motor nodes instead.

##### Musuyaba 12/03/2019 01:37:29
Hi, i get this error, 
```   11 | #include <webots/servo.h>
      |          ^~~~~~~~~~~~~~~~ 
```
and i think i need servo library, anyone have it? thankyou

##### Tahir [Moderator] 12/01/2019 01:45:12
[https://cyberbotics.com/doc/guide/samples-demos](https://cyberbotics.com/doc/guide/samples-demos)


You can see a sample soccer simulation in webots demos as well


Hi 
`@Enger` Yes it is possible

##### Enger 11/30/2019 22:42:33
Hi Guys, I am new to Webots but I want to use it for a robot soccer simulation project. I would like to use the Robotis OP2 robot but I want to code in python. Is it possible?

##### SimonDK 11/28/2019 18:09:02
Great to hear it is in the long-term plans at least 😊 I get there are more urgent things. If time would allow it, I would give it a go 😁

##### David Mansolino [cyberbotics] 11/28/2019 07:33:06
Yes we have some long-term plans to improve the scene-tree in Webots, but those are long term plans and will not be implemented in a near future (unfortunately we have currently more urgent things to improve), but if you want to try implementing this you are welcome to contribute 😉

##### SimonDK 11/27/2019 17:01:08
Regarding the Webots graphical interface, I was wondering if there are any plans to have somethings like groups or folders in the Scene Tree, or being able to drag'n'drop to reorganize items? In big simulations you easily lose overview of all items 😀

##### Dorteel 11/20/2019 09:32:31
Thanks `@Fabi`en! I think I found something online!

##### Fabien Rohrer [Moderator] 11/20/2019 09:31:16
`@Dorteel` Hi, it's a matter to compute the robot odometry, usually using simple trigonometric functions.

##### Dorteel 11/20/2019 09:30:13
Hi guys! Does anybody here have a model for converting the two wheel velocities of a differential-drive robot to the robot's angular and linear velocity?

##### Fabien Rohrer [Moderator] 11/19/2019 16:38:52
you're welcome, good luck with your project 🙂

##### Ella 11/19/2019 16:16:53
`@Fabien Rohrer`  Thank you so much!!! Maybe I should try step by step!!!

##### Fabien Rohrer [Moderator] 11/19/2019 16:11:50
But using a predefined map to start is certainly simpler.


[https://gist.github.com/fabienrohrer/543c2a9751eefc6d352e6957e6e3dc90](https://gist.github.com/fabienrohrer/543c2a9751eefc6d352e6957e6e3dc90)


The objects of a scene and their size may be retrieved using a supervisor, like this:


Does this answer your question?


If I were you, I would first use an existing library to deal with the algorithm, and a predefined map stored in the controller. Webots can retrieve easily the robot position in the map.


`@Ella` Hi, I confirm this task may be hard. There is currently no example about such implementation.

##### Ella 11/19/2019 16:04:03
Hello~ I am a real beginner of Webot!!  I want to use a* algorithm  boxes as obstacles and find the path!! but it's really hard,,,, can you give me some tips where to start??

##### Fabien Rohrer [Moderator] 11/13/2019 12:54:00
I would go for this function first: [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_simulation\_reset](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_simulation_reset)

##### pavlos27 11/13/2019 12:52:34
no i will try

##### Fabien Rohrer [Moderator] 11/13/2019 12:51:26
did you try to revert or reload the whole simulation instead?


I fear that some error is accumulated.

##### pavlos27 11/13/2019 12:49:22
this happens after 50-60 resets


but this doesn't happen at first reset


but still collisions forces are applied after reset

##### Fabien Rohrer [Moderator] 11/13/2019 12:47:48
(the floor.resetPhysics() is certainly useless, because a floor has no physics)


it sounds very correct to me.

##### pavlos27 11/13/2019 12:46:06
i have 2 robots and a rectangle arena . The robots are moving only in one linear direction  one behind the other and sometimes they are colliding . When i reset my robots to their initial position like this

##### Fabien Rohrer [Moderator] 11/13/2019 12:29:50
Could you better describe what does not work?

##### pavlos27 11/13/2019 11:20:07
unfortunately this didn't worked either

##### Fabien Rohrer [Moderator] 11/11/2019 07:20:24
`@pavlos27` Would it be possible to translate it at the floor level, but to add a very small vertical offset, just in order to avoid collisions with the floor during the translation?

##### pavlos27 11/10/2019 10:30:11
i don't know what to do to fix that do you have any ideas?


i have reset the physics of the floor too, but still i have the same problem


it will always colliding with the floor


i see contact points with the floor but what that means?my robot can't be in the air

##### David Mansolino [cyberbotics] 11/07/2019 16:16:08
By checking the contact points you can see if it does collide with the floor. If you don't see any contact points then it means that it is not colliding: [https://cyberbotics.com/doc/guide/the-user-interface#view-menu](https://cyberbotics.com/doc/guide/the-user-interface#view-menu)

##### pavlos27t 11/07/2019 10:41:35
how can i make sure the wheels are not colliding the floor?

##### David Mansolino [cyberbotics] 11/07/2019 07:58:34
`@pavlos27t` you should make sure that the wheels are not colliding the floor when you move the robot. It is also recommended to check what is happenning with the contact points just before and just after the reset, you can see this in the view / Optional Rendering menu.

##### pavlos27t 11/06/2019 18:54:05
hello again , i resetphysics but some forces still applying after collision

##### David Mansolino [cyberbotics] 11/04/2019 13:33:39
You're welcome

##### pavlos27t 11/04/2019 13:33:23
right thanks i didn't know how to reset physics..

##### David Mansolino [cyberbotics] 11/04/2019 13:26:07
you probably want to reset the physics of the robot too indeed: wb\_supervisor\_node\_reset\_physics

##### pavlos27t 11/04/2019 13:25:22
if collision happens and forces applying should i reset the forces too?


i reset my robots like this [http://prntscr.com/ps9cqr](http://prntscr.com/ps9cqr)


the ground is flat, but there is a possiblity for the e-puck robot to collide with another e-puch robot that i have in the simulation

##### David Mansolino [cyberbotics] 11/04/2019 13:17:21
Hi `@pavlos27t`, just to make sure you are speaking about simulation right? Does the robot collide with something? is the ground flat?

##### pavlos27t 11/04/2019 13:16:17
Hello Sir, i have a strange problem: i give equal speed on left and right wheel of e-puck robot but it doesn't move linear, i print my velocities to confirm it and they are equal ,so why my robot turns?

##### Stefania Pedrazzi [cyberbotics] 11/04/2019 07:08:37
Hi `@threeal`, usually controllers are automatically started by Webots when you run the simulation if the is used by any robot node, i.e. the Robot.controller field is set to your python controller name. Does this not work for you?

##### threeal 11/02/2019 00:48:58
how can i launch python controller in webots?

##### SimonDK 10/25/2019 17:24:39
`@Flo` I sent you a PM, did you receive it? /cheers

##### Olivier Michel [cyberbotics] 10/23/2019 10:35:44
See details here: [https://cyberbotics.com/doc/guide/using-ros#standard-ros-controller](https://cyberbotics.com/doc/guide/using-ros#standard-ros-controller)


You can use the ROS controller provided in Webots with your own new robot design. There is no need to modify it. You will simply have to figure out the topics and services published by it.


See this tutorial: [https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)


Of course you can create your custom robot in Webots.

##### threeal 10/23/2019 10:31:58
`@David Mansolino` thank you for the answer, but is it possible to create a custom robot in Webots? As my robot is custom made and not based on sample robot provided by the Webots simulator.

And what about the ROS controller provided by Webots? can i use it in my custom made robot or should i create custom made controller for my robot that work as bridge between ROS and Webots?.

##### David Mansolino [cyberbotics] 10/21/2019 06:57:24
`@SimonDK` :
> Going to try Webots with ROS very soon. At least for me I have grown very tired of Gazebo. It always feels unstable, cumbersome to set up and we have to use many hacks to get it to work somewhat ok with Reinforcement Learning algorithms. I hope it will be easier to speed up simulations and build simulations with Webots. Let's see


For sur it will 😉 let us know if you have any issues doing so.


`@Flo`, that sounds very interesting, looking forward to see this too!


> anyway, is it possible to treat webots controller as ros node?


Yes of course, we have developped an interface with ROS1 which is a controller that acts as a generic bridge between Webots and ROS1: [http://wiki.ros.org/webots\_ros/](http://wiki.ros.org/webots_ros/)

And we are currently creating an interface with ROS2, for this new version we are using the Webots API directly in the ros nodes: [http://wiki.ros.org/webots\_ros2](http://wiki.ros.org/webots_ros2)

Let us know if you have any precise question


`@threeal`, Webots has several advantages compared to Gazebo, here is a non-exhaustive list:
  - Cross-platform [windows, linux, mac].
Stable physics engine.
  -An efficient rendering engine using Physically Based Rendering for realistic images.
  -A simple and intuitive user interface.
  -Wide range of simulated sensors and actuators available and ready to work.
  -Wide range of robot models available and ready to work.
  -Wide range of documented samples.

##### SimonDK 10/21/2019 04:53:16
`@Flo` sounds very interesting, will PM you

##### threeal 10/20/2019 19:11:53
anyway, is it possible to treat webots controller as ros node?

##### Flo 10/20/2019 19:09:40
`@SimonDK`  I build a gym environnement for webots. Its not released yet but let me know. If you need it for your RL I can accelerate the release

##### SimonDK 10/20/2019 18:53:29
`@threeal` Going to try Webots with ROS very soon. At least for me I have grown very tired of Gazebo. It always feels unstable, cumbersome to set up and we have to use many hacks to get it to work somewhat ok with Reinforcement Learning algorithms. I hope it will be easier to speed up simulations and build simulations with Webots. Let's see 🙂

##### threeal 10/20/2019 18:45:45
and why use webots over gazebo?


anybody ever try to use webots with ros?

##### Fabien Rohrer [Moderator] 10/17/2019 15:13:45
I'm happy to read this 😉

##### pavlos27t 10/17/2019 14:50:56
😅


sorry it doesn't crash now , everything works fine

##### Fabien Rohrer [Moderator] 10/17/2019 14:49:56
(or a screenshot ? 😉 )


Normally, when a Python controller crash, something is displayed in the Webots console. Simply copy-paste it in this chat.

##### pavlos27t 10/17/2019 14:44:38
sorry i don't know how to post a crash error

##### Fabien Rohrer [Moderator] 10/17/2019 14:44:01
Ok. Could you post the crash error here?

##### pavlos27t 10/17/2019 14:43:31
yes because it crashes on every function

##### Fabien Rohrer [Moderator] 10/17/2019 14:43:30
>>> DEF robotname Robot {}


Are you sure that "robotname" matches with an existing DEF name?


I confirm you should use Supervisor() instead of Robot() to access the Supervisor API

##### pavlos27t 10/17/2019 14:40:54
if i change robot = Robot() --> robot = Supervisor() , then webots crashes when a supervisor unction called


Hello Sir, i am trying to understand supervisor programming but i found only this example in C : [https://www.cyberbotics.com/doc/guide/supervisor-programming](https://www.cyberbotics.com/doc/guide/supervisor-programming) , could you give me something similar in python?I'm trying this on a robot controller: from controller import * ;robot = Robot();robot.getFromDef(name =  "robotname")  ---> and i get error robot object has no attribute getFromDef , but i have supervisor field :TRUE on the robot

##### David Mansolino [cyberbotics] 09/30/2019 06:40:07
We also have an example with PyCharm here: [https://www.cyberbotics.com/doc/guide/using-pycharm-with-webots](https://www.cyberbotics.com/doc/guide/using-pycharm-with-webots)

##### Fabien Rohrer [Moderator] 09/30/2019 06:35:35
`@TheDisposableScientist` Hi, I expect you mean to create a webots controller inside an IDE, such as XCode. Could you refer to this draft page of the documentation? [https://cyberbotics.com/doc/guide/using-your-ide?version=enhancement-ide-section](https://cyberbotics.com/doc/guide/using-your-ide?version=enhancement-ide-section)

##### TheDisposableScientist 09/29/2019 10:08:43
how to integrate webots with an ide in macOS?

##### Stefania Pedrazzi [cyberbotics] 09/26/2019 06:20:06
`@Anoop` The `Skin` node is a pure graphical functionality that only modifies the mesh of the robot but doesn't affect the physics.
Webots cannot simulation soft robotics because ODE (the Webots physics engine) doesn't support soft body dynamics.
However there are some tricks to transform a soft robot model into an hard model that could help in simulating soft robotics application.

##### Anoop 09/25/2019 17:45:20
Thanks `@Stefania Pedrazzi` for the information. Can I assume this as a soft robotics or more generally can SKIN node be used for generating soft robotics model?

##### Stefania Pedrazzi [cyberbotics] 09/25/2019 06:17:19
Just note that the `Skin` node used to implement the skin animation is still experimental, i.e. it can be used but it is not documentated as the functionality could still change.


Hi `@Anoop`, yes this robot is included in Webots. You can find it in the simulation `projects/samples/rendering/worlds/animated_skin.wbt`
[https://www.cyberbotics.com/doc/guide/samples-rendering#animated\_skin-wbt](https://www.cyberbotics.com/doc/guide/samples-rendering#animated_skin-wbt)

##### Anoop 09/25/2019 01:27:46
Does webots include this marine robots, exact this one? I can find salamander closest to it?
%figure
![FluidRobot.png](https://cdn.discordapp.com/attachments/565155651395780609/626228320996032512/FluidRobot.png)
%end


##### Stefania Pedrazzi [cyberbotics] 09/16/2019 06:11:06
You can find another example in samples/howto/omni\_wheels.wbt simulation:  [https://www.cyberbotics.com/doc/guide/samples-howto#omni\_wheels-wbt.](https://www.cyberbotics.com/doc/guide/samples-howto#omni_wheels-wbt.) In this case the wheel is simulated using two layers of joints and cylinders.


Hi `@SimonDK`, the youBot robot has mecanum wheels [https://www.cyberbotics.com/doc/guide/youbot](https://www.cyberbotics.com/doc/guide/youbot)
The mecanum wheel is simulated by setting asymmetric friction  to the [ContactProperties]([https://www.cyberbotics.com/doc/reference/contactproperties)](https://www.cyberbotics.com/doc/reference/contactproperties)) of the wheel.

##### SimonDK 09/15/2019 14:31:33
What models for omni-directional mecanum wheel platforms have you seen/are there in Webots? How are the mecanum wheels simulated? I would like to develop my own simulation for a platform so are looking for some examples as a starting point.

##### David Mansolino [cyberbotics] 09/10/2019 06:06:21
Do you have a particular robot in mind ?


Hi `@Derek`, I would recommend to follow our tutorial, in particular this one will explain how to add 4 joints (connected to 4 wheels) to a robot: [https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)

##### Derek 09/10/2019 01:23:04
How can I add some joints in a robot by GUI operation?

##### MerySgh 08/21/2019 11:36:36
`@Fabien Rohrer` No thank you so much 🙂

##### Fabien Rohrer [Moderator] 08/21/2019 11:31:32
`@BlackPearl` Sorry to read this, the next step could be to test if the Java examples provided in Webots are working smoothly or not. Could you simply open WEBOTS\_HOME/projects/languages/java/worlds/example.wbt ?


`@MerySgh` Ok, I understand more and more your scenario. In my knowledge, I'm not aware of similar projects in the Webots community. Do you have specific questions?

##### BlackPearl 08/21/2019 10:42:31
It still doesn’t work `@fa`

##### MerySgh 08/21/2019 10:39:32
ok i'm going to see it .. i think i'm not clear enough ... ok the car made an accident with pedestrian because it's attacked from different attacks 
thanks to this attack that the car does not see the passenger crossing the road in my case the attack is jamming

##### Fabien Rohrer [Moderator] 08/21/2019 10:30:22
Could such traffic simulation meet your expectations?


Please take a look at this link and the movie: [https://cyberbotics.com/doc/automobile/sumo-interface](https://cyberbotics.com/doc/automobile/sumo-interface)


In Webots, you can simulate vehicle traffic using SUMO (it's an open-source application which simulates vehicle traffic and which can be interfaced with Webots):


If I understand well, you need to simulate a traffic jam after the collision, right?

##### MerySgh 08/21/2019 10:26:29
sumo ? what do  you mean ?

##### BlackPearl 08/21/2019 10:24:53
`@Fabien Rohrer` one minute. We are restarting everything again

##### Fabien Rohrer [Moderator] 08/21/2019 10:23:32
`@MerySgh` are you using SUMO to manage the traffic?


`@BlackPearl` so you did downgraded to java and javac 1.8 and clean/build your controller again?

##### MerySgh 08/21/2019 10:21:55
I  made the accident but how to develop the attack I do not know yet

##### Fabien Rohrer [Moderator] 08/21/2019 10:21:46
`@MerySgh` haha makes more sense

##### BlackPearl 08/21/2019 10:21:14
`@Fabien Rohrer` it’s still not working ....

##### MerySgh 08/21/2019 10:20:57
mmm actually !! I'm supposed to develop  jamming attack in my project ... for more clarifying I have a project of autonomous vehicles and I have to make an accident with a passenger in the road according to jamming attack

##### Fabien Rohrer [Moderator] 08/21/2019 10:16:07
Do you have something else in mind?


Hi, Webots is mainly a Desktop app. A jamming attack has not much sense to me. Do you speak about the streaming sever? Our websites?

##### MerySgh 08/21/2019 10:13:23
hello everyone !  has anyone have developed jamming attack in webot  before ?

##### Fabien Rohrer [Moderator] 08/21/2019 10:10:23
(it's probably the simplest solution indeed)

##### BlackPearl 08/21/2019 10:10:07
Ok thank you very much. We are downgrading

##### Fabien Rohrer [Moderator] 08/21/2019 10:09:36
Let me know if you have troubles with this.

##### BlackPearl 08/21/2019 10:08:38
Hm ok


*ubuntu


Unbuntu

##### Fabien Rohrer [Moderator] 08/21/2019 10:05:45
You should either downgrade your Java, or either recompile the library with your Java version (normally it's quite simple, just a matter to install `swig`, define `WEBOTS_HOME` and type `make` in `WEBOTS_HOME/resources/languages/java`)


What OS are you using?  arch?


We are precompiling the Java library with OpenJDK 1.8.0\_222.


ok, this is certainly the issue


not sure. could you give us more precisely your Java version (`java -version`)? It is supposed to be `1.8`. Does it matches with your `javac -version`?

##### BlackPearl 08/21/2019 09:53:19
Hm so why the error accurs

##### Fabien Rohrer [Moderator] 08/21/2019 09:52:20
on linux, OpenJDK is supposed to work,

##### BlackPearl 08/21/2019 09:52:17
But yes we are using the openJDK


No we are running Linux

##### Fabien Rohrer [Moderator] 08/21/2019 09:51:44
oops, I think I'm wrong: I was sure you were on windows.

##### BlackPearl 08/21/2019 09:51:07
We Will change to the oracle jdk


`@Fabien Rohrer`  oh ok thanks for the quick answer

##### Fabien Rohrer [Moderator] 08/21/2019 09:49:08
(the java libcontroller is precompiled with the last 8 version of the Oracle JDK)


How did you installed Java? Your log seems to refer to OpenJDK while we recommend to use the Oracle JDK: [https://cyberbotics.com/doc/guide/using-java#windows](https://cyberbotics.com/doc/guide/using-java#windows)

##### BlackPearl 08/21/2019 09:45:02
It’s a java controller


Anything we need to pay attention?


`@David Mansolino`  so we are using the new version but we got the controller crash again

##### Deleted User 07/20/2019 10:21:57
`@BlackPearl` hello sir, are you using nao ?

##### BlackPearl 06/28/2019 05:00:03
`@David Mansolino`  thanks, will try

##### David Mansolino [cyberbotics] 06/26/2019 06:15:07
`@BlackPearl` can you please try with the latest version of Webots that we releeased yesterday: [https://github.com/omichel/webots/releases/latest](https://github.com/omichel/webots/releases/latest)

##### BlackPearl 06/21/2019 04:11:49
The latest (java 8 and webots R2019-R1)

##### David Mansolino [cyberbotics] 06/20/2019 14:05:24
`@BlackPearl` which version of Webots and which version of Java are you using ?

##### BlackPearl 06/20/2019 13:54:38
That’s why we created our own but still get this error


Yes

##### David Mansolino [cyberbotics] 06/20/2019 06:54:45
`@BlackPearl` do you get this error with the controllers distributed within Webots?

##### BlackPearl 06/19/2019 18:15:12
Actually we tried today different controllers but always getting this error

##### Fabien Rohrer [Moderator] 06/19/2019 08:54:04
Do you have any way to debug your controller?


A priori, this bug is on your side.


It seems your controller called SensorController crashes.


Hi

##### BlackPearl 06/19/2019 08:50:43
I think we found a Bug?
%figure
![image0.jpg](https://cdn.discordapp.com/attachments/565155651395780609/590825780716634132/image0.jpg)
%end


##### David Mansolino [cyberbotics] 06/12/2019 15:52:00
Thank you

##### TH0 06/12/2019 15:51:23
ok, good luck!

##### David Mansolino [cyberbotics] 06/12/2019 15:50:20
Ok, then everything is perfect with your hardware. I will check when I have time and let you know

##### TH0 06/12/2019 15:49:17
OpenGL 4.6 (415 ext) - Quadro P3200/PCIe/SSE2


yes other apps work fine and i use the vive often

##### David Mansolino [cyberbotics] 06/12/2019 15:41:28
We haven't tested it with this GPU, but it should works fine.
Do you know which OpenGL version you are using?
Are other application working fine with the Vive headset?


> 0 means infinity?


for far yes

##### TH0 06/12/2019 15:37:35
P3200 mobile


0 means infinity?

##### David Mansolino [cyberbotics] 06/12/2019 15:37:13
By the way what is your GPU ?


Ok, then yes it doesn't seems it is clipping the problem, but rather an issue with OpenGL


(value are in meters)

##### TH0 06/12/2019 15:36:44
same result with 100

##### David Mansolino [cyberbotics] 06/12/2019 15:36:30
Can you try increasing it to something like 100 ?

##### TH0 06/12/2019 15:36:24
that means for me, clipping is not the problem, right?


see difference far 0 (default) vs. far 1

##### David Mansolino [cyberbotics] 06/12/2019 15:32:14
> the good news: you can simulate the behaviour without a vive, right?


Kind of, but still the vive API in itself interfere with OpenGL and this is maybe the cause of the problem.
But as soon as I have time I will try to reproduve this.


Can you try playing with the Viewpoint, near and far value ?

##### TH0 06/12/2019 15:31:10
the good news: you can simulate the behaviour without a vive, right?

##### David Mansolino [cyberbotics] 06/12/2019 15:30:44
> sorry, english is not my mother language, i thought you mean the vive handles (controllers)


No problem that was not clear from my side too.

##### TH0 06/12/2019 15:30:42
i think so


yes

##### David Mansolino [cyberbotics] 06/12/2019 15:30:31
Ok, than this make more sense, so the problem is really what is rendered in Webots

##### TH0 06/12/2019 15:30:26
sorry, english is not my mother language, i thought you mean the vive handles (controllers)

##### David Mansolino [cyberbotics] 06/12/2019 15:30:04
> i do not see the rendered vive controllers in both the webots preview and in the vive


that's normal we did not yet interfaced the vive controller with webots.

##### TH0 06/12/2019 15:30:03
yes, i also see them in the vive (its exacly the same image)

##### David Mansolino [cyberbotics] 06/12/2019 15:29:38
No sorry, I mean the red,green and blue arrows you can see in Webots.

##### TH0 06/12/2019 15:29:37
i do not see the rendered vive controllers in both the webots preview and in the vive


just to be sure: you mean with "handles" the two controllers from the vive?

##### David Mansolino [cyberbotics] 06/12/2019 15:27:56
Ok, so you can see the handle in the preview but not in the headset ?

##### TH0 06/12/2019 15:27:23
the vive preview in webots and what i see in the vive is the same

##### David Mansolino [cyberbotics] 06/12/2019 15:23:28
that's very strange, you should see exactly the same in the Webots view and in one of the two eyes in the headset. Probably there is something not clean with OpenGL on our side.

##### TH0 06/12/2019 15:13:10
oh, and i can rotate the cameraview per mouse while vr is on


(but i set both options to true in the webots gui)


and the position is not tracked, but the rotation from the vive


no handles

##### David Mansolino [cyberbotics] 06/12/2019 15:10:35
Can you see the handle in the headset too?

##### TH0 06/12/2019 15:07:26
interessting: when i click on some objects in the treeview, the coordinate systems are shown in the vive


set to 50%, same issue

##### David Mansolino [cyberbotics] 06/12/2019 15:02:05
Yes, maybe near-clip, or maybe a resolution issue. can you try setting the VIve resolution to 50% (you can set this in the preference of SteamVR). and then reboot Webots.

##### TH0 06/12/2019 15:01:42
exact

##### David Mansolino [cyberbotics] 06/12/2019 15:01:22
At least if shows that the connexion with the Vive is correctly established

##### TH0 06/12/2019 14:55:26
maybe a near-clipping problem because of different scales?


ok. the color shown in the vive is the same as the "/Background/SkyColor" (i changed it to red to test it)

##### David Mansolino [cyberbotics] 06/12/2019 14:53:18
Then if you are using the 'develop' branch you are indeed very close to the R2019b version 😕

##### TH0 06/12/2019 14:51:43
world file was "nao\_robocup.wbt"


i just tested the vive in the 2019b, same problem: colored background but no objects


on the develop

##### David Mansolino [cyberbotics] 06/12/2019 14:48:16
It depends on which git branch you are ?

##### TH0 06/12/2019 14:47:19
i'm not sure, but isn't the version i just build from webots the 2019b you mean?


ok, thats the one i used too

##### David Mansolino [cyberbotics] 06/12/2019 14:42:11
Many of them, but I remember I tested I lot the 'robotis/soccer.wbt' world

##### TH0 06/12/2019 14:41:36
whats the world you tested the VR?

##### David Mansolino [cyberbotics] 06/12/2019 14:41:33
That's indeed not normal, but we changed some part of the VRHeadset code for the next release, so please try again.

##### TH0 06/12/2019 14:40:54
only a light blue/grey background color


when i move the headset, nothing happens


ok

##### David Mansolino [cyberbotics] 06/12/2019 14:39:55
Not yet, but we will release R2019b soon. Unfortunately I don't have the time right now, but if you test with R2019b when we will release it and the problem is still present, please do not hesitate to tell us.

##### TH0 06/12/2019 14:37:41
i'm using the webots 2019a Rev 1 from some weeks ago. is there a newer version?

##### David Mansolino [cyberbotics] 06/12/2019 14:37:02
Where you able to view the simulation in Webots moving when you were moving the headset ?


That's strange we did test it a few weeks ago and it was working fine. Unfortunately we don't have any VIve available right now to test.

##### TH0 06/12/2019 14:35:07
i think the vr-function in webots is broken. i tried to view a simulation with the htc vive and steam vr but only a blue background appers in the view. when i deactivate position and rotation tracking in the webot gui, a coordinate system (webots style) is displayed in the vive in the center of the display, but no other objects from the scene)


time for another question? 😉

##### David Mansolino [cyberbotics] 06/12/2019 14:31:54
Perfect !

##### TH0 06/12/2019 14:31:48
make is completed and was successful!


btw.: great that webots is open source now!

##### David Mansolino [cyberbotics] 06/12/2019 14:26:49
Perfect, we will fix this in our repo !

##### TH0 06/12/2019 14:26:47
look good so far


👌


😄

##### David Mansolino [cyberbotics] 06/12/2019 14:20:22
Yes, indeed, can you try changing in 'src/webots/Makefile:
OPEN\_VR\_INCLUDE = -isystem $(WEBOTS\_PATH)/include/openvr
Into
OPEN\_VR\_INCLUDE = -isystem $(WEBOTS\_PATH)/include/openvr/headers
and try to recompile ?

##### TH0 06/12/2019 14:19:35
seems like an error in the makefile (missing include directory?)


right

##### David Mansolino [cyberbotics] 06/12/2019 14:18:54
Ok, no problem, then it seems they are correctly downloaded and then copied in the 'include' fodler

##### TH0 06/12/2019 14:17:58
sorry for the confusion with the find command, i updated the message from above


yes, its there

##### David Mansolino [cyberbotics] 06/12/2019 14:15:59
From the log you sent it seems they are there, can you confirm ?


The missing headers should be located in '/C/msys64/home/Thomas/webots/openvr-1.0.7/headers' after the make -j6

##### TH0 06/12/2019 14:13:30
[https://0bin.net/paste/DeUwrdi11b19n91y#XDrDCDC3pOVgGQ3a1vqIJNM6k7hJ0j1e1qgjBpNhMQh](https://0bin.net/paste/DeUwrdi11b19n91y#XDrDCDC3pOVgGQ3a1vqIJNM6k7hJ0j1e1qgjBpNhMQh)


first of all:
Thomas@DESKTOP-O5VO47I MSYS /C/msys64/home/Thomas/webots
$ find . -name openvr.h
./dependencies/openvr-1.0.7/headers/openvr.h
./include/openvr/headers/openvr.h

Thomas@DESKTOP-O5VO47I MSYS /C/msys64/home/Thomas/webots
$

##### David Mansolino [cyberbotics] 06/12/2019 14:08:42
what is the output of your make -j6 command ?


It should not impact openvr, but it is worse trying

##### TH0 06/12/2019 14:07:33
ok, but i think, its not the solution for the openvr problem, right?

##### David Mansolino [cyberbotics] 06/12/2019 14:07:10
if you want to have npn install (and some other optionnal dependencies) you can install them by running './src/install\_scripts/msys64\_installer.sh --dev' from the webots directory


Npn is completely optionnal


python is higly recommended (even so it is not required to compile the core of Webots).

##### TH0 06/12/2019 14:05:53
ok, how ever, is python and npm needed or only optional?

##### David Mansolino [cyberbotics] 06/12/2019 14:04:52
Not really, but you can send gist link: [https://gist.github.com/](https://gist.github.com/)

##### TH0 06/12/2019 14:04:45
like a block or something like that


is there some trick in discord to send code in a more compact way in the chat? 😄


ok, same error after make -j6

##### David Mansolino [cyberbotics] 06/12/2019 14:00:51
if they was no error its not needed to send the output


Ok, can you retry to do a 'make'

##### TH0 06/12/2019 14:00:30
should i send the output in the chat?


done

##### David Mansolino [cyberbotics] 06/12/2019 13:59:40
(this will clean the 'dependencies' folder so that you ca retry from a clean install)


Ok, let's first try something simple, can you do a 'make cleanse' from the 'webots' folder?

##### TH0 06/12/2019 13:59:09
and there where no errors until the one i pasted here


yes

##### David Mansolino [cyberbotics] 06/12/2019 13:58:39
Just to make sure, you did follow all the instructions step by step from [https://github.com/omichel/webots/wiki/Windows-installation](https://github.com/omichel/webots/wiki/Windows-installation) right ?


Ok, perfect, you have it the includes are located in the 'openvr-1.0.7' directories.

##### TH0 06/12/2019 13:57:29
$ ls dependencies/
Cyberbotics.Webots.Mingw64.Libraries.manifest  libpico.zip  lua-gd-windows.zip  Makefile.mac      openvr-1.0.7
libOIS.zip                                     lua-5.2.3    Makefile.linux      Makefile.windows

##### David Mansolino [cyberbotics] 06/12/2019 13:55:04
this dependency should be downloaded automatically. Can you check the content of your 'dependencies' folder ?

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

##### Fabien Rohrer [Moderator] 06/04/2019 07:56:42
But first of all, please do our tutorial 😉 [https://cyberbotics.com/doc/guide/tutorials](https://cyberbotics.com/doc/guide/tutorials)

##### Arsalan.b.r 06/04/2019 07:54:22
thanks again


yes

##### Fabien Rohrer [Moderator] 06/04/2019 07:53:17
The trucks trailer are probably closer from your expectations.


[https://www.cyberbotics.com/doc/automobile/car#heavy-weights](https://www.cyberbotics.com/doc/automobile/car#heavy-weights)


This other example may interest you:


Yes, but adding wheels to the yamor module is not an issue

##### Arsalan.b.r 06/04/2019 07:51:01
do you have a picture of that?


yamor is like snake?


like tractor and trailer

##### Fabien Rohrer [Moderator] 06/04/2019 07:50:29
I think you could achieve something by studying yamor.wbt

##### Arsalan.b.r 06/04/2019 07:50:21
it should be wheeled

##### Fabien Rohrer [Moderator] 06/04/2019 07:49:26
[https://cyberbotics.com/doc/reference/connector](https://cyberbotics.com/doc/reference/connector)


The robot model is encapsulated in a PROTO node. The Robots are linked with Connector nodes:


[https://cyberbotics.com/doc/guide/yamor](https://cyberbotics.com/doc/guide/yamor)


I think the closest robot we have in Webots yet is the yamor:

##### Arsalan.b.r 06/04/2019 07:47:13
a passive joint between carts

##### Fabien Rohrer [Moderator] 06/04/2019 07:46:39
Do you have a screenshot of a real setup?


Is it the same robot replicated several time? How is the link between the robots? A HingeJoint?


First of all it's possible to do so. There are even several possibilities to achieve this.

##### Arsalan.b.r 06/04/2019 07:44:25
is there such a robot in webots?


how can i do it simply?


its a chained combination of wheeled robots


i want to model a Tractor trailer mobile robot

##### Fabien Rohrer [Moderator] 06/04/2019 07:42:51
Hi again ^^

##### Arsalan.b.r 06/04/2019 07:42:35
hello

##### Olivier Michel [cyberbotics] 04/25/2019 11:22:48
Eventually, you will be able to open a pull request to merge your contribution.


I would propose you to open an issue on [https://github.com/omichel/webots/issues](https://github.com/omichel/webots/issues) to follow-up with this discussion.


Hi, welcome.

##### Thelm76 04/25/2019 10:40:32
also, on the wiki, I've seen that the accelerometer and the gyro are encoded on 6 bytes so 2 bytes for X, Y and Z. But the magnetometer is encoded on 12 bytes, which doesn't make sense to me as the raw values are between -32460 and 32760... So are there 3 axis encoded on 4 bytes, or am I missing something?


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

