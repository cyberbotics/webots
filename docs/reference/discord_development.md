# Development

This is an archive of the `development` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m).

## 2020

##### David Mansolino 05/15/2020 04:57:11
`@Sanket Khadse` the controller of the drone is very simple, it doesn't do any feedback using the GPS position or any inertial unit, it might therefore easily drift.

##### David Mansolino 05/15/2020 04:55:43
`@Jesusmd` you want to get the type of distance sensor? If so you should use the ``getType`` function: https://cyberbotics.com/doc/reference/distancesensor?tab-language=python#wb_distance_sensor_get_type

##### Sanket Khadse 05/15/2020 03:02:49
Hey, may I know, why does a drone, (let it be DJI Mavic 2 Pro, which is already available into Webots, or a custom made) moves up and down and shifts constantly in one direction even if no such behaviour is defined in the controller code?

##### Jesusmd 05/15/2020 01:49:44
`@David Mansolino` Hi, I am using python, I would like to work with several distance sensors at the same time and comparing their data in console. But instead of obtain the type, I got a number.

##### Luftwaffel 05/14/2020 12:48:46
> `@Luftwaffel` Although working with ROS, we decided to split the simulation projects with minimal dependencies over ROS other than communication. For 3D Math in Cpp, we're using Eigen, and in python either numpy or numpy + transformations.py (which is standalone of tf)
`@Axel M` 

Awesome, thank you so much. That makes things easier

##### Axel M 05/14/2020 09:11:01
The integration with ROS is super easy too with http://wiki.ros.org/eigen_conversions 🙂

##### Axel M 05/14/2020 09:09:52
Regarding your example of getting relative position between two nodes, that can be easily achieved with Eigen in cpp (3x3 Matrix -> Quaternion, quaternion / vector product)

##### Axel M 05/14/2020 09:08:34
`@Luftwaffel` Although working with ROS, we decided to split the simulation projects with minimal dependencies over ROS other than communication. For 3D Math in Cpp, we're using Eigen, and in python either numpy or numpy + transformations.py (which is standalone of tf)

##### David Mansolino 05/14/2020 05:34:23
`@Luftwaffel` instead of relying on ROS, if you are using Python you ca probably use the `transforms3d` python package which allows for example to convert from a rotation matrix to quaternions: https://matthew-brett.github.io/transforms3d/reference/transforms3d.quaternions.html#transforms3d.quaternions.mat2quat

##### Luftwaffel 05/13/2020 18:04:16
`@Sanket Khadse`  perhaps direct .proto file edit can help

##### Sanket Khadse 05/13/2020 17:11:36
`@Luftwaffel` that is what my problem is about. The nodes I have to copy paste one by one are in "hundreds".

##### Luftwaffel 05/13/2020 16:47:12
Perhaps add a 'Group' base node, put all your nodes in, and copy paste that

##### Sanket Khadse 05/13/2020 16:45:26
Oh, thank you for letting me know!
Keep it as a suggestion for the next update. 😄

##### Olivier Michel 05/13/2020 16:44:11
Unfortunately, this is not possible.

##### Sanket Khadse 05/13/2020 16:43:42
Hey, could you tell me how to multi-select things in scene-tree? 
I can't find a way to. I imported a VRML97 model into Webots, the model was quite large, and it's difficult to select, cut and paste each 'transform' object into a robot node's children attribute.

##### Luftwaffel 05/13/2020 16:21:05
I'll give it a try

##### Luftwaffel 05/13/2020 16:20:54
oh lordy 😅

##### Olivier Michel 05/13/2020 16:20:18
Yes, if you have an idea to achieve this... Now you know how to open a pull request 😉

##### Luftwaffel 05/13/2020 16:19:08
yes, but it requires runtime.ini edits. Would be nice if people can have it run as an example out of the box

##### Olivier Michel 05/13/2020 16:18:13
If you use Webots with ROS, you need to install ROS, and you should get 'tf' for free, isn't it?

