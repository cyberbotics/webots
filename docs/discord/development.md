# Development

This is an archive of the `development` channel of the [Webots Discord server](https://discordapp.com/invite/nTWbN9m).

## 2020

##### Simon Steinmann [Moderator] 08/18/2020 19:35:45
but we will have to check and recompile several official robot arms, as they dont include the last link, when converting to urdf. Good idea anyways, using multi-file, so viewing and editing PROTO-source is less laggy

##### David Mansolino [cyberbotics] 08/18/2020 19:30:11
> it works super well now. you just have to assignt the controller and do nothing else. If there is no target node, it adds it for you. Even tells you to enable supervisor, if not done so already

`@Simon Steinmann` looks very nice 🙂

##### Simon Steinmann [Moderator] 08/18/2020 19:30:08
that's cleaner

##### David Mansolino [cyberbotics] 08/18/2020 19:29:52
> that returns the worldpath including the filename though, that's why I needed the next 2 lines to remove it

`@Simon Steinmann` a simplest solution would be:

```Python
worldPath = os.path.dirname(supervisor.getWorldPath())
```

##### Simon Steinmann [Moderator] 08/18/2020 19:28:29
it works super well now. you just have to assignt the controller and do nothing else. If there is no target node, it adds it for you. Even tells you to enable supervisor, if not done so already

##### David Mansolino [cyberbotics] 08/18/2020 19:27:23
> what I used 😉

`@Simon Steinmann` just saw it (I was reading messages from top to bottom)

##### Simon Steinmann [Moderator] 08/18/2020 19:27:17
that returns the worldpath including the filename though, that's why I needed the next 2 lines to remove it


what I used 😉

##### David Mansolino [cyberbotics] 08/18/2020 19:25:45
> Can you try to getenv and print WEBOTS\_PROJECT environment variable? Then append "/worlds" to it should do the trick.

`@Olivier Michel` `@Simon Steinmann`  we have an API function for this: Robot.getWorldPath()

##### Simon Steinmann [Moderator] 08/18/2020 18:22:51

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/745346993257185430/unknown.png)
%end


btw, figured out the world path problem


figured it out, have to parse it. Can't init supervisor twice


meaning, do I have to parse the supervisor for the module-class \_\_init\_\_ function or can it just import it directly


is there a conflict, if I inialize a supervisor in my main controler, and also in a imported module?


I think I'll just remove the texture and make the sphere yellow


shucks

##### Olivier Michel [cyberbotics] 08/18/2020 16:17:17
Unfortunately, I don't see any better way to do this...

##### Simon Steinmann [Moderator] 08/18/2020 16:14:49
well, using it for the generic controller 😉

##### Olivier Michel [cyberbotics] 08/18/2020 16:14:29
Otherwise, you can assume it's `../../worlds/` relative to your controller.

##### Simon Steinmann [Moderator] 08/18/2020 16:13:36
it's not a system variable

##### Olivier Michel [cyberbotics] 08/18/2020 16:13:24
No, that won't work...


Can you try to getenv and print WEBOTS\_PROJECT environment variable? Then append "/worlds" to it should do the trick.


There must be a project path environment variable... Let me search...

##### Simon Steinmann [Moderator] 08/18/2020 16:07:39
how can I get the path to the world?

##### Olivier Michel [cyberbotics] 08/18/2020 16:06:54
It's better to copy them in the worlds folder under a textures subfolder.

##### Simon Steinmann [Moderator] 08/18/2020 16:06:15
can I link textures relative to the controller? or would it be easier to copy the files into the world folder?


okay works beautifully


just saw, thx 🙂

##### Olivier Michel [cyberbotics] 08/18/2020 15:58:50
`children.getCount()`


Get the root node, its children field and get the number of items (see my code sample)

##### Simon Steinmann [Moderator] 08/18/2020 15:57:23
how can I get the number of children? I wanna insert the target node on the bottom


gonna try 🙂

##### Olivier Michel [cyberbotics] 08/18/2020 15:46:37
Yes.

##### Simon Steinmann [Moderator] 08/18/2020 15:46:14
as a string I can just use the world file part of the node then?

##### Olivier Michel [cyberbotics] 08/18/2020 15:44:20
Probably with `importSFNodeFromString`: [https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_field\_import\_sf\_node\_from\_string](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_import_sf_node_from_string)

##### Simon Steinmann [Moderator] 08/18/2020 15:42:33
how would I spawn that target sphere?


finally done 😄

##### Olivier Michel [cyberbotics] 08/18/2020 15:26:02
Simply commit the previous version of the file.

##### Simon Steinmann [Moderator] 08/18/2020 15:25:00
how do I revert commits?


[https://github.com/cyberbotics/community-projects/pull/12](https://github.com/cyberbotics/community-projects/pull/12)

##### Olivier Michel [cyberbotics] 08/18/2020 14:28:18
Then, you will be able to upload your files and delete the dummy.txt file.

##### Simon Steinmann [Moderator] 08/18/2020 14:28:06
that seems to work

##### Olivier Michel [cyberbotics] 08/18/2020 14:27:53
Enter "default/controllers/generic\_inverse\_kinematics/dummy.txt"


Click "Add file" / "Create new files".


You should create a temporary dummy file.


Yes, I see.

##### Simon Steinmann [Moderator] 08/18/2020 14:26:19
Something went really wrong, and we can’t process that file.


doesnt let me


still something wrong

##### Olivier Michel [cyberbotics] 08/18/2020 14:25:47
4. Upload your two python files.


3. On this branch, click "Add files" popup menu.


2. Click on the "master" pop-up menu and create a branch (give it a name)


1. Go on the "Code" tab.


I see. So from GitHub web page, you should be able to create a branch and upload your controller files on this branch (creating the folder structure at the same time).

##### Simon Steinmann [Moderator] 08/18/2020 14:23:10
I dont wanna pull, because I have many working changes, guess I could clone again for upload

##### Olivier Michel [cyberbotics] 08/18/2020 14:22:43
Did you checked out locally the community-projects repo?

##### Simon Steinmann [Moderator] 08/18/2020 14:22:21
you guys do it 😉
> **Attachment**: [default.zip](https://cdn.discordapp.com/attachments/565155651395780609/745286472424489010/default.zip)


usually I can upload and crerate a new branch from those files

##### Olivier Michel [cyberbotics] 08/18/2020 14:21:26
You need to create a new branch


(it may be optional, triggered by a controllerArg)

##### Simon Steinmann [Moderator] 08/18/2020 14:20:50
i can't upload to github through the webpage 😦

##### Olivier Michel [cyberbotics] 08/18/2020 14:20:29
Yes, that could be a nice demo.

##### Simon Steinmann [Moderator] 08/18/2020 14:19:43
perhaps that target sphere can be generated by the controller. that would make the implementation even simpler

##### Olivier Michel [cyberbotics] 08/18/2020 14:17:59
OK, so let's go for the solution proposed by `@David Mansolino`.

##### David Mansolino [cyberbotics] 08/18/2020 14:17:18
And as we already have a n'inverse\_kinematic' controller in Webots looks safer to avoid duplicating it.

##### Simon Steinmann [Moderator] 08/18/2020 14:17:05
but up to you guys


I like consistency

##### David Mansolino [cyberbotics] 08/18/2020 14:16:27
That's fine too, I was suggesting 'default' to be like in Webots ([https://github.com/cyberbotics/webots/tree/master/projects](https://github.com/cyberbotics/webots/tree/master/projects)), but it is not mandatory to be exactly the same.

##### Olivier Michel [cyberbotics] 08/18/2020 14:15:37
OK, in that case, I would prefer something like `generic/controllers/inverse_kinematic`

##### Simon Steinmann [Moderator] 08/18/2020 14:15:27
putting controller in the base directory, removes everything


that was it

##### David Mansolino [cyberbotics] 08/18/2020 14:14:16
You should rather put it in somethig like 'default/controllers/generic\_inverse\_kinematic'

##### Simon Steinmann [Moderator] 08/18/2020 14:14:10
wait, it doesnt find anything in the extra path now

##### David Mansolino [cyberbotics] 08/18/2020 14:13:38
> Like [https://github.com/cyberbotics/community-projects/tree/master/controllers/generic\_inverse\_kinematic/](https://github.com/cyberbotics/community-projects/tree/master/controllers/generic_inverse_kinematic/)

`@Olivier Michel` this does not respect the Webots project hierarchy

##### Simon Steinmann [Moderator] 08/18/2020 14:11:58
does not find the controller

##### Olivier Michel [cyberbotics] 08/18/2020 14:09:02
Feel free to open a PR to do so.


Sure, you need to create it.

##### Simon Steinmann [Moderator] 08/18/2020 14:08:46
404

##### Olivier Michel [cyberbotics] 08/18/2020 14:08:31
Like [https://github.com/cyberbotics/community-projects/tree/master/controllers/generic\_inverse\_kinematic/](https://github.com/cyberbotics/community-projects/tree/master/controllers/generic_inverse_kinematic/)

##### Simon Steinmann [Moderator] 08/18/2020 14:08:13
that would be good

##### Olivier Michel [cyberbotics] 08/18/2020 14:07:55
Probably we should create a `controllers` folder at the level and put it in there.

##### Simon Steinmann [Moderator] 08/18/2020 14:07:00
new\_lower = armChain.links[i].bounds[0] + 0.0000001

        new\_upper = armChain.links[i].bounds[1] - 0.0000001       

        armChain.links[i].bounds = (new\_lower, new\_upper)

this should be enough, since lower is always smaller than upper


where would that folder be? I added the whole comm.proj. git to extra paths

##### Olivier Michel [cyberbotics] 08/18/2020 14:05:20
Yes, there might be rounding problems with small values.

##### Simon Steinmann [Moderator] 08/18/2020 14:04:50
yeah

##### Olivier Michel [cyberbotics] 08/18/2020 14:04:50
Yes, I would recommend you to add such a controller in a folder named "generic\_inverse\_kinematic" or the like.

##### Simon Steinmann [Moderator] 08/18/2020 14:04:48
because of 0


ohhhhh


i'm not dividing, so there should be no issues?!

##### Olivier Michel [cyberbotics] 08/18/2020 14:03:54
```python
        new_lower = armChain.links[i].bounds[0] - 0.0000001 if armChain.links[i].bounds[0] > 0 else armChain.links[i].bounds[0] + 0.0000001
        new_upper = armChain.links[i].bounds[1] - 0.0000001 if armChain.links[i].bounds[1] > 0 else armChain.links[i].bounds[1] + 0.0000001
        armChain.links[i].bounds = (new_lower, new_upper)
```

##### Simon Steinmann [Moderator] 08/18/2020 14:03:49
hmm, so my IK controller now works flawlessly with all the robots (at least kuka) in the community projects. Is it possible to add a controller on a higher abstraction level and not a per-world basis?

##### Olivier Michel [cyberbotics] 08/18/2020 14:01:40
It's maybe less dangerous to do:

##### Simon Steinmann [Moderator] 08/18/2020 14:00:56
problem fixed :p still makes me smile


😄


\# ikpy includes the bounds as valid, In Webots they have to be less than the limit

        new\_lower = armChain.links[i].bounds[0] * 0.9999999

        new\_upper = armChain.links[i].bounds[1] * 0.9999999        

        armChain.links[i].bounds = (new\_lower, new\_upper)


exactly 🙂

##### Olivier Michel [cyberbotics] 08/18/2020 13:57:28
Yes, something like `wb_supervisor_node_get_from_proto_root`.

##### Simon Steinmann [Moderator] 08/18/2020 13:55:14
requireing no argument would be good, as we arleady specify which node.

##### Olivier Michel [cyberbotics] 08/18/2020 13:54:19
I know. Then we probably need another API function to get the root proto node even if it has no DEF...

##### Simon Steinmann [Moderator] 08/18/2020 13:53:27
the screenshot above is your model 😉


trying to make it work for general and existing protos

##### Olivier Michel [cyberbotics] 08/18/2020 13:49:48
You should add one. Or use one if any at a lower level.

##### Simon Steinmann [Moderator] 08/18/2020 13:49:04

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/745278094939783168/unknown.png)
%end


there is no DEF in most proto files

##### Olivier Michel [cyberbotics] 08/18/2020 13:47:55
```
PROTO MyRobot [ ... ] { DEF MY_ROBOT_ROOT Robot { ... } }
```


And in your `MyRobot.proto` file:


```
DEF MY_PROTO_ROBOT MyRobot { ... }
```


No, it's not the same. In your world file, you will have:

##### Simon Steinmann [Moderator] 08/18/2020 13:43:59
would that not be the same as MY\_PROTO\_ROBOT?


oh so the DEF of the robot with the supervisor controller?

##### Olivier Michel [cyberbotics] 08/18/2020 13:40:13
The one that contains the node you are looking for.


It's the DEF node inside your proto definition.

##### Simon Steinmann [Moderator] 08/18/2020 13:38:29
what is MY\_ROBOT\_ROOT?

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

##### Simon Steinmann [Moderator] 08/18/2020 13:33:14
can you paste it again?

##### Olivier Michel [cyberbotics] 08/18/2020 13:33:01
No. It requires R2020b.

##### Simon Steinmann [Moderator] 08/18/2020 13:32:42
is it compatible with 2020a? I'm not running it, but would be nice for other people to be able to use it

##### Olivier Michel [cyberbotics] 08/18/2020 13:31:59
I just fixed a couple of errors in my code snippet (it should be slightly more correct now).

##### Simon Steinmann [Moderator] 08/18/2020 13:31:04
added a fix for singularities, and generating the active link mask is now independant of the naming scheme

##### David Mansolino [cyberbotics] 08/18/2020 13:30:32
🙂

##### Simon Steinmann [Moderator] 08/18/2020 13:26:42
omg it works!!!


[https://tenor.com/view/yay-market-yellow-man-gif-6195464](https://tenor.com/view/yay-market-yellow-man-gif-6195464)

##### David Mansolino [cyberbotics] 08/18/2020 13:20:11
Yes: [https://cyberbotics.com/doc/reference/motor#wb\_motor\_get\_position\_sensor](https://cyberbotics.com/doc/reference/motor#wb_motor_get_position_sensor)

##### Simon Steinmann [Moderator] 08/18/2020 13:19:04
is there a way to get the motor name, given the sensorname?


...testing with irb... the whole calling the joints after the sensors is really screwing with the ik controller here, because it is not using the convention of the sensor being named 'motorname\_sensor'


perhaps adding it to the gui would be good. Right click robot - export - as urdf


that urdf export function is awesome!


full world, easier to test 🙂



> **Attachment**: [lbr\_iiwa\_IK.zip](https://cdn.discordapp.com/attachments/565155651395780609/745260355437199380/lbr_iiwa_IK.zip)


only requires the robot to be supervisor, and a target node, where it gets position and orientation


can take it for a spin
> **Attachment**: [webots\_IK\_controller2.py](https://cdn.discordapp.com/attachments/565155651395780609/745258824138489926/webots_IK_controller2.py)


finished with the universal IK controller


😄


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


oh man, ikpy uses >= for limits, webots uses just >

##### David Mansolino [cyberbotics] 08/18/2020 12:30:45
Only on the develop branch

##### Olivier Michel [cyberbotics] 08/18/2020 12:30:40
I believe it is implemented in the nightly build of R2021a.

##### Simon Steinmann [Moderator] 08/18/2020 12:29:47
node from device is not implemented in the main build yyet right?

##### David Mansolino [cyberbotics] 08/18/2020 12:29:15
No, you can only get the min and max, but the robot is not able to change them (except using the Supervisor API).

##### Simon Steinmann [Moderator] 08/18/2020 12:28:27
is there a quick way to set max and min position of a motor from code? or to ignore limits

##### David Mansolino [cyberbotics] 08/18/2020 10:56:23
You're welcome

##### Simon Steinmann [Moderator] 08/18/2020 10:56:00
okay, thx ^

##### David Mansolino [cyberbotics] 08/18/2020 10:55:55
*should 😉


That's indeed not clearly said

##### Simon Steinmann [Moderator] 08/18/2020 10:55:45
can or should? :p

##### David Mansolino [cyberbotics] 08/18/2020 10:55:27
For up you can take webots-R2020b-rev1-x86-64.tar.bz2


For ubuntu 18.04 you can take the webots-R2020b-rev1-x86-64\_ubuntu-16.04.tar.bz2

##### Simon Steinmann [Moderator] 08/18/2020 10:54:40
for ubuntu 18.04 and up I take 

webots-R2020b-rev1-x86-64.tar.bz2 

right?

##### David Mansolino [cyberbotics] 08/18/2020 10:53:05
Yes exactly

##### Simon Steinmann [Moderator] 08/18/2020 10:52:47
would have to install 2020b, With the tar install, I just have to extract and change environment variable right?

##### David Mansolino [cyberbotics] 08/18/2020 10:46:08
But the code is probably wrong:

=> root = supervisor.getProtoFromDef('MY\_ROBOT\_ROOT')

##### Simon Steinmann [Moderator] 08/18/2020 10:46:00
oh, running a

##### David Mansolino [cyberbotics] 08/18/2020 10:45:39
no, it is in R2020b

##### Simon Steinmann [Moderator] 08/18/2020 10:45:14
is the getProto a new develop-branch function?


what would be 'MY\_ROBOT\_ROOT'


root = proto.getProtoFromDef('MY\_ROBOT\_ROOT') 

this is not a valid function

##### David Mansolino [cyberbotics] 08/18/2020 10:38:01
I think there is an issue:

n = getSFNode(child, name)  = > n = getNode(child, name)

##### Simon Steinmann [Moderator] 08/18/2020 10:37:03
thx I will test it

##### Olivier Michel [cyberbotics] 08/18/2020 10:35:41
Something like this should do the job (not tested).


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

##### Simon Steinmann [Moderator] 08/18/2020 10:34:57
those are correct. the issue will be the tool slot. with the webots model, the y-axis points out, with the urdff model, the x-axis points out. But we can adjust that to be the same, like the old webots model

##### David Mansolino [cyberbotics] 08/18/2020 10:33:30
We can indeed add the tool link, but we should be carreful to not break compatibility (e.g. changing motor/sensor names) when updating it.

##### Simon Steinmann [Moderator] 08/18/2020 10:32:08
the structure of the proto is not like the official urdf. mainly the tool link is missing.


okay, so the ur robots will have to be created again.


but the supervisor loop would be great in general, and perhaps preferable here


would have liked to prevent more libraries, but that is an option as well of course


we might have to update all official robotic arms with the new urdf2webots convention

##### David Mansolino [cyberbotics] 08/18/2020 10:19:30
> I especially need the rotation matrix, urdf only has rpy

`@Simon Steinmann` can't you convert the rpy into matrix? (in python e.g. `transforms3d` allows to do this: [https://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#terms-used-in-function-names](https://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#terms-used-in-function-names))

##### Olivier Michel [cyberbotics] 08/18/2020 10:19:08
OK.

##### Simon Steinmann [Moderator] 08/18/2020 10:19:02
i guess name

##### Olivier Michel [cyberbotics] 08/18/2020 10:18:38
How will your recognize the node you want to get the orientation/position? By its name, any other field?

##### Simon Steinmann [Moderator] 08/18/2020 10:16:30
thank you

##### Olivier Michel [cyberbotics] 08/18/2020 10:13:06
OK, let me have a look.

##### Simon Steinmann [Moderator] 08/18/2020 10:12:47
python

##### Olivier Michel [cyberbotics] 08/18/2020 10:12:41
Which programming language are you using?

##### Simon Steinmann [Moderator] 08/18/2020 10:12:20
could you help me with that loop?


trying to make a universal-IK controller for you guys

##### Olivier Michel [cyberbotics] 08/18/2020 10:11:54
A fairly simple loop should do the job.

##### Simon Steinmann [Moderator] 08/18/2020 10:11:32
problem is, I need it for the last solid 😄

##### Olivier Michel [cyberbotics] 08/18/2020 10:11:29
Yes.

##### Simon Steinmann [Moderator] 08/18/2020 10:11:23
I would have to go through the whole tree right?

##### Olivier Michel [cyberbotics] 08/18/2020 10:11:05
If an upper parent has a DEF name, this may be workable though.

##### Simon Steinmann [Moderator] 08/18/2020 10:10:35
damn

##### Olivier Michel [cyberbotics] 08/18/2020 10:10:30
Yes.

##### Simon Steinmann [Moderator] 08/18/2020 10:10:25
but  the solid needs to have a defined DEF right?

##### Olivier Michel [cyberbotics] 08/18/2020 10:10:20
And [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_position](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_position)


See [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_from\_proto\_def](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_from_proto_def)


Yes, you should be able to do this from a supervisor controller.

##### Simon Steinmann [Moderator] 08/18/2020 10:06:27
I especially need the rotation matrix, urdf only has rpy


is there a easy way to get translation and orientation of a specific solid in a proto file?


automatically


universal IK controller nearly done :D. already requires no urdf import (used your armChain creation). only thing left is to figure out, which axis points out of the robot


true I guess, still would be usefull. I might update the documentation and link to a implementation. Or we update the irb inverse kinematics world with this function, showing a general implementation on how to get relative positions and orientationis

##### David Mansolino [cyberbotics] 08/18/2020 08:11:04
It is indeed useful, but we somehow have to keep the number of API functions low (as to keep the quality level high it requires maintenance time) so we have to be careful when adding function to get information that can already be extracted from other API functions.

##### Simon Steinmann [Moderator] 08/18/2020 08:10:07
Doing all this linear algebra manually is a big obstacle for many people. Took me a while to figure out too. First time I needed linear algebra since uni pretty much 😄


In the controller I sent you, there is the extra file 'get\_relative\_position.py'. I think this is a much needed functionality for the supervisor. Could look something like this:

supervisor.getPositionRelative(self, node)


glad to be of service 😄

##### David Mansolino [cyberbotics] 08/18/2020 07:54:21
indeed good point, I should admit that the name of the joint was chosen without particularly thinking about this, it need indeed a bit more thinking about this.

##### Simon Steinmann [Moderator] 08/18/2020 07:53:16
if you were to convert it back, the motors would be called 'joint\_sensor' and the sensors 'joint\_sensor\_sensor'

##### David Mansolino [cyberbotics] 08/18/2020 07:53:02
Makes sense indeed.

##### Simon Steinmann [Moderator] 08/18/2020 07:52:41
converting urdf2webots uses joint names as motor names, that's why I'd assume the other way around would do the same

##### David Mansolino [cyberbotics] 08/18/2020 07:52:18
Sure, here it is: [https://github.com/cyberbotics/webots/blob/master/projects/robots/abb/irb/controllers/inverse\_kinematics/inverse\_kinematics.py](https://github.com/cyberbotics/webots/blob/master/projects/robots/abb/irb/controllers/inverse_kinematics/inverse_kinematics.py)


No, it was either the motor or sensor name, we decided sensor name.

##### Simon Steinmann [Moderator] 08/18/2020 07:51:46
oh and can you link me the IK controller you're working on?


particular reason it does this?

##### David Mansolino [cyberbotics] 08/18/2020 07:50:52
No, it uses indeed the sensor as names, but you should be able to easily switch from one to the other (removing the 'sensor' postfix).

##### Simon Steinmann [Moderator] 08/18/2020 07:49:13
the version I have uses the sensor names for the joints. has this been fixed in the new nightlies?

##### David Mansolino [cyberbotics] 08/18/2020 07:35:24
(and improved in the nightly)


Yes, that's new in R2020b 😉

##### Simon Steinmann [Moderator] 08/18/2020 07:33:02
😮 that exists? That would be awesome, I'll check it out later when I have time

##### David Mansolino [cyberbotics] 08/18/2020 05:48:43
> I wonder if the link-setup information could be extracted straight from the protofile. Then it would truly be an universal controller, that wouldn't need any adjustments between robot arms

`@Simon Steinmann` what about using the new `wb_robot_get_urdf` function?


That looks very cool, I have been trying to do the same using IKPY last week ([https://github.com/cyberbotics/webots/blob/master/projects/robots/abb/irb/controllers/inverse\_kinematics/inverse\_kinematics.py](https://github.com/cyberbotics/webots/blob/master/projects/robots/abb/irb/controllers/inverse_kinematics/inverse_kinematics.py)) but I am not 100% happy with the result, I would be very interested to see which library you are using and how.

##### Simon Steinmann [Moderator] 08/17/2020 19:38:03
do you need help with IK?


`@Dennet` have not uploaded it yet

##### Dennet 08/17/2020 19:31:23
> I wonder if the link-setup information could be extracted straight from the protofile. Then it would truly be an universal controller, that wouldn't need any adjustments between robot arms

`@Simon Steinmann` where did you upload the controller?

##### Simon Steinmann [Moderator] 08/17/2020 17:30:13

> **Attachment**: [ik3.mp4](https://cdn.discordapp.com/attachments/565155651395780609/744971358995349504/ik3.mp4)


I wonder if the link-setup information could be extracted straight from the protofile. Then it would truly be an universal controller, that wouldn't need any adjustments between robot arms


I created a universal IK controller. only parameters that need to be changed are the urdf filename and which axis points out of the toolSlot. Works really well

##### David Mansolino [cyberbotics] 08/17/2020 06:11:46
Just opened an issue about this: [https://github.com/cyberbotics/urdf2webots/issues/72](https://github.com/cyberbotics/urdf2webots/issues/72)


Oh yes indeed, we should open the 'name' field too.

##### Simon Steinmann [Moderator] 08/14/2020 16:06:00
Can this be an issue? The base node of every robot is called 'base\_link'. Could this lead to problems?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/743863005908959352/unknown.png)
%end


Oh I didn't see that one still being open. I replaced all files. pull/9 can be discarded

##### David Mansolino [cyberbotics] 08/13/2020 14:02:09
Can you also update the one of [https://github.com/cyberbotics/community-projects/pull/9](https://github.com/cyberbotics/community-projects/pull/9) ?

##### Simon Steinmann [Moderator] 08/13/2020 13:04:28
converted and added bunch of KUKA arms:

[https://github.com/cyberbotics/community-projects/pull/11](https://github.com/cyberbotics/community-projects/pull/11)


done

##### David Mansolino [cyberbotics] 08/13/2020 12:14:29
That's already a good start

##### Simon Steinmann [Moderator] 08/13/2020 12:13:46
well at least for robot arms 🙂

##### David Mansolino [cyberbotics] 08/13/2020 12:13:26
> No manual edits needed after converting

`@Simon Steinmann` 🎉

##### Simon Steinmann [Moderator] 08/13/2020 12:08:13
No manual edits needed after converting


after this I'll be happy, I swear


[https://github.com/cyberbotics/urdf2webots/pull/71](https://github.com/cyberbotics/urdf2webots/pull/71)


... Sorry to do this, but one final PR 😄


merged


pushed it


if float(limitElement.getAttribute('effort')) != 0:


jup, adding this line works:


line 825 in parserURDF needs a condition I think

##### David Mansolino [cyberbotics] 08/13/2020 11:29:32
oups, will try

##### Simon Steinmann [Moderator] 08/13/2020 11:28:51
the torque limits are still 0.0, not 10000


max torque implementation doesnt work for me

##### David Mansolino [cyberbotics] 08/13/2020 11:06:55
I just finished, if you're fine with the changes feel free to merge it.

##### Simon Steinmann [Moderator] 08/13/2020 10:09:27
implemented the physics node thing too. Not sure it is the cleanest solution, but at least it only needs 4 lines of extra code

##### David Mansolino [cyberbotics] 08/13/2020 09:39:37
Perfect, I will check it just after lunch

##### Simon Steinmann [Moderator] 08/13/2020 09:39:35
now we just have to remove the physics nodes from epmty liks


added it

##### David Mansolino [cyberbotics] 08/13/2020 09:36:50
(I was going to do it ^^, but if you can do it while I am testing it's cool

##### Simon Steinmann [Moderator] 08/13/2020 09:36:36
already tested it. change 1 indet + change 2 lines


I can add the torque fix while we are at it.

##### David Mansolino [cyberbotics] 08/13/2020 09:33:28
That looks all good, I will try and update the test.

##### Simon Steinmann [Moderator] 08/13/2020 09:29:55
implemented and tested. works 🙂


[https://github.com/cyberbotics/urdf2webots/pull/70](https://github.com/cyberbotics/urdf2webots/pull/70)

##### David Mansolino [cyberbotics] 08/13/2020 09:09:47
Perfect, thank  you 🙂

##### Simon Steinmann [Moderator] 08/13/2020 09:09:34
already at it 😄

##### David Mansolino [cyberbotics] 08/13/2020 09:09:05
We are are going to re-work this indeed (that's on our medium term plan), but in any case avoiding duplcating PROTO names is a good practice (think for example if a user then move the files to put them all in the same folder).

It should be sufficient to prepend the name of the robot to the `name` variable here: [https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/writeProto.py#L461](https://github.com/cyberbotics/urdf2webots/blob/master/urdf2webots/writeProto.py#L461)

Would you give a try? Or I can do this if you want.

##### Simon Steinmann [Moderator] 08/13/2020 09:05:56
but perhaps it would be better to (or in addition), to change the mesh-lookup algorithm. So it starts in the same directory as the main proto file

##### David Mansolino [cyberbotics] 08/13/2020 09:04:54
Yes, that shouldn't be too dificult to include this prefix

##### Simon Steinmann [Moderator] 08/13/2020 09:04:24
fix should be easy enough


okay, then names have to be unique


ohhhh, it doesnt start searching at the location of the proto file, but at the base level

##### David Mansolino [cyberbotics] 08/13/2020 09:03:12
We should probably improve the name of the sub-PROTO to include as prefix the name of the robot.

##### Simon Steinmann [Moderator] 08/13/2020 09:03:06
yes, same

##### David Mansolino [cyberbotics] 08/13/2020 09:02:47
Ok, I think I found the reason. The problem is that both robot are defining the same PROTO names, and therefore in the case of the `lrb_iiwa` some meshes of the `kr5` are used.

##### Simon Steinmann [Moderator] 08/13/2020 09:01:23
hold on

##### David Mansolino [cyberbotics] 08/13/2020 09:01:14
Ohhh I can reprocude the issue with the `lrb_iiwa` but not with the `kr5` is it the same for you?


> from my understanding, if not defined in urdf, it is handled as not having a limit

`@Simon Steinmann`ok then in that case, if not defined we should probably set a very high value (e.g. 10'000)

##### Simon Steinmann [Moderator] 08/13/2020 08:59:31
restarting webots didnt fix the issue weirdly enough

##### David Mansolino [cyberbotics] 08/13/2020 08:59:06
> simply copy pasting the meshes folder into my current project's protos folder, fixes all the issues and looks like this:

`@Simon Steinmann` for me it works fine from the commuity project without any changes too. Maybe you need to close and re-open Webots? It is possible that if you made some changes to the 'extern project' files they are updated only next time you restart Webots.

##### Simon Steinmann [Moderator] 08/13/2020 08:58:52
I mean, Gazebo handles it that way


from my understanding, if not defined in urdf, it is handled as not having a limit


ahhh I understand

##### David Mansolino [cyberbotics] 08/13/2020 08:54:08
In the urdf2webtos, if the maxtorque is not specified in th eurdf file, we just do not specify it in the PROTO file, Webots will then use the fedault value  which is quite small and probably doesn't correspond to the URDF default value. Instead, if the maxtorque is not defined in the URDF file, we can still export the maxTorque to a higher value (ideally corresponding to the URDF default maxTorque).

##### Simon Steinmann [Moderator] 08/13/2020 08:51:17
what do you mean?

##### David Mansolino [cyberbotics] 08/13/2020 08:50:49
ok, maybe if we find it, we can export to the standard urdf maxtorque if not specified in the urdf file.

##### Simon Steinmann [Moderator] 08/13/2020 08:50:44
not defined


nope

##### David Mansolino [cyberbotics] 08/13/2020 08:50:12
ok, and do you know what is the default maxtorque in URDF ?

##### Simon Steinmann [Moderator] 08/13/2020 08:49:57
no it is not

##### David Mansolino [cyberbotics] 08/13/2020 08:49:52
the maxtorque is not defined in the urdf ?

##### Simon Steinmann [Moderator] 08/13/2020 08:49:49
on the kr5 I changed it to maxTorque 1000, and it fixes it (not pushed yet)


There is also a issue with the standard maxTorque  of 100 being too low

##### David Mansolino [cyberbotics] 08/13/2020 08:48:53
I am currently reviewing your PR and checking this.

##### Simon Steinmann [Moderator] 08/13/2020 08:14:38
simply copy pasting the meshes folder into my current project's protos folder, fixes all the issues and looks like this:
%figure
![Screenshot_from_2020-08-13_10-10-20.png](https://cdn.discordapp.com/attachments/565155651395780609/743381994280321034/Screenshot_from_2020-08-13_10-10-20.png)
%end


another strange issue. This is the kr5 robot, loaded from the community projects folder
%figure
![Screenshot_from_2020-08-13_10-09-47.png](https://cdn.discordapp.com/attachments/565155651395780609/743381846217457694/Screenshot_from_2020-08-13_10-09-47.png)
%end

##### David Mansolino [cyberbotics] 08/13/2020 07:43:14
That's strange, I will try thsi too if I can reproduce the problem

##### Simon Steinmann [Moderator] 08/13/2020 07:42:54
[https://tenor.com/view/elmo-shrug-gif-5094560](https://tenor.com/view/elmo-shrug-gif-5094560)


removing and adding the hidden tag again seemed to have fixed the issue

##### David Mansolino [cyberbotics] 08/13/2020 07:39:11
ok, that's strange, will have a look as soon as I have the time to.

##### Simon Steinmann [Moderator] 08/13/2020 07:38:17
I can load the link itself as well, but for some reason, it doesnt work loading it into the robot link


the kr5 robot


[https://github.com/cyberbotics/community-projects/pull/9](https://github.com/cyberbotics/community-projects/pull/9)


uploading to community, one sec

##### David Mansolino [cyberbotics] 08/13/2020 07:30:05
To me it works fine, without any error in the console
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/743370782125719562/unknown.png)
%end

##### Simon Steinmann [Moderator] 08/13/2020 07:25:43
The file looks correct. I have no idea why this error comes up


DEF link\_4\_0 IndexedFaceSet: Cannot create IndexedFaceSet because: "'coordIndex' is empty.".
> **Attachment**: [link\_4\_0Mesh.proto](https://cdn.discordapp.com/attachments/565155651395780609/743369609666625587/link_4_0Mesh.proto)

##### David Mansolino [cyberbotics] 08/13/2020 07:06:45
perfect, thank you

##### Simon Steinmann [Moderator] 08/13/2020 07:05:30
done, opened issue

##### David Mansolino [cyberbotics] 08/13/2020 07:00:07
thank you

##### Simon Steinmann [Moderator] 08/13/2020 07:00:02
will do

##### David Mansolino [cyberbotics] 08/13/2020 06:59:43
Please open an issue on the urdf2webots repo about this.

##### Simon Steinmann [Moderator] 08/13/2020 06:59:14
I agree. giving them mass, doesnt seem logical

##### David Mansolino [cyberbotics] 08/13/2020 06:58:39
We should probably improve the importer to handle this exception and avoid exporting the physis in this case.

##### Simon Steinmann [Moderator] 08/13/2020 06:56:36
Question is, how should empty links be handled properly? And can we automate this


converting the kuka repo atm. Getting these errors:

WARNING: KukaLbrIiwa14R820 (PROTO) > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > HingeJoint > Solid > Solid: Undefined inertia matrix: using the identity matrix. Please specify 'boundingObject' or 'inertiaMatrix' values.

WARNING: KukaLbrIiwa14R820 (PROTO) > Solid: Undefined inertia matrix: using the identity matrix. Please specify 'boundingObject' or 'inertiaMatrix' values.



I'm pretty sure, these are due to the two 'empty' links in the urdf:

<link name="tool0"/>

<link name="base"/>

##### David Mansolino [cyberbotics] 08/13/2020 05:17:10
Hi `@MarioAndres7`, welcome? Do you mean mobile robot?

##### MarioAndres7 08/12/2020 23:14:33
Hello Im new. Does anyone knows if exists and example of a movile robot? i want to give it 

coordinates

##### Olivier Michel [cyberbotics] 08/12/2020 16:58:51
Thanks. I will check it tomorrow.

##### Simon Steinmann [Moderator] 08/12/2020 16:21:17
okay done. Accepted all changes except 2 minor ones

##### Olivier Michel [cyberbotics] 08/12/2020 16:17:30
OK, thank you.

##### Simon Steinmann [Moderator] 08/12/2020 16:17:16
i'll do it manually

##### Olivier Michel [cyberbotics] 08/12/2020 16:16:35
Probably...

##### Simon Steinmann [Moderator] 08/12/2020 16:07:59
outdated?!


can't commit the last one

##### Olivier Michel [cyberbotics] 08/12/2020 15:43:19
Reviewed.

##### Simon Steinmann [Moderator] 08/12/2020 15:06:12
PR is ready for review [https://github.com/cyberbotics/urdf2webots/pull/68](https://github.com/cyberbotics/urdf2webots/pull/68)


sounds good

##### David Mansolino [cyberbotics] 08/12/2020 13:42:49
Like it is currently the case for the 'webots' repository: [https://github.com/cyberbotics/webots/tree/master/docs](https://github.com/cyberbotics/webots/tree/master/docs)


Yes, a 'docs' folder with an 'images' folder inside look good.

##### Simon Steinmann [Moderator] 08/12/2020 13:41:18
I could create a new directory called 'docs' and put everything, including the images there


When I include images in a tutorial, where do I put them? I'm currently creating a 'tutorial.md' for urdf2webots. My plan is to have it in the base directory and link it in the README.md.

##### Olivier Michel [cyberbotics] 08/07/2020 13:47:32
Yes, that would be great.

##### Simon Steinmann [Moderator] 08/07/2020 13:45:50
I could add this example code to the robot.getBasicTimeStep documentation. In case someone else runs into that issue
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/741291016308523018/unknown.png)
%end

##### Olivier Michel [cyberbotics] 08/07/2020 13:37:57
Because this is a rare use case and a workaround exists using a supervisor.

##### Simon Steinmann [Moderator] 08/07/2020 13:37:52
ohh I know what you mean

##### Olivier Michel [cyberbotics] 08/07/2020 13:37:27
No, I believe we won't implement it.

##### Simon Steinmann [Moderator] 08/07/2020 13:37:02
want me to open request?


because setting it is a bit clonky


Supervisor function to set and get the BasicTimeStep


that would be great

##### Olivier Michel [cyberbotics] 08/07/2020 13:35:55
A solution could be to read the "basicTimeStep" field of the WorldInfo node from a supervisor process.

##### Simon Steinmann [Moderator] 08/07/2020 13:35:21
benchmark runs at different timesteps. I worked around it, by not using 

robot.getBasicTimeStep()  for steps, but the manually set interval

##### Olivier Michel [cyberbotics] 08/07/2020 13:34:39
It's pretty rare to dynamically change the time step of a simulation. Why do your need to do that?

##### Simon Steinmann [Moderator] 08/07/2020 13:34:36
or add it to the supervisor and have 

robot.getBasicTimeStep()

grab the value directly from the world instance

##### Olivier Michel [cyberbotics] 08/07/2020 13:34:10
Yes, this is a known issue.

##### Simon Steinmann [Moderator] 08/07/2020 13:33:43
is there already an issue related to this? My suggestion would be to add a function:

robot.setBasicTimeStep


returns the Timestep on robot init, not the actual timestep of the simulation


I ran into an issue, where 

robot.getBasicTimeStep()


It worked, but needed to specify no-cache. For anyone else who wants to use the new and fixed version of urdf2webots, install it like this:

pip install --no-cache-dir --upgrade urdf2webots

##### David Mansolino [cyberbotics] 08/06/2020 12:13:50
It's now live (it reflects the current state of master): [https://pypi.org/project/urdf2webots/#history](https://pypi.org/project/urdf2webots/#history)


I just launched the process, now Github Action will create and upload the pip package

##### Simon Steinmann [Moderator] 08/06/2020 12:10:15
would be easier with pip. please ping me when it's done


making a  tutorial right now


hehe good

##### David Mansolino [cyberbotics] 08/06/2020 12:09:39
I will create it right now 😉


Not yet.

##### Simon Steinmann [Moderator] 08/06/2020 12:09:16
`@David Mansolino` is the new fixed version of urdff2webots already available on pip install? because I get this error:

importer.py: error: no such option: --multi-file

##### željko 08/06/2020 11:27:22
[https://github.com/ros-industrial/kuka\_experimental](https://github.com/ros-industrial/kuka_experimental)

The repo has other arms as well, all in separate folders


just a sec

##### Simon Steinmann [Moderator] 08/06/2020 11:26:00
link?

##### željko 08/06/2020 11:25:49
OK, thanks a lot 😄 You've just saved me countless hours hahah :)

I found a github repo with the kuka arm which I'm trying to get to work

##### Simon Steinmann [Moderator] 08/06/2020 11:24:42
you got a link to the kuka repo?


Yesterday I fixed a huge issue in the converter. The joints should not be wonky anymore, so make sure you pull the newest version of urdf2webots

##### željko 08/06/2020 11:23:39
Hey! I'm just in the middle of trying to wrap my head around urf2webots and a tutorial would be of great help to me! The arms I would like to use are KUKA LBR iiwa and Kinova Jaco and I belive that they would be a good addition as they are quite popular. BTW right now reading older messages as I see that you've had similar problems 🙂

##### Simon Steinmann [Moderator] 08/06/2020 11:19:38
Since I have done quite a few urdf2webots conversions lately, I might as well create a tutorial for it, as the process can be complicated if one has not much experience. Is there any robotic arm that people want to see added to the community projects?

##### Buzzer 08/05/2020 16:03:09
thanks `@Simon Steinmann`, I will follow this

##### Simon Steinmann [Moderator] 08/05/2020 16:01:00
because the function will only return values relative to the world


there is a python example I wrote, on how to get the position of an object relative to another. That might come in handy
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/740600155841101904/unknown.png)
%end

##### Buzzer 08/05/2020 15:58:49
Ow nice, thanks

##### Simon Steinmann [Moderator] 08/05/2020 15:58:32
there is. The controller has to be a supervisor though


[https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb\_supervisor\_node\_get\_orientation](https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_node_get_orientation)

##### Buzzer 08/05/2020 15:58:20
?


there is something like GetNode(Node*)


I want to built a bot to go to a specific node location


?


Hi, how can I get a node position

##### Simon Steinmann [Moderator] 08/04/2020 17:04:49
looking at the source code, the robot would have to be re-initialized. I think this function should get this value directly from the world instance instead from a value that get's defined on controller initialization


Okay I figured out the issue:

robot.getBasicTimeStep()

This always returns the timestep that a controller was initialized with. Changing the timestep does NOT change the return of this function.


There seems to be a something strange going on with the sim\_time and timestep


Suuuper weird issue. So I run the exact same task several times with increasing timesteps (4, 8, 16, 32). If everything is set correctly, it takes 43 seconds. I use the supervisor to reset the simulation after each run, and set the timestep before each run. Now here is the strange part. If the timestep is 4 or smaller, the time for each run is the same. However, if reset the controller and manually put the timestep to 32 BEFORE i start my script, it uses 32ms increments for the simulation time, even if the steps are actually smaller. This results in times of like 5min, 2,5 min 1,2 min and then finally 43 seconds when actually using 32 timestep again.

##### Olivier Michel [cyberbotics] 08/04/2020 16:05:46
In general yes, but your case seems tricky...

##### Simon Steinmann [Moderator] 08/04/2020 16:04:35
I just dont understand how decreasing the timestep can cause issues. That's how you're supposed to get rid of these issues

##### Stefania Pedrazzi [cyberbotics] 08/04/2020 16:01:05
unfortunately not, ODE doesn't give any details. But you can find it by simplifying your simulation until the problem disappears.

##### Simon Steinmann [Moderator] 08/04/2020 15:59:52
is there a way to get more detailed info? where exactle the issue is?

##### Stefania Pedrazzi [cyberbotics] 08/04/2020 15:58:04
it's a simple task, but it is complex from the physical computation

##### Simon Steinmann [Moderator] 08/04/2020 15:57:28
at something like 4 or 8 timestep, the simulation outcome is also exactly the same. At like 32 there is a bit of variance, which is expected. I find it weird that at timestep 1 it has issues


with cylinder objects, and gripper tips are boxes


a simple stacking task
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/740236647781630022/unknown.png)
%end

##### Stefania Pedrazzi [cyberbotics] 08/04/2020 15:54:36
it depends on the simulation and the collisions between the objects at the computation time. Maybe in your case increasing the time step makes skipping some problematic contact or object's intersection.

##### Simon Steinmann [Moderator] 08/04/2020 15:47:22
the issue is, that I get these errors at timestep 1, which is much smaller than let's say 8, where everythign runs fine

##### Stefania Pedrazzi [cyberbotics] 08/04/2020 15:46:43
As reported by the warning message, the ODE physics engine has issues in computing the simulation step due to the complexity of the world.

Reducing the time step also reduces the complexity of the computations. Other suggestions to improve the stability of the simulation and avoid this error can be found here:

[https://www.cyberbotics.com/doc/guide/modeling#my-robotsimulation-explodes-what-should-i-do](https://www.cyberbotics.com/doc/guide/modeling#my-robotsimulation-explodes-what-should-i-do)

##### Simon Steinmann [Moderator] 08/04/2020 15:17:17
Found a strange behavior. Running EXACTLY the same simulation, I get several of these errors:

WARNING: The current physics step could not be computed correctly. Your world may be too complex. If this problem persists, try simplifying your bounding object(s), reducing the number of joints, or reducing WorldInfo.basicTimeStep.



if the timeStep=1    Timestep 4-16  are fine

##### David Mansolino [cyberbotics] 07/31/2020 08:46:40
You're welcome

##### Yanick Douven 07/31/2020 08:46:31
Perfect, thank you very much!

##### David Mansolino [cyberbotics] 07/31/2020 08:45:59
> Or, if not, is there some Python implementation of the generic lidar controller?

`@Yanick Douven` unfortunately not, but you should be able to convert it quite simply, here is the source code: [https://github.com/cyberbotics/webots/blob/master/projects/default/controllers/ros/RosLidar.cpp](https://github.com/cyberbotics/webots/blob/master/projects/default/controllers/ros/RosLidar.cpp)


Yes and no, you can split your robot into two robots (one been a child of the other) and assign one controller per robot.

##### Yanick Douven 07/31/2020 08:45:12
Or, if not, is there some Python implementation of the generic lidar controller?


Thanks! I didn't know there was a standard ROS controller. I'm currently writing a custom one in Python, since I need some non-generic interfacing with other components. Is it possible to combine the two? I.e. run the standard and custom ROS controllers side by side?

##### David Mansolino [cyberbotics] 07/31/2020 08:29:32
Ok, yes in that case the default ros controller does implement an interface for any kind of lidars: 

  - [https://www.cyberbotics.com/doc/guide/using-ros#standard-ros-controller](https://www.cyberbotics.com/doc/guide/using-ros#standard-ros-controller)

  - [https://www.cyberbotics.com/doc/reference/lidar?tab-language=ros#lidar-functions](https://www.cyberbotics.com/doc/reference/lidar?tab-language=ros#lidar-functions)

##### Yanick Douven 07/31/2020 08:28:12
Hi David,

For ROS 1

##### David Mansolino [cyberbotics] 07/31/2020 08:25:34
hi, for ROS 1 or 2?

##### Yanick Douven 07/31/2020 07:57:06
Hi everyone!

I'm trying to implement a ROS controller for a 3D lidar (like the Velodyne). Does anything like this already exist?

Thank you!

##### David Mansolino [cyberbotics] 07/29/2020 10:48:29
You're welcome

##### Lukulus 07/29/2020 10:46:56
ok, thank you 🙂

##### David Mansolino [cyberbotics] 07/29/2020 10:43:48
Hi `@Lukulus`, Webots APi supports well multi-threading, however, it is strongly recommended to call the step function from one thread only, doing is contrary is extremely error-prone.

##### Lukulus 07/29/2020 10:33:21
Hello I am trying to use multiple threads for different tasks for an robot. 

But it seems like calling the robot->step() funktion in diffenrent threads leads to stop the Simulation.

Sometimes it runs, sometimes time stops and also the simulation.

So can you tell me if its possible to use different threads in a robot controller and how to use them in a right way?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/737981083752202241/unknown.png)
%end

##### Simon Steinmann [Moderator] 07/27/2020 16:30:05
read out the appropiate sensor

##### csnametala 07/27/2020 16:11:10
Hello everyone!

I have an e-puck robot in Webots simulations, and I wanted to detect when it hits a wall. How can I solve this?

##### JSON Derulo 07/23/2020 08:57:35
Awesome, thanks

##### Darko Lukić [cyberbotics] 07/23/2020 08:57:02
`@JSON Derulo` Yes, you can check this example: [https://www.youtube.com/watch?v=gO0EXKv8x70](https://www.youtube.com/watch?v=gO0EXKv8x70)



In general, Webots is compatible with ROS and ROS2, therefore you can use ROS packages. A few more links:

- Webots with ROS1 tutorial: [https://cyberbotics.com/doc/guide/tutorial-8-using-ros](https://cyberbotics.com/doc/guide/tutorial-8-using-ros)

- Webots support for ROS2: [http://wiki.ros.org/webots\_ros2](http://wiki.ros.org/webots_ros2)

##### JSON Derulo 07/23/2020 08:54:46
Hi, does WeBots have to the ability to be integrated with by planners?

##### Olivier Michel [cyberbotics] 07/23/2020 05:33:41
You are welcome. We plan to release Webots R2020b (including this feature) in the next couple of days. We will make sure we announce it (including this ROS-friendly feature and a few others) on ROS Discourse.

##### Emiliano Borghi 07/22/2020 19:33:46
Hey, I saw you merged this PR: [https://github.com/cyberbotics/webots/pull/1643](https://github.com/cyberbotics/webots/pull/1643)

I would recommend you to make a post in ROS Discourse because it will help the ROS community with their simulations (included myself).



Thanks for merging it BTW!

##### David Mansolino [cyberbotics] 07/14/2020 14:31:19
You're welcome

##### Simon Steinmann [Moderator] 07/14/2020 14:23:27
thanks for the infos 🙂


hmm, perhaps we have to copy and save the setup file ourselves then

##### David Mansolino [cyberbotics] 07/14/2020 14:23:01
But we expect to release the official version of R2020b in ~3 weeks from now.


Unfortunately not.

##### Simon Steinmann [Moderator] 07/14/2020 14:21:39
that one doesnt have the simulation reset fix included yet right?

##### David Mansolino [cyberbotics] 07/14/2020 14:21:22
ok, in that case, only the official stable versions will be kept (e.g. latest R2020a-rev1).

##### Simon Steinmann [Moderator] 07/14/2020 14:20:24
collegues are setting up their systems and environments, we need to be able to install the same version, and preferably not have to change it frequently


is there a ETA?

##### David Mansolino [cyberbotics] 07/14/2020 14:19:20
But once we will release the official stable version of R2020b, this one will stay up indefinetely.

##### Simon Steinmann [Moderator] 07/14/2020 14:19:05
hmm

##### David Mansolino [cyberbotics] 07/14/2020 14:18:49
no, we keep only the one of the last 3 days.

##### Simon Steinmann [Moderator] 07/14/2020 14:18:33
do nightly build files stay up indefinetely?

##### David Mansolino [cyberbotics] 07/14/2020 14:17:25
yes

##### Simon Steinmann [Moderator] 07/14/2020 14:17:20
version b is the develop branch?

##### David Mansolino [cyberbotics] 07/14/2020 14:16:37
We usually do 2 version per year, first version a, then aroudn the middle of the year we create version b, then in between these version if needed we create patch release (e.g. a-revision1, a-revision2, etc.). If you need the patch, I would stick to the version b nightly, we will soon release an official version of R2020b when this is the case, I woudl stick to this official and stable R2020b.

##### Simon Steinmann [Moderator] 07/14/2020 14:14:17
Should I just get the latest nightly build of the master branch?


For our simulation benchmark project, we have to decide on a  version of webots. Could you give a short explanation on how your versions work?  For example, what is 2020a vs 2020b? When does the official version get updated? And lastly, which version would you recommend? I know that I needed to install a nightly build at the end of mai, to fix the simulation reset issue, and that some proto file updates are essential for our benchmark.


A universal(common, not the company) trajectory-arm controller might be something interesting


On that node, I highly modified the universal\_ROS controller you guys created. Made it extern and modified the trajectory and pose\_publisher slightly. Now all it takes to switch from ur10e to kinova-Gen3, is to change the joint names and the same controller works.


I might make a kinova\_webots git. A bunch of the launch files have to be altered in order for the IK to work. Plus it needs a custom ROS controller for webots.

##### David Mansolino [cyberbotics] 07/09/2020 09:42:36
> Btw, successfully created a working GEN3 kinova arm. Even got IK through moveIt to run. Any news on the 'unofficial' robot model repo?

`@Simon Steinmann` Very nice! No news yet (several of us are in holidays this week, so we will wait next week to discuss about this), but I will for sure let you know, it would be nice to include your model 🙂


> I think I had to flip 4/6 joints. Inverse kinematics was broken before, now it works. This should be looked into though in my opinion. It takes deeper knowledge to be able to figure out the issue and fix it. Would be great if it works out of the box and control + IK code can be directly used from the existing urdf based repositories.

`@Simon Steinmann` ok thank you for the feedback, we will try to fix this in the URDF exporter directly.

##### Simon Steinmann [Moderator] 07/09/2020 09:37:31
Btw, successfully created a working GEN3 kinova arm. Even got IK through moveIt to run. Any news on the 'unofficial' robot model repo?
%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/730719273768452136/unknown.png)
%end


I think I had to flip 4/6 joints. Inverse kinematics was broken before, now it works. This should be looked into though in my opinion. It takes deeper knowledge to be able to figure out the issue and fix it. Would be great if it works out of the box and control + IK code can be directly used from the existing urdf based repositories.


Yes, that's exactly the same issue. For some joints the direction of rotation is flipped. I fixed it manually, by flipping the rotational axis (0 1 0 --> 0 -1 0)

##### David Mansolino [cyberbotics] 07/09/2020 05:33:58
Hi, just to make sur I understood correctly, the joint axes were correct, but the direction wrong right? Is the error similar to [https://github.com/cyberbotics/urdf2webots/issues/42](https://github.com/cyberbotics/urdf2webots/issues/42) ?

##### Simon Steinmann [Moderator] 07/08/2020 15:55:06
I had a weird issue with urdf2webots. I converted the kinova gen3 6dof arm and everything worked. However when implementing it with moveit, I noticed that several hingeJoints were rotating in the wrong direction. I had to switch the axis manually to negative (or positve) to change direction.  It might have something to do with the urdf using the z-axis and the proto using the y-axis for the joint.

##### David Mansolino [cyberbotics] 07/07/2020 05:54:27
> `@David Mansolino` I seem to have figured it out. This shows joint 1 and 3 being changed in percent of their valid range. Looks correct now. Do you want me to make a PR with the new, 'correct' robotiq gripper?

`@Simon Steinmann` yes sure if you have any correction to bring to the model any PR is highly appreciated!


> I was trying to see what's inside this - "Visual\_tracking.wbt" sample world inside Webots. Unfortunately I didn't found a github link to the file for reference, but this screenshot should be helpful.

> My question was, how does the RubberDuck move? I noticed it has an immersion properties node, with fluid name - "water". But I don't see any fluid node in the scene tree. Also there's no controller code, for it's movement.

> Any explanation please?

`@ContrastNull` here is the link: [https://github.com/cyberbotics/webots/tree/master/projects/samples/robotbenchmark/visual\_tracking](https://github.com/cyberbotics/webots/tree/master/projects/samples/robotbenchmark/visual_tracking)

The duck is moved by with a Supervisor, here is the controller of the Supervisor: [https://github.com/cyberbotics/webots/blob/master/projects/samples/robotbenchmark/visual\_tracking/controllers/visual\_tracking\_benchmark/visual\_tracking\_benchmark.py](https://github.com/cyberbotics/webots/blob/master/projects/samples/robotbenchmark/visual_tracking/controllers/visual_tracking_benchmark/visual_tracking_benchmark.py)

##### ContrastNull 07/07/2020 04:53:42
I was trying to see what's inside this - "Visual\_tracking.wbt" sample world inside Webots. Unfortunately I didn't found a github link to the file for reference, but this screenshot should be helpful.

My question was, how does the RubberDuck move? I noticed it has an immersion properties node, with fluid name - "water". But I don't see any fluid node in the scene tree. Also there's no controller code, for it's movement.

Any explanation please?
%figure
![Screenshot_from_2020-07-07_10-17-01.png](https://cdn.discordapp.com/attachments/565155651395780609/729923074715025518/Screenshot_from_2020-07-07_10-17-01.png)
%end

##### zeynepp 07/06/2020 20:17:10
where is the problem



> **Attachment**: [empty\_.mp4](https://cdn.discordapp.com/attachments/565155651395780609/729792436452196372/empty_.mp4)


what do u think


I am trying to write the kinematics for the lrb robot for ipr but it is not directed to the target



%figure
![2.JPG](https://cdn.discordapp.com/attachments/565155651395780609/729791411846774824/2.JPG)
%end



%figure
![1.JPG](https://cdn.discordapp.com/attachments/565155651395780609/729791407081914508/1.JPG)
%end

##### Simon Steinmann [Moderator] 07/06/2020 17:30:54
`@David Mansolino` I seem to have figured it out. This shows joint 1 and 3 being changed in percent of their valid range. Looks correct now. Do you want me to make a PR with the new, 'correct' robotiq gripper?
> **Attachment**: [3f\_gripper.mp4](https://cdn.discordapp.com/attachments/565155651395780609/729751244406521887/3f_gripper.mp4)

##### David Mansolino [cyberbotics] 07/06/2020 15:35:44
Yes, sure I will answer it tomorrow morning

##### Simon Steinmann [Moderator] 07/06/2020 14:32:51
[https://github.com/cyberbotics/webots/issues/1841](https://github.com/cyberbotics/webots/issues/1841)


`@David Mansolino` hi david, could you take a quick look at this issue? If I know how to solve it, I can write up a short tutorial

##### David Mansolino [cyberbotics] 07/06/2020 05:40:57
> i need to create a 3d environment from a 2d image..Is it possible

`@starstuff_0903` can you tell us more, what does represent your image? The height of the terrain ?


> Is it possible to use just python3 with webots?  Could not build the python2 language binding due to my libpython2 missing symbols, and since ROS2 is all python3, figured I would not need it.  But finding python2 dependenceis in webots\_ros2 package

`@henry10210` which python2 dependencies do you see in the webots\_ros2 package?


> I set the WEBOTS\_HOME to the root of the source code folder (where I ran the ./webots command).  I wonder if this is an incorrect WEBOTS\_HOME

`@henry10210` yes this is correct.


> I got my computer back and it was in fact a hardrive issue. It was just a coincidence that I was running an extensive Webots  simulation

> Thanks for the tips

`@Clara Cardoso Ferreira` you're welcome! thank you for the feedback!

##### mint 07/06/2020 05:01:03
nevermind, I just  figured out that it is VRML97. my bad XD


Hello, is there any ways of writing a proto file  from an external design tool like CAD? What  kind of language does it belong to?

##### starstuff\_0903 07/05/2020 10:26:14
i need to create a 3d environment from a 2d image..Is it possible

##### Simon Steinmann [Moderator] 07/05/2020 09:34:48
`@henry10210` just select extern as the controller and run your controller outside in your own environment

##### henry10210 07/05/2020 04:36:03
Is it possible to use just python3 with webots?  Could not build the python2 language binding due to my libpython2 missing symbols, and since ROS2 is all python3, figured I would not need it.  But finding python2 dependenceis in webots\_ros2 package


I set the WEBOTS\_HOME to the root of the source code folder (where I ran the ./webots command).  I wonder if this is an incorrect WEBOTS\_HOME

##### Clara Cardoso Ferreira 07/03/2020 23:17:42
> Thanks I'll try that when I get my computer back.

> My controller was not writing anything at the moment it crashed, but it was probably storing data. Yeah, it may have been the harddrive



I got my computer back and it was in fact a hardrive issue. It was just a coincidence that I was running an extensive Webots  simulation

Thanks for the tips

##### Marian 06/30/2020 09:45:52
Ok, thanks a lot for the explanation. It works!

##### lojik 06/30/2020 09:32:56
Your "shape" node should be in the "children" one and not "boundingObject". Then you can select it in "boundingObject" because bounding object is just the virtual shape taken into account when performing contact simulation. It is not the visual rendering of the object.

##### Marian 06/30/2020 09:29:18
Hi, I'm trying to create some simple cylinders in WeBots. It works but unfortunatly it only creates a wireframe but no texture. What is missing here?
%figure
![webots_cylinderr.png](https://cdn.discordapp.com/attachments/565155651395780609/727455716390076437/webots_cylinderr.png)
%end

##### Clara Cardoso Ferreira 06/29/2020 15:04:28
Thanks I'll try that when I get my computer back.

My controller was not writing anything at the moment it crashed, but it was probably storing data. Yeah, it may have been the harddrive

##### David Mansolino [cyberbotics] 06/25/2020 04:55:19
About your other question, was your controller writing to a file? Would it be possibl that it saturated the hard drive?


You can do this directly from the proto itself, you simply have to set the 'randomSeed' field to 0 as explained here: [https://www.cyberbotics.com/doc/guide/object-floors#uneventerrain-field-summary](https://www.cyberbotics.com/doc/guide/object-floors#uneventerrain-field-summary)

##### Clara Cardoso Ferreira 06/24/2020 19:00:41
> `@Clara Cardoso Ferreira` to reset the terrain from the proto file is indeed possible, unfortunately it is not possible to move the robot from there, but you can move it then at the first step of the simulation with a Supervisor controller.

`@David Mansolino` 



Your answer somewhat helps. I understand I can reset the terrain, but would I have to reset it manually or is there a way to make a loop so that a different terrain is used in every simulation iteration? I imagine I would have to connect the supervisor code to the proto somehow



Another big question, I forgot a simulation running overnight on my computer and the next morning, it would not turn on. It may have been a coincidence that my computer failed even though it is under a year of use. Would you have any suggestions of what may have gone wrong?

##### David Mansolino [cyberbotics] 06/24/2020 14:06:16
You're welcome 😉

##### black\_hammer\_67 06/24/2020 14:06:02
thank you clarifying that 😁

##### David Mansolino [cyberbotics] 06/24/2020 14:04:52
Yes, that sounds good.

##### black\_hammer\_67 06/24/2020 14:04:12
ok i see, so I am looking for points that they have a certain value on y axis > 0.1 for example ?

##### David Mansolino [cyberbotics] 06/24/2020 14:02:59
That's unfortunately not so simple, the number of contact points of the wheel can vary, you should rather check the location of the contact points.

##### black\_hammer\_67 06/24/2020 14:01:48
ok so if I have a pioneer 3dx robot the default contact points are 3 , these that the wheels have with the floor and if I detect 4 or more points I can assume this is a colision right ?

##### David Mansolino [cyberbotics] 06/24/2020 14:00:20
Yes, you can get the number and location of the contact points of a node:  [https://cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_contact\_point](https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_contact_point)

##### black\_hammer\_67 06/24/2020 13:59:32
hello, is there a way to detect robot colisions in the suppervisor mode ?

##### David Mansolino [cyberbotics] 06/24/2020 05:40:46
> Also, what is the benefit of using the supervisor vs a sensor (like the GPS) to get states such as position?

`@Clara Cardoso Ferreira` no benefit, if you are trying to get your own position, GPS is the way to go, if you are trying to get the position of another node supervisor is the way to go.


> Does anyone know if it's possible to reset the terrain in the proto for every simulation and set up the robot's wheels to touch the ground to start the simulation?

`@Clara Cardoso Ferreira` to reset the terrain from the proto file is indeed possible, unfortunately it is not possible to move the robot from there, but you can move it then at the first step of the simulation with a Supervisor controller.


> Can somebody help me to figure out how to reset the simulation without having to stop the controller, actually I want every time a condition is true in the controller to reset the robots initial position only, not the simulation time

`@black_hammer_67` yes, you can use the `wb_supervisor_simulation_reset` Supervisor function: [https://www.cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb\_supervisor\_simulation\_reset](https://www.cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb_supervisor_simulation_reset)

##### Andrei 06/23/2020 23:34:32
I guess it depends what you want to do. I wanna make a pathfinder so I wanna know the whole map when I determine the path I take not just what sensors can give me.

##### Clara Cardoso Ferreira 06/23/2020 23:33:02
Also, what is the benefit of using the supervisor vs a sensor (like the GPS) to get states such as position?


Does anyone know if it's possible to reset the terrain in the proto for every simulation and set up the robot's wheels to touch the ground to start the simulation?


I think so`@black_hammer_67` , check the supervisor API and look for position

##### black\_hammer\_67 06/23/2020 16:33:46
Can somebody help me to figure out how to reset the simulation without having to stop the controller, actually I want every time a condition is true in the controller to reset the robots initial position only, not the simulation time

##### David Mansolino [cyberbotics] 06/23/2020 05:01:36
No, Webots does not simulate microphones. But you micreate an interface to get the audio from your computer michrophone.

##### webotspro9999 06/22/2020 22:02:26
> hello `@webotspro9999`, this is unfortunately not available out of the box, you will have to implement your own speach recognition algorithm or use a library implementign this.

`@David Mansolino` thanks for your reply, does the NAO in Webots have sound listening functions? For example, when I say hello can the NAO hear this and take it as an input data?

##### Olivier Michel [cyberbotics] 06/22/2020 15:38:09
You can simply write a supervisor controller that will move the robot to a random position at the beginning of the simulation.

##### aalmanso 06/22/2020 15:36:42
Hi everyone,

I want to place E-puck randomly inside arena every time I start or reload the simulation, I need that to test some behaviour. So, i wander how can I find the help about using random numbers and 2D coordinates in webots simulator?

##### David Mansolino [cyberbotics] 06/22/2020 05:27:14
hello `@webotspro9999`, this is unfortunately not available out of the box, you will have to implement your own speach recognition algorithm or use a library implementign this.

##### webotspro9999 06/20/2020 22:56:16
I'm new in Webots and I want to develop speech recognition applications using Softbank Robotics NAO. Is this feature available in Webots?


hello all

##### lojik 06/11/2020 12:29:35
Okay, thank's 👍

##### David Mansolino [cyberbotics] 06/11/2020 12:01:48
Uneven terrai is using an EleveationGrid as mesh, this mesh is way more stable/efficient than IndexedFaceSet for collision but has more constraints ([https://www.cyberbotics.com/doc/reference/elevationgrid](https://www.cyberbotics.com/doc/reference/elevationgrid))

##### lojik 06/11/2020 12:00:12
I also see that uneven terrain seems to use a symetric grid mesh as a bounding object, is it right?


Yes, exactly, you use quite the same idea behind the road PROTO. So I will keep in mind this way to create meshes. Thank you !

##### David Mansolino [cyberbotics] 06/11/2020 11:51:11
OK, it make sense, for we are using similar meshes to create roads and it works quite well. Note that you might be interested by the road PROTO as it allows to create similar meshes (when varying the height of the road): [https://cyberbotics.com/doc/guide/object-road](https://cyberbotics.com/doc/guide/object-road)

##### lojik 06/11/2020 11:48:58
Ok thank you, so my intuition is not right, but it works much better with this mesh 👍

##### David Mansolino [cyberbotics] 06/11/2020 11:45:25
The mesh looks indeed to be very clean like this. Just for the note, for the cylinder, the parallel lines are just for visualization, for the physics it is a perfect cylinder.

##### lojik 06/11/2020 10:34:08
PS: The first run after importing such an object will provoc a webots' crash.



%figure
![Screenshot_from_2020-06-11_12-32-27.png](https://cdn.discordapp.com/attachments/565155651395780609/720586430224793600/Screenshot_from_2020-06-11_12-32-27.png)
%end


Ok, so after playing a bit with my meshes I have a first intuition. I realize that your cylinders are composed with only parallel lines. So I did the same in blender for my sinusoid ground and it seems to work much better. Now it is detected every time instead of 1/3 run.



Unfortunately it will keep to have triangles by exportations. But since there is a huge number of parallels in  my mesh, it works fine now.



I do not know if I explain well enough .. Here is my final mesh :

##### David Mansolino [cyberbotics] 06/11/2020 09:03:22
You're welcome

##### lojik 06/11/2020 09:02:53
Ok, well, I did not expected that. I have some good results with my meshes. But from one simulation to an other one the results are not the same so I was wondering where it was wrong.



Thank you for your answer, I will have a deeper look to find a good solution between approximation geometries and find a good mesh.

##### David Mansolino [cyberbotics] 06/11/2020 09:00:04
> I will always have to approximate the bounding object with a circle

Not exaclty, you can use several geometries to approximate the shape (such as for example several circles to approximate an ellipsoid).

##### lojik 06/11/2020 08:58:23
If I understand correctly it is actually not possible to create more complexe shapes with their own bounding object? If I would like to have a ramp which look like an ellipsoid more than a circle. I will always have to approximate the bounding object with a circle or take time too find a good threshold with the mesh to make it works?

##### David Mansolino [cyberbotics] 06/11/2020 08:51:10
I am sorry but there is no magic solution for this, using meshes as bounding object is not the ideal solution.

##### lojik 06/11/2020 08:48:00
I already have a basicTimeStep set to 1ms 😅


I work with blender to do these meshes. My goal is to have well designed unflat terrain to benchmark my robot in that condition.

##### David Mansolino [cyberbotics] 06/11/2020 08:46:16
it all depends on the complexity of the mesh, but there is a tradeoff (which is not always easy to find).

But maybe a simpler solution for recording the video is to decrease the WeorldInfo.timestep.

##### lojik 06/11/2020 08:46:02
Because I would like to import custom models to have more freedom on the form of obstacles I have. It would not be a good deal if I have to redo the bounding object after importing them..


Would it be better if I have a finner mesh? Or worst?

##### David Mansolino [cyberbotics] 06/11/2020 08:44:01
This is because the bounding object created automatically when imported are based on the actual mesh, this is known to be not very accurate and stable, for better collisions, you should re-create the key bounding objects using sets of basic geometries (e.g. boxes, spheres, etc.)

##### lojik 06/11/2020 08:42:15
I just try now to begin recording the video after that the robot well detected my objects. In that case it works fine, but I have to play simulation until the robot detects my objects and then begin to record video.


And this effect is worst when I would like to record a video. The object are never well detected.


As seen on this picture, the wheel on the left is inside the object and the bounding object (in white) should be in red if it is well detected by the wheel.



On the right, the wheel is correctly on top of the object.



%figure
![Screenshot_from_2020-06-11_10-34-54.png](https://cdn.discordapp.com/attachments/565155651395780609/720557335977787433/Screenshot_from_2020-06-11_10-34-54.png)
%end


Hi `@David Mansolino` sorry, I did not explain well...  My problem is that I actually use the objects shown in the previous picture. With the bounding object as their own shape. But when my robot has to climb them, sometimes the wheel go through the object instead of climb it. It seems that the bounding object is not here.



I actually use webots nightls R2020b 9\_6\_2020

##### David Mansolino [cyberbotics] 06/11/2020 04:57:15
Hi `@lojik` what is exactly the problem you have with the bounding object,?We can't see them on this picture.

##### lojik 06/10/2020 16:48:44
Here you have the kind of objects I imported. These are kind of sinus waves. I put a bit of each pieces down the floor to have smaller mountains.



%figure
![Screenshot_from_2020-06-10_18-47-22.png](https://cdn.discordapp.com/attachments/565155651395780609/720318368837533766/Screenshot_from_2020-06-10_18-47-22.png)
%end


Hello there, I imported a 3d object in webots by using webots 2020b. It happen frequently that an object is suddently not takent into account with contacts. It seems that there is some problems with the bounding object.



Could it be a bug on this nightly version or is it a problem on my side?

##### David Mansolino [cyberbotics] 06/09/2020 11:35:20
You can either use physics plugins: 

[https://cyberbotics.com/doc/reference/physics-plugin](https://cyberbotics.com/doc/reference/physics-plugin)

Either extend Webots with new nodes:

[https://github.com/cyberbotics/webots/blob/master/CONTRIBUTING.md](https://github.com/cyberbotics/webots/blob/master/CONTRIBUTING.md)

##### hrsh12 06/09/2020 11:33:40
Hi, `@David Mansolino` , To make a rack and pinion arrangement, i require more kind of joints than those that are available in the documentation. Can i model my own basic joints?

##### David Mansolino [cyberbotics] 06/05/2020 05:16:30
We plan to add a new feature which will allow to link joint together and which might help you doing this ([https://github.com/cyberbotics/webots/issues/1365](https://github.com/cyberbotics/webots/issues/1365)).


Hi `@hrsh12`, this is not possible directly, however you can combine several base joint to make such system.

##### hrsh12 06/04/2020 21:23:54
Hello everyone, could someone tell me if we can implement a rack and pinion system for steering in Car. For the jointsI'm trying to recreate a  steer-by-wire system. Thanks

##### David Mansolino [cyberbotics] 06/02/2020 10:51:52
You're welcome

##### lojik 06/02/2020 10:51:08
Perfect, thank you 👍

##### David Mansolino [cyberbotics] 06/02/2020 10:50:28
Hi, yes this is indeed a known bug, it was fixed very recently, you can already download a beta of R2020a-revision2 which incudes the fix here: [https://github.com/cyberbotics/webots/releases](https://github.com/cyberbotics/webots/releases)

##### lojik 06/02/2020 10:48:45
Hello everyone, I do not know if someone else have the same problem as me but it is easy to verify.



I am on ubuntu 18.04, using webots R2020a-revision1. When I right-click to add new features --> search in the 'find' box and type an underscore '\_' Webots crashes.



Do someone has the same issue?

##### David Mansolino [cyberbotics] 06/02/2020 05:54:32
> is there any depth camera that i can use for skeletal tracking?

`@Sergen Aşık` yes of course, you can use range-finders: [https://www.cyberbotics.com/doc/reference/rangefinder](https://www.cyberbotics.com/doc/reference/rangefinder)


> How can i run simulator using Qt Creator?

`@Sergen Aşık` do you want to start Webots from Qt Creator, or use Qt in your controller?


The supervisor also send plume information to every robot using emitter-receivers.


> Hi! Webots noob here, could someone tell me how to add an odor plume in the simulation and make the robot follow it? (Kinda trying to recreate this : [https://en.wikibooks.org/wiki/Webots\_Odor\_Simulation](https://en.wikibooks.org/wiki/Webots_Odor_Simulation))

`@Ans` this is simply emulated from a Supervisor controller that imports shapes to visualize the plume.


> does anyone have implemented some kind of pathfinding and can help me ? pls dm

`@Wesst` you will find some examples here: [https://en.wikibooks.org/wiki/Cyberbotics%27\_Robot\_Curriculum/Advanced\_Programming\_Exercises#Path\_planning\_](https://en.wikibooks.org/wiki/Cyberbotics%27_Robot_Curriculum/Advanced_Programming_Exercises#Path_planning_)[Advanced]

##### Sergen Aşık 05/31/2020 22:28:06
is there any depth camera that i can use for skeletal tracking?


How can i run simulator using Qt Creator?

##### Ans 05/30/2020 20:39:57
Hi! Webots noob here, could someone tell me how to add an odor plume in the simulation and make the robot follow it? (Kinda trying to recreate this : [https://en.wikibooks.org/wiki/Webots\_Odor\_Simulation](https://en.wikibooks.org/wiki/Webots_Odor_Simulation))

##### Wesst 05/30/2020 16:24:35
does anyone have implemented some kind of pathfinding and can help me ? pls dm

##### Iguins 05/29/2020 05:37:04
thank you


I was afraid of that

##### David Mansolino [cyberbotics] 05/29/2020 05:36:28
You can, but heightmap is much more efficient and stable than random indexedFaceSet.

##### Iguins 05/29/2020 05:35:32
do you think that instead of a heightmap I could use a custom mesh to also change the x and z values?

##### David Mansolino [cyberbotics] 05/29/2020 05:33:58
You're welcome

##### Iguins 05/29/2020 05:33:39
nice thank you!

##### David Mansolino [cyberbotics] 05/29/2020 05:32:59
In that case the robots should be Supervisors and they will be able to change the height values.

##### Iguins 05/29/2020 05:31:28
I wanted the robots to modify an uneven terrain

##### Clara Cardoso Ferreira 05/29/2020 05:20:52
Ill give it a try, thank you David!

##### David Mansolino [cyberbotics] 05/29/2020 05:06:52
For the whole moon, it is indeed going to be slow, but if you import only a reasonable sub-section, the performance should be ok. See this example: [https://cyberbotics.com/doc/automobile/ch-vens](https://cyberbotics.com/doc/automobile/ch-vens)

##### Clara Cardoso Ferreira 05/29/2020 05:05:26
with real data


so that my robot can learn how to navigate in unknown terrain


I could import the whole moon, but I plan to have at least sections of the moon


For my case, it changes every meter. I am trying to import the map of the moon to Webots

##### David Mansolino [cyberbotics] 05/29/2020 05:03:20
Will the terrain change at each step ?

##### Iguins 05/28/2020 22:10:01
I was facing that same problem

##### Clara Cardoso Ferreira 05/28/2020 21:34:45
My guess would be that his solution would work, but it would probably be extremely slow since you would have to generate each individual slide of the terrain at every render pass


> One possible solution would be to create a procedural PROTO ([https://www.cyberbotics.com/doc/reference/procedural-proto-nodes](https://www.cyberbotics.com/doc/reference/procedural-proto-nodes)) which will read the float image and generate an ElevationGrid ([https://www.cyberbotics.com/doc/reference/elevationgrid](https://www.cyberbotics.com/doc/reference/elevationgrid)) from it.

`@David Mansolino`

##### AngelAyala 05/28/2020 06:54:54
I wasn't considering as real context, thanks


ok I get it, both running in different controllers

##### Stefania Pedrazzi [cyberbotics] 05/28/2020 06:53:39
What you can do is to send the GPS data from the Mavic2Pro controller to the supervisor controller using for example the Emitter/Receiver devices


This is like in real context.


if yes, then you cannot read the Mavic2Pro GPS device values from another robot/supervisor.

##### AngelAyala 05/28/2020 06:52:31
yes

##### Stefania Pedrazzi [cyberbotics] 05/28/2020 06:52:23
Is the Supervisor controller different from the Mavic2Pro controller?

##### AngelAyala 05/28/2020 06:51:35
using python


is there anyway to do this?


now, how can I access to the GPS node inside the Mavic2Pro PROTO using the supervisor node?

##### David Mansolino [cyberbotics] 05/28/2020 06:20:42
You're welcome

##### AngelAyala 05/28/2020 06:20:33
oh greats, thanks

##### David Mansolino [cyberbotics] 05/28/2020 06:20:00
but you can simply check the return value


this s indeed just a warning which doesnt  raise any exception

##### AngelAyala 05/28/2020 06:19:11
is maybe a especial ExceptionError?


I'm currently using, but when appears this error Error: wb\_gps\_enable(): invalid device tag, the code is unable to catch it

##### David Mansolino [cyberbotics] 05/28/2020 06:18:04
you can always use a  `try` and `except` on the sensitive part

##### AngelAyala 05/28/2020 06:15:18
Is there anyway to catch the runtime errors in python in order to stop and reset the simulation? I can control the execution of the simulation with Supervisor node

##### David Mansolino [cyberbotics] 05/28/2020 05:25:26
You will find an example of this here: [https://gist.github.com/DavidMansolino/46d7e0df4c48bbaac4611b3872878347](https://gist.github.com/DavidMansolino/46d7e0df4c48bbaac4611b3872878347)


One possible solution would be to create a procedural PROTO ([https://www.cyberbotics.com/doc/reference/procedural-proto-nodes](https://www.cyberbotics.com/doc/reference/procedural-proto-nodes)) which will read the float image and generate an ElevationGrid ([https://www.cyberbotics.com/doc/reference/elevationgrid](https://www.cyberbotics.com/doc/reference/elevationgrid)) from it.

##### Clara Cardoso Ferreira 05/27/2020 18:21:49
[https://pgda.gsfc.nasa.gov/products/54](https://pgda.gsfc.nasa.gov/products/54)

Im trying to retrieve height maps from the above data and import it to Webots to create a custom terrain. I am not sure if this is possible and easy to do. Please provide me some guidance.



I was thinking float img files formats would work best since I could import it to Opend CV if necessary.

##### Axel M [Premier Service] 05/27/2020 16:43:13
Review / PR / issues are welcome 🙂


I ported the Atom PROTO extension to VSCode : [https://marketplace.visualstudio.com/items?itemName=pymzor.language-proto-webots](https://marketplace.visualstudio.com/items?itemName=pymzor.language-proto-webots)

##### David Mansolino [cyberbotics] 05/25/2020 06:25:16
You can either convert your car node to base node (right click on the node in the scene-tree) and then you will be able to change the structure, either change directly the `Car.proto` file.

##### hrsh12 05/24/2020 21:21:56
Is there a way of modifying the usual steering system used in car models?

##### ContrastNull 05/22/2020 16:03:44
Alright, thank you for your understanding!

##### Olivier Michel [cyberbotics] 05/22/2020 16:02:54
OK, then you should probably run Webots in the cloud and stream your simulation to the web.

##### ContrastNull 05/22/2020 16:01:57
I am a competitor, sir.

##### Olivier Michel [cyberbotics] 05/22/2020 16:00:06
Are you a competitor or an organizer/developer of this contest?

##### ContrastNull 05/22/2020 15:50:50
`@Olivier Michel` , would you please take a look at this. So it will be easy to help me out.

[http://www.aerialroboticscompetition.org/assets/downloads/simulation\_challenge\_rules\_1.1.pdf](http://www.aerialroboticscompetition.org/assets/downloads/simulation_challenge_rules_1.1.pdf)

Go for " Challenge Rules - 1 ".

##### Olivier Michel [cyberbotics] 05/22/2020 15:46:11
This is not possible.

##### ContrastNull 05/22/2020 15:45:40
So could you tell me how can I make this static scene to moving? (by using my controller code of course).

No, as I said, its none of these two cases.

##### Olivier Michel [cyberbotics] 05/22/2020 15:41:18
Exporting a to HTML will simply export a static scene (no motion), so you probably want to either export an animation or stream a live simulation?

##### ContrastNull 05/22/2020 15:37:09
Yes, I have already read it. I need to simulate my world on a web browser using HTML file. I mean anyone with the file can open the simulation on the web. Server streaming or already recorded animation isn't the case.

##### Olivier Michel [cyberbotics] 05/22/2020 15:32:27
Please refer to the user guide: [https://cyberbotics.com/doc/guide/web-interface](https://cyberbotics.com/doc/guide/web-interface)


Do you want to export an animation or to stream a live simulation?


That's because you exported to HTML but you are not streaming the simulation.

##### ContrastNull 05/22/2020 15:28:54
`@Olivier Michel` you see, no other option or window opens. neither the drone moves. But in webots, it moves. Any suggestions?
%figure
![Screenshot_from_2020-05-22_20-57-46.png](https://cdn.discordapp.com/attachments/565155651395780609/713413087914491904/Screenshot_from_2020-05-22_20-57-46.png)
%end

##### Olivier Michel [cyberbotics] 05/22/2020 14:27:44
If your robots move in Webots, they should also move in the web view. Don't they?

##### ContrastNull 05/22/2020 13:18:06
Hey, I was trying to simulate a world on a HTTP web page. But I couldn't link the controller to the world. I can only move the viewpoint. How should I link the controllers to that file? So that the robots in my simulation should work.

##### Olivier Michel [cyberbotics] 05/22/2020 06:48:43
Please open an issue on [https://github.com/cyberbotics/urdf2webots/issues/new](https://github.com/cyberbotics/urdf2webots/issues/new) and upload your URDF file as a ZIP file.

##### ChapoGuzman 05/21/2020 23:44:29
Thanks in advance


I did a good install because I tried the "Human" file and it worked correctly



%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/713174651731181749/unknown.png)
%end


But i have this error


Hi guys im using the urdf package to convert my files between solidworks and webots

##### Alfian 05/21/2020 04:09:02
There is no controller for lock position of the DJI Mavic 2 Pro

##### David Mansolino [cyberbotics] 05/15/2020 04:57:11
`@ContrastNull` the controller of the drone is very simple, it doesn't do any feedback using the GPS position or any inertial unit, it might therefore easily drift.


`@Jesusmd` you want to get the type of distance sensor? If so you should use the ``getType`` function: [https://cyberbotics.com/doc/reference/distancesensor?tab-language=python#wb\_distance\_sensor\_get\_type](https://cyberbotics.com/doc/reference/distancesensor?tab-language=python#wb_distance_sensor_get_type)

##### ContrastNull 05/15/2020 03:02:49
Hey, may I know, why does a drone, (let it be DJI Mavic 2 Pro, which is already available into Webots, or a custom made) moves up and down and shifts constantly in one direction even if no such behaviour is defined in the controller code?

##### Jesusmd 05/15/2020 01:49:44
`@David Mansolino` Hi, I am using python, I would like to work with several distance sensors at the same time and comparing their data in console. But instead of obtain the type, I got a number.

##### Simon Steinmann [Moderator] 05/14/2020 12:48:46
> `@Simon Steinmann` Although working with ROS, we decided to split the simulation projects with minimal dependencies over ROS other than communication. For 3D Math in Cpp, we're using Eigen, and in python either numpy or numpy + transformations.py (which is standalone of tf)

`@Axel M` 



Awesome, thank you so much. That makes things easier

##### Axel M [Premier Service] 05/14/2020 09:11:01
The integration with ROS is super easy too with [http://wiki.ros.org/eigen\_conversions](http://wiki.ros.org/eigen_conversions) 🙂


Regarding your example of getting relative position between two nodes, that can be easily achieved with Eigen in cpp (3x3 Matrix -> Quaternion, quaternion / vector product)


`@Simon Steinmann` Although working with ROS, we decided to split the simulation projects with minimal dependencies over ROS other than communication. For 3D Math in Cpp, we're using Eigen, and in python either numpy or numpy + transformations.py (which is standalone of tf)

##### David Mansolino [cyberbotics] 05/14/2020 05:34:23
`@Simon Steinmann` instead of relying on ROS, if you are using Python you ca probably use the `transforms3d` python package which allows for example to convert from a rotation matrix to quaternions: [https://matthew-brett.github.io/transforms3d/reference/transforms3d.quaternions.html#transforms3d.quaternions.mat2quat](https://matthew-brett.github.io/transforms3d/reference/transforms3d.quaternions.html#transforms3d.quaternions.mat2quat)

##### Simon Steinmann [Moderator] 05/13/2020 18:04:16
`@ContrastNull`  perhaps direct .proto file edit can help

##### ContrastNull 05/13/2020 17:11:36
`@Simon Steinmann` that is what my problem is about. The nodes I have to copy paste one by one are in "hundreds".

##### Simon Steinmann [Moderator] 05/13/2020 16:47:12
Perhaps add a 'Group' base node, put all your nodes in, and copy paste that

##### ContrastNull 05/13/2020 16:45:26
Oh, thank you for letting me know!

Keep it as a suggestion for the next update. 😄

##### Olivier Michel [cyberbotics] 05/13/2020 16:44:11
Unfortunately, this is not possible.

##### ContrastNull 05/13/2020 16:43:42
Hey, could you tell me how to multi-select things in scene-tree? 

I can't find a way to. I imported a VRML97 model into Webots, the model was quite large, and it's difficult to select, cut and paste each 'transform' object into a robot node's children attribute.

##### Simon Steinmann [Moderator] 05/13/2020 16:21:05
I'll give it a try


oh lordy 😅

##### Olivier Michel [cyberbotics] 05/13/2020 16:20:18
Yes, if you have an idea to achieve this... Now you know how to open a pull request 😉

##### Simon Steinmann [Moderator] 05/13/2020 16:19:08
yes, but it requires runtime.ini edits. Would be nice if people can have it run as an example out of the box

##### Olivier Michel [cyberbotics] 05/13/2020 16:18:13
If you use Webots with ROS, you need to install ROS, and you should get 'tf' for free, isn't it?

##### Simon Steinmann [Moderator] 05/13/2020 16:16:44
it is almost a must if working with ROS and robots


how big of a deal would it be to add the 'tf' package to the webots ROS environment?

##### Olivier Michel [cyberbotics] 05/13/2020 16:15:26
Oops...

##### Simon Steinmann [Moderator] 05/13/2020 16:15:08
Problem is, the link to their liscense is broken 😅


[https://sscc.nimh.nih.gov/pub/dist/bin/linux\_gcc32/meica.libs/nibabel/quaternions.py](https://sscc.nimh.nih.gov/pub/dist/bin/linux_gcc32/meica.libs/nibabel/quaternions.py)


specifically this:


[https://nipy.org/nibabel/reference/nibabel.quaternions.html](https://nipy.org/nibabel/reference/nibabel.quaternions.html)


The thing is, I want it to be able to run in the native webots environment, so no extra packages. I got it to work using their code:

##### Olivier Michel [cyberbotics] 05/13/2020 16:13:18
OK, so you need to convert this 3x3 rotation matrix to a quaternion then.


Yes, you are right. Sorry, my bad.

##### Simon Steinmann [Moderator] 05/13/2020 16:12:12
but that returns the 3x3 matrix

##### Olivier Michel [cyberbotics] 05/13/2020 16:11:40
If you need absolute rotation (in axis-angle notation), use:  [https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_node\_get\_orientation](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_orientation)


(gives relative rotation in axis-angle notation)


[https://www.cyberbotics.com/doc/reference/supervisor#wb\_supervisor\_field\_get\_sf\_rotation](https://www.cyberbotics.com/doc/reference/supervisor#wb_supervisor_field_get_sf_rotation)

##### Simon Steinmann [Moderator] 05/13/2020 16:09:36
how do we get axis angles? I have the 3x3 matrix

##### Olivier Michel [cyberbotics] 05/13/2020 16:09:02
```python
def axis_angle_to_quaternion(axis, theta):
    axis = numpy.array(axis) / numpy.linalg.norm(axis)
    return numpy.append([numpy.cos(theta/2)], numpy.sin(theta/2) * axis)
```


No, you have to get it as an axis-angle representation, but that's super easy to translate into quaternion.

##### Simon Steinmann [Moderator] 05/13/2020 16:04:22
Btw, is there a way to get the quaternion orientation directly from webots? That would make things much simpler


Still a bit crude with little error correction, but it works 🙂


<@&568329906048598039> Btw, I'm currently working on a python program, that publishes the PoseStamped of one or more nodes into a rostopic. It also allows to publish the Pose relative to a specific node.

##### David Mansolino [cyberbotics] 05/11/2020 06:43:54
> Is there an quickly way to get the attribute instead of  predefined values as for example in  Keyboard class.?  I would preffer enum as in c or c++

`@Jesusmd` which language are you using?

##### Jesusmd 05/10/2020 06:29:26
Is there an quickly way to get the attribute instead of  predefined values as for example in  Keyboard class.?  I would preffer enum as in c or c++

##### Simon Steinmann [Moderator] 04/29/2020 22:31:07
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


Hi `@ContrastNull`, the VRML import does indeed import only the visual meshes of the object, you then have to recreate the structure of the object yourself. I would recommend to follow these tutorials to create an objet (1 to 7): [https://cyberbotics.com/doc/guide/tutorials](https://cyberbotics.com/doc/guide/tutorials)

##### ContrastNull 04/12/2020 06:30:27
Hey, I tried importing a door hinge as a VRML97 file, which was converted from Solidworks model. 

The problem is, it is considering each part of the hinge, like screws, doors as separate individual. (As in the scene tree, it is showing each part differently with same name - "transform") 

Neither it is considered as other objects in the world's, because while I tried to click on the hinge, it won't click and show it's origin and all. 

What must be the problem here?


Sure,  I will look into them and let you know if it helps! 😊

##### David Mansolino [cyberbotics] 04/09/2020 12:49:19
Did you follow our tutorials? If not I would strongly advice to follow tutorials 1 to 6: [https://cyberbotics.com/doc/reference/supervisor](https://cyberbotics.com/doc/reference/supervisor)


You can either use the Supervisor API to change it's rotation field either mount the radar on a controllable joint.

##### ContrastNull 04/09/2020 12:43:17
`@David Mansolino`,  in the first question, after I imported the model into webots,  how should I define the movement of that radar part as needed?

##### David Mansolino [cyberbotics] 04/09/2020 12:38:42
About the second issue, I would use the Supervisor API to move the whole mast: [https://cyberbotics.com/doc/reference/supervisor](https://cyberbotics.com/doc/reference/supervisor)


About the first issue, you mean you don't now how to make the radar moving ?

##### ContrastNull 04/09/2020 12:35:55
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
In Python I personnally often use the ``transforms3d``module (available on PIP: [https://pypi.org/project/transforms3d](https://pypi.org/project/transforms3d)) which allows to convert between many 3D representations, you will probably find what you need in the doc: [http://matthew-brett.github.io/transforms3d/](http://matthew-brett.github.io/transforms3d/)

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
I have acquired an EZgripper ([https://sakerobotics.com/](https://sakerobotics.com/)) for my robot. What's the best way to simulate such an under actuated gripper? I've written a small Python controller, but I can't really make it grasp properly. Any ideas? I don't really need the physics aspect of the grasping itself, just being able to move objects around.

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

- samples/devices/motor.wbt ([https://cyberbotics.com/doc/guide/samples-devices#motor-wbt](https://cyberbotics.com/doc/guide/samples-devices#motor-wbt))

- samples/devices/motor2.wbt ([https://cyberbotics.com/doc/guide/samples-devices#motor2-wbt](https://cyberbotics.com/doc/guide/samples-devices#motor2-wbt))

- samples/devices/motor3.wbt ([https://cyberbotics.com/doc/guide/samples-devices#motor3-wbt](https://cyberbotics.com/doc/guide/samples-devices#motor3-wbt))

- samples/devices/linear\_motor.wbt ([https://cyberbotics.com/doc/guide/samples-devices#linear\_motor-wbt](https://cyberbotics.com/doc/guide/samples-devices#linear_motor-wbt))


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

##### pavlos27 11/13/2019 12:46:39

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/644156174123532351/unknown.png)
%end


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

##### pavlos27t 11/06/2019 18:54:11

%figure
![unknown.png](https://cdn.discordapp.com/attachments/565155651395780609/641711950677409813/unknown.png)
%end


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
`@ThePolymath` Hi, I expect you mean to create a webots controller inside an IDE, such as XCode. Could you refer to this draft page of the documentation? [https://cyberbotics.com/doc/guide/using-your-ide?version=enhancement-ide-section](https://cyberbotics.com/doc/guide/using-your-ide?version=enhancement-ide-section)

##### ThePolymath 09/29/2019 10:08:43
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
You can find another example in samples/howto/omni\_wheels.wbt simulation:  [https://www.cyberbotics.com/doc/guide/samples-howto#omni\_wheels-wbt](https://www.cyberbotics.com/doc/guide/samples-howto#omni_wheels-wbt). In this case the wheel is simulated using two layers of joints and cylinders.


Hi `@SimonDK`, the youBot robot has mecanum wheels [https://www.cyberbotics.com/doc/guide/youbot](https://www.cyberbotics.com/doc/guide/youbot)

The mecanum wheel is simulated by setting asymmetric friction  to the [ContactProperties]([https://www.cyberbotics.com/doc/reference/contactproperties](https://www.cyberbotics.com/doc/reference/contactproperties)) of the wheel.

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



> **Attachment**: [city\_2.mp4](https://cdn.discordapp.com/attachments/565155651395780609/613680130568617984/city_2.mp4)

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

##### BlackPearl 08/21/2019 09:57:46

%figure
![image0.png](https://cdn.discordapp.com/attachments/565155651395780609/613673091335323648/image0.png)
%end

##### Fabien Rohrer [Moderator] 08/21/2019 09:56:07
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

##### BlackPearl 06/19/2019 08:51:47

%figure
![image0.jpg](https://cdn.discordapp.com/attachments/565155651395780609/590826051312025620/image0.jpg)
%end


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



%figure
![webotsVR03.png](https://cdn.discordapp.com/attachments/565155651395780609/588391025089708196/webotsVR03.png)
%end



%figure
![webotsVR02.png](https://cdn.discordapp.com/attachments/565155651395780609/588391019444174858/webotsVR02.png)
%end

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

##### TH0 06/12/2019 15:10:10

%figure
![webotsVR01.png](https://cdn.discordapp.com/attachments/565155651395780609/588384559884926987/webotsVR01.png)
%end


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

##### Arsalan.b.r 06/04/2019 07:49:01

%figure
![tractor-trailer.JPG](https://cdn.discordapp.com/attachments/565155651395780609/585374436270800906/tractor-trailer.JPG)
%end

##### Fabien Rohrer [Moderator] 06/04/2019 07:48:04
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

