JetBot is an open-source robot based on NVIDIA Jetson Nano.
The robot is an affordable two-wheeled robot distributed as a DIY kit.

More information on the JetBot robot can be found on this [website](https://jetbot.org).

#### JetBot PROTO

Derived from [Robot](https://cyberbotics.com/doc/reference/robot).

```
JetBot {
  SFVec3f      translation        0 0 0
  SFRotation   rotation           0 0 1 0
  SFString     name               "JetBot"
  SFString     controller         "jetbot_basic_motion"
  MFString     controllerArgs     []
  SFString     customData         ""
  SFBool       supervisor         FALSE
  SFBool       synchronization    TRUE
  SFBool       chassis            TRUE
  SFColor      chassis_color      0.571 0.817 0.479
  SFNode       cameraSlot         JetBotRasperryPiCamera  {
                                    rotation 0 1 0 0.316799
                                  }
  MFNode       extensionSlot      []
  SFString     window             "<generic>"
}
```

##### JetBot Field Summary

- `chassis`: Defines if the robot has a chassis.
- `chassis_color`:  Defines the color of the robot's chassis.
- `cameraSlot`:  Extends the robot with a camera on the front. For example the [JetBotRaspberryPiCamera](https://webots.cloud/run?url=https://github.com/cyberbotics/webots/blob/released/projects/robots/nvidia/jetbot/protos/JetBotRaspberryPiCamera.proto)
- `extensionSlot`: Extends the robot with new nodes.

### Sample

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/nvidia/jetbot/worlds]({{ url.github_tree }}/projects/robots/nvidia/jetbot/worlds)".
An additional example using PyTorch CNN to perform collision avoidance is available [here](https://github.com/cyberbotics/webots-projects/tree/master/projects/nvidia-jetbot-collision-avoidance).

#### [jetbot.wbt]({{ url.github_tree }}/projects/robots/nvidia/jetbot/worlds/jetbot.wbt)

![jetbot.wbt.png](images/jetbot/jetbot.wbt.thumbnail.jpg) In this example, you can see a JetBot robot performing basic motion.
The `jetbot` C controller replicates the basic motion provided by the official ["Basic Motion"](https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/notebooks/basic_motion/basic_motion.ipynb) Jupyter notebook.
