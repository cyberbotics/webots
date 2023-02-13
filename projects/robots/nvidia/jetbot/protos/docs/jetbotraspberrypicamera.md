JetBot is an open-source robot based on NVIDIA Jetson Nano.
The robot is an affordable two-wheeled robot distributed as a DIY kit.

More information on the JetBot robot can be found on this [website](https://jetbot.org).

### JetBot PROTO Nodes

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
- `cameraSlot`:  Extends the robot with a camera on the front.
- `extensionSlot`: Extends the robot with new nodes.

#### JetBotRaspberryPiCamera PROTO

Derived from [Transform](https://cyberbotics.com/doc/reference/transform).
It contains a [Camera](https://cyberbotics.com/doc/reference/camera) device.

%figure

![JetBotRaspberryPiCamera.png](images/jetbot/JetBotRaspberryPiCamera.thumbnail.png)

%end

```
JetBotRaspberryPiCamera {
  SFVec3f      translation        0 0 0
  SFRotation   rotation           0 0 1 0
  SFString     name               "camera"
  SFFloat      fieldOfView        2.79253
  SFInt32      width              1280
  SFInt32      height             720
  SFFloat      near               0.01
  SFFloat      far                1.0
}
```

##### JetBotRaspberryPiCamera Field Summary

- `name`: Defines the [Camera.name](https://cyberbotics.com/doc/reference/camera) value.
- `fieldOfView`:  Defines the [Camera.fieldOfView](https://cyberbotics.com/doc/reference/camera) value.
- `width`: Defines the [Camera.width](https://cyberbotics.com/doc/reference/camera) value.
- `height`: Defines the [Camera.height](https://cyberbotics.com/doc/reference/camera) value.
- `near`: Defines the [Camera.near](https://cyberbotics.com/doc/reference/camera) value.
- `far`: Defines the [Camera.far](https://cyberbotics.com/doc/reference/camera) value.

### Sample

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/nvidia/jetbot/worlds]({{ url.github_tree }}/projects/robots/nvidia/jetbot/worlds)".
An additional example using PyTorch CNN to perform collision avoidance is available [here](https://github.com/cyberbotics/webots-projects/blob/projects/nvidia-jetbot-collision-avoidance).

#### jetbot.wbt

![jetbot.wbt.png](images/jetbot/jetbot.wbt.thumbnail.jpg) In this example, you can see a JetBot robot performing basic motion.
The `jetbot` C controller replicates the basic motion provided by the official ["Basic Motion"](https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/notebooks/basic_motion/basic_motion.ipynb) Jupyter notebook.
