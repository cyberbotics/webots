## NVIDIA JetBot

%robot jetbot images/robots/jetbot/model.thumbnail.png

JetBot is an open-source robot based on NVIDIA Jetson Nano.
The robot is an affordable two-wheeled robot distributed as DIY kit.

More information on the JetBot robot can be found on this [website](https://jetbot.org/v0.4.3/).

### JetBot PROTO

Derived from [Robot](../reference/robot.md).

```
JetBot {
  SFVec3f      translation        0 0 0
  SFRotation   rotation           0 1 0 0
  SFString     name               "JetBot"
  SFString     controller         "jetbot"
  MFString     controllerArgs     []
  SFString     customData         ""
  SFBool       supervisor         FALSE
  SFBool       synchronization    TRUE
  SFColor      chassis_color      0.571 0.817 0.479
  SFFloat      camera_fieldOfView 2.79253
  SFInt32      camera_width       1280
  SFInt32      camera_height      720
  SFFloat      camera_near        0.01
  SFFloat      camera_far         1.0
  MFNode       extensionSlot      []
  SFString     window             "generic"
}
```

> **File location**: "[WEBOTS\_HOME/projects/robots/nvidia/jetbot/protos/JetBot.proto]({{ url.github_tree }}/projects/robots/nvidia/jetbot/protos/JetBot.proto)"

#### JetBot Field Summary

- `chassis_color`:  Defines the color of the robot's chassis.
- `camera_fieldOfView`:  Defines the `fieldOfView` field of the [Camera](../reference/camera.md).
- `camera_width`: Defines the `width` field of the [Camera](../reference/camera.md).
- `camera_height`: Defines the `height` field of the [Camera](../reference/camera.md).
- `camera_near`: Defines the `near` field of the [Camera](../reference/camera.md).
- `camera_far`: Defines the `far` field of the [Camera](../reference/camera.md).
- `extensionSlot`: Extends the robot with new nodes.

### Sample

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/nvidia/jetbot/worlds]({{ url.github_tree }}/projects/robots/nvidia/jetbot/worlds)".

#### jetbot.wbt

![jetbot.wbt.png](images/robots/jetbot/jetbot.wbt.thumbnail.jpg) This simulation shows a JetBot TODO.
