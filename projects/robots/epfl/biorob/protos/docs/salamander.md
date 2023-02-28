The "Salamander" robot is a salamander-like robot developed by the [EPFL BioRob laboratory](https://biorob.epfl.ch/).

### Salamander PROTO

Derived from [Robot](https://cyberbotics.com/doc/reference/robot).

```
Salamander {
  SFVec3f    translation     0 0 0.044
  SFRotation rotation        0 0 1 0
  SFString   name            "Salamander"
  SFString   controller      "salamander"
  MFString   controllerArgs  []
  SFString   customData      ""
  SFBool     supervisor      FALSE
  SFBool     synchronization TRUE
  MFNode     extensionSlot   []
}
```

#### Salamander Field Summary

- `extensionSlot`: Extends the robot with new nodes in the extension slot.

### Samples

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/epfl/biorob/worlds]({{ url.github_tree }}/projects/robots/epfl/biorob/worlds)".

#### [salamander.wbt]({{ url.github_tree }}/projects/robots/epfl/biorob/worlds/salamander.wbt)

![salamander.wbt.png](images/salamander/salamander.wbt.thumbnail.jpg) A salamander-shaped robot walks down a slope and reaches a pool where it starts to swim.
The controller uses two different types of locomotion: it walks on the ground and swims in the water.
This demo simulates propulsive forces caused by the undulations of the body and the resistance caused by the robot's shape.
In addition, the buoyancy of the robot's body is also simulated using Archimedes' principle.
