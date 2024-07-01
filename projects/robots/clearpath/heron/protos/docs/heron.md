The [clearpath's Heron USV](https://robots.ros.org/clearpath-heron-usv/) is a USV platform.

### Motors

  There are 2 engines, engine name ```left_motor```and ```right_motor``.
  You can add new Camera, GPS or other sensors you want under ``body_slot```.

### Movie Presentation

![youtube video](https://www.youtube.com/watch?v=qWRyCnJWVuM)

### Heron PROTO

Derived from [Robot](https://cyberbotics.com/doc/reference/robot).

```
Heron {
  SFVec3f    translation     0 0 0
  SFRotation rotation        0 1 0 1.5708
  SFString   name            "heron"
  SFString   controller      "heron_usv_controller"
  MFString   controllerArgs  []
  SFBool     synchronization TRUE
  SFColor    color           0.8 0.5 0.1
  MFNode     bodySlot        []
}
```

#### Heron Field Summary

- `bodySlot`: Extends the robot with new nodes.

### Samples

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/clearpath/heron/worlds]({{ url.github_tree }}/projects/robots/clearpath/heron/worlds)".

#### [heron\ocean.wbt]({{ url.github_tree }}/projects/robots/clearpath/moose/worlds/heron\ocean.wbt)

![swarm.png](images/heron/swarm.jpg) This simulation shows a basicly swarm robotics.
