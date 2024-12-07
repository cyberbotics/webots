The [Blue Robotics BlueBoat USV](https://bluerobotics.com/product-category/boat/) is a USV platform.

### Motors

There are 2 motor, motor name `left_motor`and `right_motor`.
You can add new Camera, GPS or other sensors you want under `body_slot`.

### Movie Presentation

![youtube video](https://youtu.be/kcKopzJZTEM)

### BlueBoat PROTO

Derived from [Robot](https://cyberbotics.com/doc/reference/robot).

```js
BlueBoat {
  SFVec3f    translation     0 0 0
  SFRotation rotation        0 1 0 1.5708
  SFString   name            "blueboat"
  SFString   controller      "blueboat_controller"
  MFString   controllerArgs  []
  SFBool     synchronization TRUE
  MFNode     bodySlot        []
}
```

#### BlueBoat Field Summary

- `bodySlot`: Extends the robot with new nodes.

### Samples

You will find the following sample in this folder: "[WEBOTS\_HOME/projects/robots/blue_robotics/blueboat/worlds]({{ url.github_tree }}/projects/robots/blue_robotics/blueboat/worlds)".

Ocean

#### [blueboat\ocean.wbt]({{ url.github_tree }}/projects/robots/blue_robotics/blueboat/ocean.wbt)

![ocean.png](images/blueboat/ocean.jpg) This simulation shows a basicly usv.
