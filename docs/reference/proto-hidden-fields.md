## PROTO Hidden Fields

Regular PROTO fields let you change, save and restore, chosen characteristics of your model.
In contrast, PROTO encapsulation prevents field values which are not accessible through PROTO fields, but which may change during simulation, from being saved and subsequently restored.
Still, this is not true for all field values, since Webots saves for you hidden PROTO fields which are bound to change over simulation time.
Namely the `translation` and `rotation` fields of [Solid](solid.md) nodes as well as the `position` fields of [Joint](joint.md) nodes are saved as hidden PROTO fields in the field scope of every top-level PROTO.
In case of [solid merging](physics.md#implicit-solid-merging-and-joints), note that hidden `translation` and `rotation` fields are saved only for the [Solid](solid.md) placed at the top of the solid assembly.

As in the case of non-PROTO objects, initial velocities of physical subparts of a PROTO are saved and can be subsequently restored when reloading your world file.
Like the other hidden fields, velocities are saved in the field scope of every top-level PROTO.

Each hidden field appends an index to its name which encodes the location of the [Solid](solid.md) to which it belongs inside the tree hierarchy rooted at the PROTO node.
This index corresponds is the depth-first pre-order traversal index of the [Solid](solid.md) in this tree.
If a hidden field corresponds to the `position` of [Joint](joint.md), an additional index is appended to its name, namely the index of the [Joint](joint.md) in the list of [Joint](joint.md) nodes originating from the [Solid](solid.md) sorted by means of pre-order traversal.
As an example, we display below an excerpt of "projects/robots/pioneer/pioneer3at/worlds/pioneer3at.wbt" when saved after one simulation step.

```
DEF PIONEER_3AT Pioneer3at {
  hidden position_0_0 -2.88177e-07
  hidden position_0_1 -4.63679e-07
  hidden position_0_2 -3.16282e-07
  hidden position_0_3 -4.91785e-07
  hidden linearVelocity_0 -0.00425142 -0.0284347 0.0036279
  hidden angularVelocity_0 0.0198558 -9.38102e-07 0.0232653
  hidden translation_2 -0.197 4.04034e-06 0.1331
  hidden rotation_2 -0.013043 0.00500952 0.999902 -2.88177e-07
  hidden linearVelocity_2 -0.00255659 -0.0214607 0.00218164
  hidden angularVelocity_2 0.0198598 -9.84272e-07 0.0232733
  hidden translation_3 0.197 7.85063e-06 0.1331
  hidden rotation_3 -0.00949932 0.00367208 0.999948 -4.63679e-07
  hidden linearVelocity_3 -0.00255694 -0.0330774 0.00218161
  hidden angularVelocity_3 0.0198623 -9.92987e-07 0.0232782
  hidden translation_4 -0.197 4.64107e-06 -0.1331
  hidden rotation_4 -0.011884 0.0045545 0.999919 -3.16282e-07
  hidden linearVelocity_4 -0.00255674 -0.0232922 0.00218172
  hidden angularVelocity_4 0.0198602 -9.84272e-07 0.0232741
  hidden translation_5 0.197 8.45135e-06 -0.1331
  hidden rotation_5 -0.00895643 0.00345587 0.999954 -4.91785e-07
  hidden linearVelocity_5 -0.0025571 -0.0349089 0.00218169
  hidden angularVelocity_5 0.0198627 -9.92987e-07 0.023279
  translation 2.61431 0.109092 18.5514
  rotation -0.000449526 1 0.000227141 -2.66435
  controller "obstacle_avoidance_with_lidar"
  extensionSlot [
    SickLms291 {
      translation 0 0.24 -0.136
      pixelSize 0
    }
  ]
}
```

The names of the first six hidden fields all contain 0 as primary index, which is the index of the `Pioneer3at` PROTO itself.
The additional secondary indices for the four hidden `position` fields correspond to the four [HingeJoint](hingejoint.md) nodes used for the wheels and numbered by means of pre-order traversal.
There is no hidden field associated to the [Solid](solid.md) node with index 1, namely the `SickLms291` PROTO, since its relative position and orientation are kept fixed during simulation.
The indices ranging from 2 to 5 correspond to the four [Solid](solid.md) wheels of the `Pioneer3at`.
