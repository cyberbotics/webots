## Muscle

```
Muscle {
  SFDouble volume      0.01    # [0, inf)
  SFVec3f  startOffset 0 0 0   # any vector
  SFVec3f  endOffset   0 0 0   # any vector
  MFColor  color       [ ]     # any color
  SFBool   castShadows TRUE    # {TRUE, FALSE}
  SFBool   visible     TRUE    # {TRUE, FALSE}
}
```

### Description

A [Muscle](#muscle) node can be used to graphically display the contraction of an artificial muscle implemented using [Joint](joint.md) and [Motor](motor.md) nodes.
The artificial muscle is represented using a spheroid where the symmetry axis is the vector between the joint's closest upper [Pose](pose.md) node and the `endPoint` [Solid](solid.md) node.
The other two axes have the same length computed based on the symmetry axis length so that the volume remains constant during stretching.

Note that the [Muscle](#muscle) node cannot be used in case of a [Motor](motor.md) device included in a [Track](track.md) node.

### Field Summary

- The `volume` field specifies the constant volume of the graphical spheroid.
This value is used to recompute the shape of the muscle when the joint moves.

- The `startOffset` specifies the position of the bottom point of the muscle spheroid in the coordinate system of the closest upper [Pose](pose.md) node.
If the `startOffset` is `[0, 0, 0]`, then the spheroid bottom point corresponds to the closest upper [Pose](pose.md) origin.

- The `endOffset` specifies the position of the top point of the muscle spheroid in the coordinate system of the [Joint](joint.md).`endPoint` [Solid](solid.md) node.
If the `endOffset` is `[0, 0, 0]`, then the spheroid top point corresponds to the `endPoint` [Solid](solid.md) origin.

- The `color` field specifies the color of the spheroid at the three different muscle states: idle (item 0), contracting (item 1), and relaxing (item 2).
The displayed color results by mixing the idle color and the current state color with a percentage depending on the force applied by the motor:

    ```
color = idle_color * (1 - percentage) + other_color * percentage
    ```

    Where ``other_color`` is contracting or relaxing color.
    Only three colors are used, so if more items are specified then they will be ignored.
    If only two colors are defined, then same color (item 1) is used when the muscle is contracting or relaxing.
    If only one color is defined, then the specified color is be used for all the muscle states.
    If `color` field is empty, the default color (pure red) is used for all the muscle states.

- The `castShadows` field allows the user to turn on (TRUE) or off (FALSE) shadows casted by the muscle spheroid mesh.

- The `visible` field is used to show (TRUE) or hide (FALSE) the muscle in the 3D scene.
