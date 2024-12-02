## Damping

```
Damping {
  SFFloat linear  0.2    # [0, 1]
  SFFloat angular 0.2    # [0, 1]
}
```

### Description

A [Damping](#damping) node can be used to slow down a body (a [Solid](solid.md) node with [Physics](physics.md)).
The speed of each body is reduced by the specified amount (between 0.0 and 1.0) every second.
A value of 0.0 means "no slowing down" and value of 1.0 means a "complete stop", a value of 0.1 means that the speed should be decreased by 10 percent every second.
Note that the behavior of this value on the solid speeds is nonlinear: a linear damping of 0.99 is far to affect the solids speeds as a linear damping of 1.0.
A damped body will possibly come to rest and become disabled depending on the values specified in [WorldInfo](worldinfo.md).
Damping does not add any force in the simulation, it directly affects the velocity of the body.
The damping effect is applied after all forces have been applied to the bodies.
Damping can be used to reduce simulation instability.

> **Note**: When several rigidly linked [Solid](solid.md)s are merged (see [Physics](physics.md)'s [solid merging](physics.md#implicit-solid-merging-and-joints) section) damping values of the aggregate body are averaged over the volumes of all [Solid](solid.md) components.
The volume of a [Solid](solid.md) is the sum of the volumes of the geometries found in its `boundingObject`; overlaps are not handled.

The `linear` field indicates the amount of damping that must be applied to the body's linear motion.
The `angular` field indicates the amount of damping that must be applied to the body's angular motion.
The linear damping can be used, e.g., to slow down a vehicle by simulating air or water friction.
The angular damping can be used, e.g., to slow down the rotation of a rolling ball or the spin of a coin.
Note that the damping is applied regardless of the shape of the object, so damping cannot be used to model complex fluid dynamics (use [ImmersionProperties](immersionproperties.md) and [Fluid](fluid.md) nodes instead).

A [Damping](#damping) node can be specified in the `defaultDamping` field of the [WorldInfo](worldinfo.md) node; in this case it defines the default damping parameters that must be applied to every body in the simulation.
A [Damping](#damping) node can be specified in the `damping` field of a [Physics](physics.md) node; in this case it defines the damping parameters that must be applied to the [Solid](solid.md) that contains the [Physics](physics.md) node.
The damping specified in a [Physics](physics.md) node overrides the default damping.
