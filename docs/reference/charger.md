## Charger

Derived from [Solid](solid.md).

```
Charger {
  MFFloat battery       [ ]     # [0, inf)
  SFFloat radius        0.04    # [0, inf)
  SFColor emissiveColor 0 1 0   # any color
  SFBool  gradual       TRUE    # {TRUE, FALSE}
}
```

### Description

The [Charger](#charger) node is used to model a special kind of battery charger for the robots.
A robot has to get close to a charger in order to recharge itself.
A charger is not like a standard battery charger connected to a constant power supply.
Instead, it is a battery itself: it accumulates energy with time.
It could be compared to a solar power panel charging a battery.
When the robot comes to get energy, it can't get more than the charger has presently accumulated.
Note that only one robot can be charged at a time.
Additionally, if a robot is connected, the charger cannot recharge itself.

The appearance of the [Charger](#charger) node can be altered by its current energy.
When the [Charger](#charger) node is full, the resulting color corresponds to its `emissiveColor` field, while when the [Charger](#charger) node is empty, its resulting color corresponds to its original one.
Intermediate colors depend on the `gradual` field.
Only the first child of the [Charger](#charger) node is affected by this alteration.
The resulting color is applied only on the first child of the [Charger](#charger) node.
If the first child is a [Shape](shape.md) node, the `emissiveColor` field of its [Material](material.md) node is altered.
If the first child is a [Light](light.md) node, its `color` field is altered.
Otherwise, if the first child is a [Group](group.md) node, a recursive search is applied on this node and every [Light](light.md), [Shape](shape.md) and [Group](group.md) nodes are altered according to the two previous rules.
Note that [Material](material.md) and [Light](light.md) nodes that are going to be automatically altered by the [Charger](#charger) functionality cannot be [USE](def-and-use.md) nodes.

### Field Summary

The fields specific to the [Charger](#charger) node are:

- `battery`: this field should contain three values, namely the present energy of the charger (*J*), its maximum energy (*J*) and its charging speed (*W=J/s*).

- `radius`: radius of the charging area in meters.
The charging area is a sphere centered on the origin of the charger coordinate system.
The robot can recharge itself if its origin is in the charging area (see [this figure](#the-sensitive-area-of-a-charger)).

%figure "The sensitive area of a charger"

![charger.png](images/charger.png)

%end

- `emissiveColor`: color of the first child node (see above) when the charger is full.

- `gradual`: defines the behavior of the indicator.
If set to TRUE, the indicator displays a progressive transition between its original color and the `emissiveColor` specified in the [Charger](#charger) node, corresponding to the present level of charge.
If set to FALSE, the indicator remains its original color until the charger is fully charged (i.e., the present energy level equals the maximum energy level).
Then, it switches to the specified `emissiveColor`.
