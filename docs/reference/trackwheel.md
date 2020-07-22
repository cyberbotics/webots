## TrackWheel

Derived from [Group](group.md).

```
TrackWheel {
  SFVec2f position 0 0    # any vector
  SFFloat radius   0.1    # [0, inf)
  SFBool  inner    TRUE   # {TRUE, FALSE}
}
```

### Description

The [TrackWheel](#trackwheel) node is a utility node helping the setup of a wheel of a track system defined by a [Track](track.md) node.

By default this node doesn't have any graphical shape and the shape of the wheel has to be defined inside the `children` field of the [TrackWheel](#trackwheel) node.
All the children nodes will be automatically translated at the relative location specified in the `position` field and automatically rotated along the z-axis based on the speed of the parent [Track](track.md) node.
Additionally this node is used by the parent [Track](track.md) node to compute the track belt path used for geometries animation and representing the track itself.
Note that in order to automatically generate the track belt, the geometries animation has to be enabled, i.e. the `animatedGeometry` field of the parent [Track](track.md) node has to be defined and the `geometriesCount` field has to be greater than 0.

The [TrackWheel](#trackwheel) node is a pure graphical object and doesn't have any physical behavior.
All the physical properties are defined in the [Track](track.md) node.

A [TrackWheel](#trackwheel) node can only be inserted in the `children` field of a [Track](track.md) node.

### Field Summary

The fields of the [TrackWheel](#trackwheel) are mainly used to automatically generate the track belt path in case the geometries animation is defined in the [Track](track.md) node:

- `position`: defines the position of the wheel on the x-y plane of the [Track](track.md) node coordinate system.
The `position`field is also used to compute the relative translation of the `children` nodes.

- `radius`: defines the radius of the wheel.

- `inner`: defines the location of the wheel with respect to the track belt.
If the value is TRUE then the wheel is inside the track belt, otherwise it is outside.
The value of this field also affects the automatic rotation direction of the user-defined wheel shape.
If TRUE, the shape will rotate in the same direction of the track, otherwise it will rotate in the opposite direction.
