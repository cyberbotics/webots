## DEF and USE

A node which is named using the DEF keyword can be referenced later by its name in the same file with USE statements.
The DEF and USE keywords can be used to reduce redundancy in ".wbt" and ".proto" files.
DEF name are limited in scope to a single ".wbt" or ".proto" file.
If multiple nodes are given the same DEF name, each USE statement refers to the closest node with the given DEF name preceding it in the ".wbt" or ".proto" file.

```
[DEF defName] nodeName { nodeBody }
```

```
USE defName
```

### USE Exceptions

Although it is permitted to name any node using the DEF keyword, there are some exceptions where USE nodes are not valid.
A USE node is an exact copy of the corresponding DEF node, and thus no property of a USE node can change independently from the DEF node.

For this reason it is not allowed to insert USE nodes of [Solid](solid.md), [Joint](joint.md), [JointParameters](jointparameters.md), [TrackWheel](trackwheel.md), and [BallJointParameters](balljointparameters.md) nodes and their derived nodes.
Indeed, the ability for identical solids or joints to occupy the same position is useless, if not hazardous, in a physics simulation.
To safely duplicate one of these nodes, you can design a [PROTO](proto.md) model for this node and then add different PROTO instances to your world.

The same principle applies to [Material](material.md) and [Light](light.md) nodes located in the first child of [Charger](charger.md) and [LED](led.md) nodes whose emissive color changes automatically based on the [Charger](charger.md) energy and [LED](led.md) setup.

Also the [TextureTransform](texturetransform.md) of the first [Shape](shape.md) in the [Track](track.md) device with enabled texture animation cannot be a USE node given that the translation values will be automatically updated by Webots while the [Track](track.md) device is moving.
