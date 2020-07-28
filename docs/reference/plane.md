## Plane

```
Plane {
  SFVec2f size 1 1   # any vector
}
```

### Description

The [Plane](#plane) node defines a plane in 3D-space.
The plane's normal vector is the y-axis of the local coordinate system.
The plane can be used as graphical object or as collision detection object.

When a plane is used as graphical object, the `size` field specifies the dimensions of the graphical representation.
Just like the other graphical primitives, it is possible to apply a [Material](material.md) (e.g., a texture) to a plane.

When a plane is used as collision detection object (in a `boundingObject`) then the `size` field is ignored and the plane is considered to be infinite.
The [Plane](#plane) node is the ideal primitive to simulate, e.g., the floor or infinitely high walls.
Unlike the other collision detection primitives, the [Plane](#plane) can only be used in static objects (a static object is an object without a [Physics](physics.md) node).
Note that Webots ignores collision between planes, so planes can safely cut each other.
Note that a [Plane](#plane) node is in fact not really a plane: it's a half-space.
Anything that is moving inside the half-space will be ejected out of it.
This means that planes are only planes from the perspective of one side.
If you want your plane to be reversed, rotate it by &pi; using a [Transform](transform.md) node.
