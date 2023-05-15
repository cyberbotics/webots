## Fluid

Derived from [Pose](pose.md).

```
Fluid {
  SFString description    ""        # any string
  SFString name           "fluid"   # any string
  SFString model          ""        # any string
  SFString description    ""        # any string
  SFFloat  density        1000      # {-1, [0, inf)}
  SFFloat  viscosity      0.001     # [0, inf)
  SFVec3f  streamVelocity 0 0 0     # any vector
  SFNode   boundingObject NULL      # {node, PROTO}
  SFBool   locked         FALSE     # {TRUE, FALSE}
}
```

### Description

A [Fluid](#fluid) node represents a possibly unbounded fluid volume with physical properties such as density and stream velocity.
A [Solid](solid.md) node which is partially or fully immersed in some [Fluid](#fluid)'s `boundingObject` will be subject to the static force (Archimedes' thrust) and the dynamic force (drag force) exerted by the [Fluid](#fluid) provided it has a [Physics](physics.md) node, a `boundingObject` and that its field `immersionProperties` contains an [ImmersionProperties](immersionproperties.md) node referring to the given [Fluid](#fluid).

In the 3D window, [Fluid](#fluid) nodes can be manipulated (dragged, lifted, rotated, etc) using the mouse.

### Fluid Fields

- `name`: name of the fluid.
This is the name used in a [ImmersionProperties](immersionproperties.md) to refer to a given [Fluid](#fluid).

- `model`: generic name of the fluid, e.g., "sea".

- `description`: short description (1 line) of the fluid.

- `density`: density of the fluid expressed in *kilogram per cubic meter* [kg/m³]; it defaults to water density.
The fluid density is taken into account for the computations of Archimedes' thrust, drag forces and drag torques, see [ImmersionProperties](immersionproperties.md).

- `viscosity`: dynamic viscosity of the fluid expressed in *pascal second* [Pa·s = N·s/m²].
It defaults to viscosity of water at 20 degrees Celsius.

- `streamVelocity`: fluid linear velocity, the flow being assumed laminar.
The fluid linear velocity is taken into account for the drag and viscous resistance computations, see [ImmersionProperties](immersionproperties.md).

- `boundingObject`: the bounding object specifies the geometrical primitives and their [Pose](pose.md) offset used for immersion detection.
If the `boundingObject` field is NULL, then no immersion detection is performed and that fluid will have no effect on immersed objects.
A [Solid](solid.md) will undergo static or dynamic forces exerted by a [Fluid](#fluid) only if its `boundingObject` collides with the [Fluid](#fluid)'s `boundingObject`.
The intersection volume with an individual primitive geometry is approximated by the intersection volume of this geometry with a tangent plane of equation *y = c, c > 0* in the geometry coordinate system.
This volume is used to generates Archimedes' thrust.

    This field is subject to the same restrictions as a [Solid](solid.md)'s
    `boundingObject`.

- `locked`: if `TRUE`, the fluid object cannot be moved using the mouse.
This is useful to prevent moving an object by mistake.
