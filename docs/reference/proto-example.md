## PROTO Example

A complete example of PROTO definition and instantiation is provided here.
The PROTO is called `TwoColorChair`; it defines a simple chair with four legs and a seating part.
For simplicity, this PROTO does not have bounding objects nor [Physics](physics.md) nodes.
A more complete example of this PROTO named `SimpleChair` is provided in Webots distribution.

The `TwoColorChair` PROTO allows to specify two colors: one for the legs and one for the seating surface of the chair.
The interface also defines a `translation` field and a `rotation` field that are associated with the equally named fields of the PROTO's [Solid](solid.md) base node.
This allows to store the position and orientation of the PROTO instances.

"TwoColorChair.proto":

```
#VRML_SIM R2019a utf8
# license: Creative Commons Attribution 4.0 International License.
# license url: https://creativecommons.org/licenses/by/4.0/legalcode
# documentation url: https://www.cyberbotics.com/doc/guide/object-chairs
# A two-color chair

PROTO TwoColorChair [
  field SFVec3f    translation       0 0.91 0
  field SFRotation rotation          0 1 0 0
  field SFString   name              "two-color chair"
  field SFColor    legColor          1 1 0
  field SFColor    seatColor         1 0.65 0
  field SFNode     seatGeometry      NULL
  field MFNode     seatExtensionSlot []
]
{
  Solid {
    translation IS translation
    rotation IS rotation
    children [
      Transform {
        translation 0 0 -0.27
        children IS seatExtensionSlot
      }
      Transform {
        translation 0 -0.35 0
        children [
          Shape {
            appearance Appearance {
              material Material { diffuseColor IS seatColor }
            }
            geometry IS seatGeometry
          }
        ]
      }
      Transform {
        translation 0.25 -0.65 -0.23
        children [
          DEF LEG_SHAPE Shape {
            appearance Appearance {
              material Material { diffuseColor IS legColor }
            }
            geometry Box { size 0.075 0.52 0.075 }
          }
        ]
      }
      Transform {
        translation -0.25 -0.65 -0.23
        children [ USE LEG_SHAPE ]
      }
      Transform {
        translation 0.25 -0.65 0.2
        children [ USE LEG_SHAPE ]
      }
      Transform {
        translation -0.25 -0.65 0.2
        children [ USE LEG_SHAPE ]
      }
    ]
    name IS name
  }
}
```

As you can observe in this example, it is perfectly valid to have several IS statement for one interface field (`seatColor`), as long as the types match.
It is also possible to use IS statements inside a defined (DEF) node and then to reuse (USE) that node.
This is done here with the `diffuseColor IS legColor` statement placed inside the `DEF LEG_SHAPE Shape` node which is then reused (USE) several times below.

The "ProtoInstantiationExample.wbt" file below exemplifies the instantiation of this PROTO.
PROTO nodes are instantiated using the regular node syntax.
Fields with the default value can be omitted.
Field values which differ from the default must be specified.

"TwoChairs.wbt":

```
#VRML_SIM R2019a utf8

WorldInfo {
}
Viewpoint {
  orientation 0.628082 0.772958 0.089714 5.69177
  position -0.805359 1.75254 2.75772
}
Background {
  skyColor [
    0.4 0.7 1
  ]
}
DirectionalLight {
  direction -0.3 -1 -0.5
  castShadows TRUE
}
TwoColorChair {
  seatGeometry Cylinder {
    height 0.075
    radius 0.38
  }
}
TwoColorChair {
  translation 1.2 0.91 0
  seatColor 0.564706 0.933333 0.564706
  seatGeometry Box { size 0.6 0.075 0.52 }
  seatExtensionSlot [
    Shape {
      appearance Appearance {
        material Material { diffuseColor 0.564706 0.933333 0.564706}
      }
      geometry Box { size 0.6 0.67 0.0275 }
    }
  ]
}
```

The "TwoChairs.wbt" file once loaded by Webots appears as shown in [this figure](#two-instances-of-the-twocolorchair-proto-in-webots).

As you can observe in this example, defining MFNode fields in the PROTO interface allows to reuse the same model for slightly different objects or robots.
Extenstion slots like `seatExtensionSlot` field could, for example, be used to add additional devices to a base robot without needing to copy the robot definition or creating a new PROTO.

%figure "Two instances of the TwoColorChair PROTO in Webots"

![two_chairs_v7-2-0.png](images/two_chairs_v7-2-0.png)

%end
