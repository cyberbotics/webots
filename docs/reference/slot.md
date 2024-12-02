## Slot

```
Slot {
  SFString type     ""     # any string
  SFNode   endPoint NULL   # {node, PROTO}
}
```

### Description

[Slot](#slot) nodes always works with pairs, only a second [Slot](#slot) can be added in the `endPoint` field of a [Slot](#slot) before to be able to add any node in the `endPoint` field of the second [Slot](#slot).
Furthermore, the second [Slot](#slot) can be added only if it has the same `type` as the first one.
The [Slot](#slot) node is particularly useful with PROTO nodes, it allows the user to constrain the type of nodes that can be added in an extension field of the PROTO.
Imagine for example that you have an armed robot in which you can plug different kinds of hands.
In order to do so you will put the hand as an extension field of your robot, you will then be able to add all the different PROTO models of hand that you have made.
But nothing prevents you from adding a PROTO of table in the hand extension field.
The [Slot](#slot) is made for preventing this kind of problems.
By encapsulating your extension field in a [Slot](#slot) and using the [Slot](#slot) node as base node for all your hands PROTO nodes and defining the same `type` for the field [Slot](#slot) and the PROTO [Slot](#slot), only hands can be inserted in the extension field.
This is illustrated in the [example](#example) section.

### Field Summary

- `type`: defines the type of the [Slot](#slot).
Two [Slot](#slot) nodes can be connected only if their types match.
It is possible to specify a gender by ending the string with a '`+`' or a '`-`'.
In this case, two [Slot](#slot) nodes can be connected only if they are of opposite gender (e.g. a [Slot](#slot) with a type ending with '`+`' can only be connected to a [Slot](#slot) with the same type, except that it ends with '`-`' instead of '`+`').
The default empty type matches any type.

- `endPoint`: The node inserted in the endPoint of a [Slot](#slot) should be another [Slot](#slot) if this [Slot](#slot) is not already connected to another [Slot](#slot) (i.e., its parent is a [Slot](#slot)).
If the pair of [Slot](#slot) nodes is already connected, any node that can usually be inserted in a `children` field can be inserted in the `endPoint` field of the second [Slot](#slot).

### Example

If you want to write a proto of a robot called `MyRobot` that accepts only hands in its field `handExtension`, you have to set the field `handExtension` to be the `endPoint` of a [Slot](#slot).

```
PROTO MyRobot [
  field SFNode handExtension NULL
]
Robot {
  children [
    Slot {
      type "robotHand"
      endPoint IS handExtension
    }
    ...
  ]
}
```

Then any PROTO of a hand needs to use the [Slot](#slot) as base node and the `type` of this [Slot](#slot) should match the one in `MyRobot`.

```
PROTO RobotHand [
]
{
  Slot {
    type "robotHand"
    endPoint Solid {
      ...
    }
  }
}
```
