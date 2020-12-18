## ContactProperties

```
ContactProperties {
  SFString material1          "default"            # any string
  SFString material2          "default"            # any string
  MFFloat  coulombFriction    1                    # [0, inf)
  SFVec2f  frictionRotation   0 0                  # any positive vector
  SFFloat  bounce             0.5                  # [0, 1]
  SFFloat  bounceVelocity     0.01                 # [0, inf)
  MFFloat  forceDependentSlip 0                    # [0, inf)
  SFFloat  softERP            0.2                  # [0, 1]
  SFFloat  softCFM            0.001                # (0, inf)
  SFString bumpSound          "sounds/bump.wav"    # any string
  SFString rollSound          "sounds/roll.waw"    # any string
  SFString SlideSound         "sounds/slide.wav"   # any string
}
```

### Description

[ContactProperties](#contactproperties) nodes define the contact properties to use in case of contact between [Solid](solid.md) nodes (or any node derived from [Solid](solid.md)).
[ContactProperties](#contactproperties) nodes are placed in the `contactProperties` field of the [WorldInfo](worldinfo.md) node.
Each [ContactProperties](#contactproperties) node specifies the name of two *materials* for which these [ContactProperties](#contactproperties) are valid.

When two [Solid](solid.md) nodes collide, a matching [ContactProperties](#contactproperties) node is searched in the [WorldInfo](worldinfo.md).`contactProperties` field.
A [ContactProperties](#contactproperties) node will match if its `material1` and `material2` fields correspond (in any order) to the `contactMaterial` fields of the two colliding [Solid](solid.md)s.
The values of the first matching [ContactProperties](#contactproperties) are applied to the contact.
If no matching node is found, default values are used.
The default values are the same as those indicated above.

> **Note**: In older Webots versions, contact properties used to be specified in [Physics](physics.md) nodes.
For compatibility reasons, contact properties specified like this are still functional in Webots, but they trigger deprecation warnings.
To remove these warning you need to switch to the new scheme described in this page.
This can be done in three steps: 1.
Add [ContactProperties](#contactproperties) nodes in [WorldInfo](worldinfo.md), 2.
Define the `contactMaterial` fields of [Solid](solid.md) nodes, 3.
Reset the values of `coulombFriction, bounce, bounceVelocity` and `forceDependentSlip` in the [Physics](physics.md) nodes.

### Field Summary

- The `material1` and `material2` fields specify the two *contact materials* to which this [ContactProperties](#contactproperties) node must be applied.
The values in this fields should match the `contactMaterial` fields of [Solid](solid.md) nodes in the simulation.
The values in `material1` and `material2` are exchangeable.

- The `coulombFriction` are the Coulomb friction coefficients.
They must be in the range 0 to infinity (use -1 for infinity).
0 results in a frictionless contact, and infinity results in a contact that never slips.
This field can hold one to four values.
If it has only one value, the friction is fully symmetric.
With two values, the friction is fully asymmetric using the same coefficients for both solids.
With three values, the first solid (corresponding to `material1`) uses asymmetric coefficients (first two values) and the other solid (corresponding to `material2`) uses a symmetric coefficient (last value).
Finally, with four values, both solids use asymmetric coefficients, first two for the first solid and last two for the second solid.
The two friction directions are defined for each faces of the geometric primitives and match with the U and V components used in the texture mapping.
Only the `Box`, `Plane` and `Cylinder` primitives support asymmetric friction.
If another primitive is used, only the first value will be used for symetric friction.
WEBOTS\_HOME/projects/sample/howto/worlds/asymmetric\_friction1.wbt contains an example of fully asymmetric friction.

- The `frictionRotation` allows the user to rotate the friction directions used in case of asymmetric `coulombFriction` and/or asymmetric `forceDependentSlip`.
By default, the directions are the same than the ones used for texture mapping (this can ease defining an asymmetric friction for a textured surface matching the rotation field of the corresponding TextureTransform node).
WEBOTS\_HOME/projects/sample/howto/worlds/asymmetric\_friction2.wbt illustrates the use of this field.

- The `bounce` field is the coefficient of restitution (COR) between 0 and 1.
The coefficient of restitution (COR), or *bounciness* of an object is a fractional value representing the ratio of speeds after and before an impact.
An object with a COR of 1 collides elastically, while an object with a COR < 1 collides inelastically.
For a COR = 0, the object effectively "stops" at the surface with which it collides, not bouncing at all.
COR = (relative speed after collision) / (relative speed before collision).

- The `bounceVelocity` field represents the minimum incoming velocity necessary for bouncing.
Solid objects with velocities below this threshold will have a `bounce` value set to 0.

- The `forceDependentSlip` field defines the *force dependent slip* (FDS) for friction, as explained in the ODE documentation: "FDS is an effect that causes the contacting surfaces to side past each other with a velocity that is proportional to the force that is being applied tangentially to that surface.
Consider a contact point where the coefficient of friction mu is infinite.
Normally, if a force f is applied to the two contacting surfaces, to try and get them to slide past each other, they will not move.
However, if the FDS coefficient is set to a positive value k then the surfaces will slide past each other, building up to a steady velocity of k*f relative to each other.
Note that this is quite different from normal frictional effects: the force does not cause a constant acceleration of the surfaces relative to each other&mdash;it causes a brief acceleration to achieve the steady velocity."

    This field can hold one to four values. If it has only one value, this
    coefficient is applied to both directions (force dependent slip is disabled if
    the value is 0). With two values, force dependent slip is fully asymmetric using
    the same coefficients for both solids (if one value is 0, force dependent slip
    is disabled in the corresponding direction). With three values, the first solid
    (corresponding to `material1`) uses asymmetric coefficients (first two values)
    and the other solid (corresponding to `material2`) uses a symmetric coefficient
    (last value). Finally, with four values, both solids use asymmetric
    coefficients, first two for the first solid and last two for the second solid.
    The friction directions and the supported geometric primitives are the same as
    the ones documented with the `coulombFriction` field.

- The `softERP` field defines the *Error Reduction Parameter* used by ODE to manage local contact joints.
See [WorldInfo](worldinfo.md) for a description of the ERP concept.

- The `softCFM` field defines the soft *Constraint Force Mixing* used by ODE to manage local contacts joints.
[WorldInfo](worldinfo.md) for a description of the CFM concept.

- The `bumpSound`, `rollSound` and `slideSound` fields define the URLs to WAVE files that are used to render the sounds of contacts.
These URLs are expressed relatively to the world or PROTO file containing the `ContactProperties` node.
`bumpSound` is the sound produced by the impact of a collision.
Its gain is modulated by the energy involved in the collision.
`rollSound` is the sound produced by a rolling object.
Its gain and pitch are modulated by the angular velocities of the bodies in contact.
`slideSound` is the sound produced by the friction of a body sliding on another body.
Its gain and pitch are modulated by the linear velocity of the contact surface.
The formulas affecting the gain and pitch of these sounds were determinated empirically to produce fairly realistic sounds.
They are subject to improvements.

> **Note**: The youBot robot is a good example of asymmetric coulombFriction and forceDependentSlip, it is located in WEBOTS\_HOME/projects/robot/youbot/worlds/youbot.wbt.
