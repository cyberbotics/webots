## RotationalMotor

Derived from [Motor](motor.md).

```
RotationalMotor {
  SFString name      "rotational motor"              # any string
  SFFloat  maxTorque 10                              # [0, inf)
  SFString sound     "sounds/rotational_motor.wav"   # any string
}
```

### Description

A [RotationalMotor](#rotationalmotor) node can be used to power either a [HingeJoint](hingejoint.md) or a [Hinge2Joint](hinge2joint.md) to produce a rotational motion around the choosen axis.

### Field Summary

- The `name` field specifies the name identifier of the motor device.
This the name to which the `wb_robot_get_device` function refer.
It defaults to `"rotational motor"`.

- The `maxTorque` field specifies both the upper limit and the default value for the motor *available torque*.
The *available torque* is the torque that is available to the motor to perform the requested motions.
It is expressed in *newton meter* [N⋅m].
When used in velocity mode, this maximum torque is always applied (with the correct sign) until the speed is reached. 
See : http://ode.org/wiki/index.php?title=Manual#Stops_and_motor_parameters
The `wb_motor_set_available_torque` function can be used to change the *available torque* at run-time.
The value of `maxTorque` should always be zero or positive (the default value is 10 N⋅m).
A small `maxTorque` value may result in a motor being unable to move to the target position because of its weight or other external forces. A motor can't go backward when used in speed mode.
