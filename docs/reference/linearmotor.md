## LinearMotor

Derived from [Motor](motor.md).

```
LinearMotor {
  SFString name     "linear motor"              # any string
  SFFloat  maxForce 10                          # [0, inf)
  SFString sound    "sounds/linear_motor.wav"   # any string
}
```

### Description

A [LinearMotor](#linearmotor) node can be used to power a [SliderJoint](sliderjoint.md) and a [Track](track.md).

### Field Summary

- The `name` field specifies the name identifier of the motor device.
This the name to which the `wb_robot_get_device` function refers.
It defaults to `"linear motor"`.

- The `maxForce` field specifies both the upper limit and the default value for the motor *available force*.
The *available force* is the force that is available to the motor to perform the requested motions.
It is expressed in *newton* [N].
The `wb_motor_set_available_force` function can be used to change the *available force* at run-time.
The value of `maxForce` should always be zero or positive (the default value is 10 N).
A small `maxForce` value may result in a motor being unable to move to the target position because of its weight or other external forces.
