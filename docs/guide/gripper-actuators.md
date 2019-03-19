## Grippers

It is quite easy to create any kind of gripper using [Joint](../reference/joint.md), [Motor](../reference/motor.md) and [Solid](../reference/solid.md) nodes.

### ROBOTIQ 3F Gripper

The `ROBOTIQ 3F Gripper` is a 3-fingers adaptive robot gripper.


%figure "Internal motor and position-sensor names"

| Motor Name             | Position Sensor Name           |
| ---------------------- | ---------------------------    |
| palm_finger_1_joint    |  palm_finger_1_joint_sensor    |
| finger_1_joint_1       |  palm_finger_1_joint_sensor    |
| finger_1_joint_2       |  finger_1_joint_2_sensor       |
| finger_1_joint_3       |  finger_1_joint_3_sensor       |
| palm_finger_2_joint    |  palm_finger_2_joint_sensor    |
| finger_2_joint_1       |  finger_2_joint_1_sensor       |
| finger_2_joint_2       |  finger_2_joint_2_sensor       |
| finger_2_joint_3       |  finger_2_joint_3_sensor       |
| finger_middle_joint_1  |  finger_middle_joint_1_sensor  |
| finger_middle_joint_2  |  finger_middle_joint_2_sensor  |
| finger_middle_joint_3  |  finger_middle_joint_3_sensor  |


%end

%figure "ROBOTIQ 3F Gripper"

![robotiq_3f_gripper.png](images/actuators/robotiq_3f_gripper.png)

%end

```
PROTO Robotiq3fGripper [
  SFVec3f    translation    0 0 0
  SFRotation rotation       0 1 0 0
  SFString   name           "ROBOTIQ 3f Gripper"
]
```
