## IMU Sensors

IMU (Inertial Measurement Unit) sensors combine multiple proprioceptive sensors.
An [Accelerometer](../reference/accelerometer.md), a [Gyro](../reference/gyro.md) and a [Compass](../reference/compass.md) are generally all combined in one single sensor.
The IMU may also be equipped with a higher level layer allowing to get the attitude of the robot (roll, pitch, yaw), which is equivalent to the behavior of the [InertialUnit](../reference/inertialunit.md).
It is indeed possible to add each of these sensors individually on a robot.
Predefined IMU models present in Webots to simplify the user experience are listed below.

### MPU-9250

%figure "MPU-9250 model"

![mpu-9250.png](images/sensors/mpu-9250.png)

%end

The [MPU-9250](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/) is a MEMS (Microelectromechanical systems) IMU device to track the orientation and motion of a body.
It is modeled as a solid chip embeding an [Accelerometer](../reference/accelerometer.md), a [Gyro](../reference/gyro.md) and a [Compass](../reference/compass.md) to retrieve, respectively, acceleration values in [m/s²], angular rates in [rad/s] and the vector indicating the north direction for heading.

Detailed specifications can be found [on this page](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf).

%figure "Internal devices names"

| Device                            | Name                                      |
| --------------------------------- | ----------------------------------------- |
| Accelerometer                     |  "`Mpu-9250.name` accelerometer"          |
| Gyro                              |  "`Mpu-9250.name` gyro"                   |
| Compass                           |  "`Mpu-9250.name` compass"                |

%end

```
Mpu-9250 {
  SFVec3f    translation   0 0 0
  SFRotation rotation      0 0 1 0
  SFString   name          "MPU-9250"
}
```
