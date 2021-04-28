#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#define TIME_STEP 16
#define ANGLE_LIMIT 0.523599f

int main() {
  wb_robot_init();

  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);
  // wait for command
  float velocity_motor = 0.0;
  float velocity_motor2 = 0.0;
  while (wb_robot_step(TIME_STEP) != -1.0) {
    if (wb_receiver_get_queue_length(receiver) > 0) {
      const float *inbuffer = (float *)wb_receiver_get_data(receiver);
      velocity_motor = inbuffer[0];
      velocity_motor2 = inbuffer[1];
      break;
    }
  }

  WbDeviceTag motor[5];
  WbDeviceTag motor2[5];
  char names_motor[5][30] = {"backlash_on_both_motor", "backlash_on_axis_motor", "backlash_on_axis2_motor",
                             "backlash_on_neither_motor", "reference_hinge2joint_motor"};
  char names_motor2[5][30] = {"backlash_on_both_motor2", "backlash_on_axis_motor2", "backlash_on_axis2_motor2",
                              "backlash_on_neither_motor2", "reference_hinge2joint_motor2"};

  for (int i = 0; i < 5; ++i) {
    motor[i] = wb_robot_get_device(names_motor[i]);
    motor2[i] = wb_robot_get_device(names_motor2[i]);
    wb_motor_set_position(motor[i], INFINITY);
    wb_motor_set_position(motor2[i], INFINITY);
    wb_motor_set_velocity(motor[i], velocity_motor);
    wb_motor_set_velocity(motor2[i], velocity_motor2);
  }

  double reference_hinge2joint_position;
  WbDeviceTag reference_hinge2joint_sensor = wb_robot_get_device("reference_hinge2joint_sensor");
  WbDeviceTag reference_hinge2joint_sensor2 = wb_robot_get_device("reference_hinge2joint_sensor2");
  wb_position_sensor_enable(reference_hinge2joint_sensor, TIME_STEP);
  wb_position_sensor_enable(reference_hinge2joint_sensor2, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1.0) {
    if (velocity_motor == 0.0 && velocity_motor2 != 0.0)  // actuating only axis2
      reference_hinge2joint_position = wb_position_sensor_get_value(reference_hinge2joint_sensor2);
    else
      reference_hinge2joint_position = wb_position_sensor_get_value(reference_hinge2joint_sensor);

    if (reference_hinge2joint_position > ANGLE_LIMIT) {
      for (int i = 0; i < 5; ++i) {
        if (velocity_motor == 0.0 && velocity_motor2 != 0.0)
          wb_motor_set_velocity(motor2[i], -velocity_motor2);  // first test, only axis2 actuated
        else
          wb_motor_set_velocity(motor[i], -velocity_motor);  // second test, only axis actuated
      }
    }
    if (reference_hinge2joint_position < -ANGLE_LIMIT) {
      for (int i = 0; i < 5; ++i) {
        if (velocity_motor == 0.0 && velocity_motor2 != 0.0)
          wb_motor_set_velocity(motor2[i], velocity_motor2);  // first test, only axis2 actuated
        else
          wb_motor_set_velocity(motor[i], velocity_motor);  // second test, only axis actuated
      }
    }
  }

  wb_robot_cleanup();

  return 0;
}
