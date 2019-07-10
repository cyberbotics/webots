#include <stdlib.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Get and enable devices.
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  wb_keyboard_enable(timestep);

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  int m;
  for (m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  // Wait one second.
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  // Constants.
  const double k = 28.7;
  const double kv1 = 2.5;
  const double kv2 = 0.0002;
  const double kr = 5.0;
  const double kp = 5.0;
  double tA = 1.0;
  double sum = 0.0;

  while (wb_robot_step(timestep) != -1) {
    bool led_state = ((int)(wb_robot_get_time())) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    double y = wb_gps_get_values(gps)[1];

    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          a = 0.3;
          break;
        case WB_KEYBOARD_DOWN:
          a = -0.3;
          break;
        case WB_KEYBOARD_RIGHT:
          b = 0.5;
          break;
        case WB_KEYBOARD_LEFT:
          b = -0.5;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
          c = -0.3;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
          c = 0.3;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          tA += 0.05;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          tA -= 0.05;
          break;
      }
      key = wb_keyboard_get_key();
    }

    double dA = tA - y;
    if (SIGN(dA) != SIGN(sum))
      sum = 0.0;
    sum += dA;

    roll += 1.5708;
    double r = kr * CLAMP(roll, -0.5, 0.5) + c;
    double p = kp * CLAMP(pitch, -0.5, 0.5) + a;
    double yF = b;
    double v = k + kv1 * CLAMP(dA, -0.05, 0.05) + kv2 * sum;

    wb_motor_set_velocity(front_left_motor, v - r - p + yF);
    wb_motor_set_velocity(front_right_motor, -(v + r - p - yF));
    wb_motor_set_velocity(rear_left_motor, -(v - r + p - yF));
    wb_motor_set_velocity(rear_right_motor, v + r + p + yF);
  };

  wb_robot_cleanup();

  return EXIT_SUCCESS;
}
