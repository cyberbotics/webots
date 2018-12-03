#include <webots/brake.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int i;
  WbDeviceTag pos_sensors[4];
  WbDeviceTag brakes[3];

  brakes[0] = wb_robot_get_device("brake 1");
  brakes[1] = wb_robot_get_device("brake 2");
  brakes[2] = wb_robot_get_device("brake 3");
  wb_brake_set_damping_constant(brakes[0], 0.0);
  wb_brake_set_damping_constant(brakes[1], 1.0);
  wb_brake_set_damping_constant(brakes[2], 10000.0);

  pos_sensors[1] = wb_brake_get_position_sensor(brakes[0]);
  ts_assert_int_equal(pos_sensors[1], wb_robot_get_device("position sensor 2"),
                      "wb_brake_get_position_sensor didn't work correctly.");
  pos_sensors[0] = wb_robot_get_device("position sensor 1");
  pos_sensors[2] = wb_robot_get_device("position sensor 3");
  pos_sensors[3] = wb_robot_get_device("position sensor 4");
  for (i = 0; i < 4; ++i)
    wb_position_sensor_enable(pos_sensors[i], TIME_STEP);

  for (i = 0; i < 8; ++i)
    wb_robot_step(TIME_STEP);

  WbDeviceTag ps3 = wb_brake_get_position_sensor(brakes[1]);
  ts_assert_int_equal(ps3, pos_sensors[2], "wb_brake_get_position_sensor after simulation start didn't work correctly.");

  double angles[4];
  for (i = 0; i < 4; ++i)
    angles[i] = wb_position_sensor_get_value(pos_sensors[i]);

  ts_assert_double_in_delta(
    angles[0], angles[1], 0.01,
    "Joint 0 and 1 should have the same dynamic, because one has no brake and the other has a brake set to 0.0.");
  ts_assert_double_is_bigger(angles[1], angles[2],
                             "Joint 2 should have a smaller angle than joint 1 because its brake is set to a non-null value.");
  ts_assert_double_in_delta(angles[3], 0, 0.001,
                            "Joint 3 should have an angle close to 0.0, because its brake is set to a very high value.");

  ts_send_success();
  return EXIT_SUCCESS;
}
