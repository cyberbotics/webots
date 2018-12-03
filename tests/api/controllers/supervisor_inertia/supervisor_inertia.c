// Test that the inertia is not reset after a supervisor set position.
// A ball is falling, and translated after 1 second, it should hit the right collider just after.

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/touch_sensor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <stdlib.h>
#include <string.h>

#define TIME_STEP 8

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbNodeRef ball_node = wb_supervisor_node_get_from_def("BALL");
  ts_assert_pointer_not_null(ball_node, "The supervisor cannot get the WbNodeRef of the ball.");

  WbFieldRef translation_field = wb_supervisor_node_get_field(ball_node, "translation");
  ts_assert_pointer_not_null(translation_field, "The supervisor cannot get the WbFieldRef of the ball translation.");

  WbDeviceTag red_collider = wb_robot_get_device("red_collider");
  wb_touch_sensor_enable(red_collider, TIME_STEP);
  WbDeviceTag green_collider = wb_robot_get_device("green_collider");
  wb_touch_sensor_enable(green_collider, TIME_STEP);

  // Wait one second.
  while (wb_robot_get_time() < 1.0)
    wb_robot_step(TIME_STEP);

  // Translate the ball, without reseting its physics.
  const double *translation_const = wb_supervisor_field_get_sf_vec3f(translation_field);
  double *translation = (double *)malloc(3 * sizeof(double));
  memcpy(translation, translation_const, 3 * sizeof(double));
  translation[0] += 2.0;
  translation[2] += 2.0;
  wb_supervisor_field_set_sf_vec3f(translation_field, translation);
  free(translation);

  // Test that the green touch sensor is touched quickly enough, i.e. the physics has not been reset.
  while (true) {
    ts_assert_double_is_bigger(1.1, wb_robot_get_time(),
                               "The ball didn't touched the sensors quickly enough: the physics has not been reset.");
    ts_assert_double_equal(wb_touch_sensor_get_value(red_collider), 0.0, "The ball hits the wrong collider.");
    if (wb_touch_sensor_get_value(green_collider) == 1.0)
      break;  // test succeed!
    wb_robot_step(TIME_STEP);
  }

  wb_robot_step(TIME_STEP);

  ts_send_success();
  return EXIT_SUCCESS;
}
