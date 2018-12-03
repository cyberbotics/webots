#include <webots/emitter.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

#include <stdio.h>
#include <string.h>

int main(int argc, char **argv) {
  wb_robot_init();

  int time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  WbDeviceTag touch_sensor = wb_robot_get_device("touch sensor");
  wb_touch_sensor_enable(touch_sensor, time_step);

  while (wb_robot_step(time_step) != -1) {
    if (wb_touch_sensor_get_value(touch_sensor) == 1) {
      printf("collision\n");

      char buffer[64];
      sprintf(buffer, "%s : collision at %lf", wb_robot_get_name(), wb_robot_get_time());
      wb_emitter_send(emitter, buffer, strlen(buffer) + 1);
      wb_robot_step(time_step);
      break;
    }
  };

  wb_robot_cleanup();

  return 0;
}
