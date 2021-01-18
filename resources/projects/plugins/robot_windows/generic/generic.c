#include <webots/robot.h>
#include <webots/robot_window_utils/generic_robot_window.h>
#include <webots/robot_window_utils/string_utils.h>
#include <webots/robot_wwi.h>

#include <stdlib.h>
#include <string.h>

void wb_robot_window_init() {
  init_robot_window();
}

// JavaScript -> C protocol description:
//   [deviceName:commandTag[=commadState][,]]*
// example:
//   "e-puck:forward,ds0:enable,myMotor0:value=1.2"
static void apply_command(const char *command) {
  char *tokens = strdup(command);
  char *token = NULL;

  WbDeviceTag tag = 0;
  bool robot = false;
  while ((token = string_utils_strsep(&tokens, ":"))) {
    if (tag == 0 && !robot) {  // first token = device or robot name
      char *name0 = string_utils_replace(token, "\\:", ":");
      char *name = string_utils_replace(name0, "\\,", ",");
      if (strcmp(name, wb_robot_get_name()) == 0)
        robot = true;
      else
        tag = wb_robot_get_device(name);
      free(name);
      free(name0);
    } else
      parse_device_command(token, &tag, &robot);
  }
}

void wb_robot_window_step(int time_step) {
  const char *message = wb_robot_wwi_receive_text();
  if (message) {
    if (!handle_generic_robot_window_messages(message)) {
      char *tokens = strdup(message);
      char *token = NULL;
      while ((token = string_utils_strsep(&tokens, ",")))
        apply_command(token);
    }
  }

  if (!robot_window_needs_update())
    return;

  update_robot_window();
}

void wb_robot_window_cleanup() {
}
