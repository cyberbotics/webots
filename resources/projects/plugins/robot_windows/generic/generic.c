#include <webots/plugins/robot_window/generic_robot_window/generic.h>
#include <webots/plugins/robot_window/robot_wwi.h>
#include <webots/robot.h>
#include <webots/utils/string.h>

#include <stdlib.h>
#include <string.h>

void wb_robot_window_init() {
  wbu_generic_robot_window_init();
}

void wb_robot_window_step(int time_step) {
  const char *message;
  while ((message = wb_robot_wwi_receive_text())) {
    if (!wbu_generic_robot_window_handle_messages(message)) {
      // JavaScript -> C protocol description:
      //   [deviceName:commandTag[=commadState][,]]*
      // example:
      //   "e-puck:forward,ds0:enable,myMotor0:value=1.2"
      char *tokens = strdup(message);
      char *token = NULL;
      while ((token = wbu_string_strsep(&tokens, ","))) {
        char *command = strdup(token);
        char *first_word = wbu_string_strsep(&command, ":");
        if (!wbu_generic_robot_window_parse_device_control_command(first_word, command))
          wbu_generic_robot_window_parse_device_command(first_word, command);
      }
    }
  }

  if (!wbu_generic_robot_window_needs_update())
    return;

  wbu_generic_robot_window_update();
}

void wb_robot_window_cleanup() {
}
