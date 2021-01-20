#include <webots/robot.h>
#include <webots/robot_window_utils/generic_robot_window.h>
#include <webots/robot_window_utils/string_utils.h>
#include <webots/robot_wwi.h>

#include <stdlib.h>
#include <string.h>

void wb_robot_window_init() {
  init_robot_window();
}

void wb_robot_window_step(int time_step) {
  const char *message = wb_robot_wwi_receive_text();
  if (message) {
    if (!handle_generic_robot_window_messages(message)) {
      char *tokens = strdup(message);
      char *token = NULL;
      while ((token = string_utils_strsep(&tokens, ","))) {
        char *command = strdup(token);
        char *first_word = string_utils_strsep(&command, ":");
        if (!parse_device_control_command(first_word, command))
          parse_device_command(first_word, command);
      }
    }
  }

  if (!robot_window_needs_update())
    return;

  update_robot_window();
}

void wb_robot_window_cleanup() {
}
