#ifndef GENERIC_H
#define GENERIC_H

#include <webots/types.h>

void configure_generic_robot_window(const char *message);
void init_generic_robot_window();
void parse_generic_command(char *token, WbDeviceTag *tag, bool *robot);

#endif
