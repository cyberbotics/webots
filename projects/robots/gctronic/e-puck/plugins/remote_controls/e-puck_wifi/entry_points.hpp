/*
 * Description:  Defines the entry point of the remote control library
 */

#ifndef LIB_ROBOT_WINDOW_HPP
#define LIB_ROBOT_WINDOW_HPP

#include <webots/remote_control.h>

extern "C" {
bool wbr_init(WbrInterface *ri);
void wbr_cleanup();
}

#endif
