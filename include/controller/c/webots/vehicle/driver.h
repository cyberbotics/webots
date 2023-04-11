/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Driver library to be used with the 'Car' proto (or any proto inherited by 'Car') and the car library
 * Comments:      Sponsored by the CTI project RO2IVSim
 *                (http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)
 */

#ifndef DRIVER_H
#define DRIVER_H

#include <webots/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { OFF, RIGHT, LEFT } WbuDriverIndicatorState;

typedef enum { UNDEFINED_CONTROL_MODE = -1, SPEED = 0, TORQUE } WbuDriverControlMode;

typedef enum { DOWN, SLOW, NORMAL, FAST } WbuDriverWiperMode;

// private function for webots_ros2 to identify robots that can use libdriver
bool wbu_driver_initialization_is_possible();

void wbu_driver_init();
void wbu_driver_cleanup();
int wbu_driver_step();

// positive: turn right, negative: turn left
void wbu_driver_set_steering_angle(double steering_angle);
double wbu_driver_get_steering_angle();

void wbu_driver_set_cruising_speed(double speed);
double wbu_driver_get_target_cruising_speed();

double wbu_driver_get_current_speed();

void wbu_driver_set_throttle(double throttle);
double wbu_driver_get_throttle();

void wbu_driver_set_brake_intensity(double intensity);
double wbu_driver_get_brake_intensity();

void wbu_driver_set_indicator(WbuDriverIndicatorState state);
void wbu_driver_set_hazard_flashers(bool state);

WbuDriverIndicatorState wbu_driver_get_indicator();
bool wbu_driver_get_hazard_flashers();

void wbu_driver_set_dipped_beams(bool state);
void wbu_driver_set_antifog_lights(bool state);

bool wbu_driver_get_dipped_beams();
bool wbu_driver_get_antifog_lights();

double wbu_driver_get_rpm();
int wbu_driver_get_gear();
void wbu_driver_set_gear(int gear);
int wbu_driver_get_gear_number();
WbuDriverControlMode wbu_driver_get_control_mode();

void wbu_driver_set_wiper_mode(WbuDriverWiperMode mode);
WbuDriverWiperMode wbu_driver_get_wiper_mode();

// kept for backward compatibility (Webots 8.6)
void wbu_driver_set_brake(double brake) WB_DEPRECATED;
double wbu_driver_get_brake() WB_DEPRECATED;

// kept for backward compatibility (Webots R2018a)
typedef WbuDriverWiperMode wbu_wipers_mode;
typedef WbuDriverIndicatorState wbu_indicator_state;
typedef WbuDriverControlMode wbu_control_mode;
typedef WbuDriverWiperMode wbu_wipers_mode;
void wbu_driver_set_wipers_mode(wbu_wipers_mode mode) WB_DEPRECATED;
wbu_wipers_mode wbu_driver_get_wipers_mode() WB_DEPRECATED;

#ifdef __cplusplus
}
#endif

#endif  // DRIVER_H
