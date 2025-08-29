/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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
 * Description:   Car library to be used with the 'Car' proto (or any proto inherited by 'Car')
 * Comments:      Sponsored by the CTI project RO2IVSim
 *                (http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)
 */

#ifndef CAR_H
#define CAR_H

#include <webots/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { WBU_CAR_TRACTION = 0, WBU_CAR_PROPULSION = 1, WBU_CAR_FOUR_BY_FOUR = 2 } WbuCarType;

typedef enum {
  WBU_CAR_COMBUSTION_ENGINE = 0,
  WBU_CAR_ELECTRIC_ENGINE = 1,
  WBU_CAR_PARALLEL_HYBRID_ENGINE = 2,
  WBU_CAR_POWER_SPLIT_HYBRID_ENGINE = 3
} WbuCarEngineType;

typedef enum {
  WBU_CAR_WHEEL_FRONT_RIGHT = 0,
  WBU_CAR_WHEEL_FRONT_LEFT = 1,
  WBU_CAR_WHEEL_REAR_RIGHT = 2,
  WBU_CAR_WHEEL_REAR_LEFT = 3,
  WBU_CAR_WHEEL_NB
} WbuCarWheelIndex;

void wbu_car_init();
void wbu_car_cleanup();

WbuCarType wbu_car_get_type();
WbuCarEngineType wbu_car_get_engine_type();

void wbu_car_set_indicator_period(double period);
double wbu_car_get_indicator_period();

bool wbu_car_get_backwards_lights();
bool wbu_car_get_brake_lights();

double wbu_car_get_track_front();
double wbu_car_get_track_rear();
double wbu_car_get_wheelbase();
double wbu_car_get_front_wheel_radius();
double wbu_car_get_rear_wheel_radius();

double wbu_car_get_wheel_encoder(WbuCarWheelIndex wheel_index);
double wbu_car_get_wheel_speed(WbuCarWheelIndex wheel_index);

void wbu_car_set_left_steering_angle(double angle);
void wbu_car_set_right_steering_angle(double angle);
double wbu_car_get_right_steering_angle();
double wbu_car_get_left_steering_angle();

void wbu_car_enable_limited_slip_differential(bool enable);
void wbu_car_enable_indicator_auto_disabling(bool enable);

// kept for backward compatibility (Webots R2018a)
typedef WbuCarType wbu_car_type;
typedef WbuCarEngineType wbu_car_engine_type;
typedef WbuCarWheelIndex wbu_car_wheel_index;

#ifdef __cplusplus
}
#endif

#endif  // CAR_H
