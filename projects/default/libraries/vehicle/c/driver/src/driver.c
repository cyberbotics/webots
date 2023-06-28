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
 * Description:   Implementation of the driver library
 * Comments:      Sponsored by the CTI project RO2IVSim
 *                (http://transport.epfl.ch/simulator-for-mobile-robots-and-intelligent-vehicles)
 */
#define _USE_MATH_DEFINES

#ifdef _MSC_VER
#define isnan(x) ((x) != (x))
#endif

#ifdef _MSC_VER
#ifndef INFINITY
static const double tmp = 1.0;
#define INFINITY (1.0 / (tmp - 1.0))
#endif
#endif

#define THROTTLE_TO_VOLUME_GAIN 0.2
#define RPM_TO_VOLUME_GAIN 0.5

#include "webots/vehicle/driver.h"

#include <webots/brake.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/speaker.h>
#include "../car/src/car_private.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ACCELERATION_THRESHOLD 50  // maximum acceleration allowed for the wheels [rad/s2]
#define INDICATOR_AUTO_DISABLING_THRESHOLD 0.1
#define BLINKER_SOUND_FILE "sounds/blinker.wav"

typedef struct {
  // Car
  car *car;
  // Devices
  WbDeviceTag engine_speaker;
  // Commands
  WbuDriverWiperMode wiper_mode;
  WbuDriverIndicatorState indicator_state;
  bool hazard_flashers_on;
  double steering_angle;
  double cruising_speed;
  double throttle;
  double brake;
  int gear;
  int dipped_beams_state;
  // Mesures
  double rpm;
  // Internal
  WbuDriverControlMode control_mode;
  double front_slip_ratio;
  double rear_slip_ratio;
  double central_slip_ratio;
  double basic_time_step;
  double indicator_angle;  // steering angle at the moment when indicator is switched on
} driver;

static driver *instance = NULL;

//***********************************//
//        Utility functions          //
//***********************************//

bool wbu_driver_initialization_is_possible() {
  // Parse vehicle caracteristics from the beginning of the data string
  int read_int;
  double read_double;
  char read_char;
  int i;

  wb_robot_init();
  const char *sub_data_string = wb_robot_get_custom_data();
  i = sscanf(sub_data_string, "%lf %lf %lf %lf %lf %lf %lf %c %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d", &read_double,
             &read_double, &read_double, &read_double, &read_double, &read_double, &read_double, &read_char, &read_double,
             &read_double, &read_double, &read_double, &read_double, &read_double, &read_double, &read_double, &read_double,
             &read_double, &read_int, &read_int);

  if (i < 20)
    return false;

  return true;
}

static double kmh_to_rads(double kmh, double wheel_radius) {
  return kmh / 3.6 / wheel_radius;
}

// compute wheels front geometric differential ratio right/left
static double differential_ratio_front() {
  if (instance->steering_angle == 0)
    return 0.5;

  double right_turning_radius = instance->car->wheelbase / sin(instance->car->right_angle);
  double left_turning_radius = instance->car->wheelbase / sin(instance->car->left_angle);

  return right_turning_radius / (left_turning_radius + right_turning_radius);
}

// compute wheels rear geometric differential ratio right/left
static double differential_ratio_rear() {
  if (instance->steering_angle == 0)
    return 0.5;

  double right_turning_radius = instance->car->wheelbase / tan(instance->steering_angle) - (instance->car->track_rear / 2);
  double left_turning_radius = instance->car->wheelbase / tan(instance->steering_angle) + (instance->car->track_rear / 2);

  return right_turning_radius / (left_turning_radius + right_turning_radius);
}

// compute central geometric differential ratio front/rear
static double differential_ratio_central() {
  if (instance->steering_angle == 0)
    return 0.5;

  double front_turning_radius = instance->car->wheelbase / sin(instance->steering_angle);
  double rear_turning_radius = instance->car->wheelbase / tan(instance->steering_angle);
  double wheels_radius_ratio = instance->car->rear_wheel_radius / instance->car->front_wheel_radius;

  return (wheels_radius_ratio * front_turning_radius) / (front_turning_radius + rear_turning_radius);
}

static double compute_output_torque() {
  // Compute available torque taking into account the current gear ratio and engine model
  double gear_ratio;
  if (instance->gear > 0)
    gear_ratio = instance->car->gear_ratio[instance->gear];
  else if (instance->gear < 0)  // reverse
    gear_ratio = instance->car->gear_ratio[0];
  else
    return 0.0;  // no gear engaged

  double engine_torque = 0;
  WbuCarEngineType engine = instance->car->engine_type;

  double real_rpm = instance->rpm;
  // cppcheck-suppress knownConditionTrueFalse
  if (instance->gear > 0 && real_rpm < 0)
    real_rpm = 0;
  double engine_rpm = real_rpm;
  if ((engine_rpm < instance->car->engine_min_rpm) &&
      (engine == WBU_CAR_COMBUSTION_ENGINE || engine == WBU_CAR_PARALLEL_HYBRID_ENGINE ||
       engine == WBU_CAR_POWER_SPLIT_HYBRID_ENGINE))
    engine_rpm = instance->car->engine_min_rpm;

  // in case of parallel hybrid engine, the combustion engine is switch on only if min-rpm is reached
  if (engine == WBU_CAR_COMBUSTION_ENGINE || (engine == WBU_CAR_PARALLEL_HYBRID_ENGINE && real_rpm == engine_rpm))
    engine_torque += instance->car->engine_coefficients[0] + engine_rpm * instance->car->engine_coefficients[1] +
                     engine_rpm * engine_rpm * instance->car->engine_coefficients[2];
  if (engine == WBU_CAR_POWER_SPLIT_HYBRID_ENGINE && real_rpm == engine_rpm)
    engine_torque +=
      (1 - instance->car->hybrid_power_split_ratio) *
      (instance->car->engine_coefficients[0] + instance->car->hybrid_power_split_rpm * instance->car->engine_coefficients[1] +
       instance->car->hybrid_power_split_rpm * instance->car->hybrid_power_split_rpm * instance->car->engine_coefficients[2]);
  if (engine == WBU_CAR_ELECTRIC_ENGINE || engine == WBU_CAR_PARALLEL_HYBRID_ENGINE ||
      engine == WBU_CAR_POWER_SPLIT_HYBRID_ENGINE) {
    double temporary_engine_torque = (instance->car->engine_max_power * 60) / (2 * M_PI * real_rpm);
    if (temporary_engine_torque > instance->car->engine_max_torque)
      temporary_engine_torque = instance->car->engine_max_torque;
    engine_torque += temporary_engine_torque;
  }

  double output_torque = engine_torque * instance->throttle * gear_ratio;

  if (real_rpm == instance->car->engine_max_rpm)  // maximum rotation speed of the motor, we don't want to increase it !
    output_torque = 0;

  // Limit torque if maximum acceleration is reached (in order to not have big jump of wheels speed when one of them temporarily
  // does not touch the ground)
  if (fabs(instance->car->max_acceleration) > ACCELERATION_THRESHOLD)
    output_torque *= (ACCELERATION_THRESHOLD / fabs(instance->car->max_acceleration));

  return output_torque;
}

//***********************************//
//          Step functions           //
//***********************************//

static void update_wheels_speed(int ms) {  // Warning speed is wrong the first two steps
  int i = 0;
  instance->car->max_acceleration = 0.0;

  // Enable positionSensors if not already done
  for (i = 0; i < 4; i++) {
    if (wb_position_sensor_get_sampling_period(instance->car->sensors[i]) == 0)
      wb_position_sensor_enable(instance->car->sensors[i], ms);
  }

  // Get current position of the wheels
  static double previous_position[4] = {0.0, 0.0, 0.0, 0.0};
  static double previous_speed[4] = {0.0, 0.0, 0.0, 0.0};
  double current_position[4];
  for (i = 0; i < 4; i++)
    current_position[i] = wb_position_sensor_get_value(instance->car->sensors[i]);

  // Compute wheels speeds
  for (i = 0; i < 4; i++) {
    instance->car->speeds[i] = 1000 * (current_position[i] - previous_position[i]) / ms;
    const double acceleration = 1000 * (instance->car->speeds[i] - previous_speed[i]) / ms;
    if (fabs(acceleration) > fabs(instance->car->max_acceleration))
      instance->car->max_acceleration = acceleration;
    previous_position[i] = current_position[i];
    previous_speed[i] = instance->car->speeds[i];
  }

  // Update speed_display needle
  if (instance->car->needle_motors[1]) {
    const double abs_speed = fabs(wbu_driver_get_current_speed());
    if (abs_speed <= 220) {  // 220 is the max speed on the speedometer texture.
      const double speed_needle_position = abs_speed * wb_motor_get_max_position(instance->car->needle_motors[1]) / 220;
      wb_motor_set_position(instance->car->needle_motors[1], speed_needle_position);
    } else
      wb_motor_set_position(instance->car->needle_motors[1], wb_motor_get_max_position(instance->car->needle_motors[1]));
  }
}

static void compute_rpm() {
  // average speed of the four wheels (rad/s)
  double average_speed =
    (instance->car->speeds[0] + instance->car->speeds[1] + instance->car->speeds[2] + instance->car->speeds[3]) / 4;
  // convert to rot/min
  average_speed *= 60 / (2 * M_PI);
  // convert to engine rpm
  if (instance->gear == 0) {
    instance->rpm = 0;  // TODO: eventually improve this in a future version of the library
    return;
  }

  double gear_ratio = 0.0;

  if (instance->gear > 0)
    gear_ratio = instance->car->gear_ratio[instance->gear];
  else if (instance->gear < 0)
    gear_ratio = -instance->car->gear_ratio[0];

  double max_rpm = instance->car->engine_max_rpm;

  // multiplication by the gear_ratio because we want motor rpm speed from wheels rpm
  double rpm = fabs(average_speed * gear_ratio);

  if (isnan(rpm))  // in order to handle the first time step
    rpm = 0;
  else if (rpm > max_rpm)
    rpm = max_rpm;
  instance->rpm = rpm;

  // Update speed_display RPM needle
  if (instance->car->needle_motors[0]) {
    if (rpm <= 11000) {  // 11 * 1000 is the max RPM on the speed_display texture.
      const double rpm_needle_position = rpm * wb_motor_get_max_position(instance->car->needle_motors[0]) / 11000;
      wb_motor_set_position(instance->car->needle_motors[0], rpm_needle_position);
    } else
      wb_motor_set_position(instance->car->needle_motors[0], wb_motor_get_max_position(instance->car->needle_motors[0]));
  }
}

static void update_slip_ratio() {
  if (instance->car->type == WBU_CAR_TRACTION || instance->car->type == WBU_CAR_FOUR_BY_FOUR) {
    // Compute and update the front slip differential ratio (between -1 and 1)
    double real_front_ratio = (instance->car->speeds[0] / (instance->car->speeds[0] + instance->car->speeds[1])) * 2 - 1;
    // for better result a PD controller can be used here
    instance->front_slip_ratio = instance->front_slip_ratio + real_front_ratio;
    if (isnan(instance->front_slip_ratio))
      instance->front_slip_ratio = 0.0;
    else if (instance->front_slip_ratio < -1)
      instance->front_slip_ratio = -1;
    else if (instance->front_slip_ratio > 1)
      instance->front_slip_ratio = 1;
  }
  if (instance->car->type == WBU_CAR_PROPULSION || instance->car->type == WBU_CAR_FOUR_BY_FOUR) {
    // Compute and update the rear slip differential ratio (between -1 and 1)
    double real_rear_ratio = (instance->car->speeds[2] / (instance->car->speeds[2] + instance->car->speeds[3])) * 2 - 1;
    // for better result a PD controller can be used here
    instance->rear_slip_ratio = instance->rear_slip_ratio + real_rear_ratio;
    if (isnan(instance->rear_slip_ratio))
      instance->rear_slip_ratio = 0.0;
    else if (instance->rear_slip_ratio < -1)
      instance->rear_slip_ratio = -1;
    else if (instance->rear_slip_ratio > 1)
      instance->rear_slip_ratio = 1;
  }
  if (instance->car->type == WBU_CAR_FOUR_BY_FOUR) {
    // Compute and update the central slip differential ratio (between -1 and 1)
    double front_speed_sum = instance->car->speeds[0] + instance->car->speeds[1];
    double rear_speed_sum = instance->car->speeds[2] + instance->car->speeds[3];
    double real_central_ratio = (front_speed_sum / (front_speed_sum + rear_speed_sum)) * 2 - 1;
    // for better result a PD controller can be used here
    instance->central_slip_ratio = instance->central_slip_ratio + real_central_ratio;
    if (isnan(instance->central_slip_ratio))
      instance->central_slip_ratio = 0.0;
    else if (instance->central_slip_ratio < -1)
      instance->central_slip_ratio = -1;
    else if (instance->central_slip_ratio > 1)
      instance->central_slip_ratio = 1;
  }
}

static void update_torque() {
  double torque = compute_output_torque();

  // Distribute the available torque to the actuated wheels using 'geometric' differential rules
  if (instance->car->type == WBU_CAR_TRACTION) {
    // Geometric differential ratio (left and right wheel should not have same torque because the rotation radius is not the
    // same)
    double ratio = differential_ratio_front();
    // Add the limited differential slip if enable
    // the formula is just in order to keep ratio between 0 and 1
    if (instance->car->limited_slip_differential)
      ratio *= 2 * (1 - ((instance->front_slip_ratio + 1) / 2));
    // Compute and apply torques
    double right_torque = torque * ratio;
    double left_torque = torque * (1 - ratio);
    wb_motor_set_torque(instance->car->wheels[0], right_torque);
    wb_motor_set_torque(instance->car->wheels[1], left_torque);

  } else if (instance->car->type == WBU_CAR_PROPULSION) {
    // Geometric differential ratio (left and right wheel should not have same torque because the rotation radius is not the
    // same)
    double ratio = differential_ratio_rear();
    // Add the limited differential slip if enable
    // the formula is just in order to keep ratio between 0 and 1
    if (instance->car->limited_slip_differential)
      ratio *= 2 * (1 - ((instance->rear_slip_ratio + 1) / 2));
    // Compute and apply torques
    double right_torque = torque * ratio;
    double left_torque = torque * (1 - ratio);
    wb_motor_set_torque(instance->car->wheels[2], right_torque);
    wb_motor_set_torque(instance->car->wheels[3], left_torque);

  } else if (instance->car->type == WBU_CAR_FOUR_BY_FOUR) {
    // Geometric differential ratio
    double ratio_front = differential_ratio_front();
    double ratio_rear = differential_ratio_rear();
    double ratio_central = differential_ratio_central();
    // Add the limited differential slip if enable
    if (instance->car->limited_slip_differential) {
      // the formula is just in order to keep ratio between 0 and 1
      ratio_front *= 2 * (1 - ((instance->front_slip_ratio + 1) / 2));
      ratio_rear *= 2 * (1 - ((instance->rear_slip_ratio + 1) / 2));
      ratio_central *= 2 * (1 - ((instance->central_slip_ratio + 1) / 2));
    }

    // Compute and apply torques
    double front_right_torque = torque * ratio_front * ratio_central;
    double front_left_torque = torque * (1 - ratio_front) * ratio_central;
    double rear_right_torque = torque * ratio_rear * (1 - ratio_central);
    double rear_left_torque = torque * (1 - ratio_rear) * (1 - ratio_central);
    wb_motor_set_torque(instance->car->wheels[0], front_right_torque);
    wb_motor_set_torque(instance->car->wheels[1], front_left_torque);
    wb_motor_set_torque(instance->car->wheels[2], rear_right_torque);
    wb_motor_set_torque(instance->car->wheels[3], rear_left_torque);
  }
}

static void update_brake() {
  // Compute and apply dampingConstant
  int i;
  for (i = 0; i < 4; i++) {
    // we divide by the rotational speed of the wheels because we want to remove the dependency of the damping friction to the
    // rotational speed
    const double damping_constant = fabs(instance->brake * instance->car->brake_coefficient / instance->car->speeds[i]);
    if (!isnan(damping_constant))
      wb_brake_set_damping_constant(instance->car->brakes[i], damping_constant);
  }
}

static void update_engine_sound() {
  if (strcmp(instance->car->engine_sound, "none") != 0) {
    double volume = 1.0;
    bool stop_sound = false;
    double rpm = 0;
    if (instance->control_mode == TORQUE) {
      rpm = instance->rpm;
      // modulate volume by the throttle position
      volume *= (1.0 - THROTTLE_TO_VOLUME_GAIN) + THROTTLE_TO_VOLUME_GAIN * instance->throttle;
      // modulate volume by the rpm
      volume *= (1.0 - RPM_TO_VOLUME_GAIN) + RPM_TO_VOLUME_GAIN *
                                               (rpm > instance->car->engine_max_rpm ? instance->car->engine_max_rpm : rpm) /
                                               instance->car->engine_max_rpm;
    } else if (instance->control_mode == SPEED) {
      // in speed mode, the rpm is estimated from the speed
      // average speed of the four wheels (rad/s)
      double average_speed =
        (instance->car->speeds[0] + instance->car->speeds[1] + instance->car->speeds[2] + instance->car->speeds[3]) / 4;
      if (isnan(average_speed))  // first step
        return;
      // convert to rot/min
      average_speed *= 60 / (2 * M_PI);
      // check which gear is the most appropriate
      double gear_ratio = 0;
      if (average_speed < 0)
        gear_ratio = -instance->car->gear_ratio[0];
      else {
        int i;
        for (i = 1; i < instance->car->gear_number; ++i) {
          gear_ratio = instance->car->gear_ratio[i];
          // the 'correct' gear is when rpm < 1 / 3 rpm range
          if (fabs(average_speed * gear_ratio) < (2 * instance->car->engine_min_rpm + instance->car->engine_max_rpm) / 3)
            break;
        }
      }
      rpm = fabs(average_speed * gear_ratio);
      // modulate volume by the difference between current speed and target speed (estimation of throttle)
      double speed_diff = instance->cruising_speed - wbu_driver_get_current_speed();
      if (speed_diff < 0)  // deceleration => no throttle
        speed_diff = 0.0;
      else if (speed_diff > 25.0)  // we assume full throttle as soon as the difference is bigger than 25
        speed_diff = 25.0;
      volume *= (1.0 - THROTTLE_TO_VOLUME_GAIN) + THROTTLE_TO_VOLUME_GAIN * speed_diff / 25.0;
      // modulate volume by the rpm
      volume *= (1.0 - RPM_TO_VOLUME_GAIN) + RPM_TO_VOLUME_GAIN *
                                               (rpm < instance->car->engine_max_rpm ? instance->car->engine_max_rpm : rpm) /
                                               instance->car->engine_max_rpm;
    }
    WbuCarEngineType engine = instance->car->engine_type;
    if (engine == WBU_CAR_POWER_SPLIT_HYBRID_ENGINE) {
      if (rpm > instance->car->engine_min_rpm)
        rpm = instance->car->hybrid_power_split_rpm;
      else
        stop_sound = true;
    } else if (engine == WBU_CAR_PARALLEL_HYBRID_ENGINE) {
      if (rpm < instance->car->engine_min_rpm)
        stop_sound = true;
    }
    if (rpm < instance->car->engine_min_rpm)
      rpm = instance->car->engine_min_rpm;
    double pitch = rpm / instance->car->engine_sound_rpm_reference;
    if (stop_sound)
      wb_speaker_stop(instance->engine_speaker, instance->car->engine_sound);
    else
      wb_speaker_play_sound(instance->engine_speaker, instance->engine_speaker, instance->car->engine_sound, volume, pitch, 0.0,
                            true);
  }
}

//***********************************//
//          API functions            //
//***********************************//

void wbu_driver_init() {
  if (instance != NULL)
    return;

  wbu_car_init();
  instance = (driver *)malloc(sizeof(driver));
  instance->car = _wbu_car_get_instance();
  instance->wiper_mode = DOWN;
  instance->indicator_state = OFF;
  instance->hazard_flashers_on = false;
  instance->steering_angle = 0.0;
  instance->cruising_speed = 0.0;
  instance->throttle = 0.0;
  instance->brake = 0.0;
  instance->gear = 0;
  instance->dipped_beams_state = 0;
  instance->rpm = 0.0;
  instance->control_mode = UNDEFINED_CONTROL_MODE;
  instance->front_slip_ratio = 0.0;
  instance->rear_slip_ratio = 0.0;
  instance->central_slip_ratio = 0.0;
  instance->basic_time_step = wb_robot_get_basic_time_step();
  instance->indicator_angle = 0.0;

  instance->engine_speaker = wb_robot_get_device("engine_speaker");
  if (instance->engine_speaker == 0) {
    fprintf(stderr, "Warning: Any car should have a 'engine_speaker' speaker.\n");
    exit(0);
  }
}

int wbu_driver_step() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_step()"))
    return 0;

  // indicator
  bool on = (((int)(wb_robot_get_time() / instance->car->indicator_period) % 2) == 0);
  bool right_indicator = false, left_indicator = false;
  if (instance->hazard_flashers_on) {  // hazard flashers mode
    right_indicator = on;
    left_indicator = on;
  } else if (instance->indicator_state == RIGHT)  // right mode
    right_indicator = on;
  else if (instance->indicator_state == LEFT)  // left mode
    left_indicator = on;

  _wbu_car_set_led_state_if_exist(RIGHT_INDICATOR_INDEX, right_indicator);
  _wbu_car_set_led_state_if_exist(LEFT_INDICATOR_INDEX, left_indicator);

  // wiper
  if ((instance->car->wiper_motors[0] != 0 && instance->car->wiper_motors[1] != 0 && instance->car->wiper_sensor != 0) &&
      instance->wiper_mode != DOWN) {
    bool stop = ((int)(wb_robot_get_time()) % 4) != 0;  // allows for wiper in slow mode to be activated once every 4 seconds.
    if (wb_position_sensor_get_value(instance->car->wiper_sensor) ==
        wb_motor_get_min_position(instance->car->wiper_motors[0])) {  // wiper go up
      wb_motor_set_position(instance->car->wiper_motors[0], wb_motor_get_max_position(instance->car->wiper_motors[0]));
      wb_motor_set_position(instance->car->wiper_motors[1], wb_motor_get_max_position(instance->car->wiper_motors[1]));
    }
    if (wb_position_sensor_get_value(instance->car->wiper_sensor) ==
          wb_motor_get_max_position(instance->car->wiper_motors[0]) ||
        (instance->wiper_mode == SLOW && stop)) {
      // wiper go/stay down
      wb_motor_set_position(instance->car->wiper_motors[0], wb_motor_get_min_position(instance->car->wiper_motors[0]));
      wb_motor_set_position(instance->car->wiper_motors[1], wb_motor_get_min_position(instance->car->wiper_motors[1]));
    }
  }

  // backward light on if control in torque is enable and gear -1 is engaged or control in speed is enable with a negative speed
  if (((instance->control_mode == TORQUE) && (instance->gear == -1)) ||
      ((instance->control_mode == SPEED) && (instance->cruising_speed < 0)))
    _wbu_car_set_led_state_if_exist(BACKWARDS_LIGHT_INDEX, true);
  else
    _wbu_car_set_led_state_if_exist(BACKWARDS_LIGHT_INDEX, false);

  // update wheels speed and compute engine rpm
  update_wheels_speed((int)instance->basic_time_step);
  compute_rpm();
  // update torque if control in torque is activated
  if (instance->control_mode == TORQUE) {
    // update the differential slip ratio if limited differential slip is enable
    if (instance->car->limited_slip_differential)
      update_slip_ratio();
    update_torque();
  }

  // update brake (dependant of the rotation speed of the wheels)
  if (instance->brake > 0.0)
    update_brake();

  // update engine sound
  update_engine_sound();

  return wb_robot_step((int)instance->basic_time_step);
}

void wbu_driver_cleanup() {
  if (instance == NULL)
    return;
  free(instance);
  instance = NULL;
  wbu_car_cleanup();
}

void wbu_driver_set_steering_angle(double steering_angle) {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_set_steering_angle()"))
    return;

  if (isnan(steering_angle)) {
    fprintf(stderr, "Warning: wbu_driver_set_steering_angle() called with an invalid 'steering_angle' argument (NaN)\n");
    return;
  }

  // move the steering wheel (if any)
  if (instance->car->steering_wheel != 0)
    wb_motor_set_position(instance->car->steering_wheel, steering_angle * 10);

  // Steering mechanism, formulas from:
  // Reza N. Jazar, Vehicle Dynamics: Theory and Application. Springer 2009
  instance->steering_angle = steering_angle;
  double cot = 1 / tan(steering_angle);
  double right_angle = atan(1 / (cot - (instance->car->track_front / (2 * instance->car->wheelbase))));
  double left_angle = atan(1 / (cot + (instance->car->track_front / (2 * instance->car->wheelbase))));
  instance->car->right_angle = right_angle;
  instance->car->left_angle = left_angle;
  wb_motor_set_position(instance->car->steering_motors[0], right_angle);  // right
  wb_motor_set_position(instance->car->steering_motors[1], left_angle);   // left

  // the differential speeds need to be recomputed
  if (instance->control_mode == SPEED)
    wbu_driver_set_cruising_speed(instance->cruising_speed);

  // indicator auto-disabling mechanism
  if (instance->car->indicator_auto_disabling) {
    if (instance->indicator_state == RIGHT) {
      if (steering_angle > instance->indicator_angle)  // continue steering in the direction of the blinker
        instance->indicator_angle = steering_angle;
      else if (steering_angle - instance->indicator_angle <= -INDICATOR_AUTO_DISABLING_THRESHOLD) {
        if (instance->car->indicator_lever_motor != 0)
          wb_motor_set_position(instance->car->indicator_lever_motor, 0.0);
        instance->indicator_state = OFF;
        if (!instance->hazard_flashers_on)
          wb_speaker_stop(instance->engine_speaker, BLINKER_SOUND_FILE);
      }
    } else if (instance->indicator_state == LEFT) {
      if (steering_angle < instance->indicator_angle)  // continue steering in the direction of the blinker
        instance->indicator_angle = steering_angle;
      else if (instance->indicator_angle - steering_angle <= -INDICATOR_AUTO_DISABLING_THRESHOLD) {
        if (instance->car->indicator_lever_motor != 0)
          wb_motor_set_position(instance->car->indicator_lever_motor, 0.0);
        instance->indicator_state = OFF;
        if (!instance->hazard_flashers_on)
          wb_speaker_stop(instance->engine_speaker, BLINKER_SOUND_FILE);
      }
    }
  }
}

double wbu_driver_get_steering_angle() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_steering_angle()"))
    return 0;
  double right_angle = wb_position_sensor_get_value(instance->car->steering_sensors[0]);
  double left_angle = wb_position_sensor_get_value(instance->car->steering_sensors[1]);
  return (right_angle + left_angle) / 2.0;
}

void wbu_driver_set_cruising_speed(double speed) {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_set_cruising_speed()"))
    return;

  // Compute and apply speeds of front wheels (if needed)
  if (instance->car->type == WBU_CAR_TRACTION || instance->car->type == WBU_CAR_FOUR_BY_FOUR) {
    double ratio = differential_ratio_front();
    double right_speed = 2 * speed * ratio;
    double left_speed = 2 * speed * (1 - ratio);

    // apply central differential if needed
    if (instance->car->type == WBU_CAR_FOUR_BY_FOUR) {
      ratio = differential_ratio_central();
      right_speed *= 2 * ratio;
      left_speed *= 2 * ratio;
    }

    if (instance->car->front_wheel_radius == -1.0) {
      fprintf(stderr, "Warning: wheel radius cannot be retrieve automatically.\n");
    } else {
      wb_motor_set_velocity(instance->car->wheels[0], kmh_to_rads(right_speed, instance->car->front_wheel_radius));
      wb_motor_set_velocity(instance->car->wheels[1], kmh_to_rads(left_speed, instance->car->front_wheel_radius));
      if (instance->control_mode == TORQUE) {
        // if control in torque was previously enable we need to reset the target position to infinite
        wb_motor_set_position(instance->car->wheels[0], INFINITY);
        wb_motor_set_position(instance->car->wheels[1], INFINITY);
      }
    }
  }

  // Compute and apply speeds of rear wheels (if needed)
  if (instance->car->type == WBU_CAR_PROPULSION || instance->car->type == WBU_CAR_FOUR_BY_FOUR) {
    double ratio = differential_ratio_rear();
    double right_speed = 2 * speed * ratio;
    double left_speed = 2 * speed * (1 - ratio);

    // apply central differential if needed
    if (instance->car->type == WBU_CAR_FOUR_BY_FOUR) {
      ratio = differential_ratio_central();
      right_speed *= 2 * (1 - ratio);
      left_speed *= 2 * (1 - ratio);
    }

    if (instance->car->rear_wheel_radius == -1.0) {
      fprintf(stderr, "Warning: wheel radius cannot be retrieve automatically.\n");
    } else {
      wb_motor_set_velocity(instance->car->wheels[2], kmh_to_rads(right_speed, instance->car->rear_wheel_radius));
      wb_motor_set_velocity(instance->car->wheels[3], kmh_to_rads(left_speed, instance->car->rear_wheel_radius));
      if (instance->control_mode == TORQUE) {
        // if control in torque was previously enable we need to reset the target position to infinite
        wb_motor_set_position(instance->car->wheels[2], INFINITY);
        wb_motor_set_position(instance->car->wheels[3], INFINITY);
      }
    }
  }

  instance->cruising_speed = speed;
  instance->control_mode = SPEED;
}

double wbu_driver_get_target_cruising_speed() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_target_cruising_speed()"))
    return 0;
  return instance->cruising_speed;
}

double wbu_driver_get_current_speed() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_current_speed()"))
    return 0.0;
  double speed = 0.0;

  switch (instance->car->type) {
    case WBU_CAR_TRACTION:
      if (instance->car->front_wheel_radius == -1.0)
        fprintf(stderr, "Warning: wheel radius cannot be retrieve automatically.\n");
      else
        speed = 0.5 * (instance->car->speeds[0] + instance->car->speeds[1]) * instance->car->front_wheel_radius;
      break;
    case WBU_CAR_FOUR_BY_FOUR:
      if (instance->car->front_wheel_radius == -1.0 || instance->car->rear_wheel_radius == -1.0)
        fprintf(stderr, "Warning: wheel radius cannot be retrieve automatically.\n");
      else
        speed = 0.25 * (instance->car->speeds[0] + instance->car->speeds[1]) * instance->car->front_wheel_radius +
                0.25 * (instance->car->speeds[2] + instance->car->speeds[3]) * instance->car->rear_wheel_radius;
      break;
    case WBU_CAR_PROPULSION:
      if (instance->car->rear_wheel_radius == -1.0)
        fprintf(stderr, "Warning: wheel radius cannot be retrieve automatically.\n");
      else
        speed = 0.5 * (instance->car->speeds[2] + instance->car->speeds[3]) * instance->car->rear_wheel_radius;
      break;
  }
  speed *= 3.6;

  return speed;
}

void wbu_driver_set_throttle(double throttle) {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_set_throttle()"))
    return;
  if (throttle < 0) {
    fprintf(stderr, "Warning: 'throttle' should be bigger or equal to 0, used 0 instead.\n");
    throttle = 0;
  } else if (throttle > 1) {
    fprintf(stderr, "Warning: 'throttle' should be smaller or equal to 1, used 1 instead.\n");
    throttle = 1;
  }

  instance->throttle = throttle;
  instance->control_mode = TORQUE;
}

double wbu_driver_get_throttle() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_throttle()"))
    return 0;
  return instance->throttle;
}

void wbu_driver_set_brake_intensity(double intensity) {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_set_brake_intensity()"))
    return;
  if (intensity < 0.0) {
    fprintf(stderr, "Warning: 'intensity' should be bigger or equal to 0, used 0 instead.\n");
    intensity = 0.0;
  } else if (intensity > 1.0) {
    fprintf(stderr, "Warning: 'intensity' should be smaller or equal to 1, used 1 instead.\n");
    intensity = 1.0;
  }
  instance->brake = intensity;

  // Update braking light
  if (intensity > 0.0)
    _wbu_car_set_led_state_if_exist(BRAKE_LIGHT_INDEX, true);
  else {
    _wbu_car_set_led_state_if_exist(BRAKE_LIGHT_INDEX, false);
    int i;
    // restore default damping constant
    for (i = 0; i < 4; i++)
      wb_brake_set_damping_constant(instance->car->brakes[i], 0.0);
  }
}

double wbu_driver_get_brake_intensity() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_brake_intensity()"))
    return 0;
  return instance->brake;
}

void wbu_driver_set_brake(double brake) {
  fprintf(stderr, "Warning: Deprecated 'wbu_driver_set_brake' use 'wbu_driver_set_brake_intensity' instead.\n");
  wbu_driver_set_brake_intensity(brake);
}

double wbu_driver_get_brake() {
  fprintf(stderr, "Warning: Deprecated 'wbu_driver_get_brake' use 'wbu_driver_get_brake_intensity' instead.\n");
  return wbu_driver_get_brake_intensity();
}

void wbu_driver_set_indicator(WbuDriverIndicatorState state) {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_set_indicator()"))
    return;
  instance->indicator_state = state;
  instance->indicator_angle = instance->steering_angle;
  if (instance->indicator_state == RIGHT && instance->car->indicator_lever_motor != 0)
    wb_motor_set_position(instance->car->indicator_lever_motor,
                          wb_motor_get_max_position(instance->car->indicator_lever_motor));
  else if (instance->indicator_state == LEFT && instance->car->indicator_lever_motor != 0)
    wb_motor_set_position(instance->car->indicator_lever_motor,
                          wb_motor_get_min_position(instance->car->indicator_lever_motor));
  else if (instance->car->indicator_lever_motor != 0)
    wb_motor_set_position(instance->car->indicator_lever_motor, 0.0);

  if (instance->hazard_flashers_on || instance->indicator_state != OFF)
    wb_speaker_play_sound(instance->engine_speaker, instance->engine_speaker, BLINKER_SOUND_FILE, 1.0, 1.0, 0.0, true);
  else
    wb_speaker_stop(instance->engine_speaker, BLINKER_SOUND_FILE);
}

void wbu_driver_set_hazard_flashers(bool state) {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_set_hazard_flashers"))
    return;
  instance->hazard_flashers_on = state;
  if (instance->hazard_flashers_on || instance->indicator_state != OFF)
    wb_speaker_play_sound(instance->engine_speaker, instance->engine_speaker, BLINKER_SOUND_FILE, 1.0, 1.0, 0.0, true);
  else
    wb_speaker_stop(instance->engine_speaker, BLINKER_SOUND_FILE);
}

WbuDriverIndicatorState wbu_driver_get_indicator() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_indicator()"))
    return OFF;
  return instance->indicator_state;
}

bool wbu_driver_get_hazard_flashers() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_hazard_flashers"))
    return false;
  return instance->hazard_flashers_on;
}

void wbu_driver_set_dipped_beams(bool state) {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_set_dipped_beams()"))
    return;
  _wbu_car_set_led_state_if_exist(FRONT_LIGHT_INDEX, state);
  _wbu_car_set_led_state_if_exist(REAR_LIGHT_INDEX, state);
  instance->dipped_beams_state = state;
}

void wbu_driver_set_antifog_lights(bool state) {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_set_antifog_lights()"))
    return;
  _wbu_car_set_led_state_if_exist(ANTIFOG_LIGHT_INDEX, state);
}

bool wbu_driver_get_dipped_beams() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_dipped_beams()"))
    return false;
  return instance->dipped_beams_state;
}

bool wbu_driver_get_antifog_lights() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_antifog_lights()"))
    return false;
  return _wbu_car_get_led_state_if_exist(ANTIFOG_LIGHT_INDEX);
}

double wbu_driver_get_rpm() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_rpm()"))
    return 0.0;
  if (instance->control_mode == SPEED) {
    fprintf(
      stderr,
      "Warning: no engine model when cruise control is enable, call to 'wbu_driver_get_rpm()' is therefore not allowed.\n");
    return 0.0;
  }
  return instance->rpm;
}

int wbu_driver_get_gear() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_gear()"))
    return 0;
  if (instance->control_mode == SPEED) {
    fprintf(
      stderr,
      "Warning: no gear model when cruise control is enable, call to 'wbu_driver_get_gear()' is therefore not allowed.\n");
    return 0;
  }
  return instance->gear;
}

void wbu_driver_set_gear(int gear) {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_set_gear()"))
    return;
  if (gear > instance->car->gear_number) {
    fprintf(stderr, "Warning: this car has only %d gears.\n", instance->car->gear_number);
    gear = instance->car->gear_number;
  } else if (gear < -1) {
    fprintf(stderr, "Warning: Minimum gear value is -1");
    gear = -1;
  }
  instance->gear = gear;
  return;
}

int wbu_driver_get_gear_number() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_gear_number()"))
    return 0;
  return instance->car->gear_number;
}

WbuDriverControlMode wbu_driver_get_control_mode() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_control_mode()"))
    return UNDEFINED_CONTROL_MODE;
  return instance->control_mode;
}

WbuDriverWiperMode wbu_driver_get_wiper_mode() {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_get_wiper_mode()"))
    return (WbuDriverWiperMode)-1;
  return instance->wiper_mode;
}

void wbu_driver_set_wiper_mode(WbuDriverWiperMode mode) {
  if (!_wbu_car_check_initialisation("wbu_driver_init()", "wbu_driver_set_wiper_mode()"))
    return;
  if (instance->car->wiper_motors[0] != 0 && instance->car->wiper_motors[1] != 0) {
    instance->wiper_mode = mode;
    double speed = 2.0;  // same speed for all other modes in order to be able to reset wiper position
    if (mode == FAST)
      speed = 4.0;
    wb_motor_set_velocity(instance->car->wiper_motors[0], speed);
    wb_motor_set_velocity(instance->car->wiper_motors[1], speed);
    if (mode == DOWN) {
      wb_motor_set_position(instance->car->wiper_motors[0], wb_motor_get_min_position(instance->car->wiper_motors[0]));
      wb_motor_set_position(instance->car->wiper_motors[1], wb_motor_get_min_position(instance->car->wiper_motors[1]));
    }
  };
}

wbu_wipers_mode wbu_driver_get_wipers_mode() {
  return wbu_driver_get_wiper_mode();
}

void wbu_driver_set_wipers_mode(wbu_wipers_mode mode) {
  wbu_driver_set_wiper_mode(mode);
}
