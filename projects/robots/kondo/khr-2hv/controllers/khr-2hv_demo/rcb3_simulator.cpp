// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rcb3_simulator.h"

#ifndef M_PI
#define _USE_MATH_DEFINES
#endif

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cctype>

#ifdef _WIN32
#include <windows.h>
#endif

#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

float estimated_speed[256];
int samples_val[10] = {1, 10, 25, 35, 50, 75, 100, 150, 200, 255};
float samples_speed[10] = {275.14f, 234.38f, 226.0f, 168.75f, 105.47f, 84.38f, 69.16f, 43.05f, 31.64f, 24.91f};

const char *joint_number_to_name[JOINT_COUNT] = {"Head",        "Right_Shoulder", "Right_Arm1", "Right_Hand", "Left_Shoulder",
                                                 "Left_Arm1",   "Left_Hand",      "Right_Hip",  "Right_Leg1", "Right_leg3",
                                                 "Right_Ankle", "Right_Foot",     "Left_Hip",   "Left_Leg1",  "Left_Leg3",
                                                 "Left_Ankle",  "Left_Foot"};

int channel_indices[CHANNEL_COUNT_RCB3] = {0,  4,  5,  6,  -1, 1, 2, 3,  -1, -1, 12, 13,
                                           14, 15, 16, -1, 7,  8, 9, 10, 11, -1, -1, -1};

void log(int verbose_level, const char *format, ...) {
  const int g_verbose_level = 0;  // Debug level.
  if (verbose_level > g_verbose_level)
    return;
  va_list arglist;
  va_start(arglist, format);
  vprintf(format, arglist);
  va_end(arglist);
}

void compute_estimated_speed() {
  estimated_speed[0] = 0;
  int si = 0;
  int val_start = samples_val[si];
  int val_end = samples_val[si + 1];
  float speed_start = samples_speed[si];
  float speed_end = samples_speed[si + 1];
  for (int i = 1; i <= 255; i++) {
    if (val_end < i) {
      si++;
      val_start = samples_val[si];
      val_end = samples_val[si + 1];
      speed_start = samples_speed[si];
      speed_end = samples_speed[si + 1];
    }

    if (i == val_end)
      estimated_speed[i] = speed_end;
    else {
      float speed = speed_start + (speed_end - speed_start) * (float)(i - val_start) / (float)(val_end - val_start);
      estimated_speed[i] = speed;
    }
  }
}

rcb3_simulator::rcb3_simulator() {
  m_current_motion = NO_MOTION_INDEX;
  m_current_scenario = NO_MOTION_INDEX;
  m_current_scenario_position = NO_MOTION_INDEX;
  compute_estimated_speed();
  m_loop_counter = 0;
  m_compare_register = 0;
  m_button = 0;
  m_run_current_motion_only = false;
  m_simulation_delay = 0;
  m_motions.resize(80, NULL);
  m_scenarios.resize(200, NULL);

  for (int ad = 0; ad < AD_INPUT_COUNT; ad++) {
    m_ad[ad] = 0;
    m_ad_origin[ad] = 0;
  }

  // 1.35v voltage when not moving for the Gyros.
  m_ad_origin[0] = (int)(1.35f * 1024.0f / 5.0f);
  m_ad_origin[1] = (int)(1.35f * 1024.0f / 5.0f);

  for (int i = 0; i < CHANNEL_COUNT_RCB3; i++) {
    m_joint_position[i] = 0.0f;
    m_desired_joint_position[i] = 0.0f;
    for (int ad = 0; ad < AD_INPUT_COUNT; ad++) {
      m_ad_mixing[i][ad] = false;
      m_ad_mixing_factor[i][ad] = 0;
      m_realtime_ad_mixing[i][ad] = false;
      m_realtime_ad_mixing_factor[i][ad] = 0;
    }
  }
  m_motion_mixing_count = 0;
  m_realtime_mixing_count = 0;

  for (int i = 0; i < JOINT_COUNT; i++) {
    m_joint[i] = 0;
    m_joint_sensor[i] = 0;
  }
  m_accelerometerTag = 0;
  m_gyroTag = 0;
}

rcb3_simulator::~rcb3_simulator() {
  for (int i = 0; i < (int)m_motions.size(); i++) {
    if (m_motions[i])
      delete m_motions[i];
  }

  m_motions.clear();
}

void rcb3_simulator::init() {
  int i;
  m_current_motion = NO_MOTION_INDEX;

  for (i = 0; i < JOINT_COUNT; i++) {
    m_joint[i] = wb_robot_get_device(joint_number_to_name[i]);
    m_joint_sensor[i] = wb_motor_get_position_sensor(m_joint[i]);
    wb_position_sensor_enable(m_joint_sensor[i], TIME_STEP);
  }

  float motor_speed = (60.0f / 0.14f) * (float)M_PI / 180.0f;

  for (i = 0; i < JOINT_COUNT; i++) {
    if (m_joint[i] == 0) {
      log(0, "Joint %d is missing", i);
    } else {
      wb_motor_set_acceleration(m_joint[i], -1);
      wb_motor_set_velocity(m_joint[i], motor_speed);
    }
  }

  wb_keyboard_enable(TIME_STEP);

  m_accelerometerTag = wb_robot_get_device("RCB3_K_ACCELEROMETER");
  wb_accelerometer_enable(m_accelerometerTag, TIME_STEP);

  m_gyroTag = wb_robot_get_device("RCB3_K_GYRO");
  wb_gyro_enable(m_gyroTag, TIME_STEP);
}

void rcb3_simulator::play_scenario(unsigned int store_index) {
  if (store_index < m_scenarios.size()) {
    log(1, "playing scenario %u\n", store_index);

    m_current_scenario = store_index;
    m_scenarios[store_index]->m_current_pos = m_scenarios[store_index]->m_start;

    // We have started a new scenario
    RCBMotion *m = m_scenarios[m_current_scenario];
    int start_pos = m->m_start;
    if (start_pos != NO_MOTION_INDEX) {
      RCBMotionItem &item = m->m_items[start_pos];
      m_current_motion = item.m_params.front();
      m_run_current_motion_only = false;
    } else  // Invalid scenario, stop here
      m_current_scenario = NO_MOTION_INDEX;
  }
}

void rcb3_simulator::play_motion(unsigned int store_index) {
  if (store_index < m_motions.size()) {
    log(1, "playing motion %u\n", store_index);

    m_current_motion = store_index;
    m_run_current_motion_only = true;
    run();
  }
}

void rcb3_simulator::run() {
  int k = wb_keyboard_get_key();
  RCBMotion *m = NULL;

  if (m_current_motion != NO_MOTION_INDEX) {
    if (m_current_motion < m_motions.size())
      m = m_motions[m_current_motion];
  }

  if (m) {
    log(1, "Playing Motion %s \n", m->m_name);
    m->m_current_pos = m->m_start;
    int pos = m->m_start;

    double start_time = wb_robot_get_time();
    m_motion_mixing_count = 0;
    float estimated_motion_time = 0;
    while (pos != -1) {
      log(2, "Position %d\n", pos);
      RCBMotionItem mt = m->m_items[pos];
      log(3, "Motion Item %d\n", pos);
      std::list<int>::iterator it = mt.m_params.begin();
      bool default_jump = true;
      int speed = *it;
      ++it;
      int j;

      get_ad_values();

      if (m_button == -1)
        return;

      for (j = 0; j < CHANNEL_COUNT_RCB3; j++) {
        if (channel_indices[j] != -1) {
          if (*it >= 32768) {
            // desired_joint_position[j] = 0.0f;
          } else {
            log(4, "Joint %d : %d\n", j, *it - 16384);
            m_desired_joint_position[j] = (*it - 16384) * 90.0f / 256.0f;
          }
        }

        int command = *it;
        if (command >= 32768) {
          bool recognized = false;
          // Special command cases
          if (command >= 33031 && command <= 33286)
            // Jump command, not used here simulated with links
            recognized = true;

          // Commands active only for 1 value
          if (j == 0) {
            if (command >= 33287 && command <= 33542) {
              recognized = true;
              // Setup Loop Counter
              m_loop_counter = command - 33287;
              log(4, "Setting Loop Counter to %d\n", m_loop_counter);
            }

            if (command >= 33543 && command <= 33798) {
              recognized = true;
              // Setup Loop Counter
              int delta = command - 33543 + 1;
              m_loop_counter -= delta;
              log(4, "Updating Loop Counter to %d\n", m_loop_counter);
              if (m_loop_counter != 0)
                default_jump = false;
            }

            if (command >= 39942 && command <= 57348) {
              recognized = true;
              // Setup Register for Compare operation
              m_compare_register = (command - 39942) - 1023;
              log(4, "Setting Compare Register to %d\n", m_compare_register);
            }

            if (command >= 35846 && command <= 36101) {
              recognized = true;
              // Compare AD1 with register
              log(4, "Checking AD1=%d against register=%d\n", m_ad[0], m_compare_register);
              if (m_ad[0] <= m_compare_register)
                default_jump = false;
            }

            if (command >= 36102 && command <= 36357) {
              recognized = true;
              // Compare AD2 with register
              log(4, "Checking AD2=%d against register=%d\n", m_ad[1], m_compare_register);
              if (m_ad[1] <= m_compare_register)
                default_jump = false;
            }

            if (command >= 36358 && command <= 36613) {
              recognized = true;
              // Compare AD3 with register
              log(4, "Checking AD3=%d against register=%d\n", m_ad[2], m_compare_register);
              if (m_ad[2] <= m_compare_register)
                default_jump = false;
            }

            if (command >= 39430 && command <= 39685) {
              recognized = true;
              // check button vs register
              // int rcb3_register = command - 35846;
              log(4, "Checking if button(simulated remote control %d) == register=%d\n", m_button, m_compare_register);
              if (m_button == m_compare_register)
                default_jump = false;
            }

            if (command >= 39686 && command <= 39941) {
              recognized = true;
              // check button vs register
              // int rcb3_register = command - 35846;
              log(4, "Checking button(simulated remote control %d) bitwise AND register=%d\n", m_button, m_compare_register);
              if (m_button & m_compare_register)
                default_jump = false;
            }

            // AD1 Mixing
            if (command >= 36614 && command <= 36869) {
              recognized = true;
              log(4, "AD1 Mixing\n");
              int mixing_value = command - 36614;
              fill_mixing_data(j, 0, mixing_value, false);
            }

            // AD2 Mixing
            if (command >= 36870 && command <= 37125) {
              recognized = true;
              log(4, "AD2 Mixing\n");
              int mixing_value = command - 36870;
              fill_mixing_data(j, 1, mixing_value, false);
            }

            // AD3 Mixing
            if (command >= 37126 && command <= 37381) {
              recognized = true;
              log(4, "AD3 Mixing\n");
              int mixing_value = command - 37126;
              fill_mixing_data(j, 2, mixing_value, false);
            }

            // Realtime AD1 Mixing
            if (command >= 64500 && command <= 64755) {
              recognized = true;
              log(4, "Realtime AD1 Mixing\n");
              int mixing_value = command - 64500;
              fill_mixing_data(j, 0, mixing_value, true);
            }

            // Realtime AD2 Mixing
            if (command >= 64756 && command <= 65011) {
              recognized = true;
              log(4, "Realtime AD2 Mixing\n");
              int mixing_value = command - 64756;
              fill_mixing_data(j, 1, mixing_value, true);
            }

            // Realtime AD3 Mixing
            if (command >= 65012 && command <= 65267) {
              recognized = true;
              log(4, "Realtime AD3 Mixing\n");
              int mixing_value = command - 65012;
              fill_mixing_data(j, 2, mixing_value, true);
            }

            // Fix origin value for AD1, AD2, AD3
            if (command >= 65268 && command <= 65270) {
              recognized = true;
              int index = command - 65268;
              m_ad_origin[index] = m_ad[index];
            }

            if (command >= 37382 && command <= 37637) {
              recognized = true;
              // Compare PAD1 with register
              log(4, "Checking PAD1=%d against register=%d\n", m_ad[0] - m_ad_origin[0], m_compare_register);
              if ((m_ad[0] - m_ad_origin[0]) <= m_compare_register)
                default_jump = false;
            }

            if (command >= 37638 && command <= 37893) {
              recognized = true;
              // Compare PAD2 with register
              log(4, "Checking PAD2=%d against register=%d\n", m_ad[1] - m_ad_origin[1], m_compare_register);
              if ((m_ad[1] - m_ad_origin[1]) <= m_compare_register)
                default_jump = false;
            }

            if (command >= 37894 && command <= 38149) {
              recognized = true;
              // Compare PAD3 with register
              log(4, "Checking PAD3=%d against register=%d\n", m_ad[2] - m_ad_origin[2], m_compare_register);
              if ((m_ad[2] - m_ad_origin[2]) <= m_compare_register)
                default_jump = false;
            }
          }

          if (j == 0 && !recognized)
            log(3, "Unrecognized command %d\n", command);
        }

        ++it;
      }

      float motor_speed = estimated_speed[speed];

      if (m_motion_mixing_count != 0)
        apply_mixing(false);
      else if (m_realtime_mixing_count)
        apply_mixing(true);

      float max_delta = 0;

      float deltas[JOINT_COUNT];
      for (int s = 0; s < JOINT_COUNT; s++)
        deltas[s] = 0;

      for (j = 0; j < CHANNEL_COUNT_RCB3; j++) {
        if (m_desired_joint_position[j] != m_joint_position[j]) {
          float delta = (float)fabs(m_desired_joint_position[j] - m_joint_position[j]);
          if (delta > max_delta)
            max_delta = delta;

          int ch = channel_indices[j];
          if (ch != -1) {
            deltas[ch] = delta;
            log(5, "Moving %s to %f\n", joint_number_to_name[ch], m_desired_joint_position[j]);
            float new_angle = m_desired_joint_position[j] * (float)M_PI / 180.0f;
            wb_motor_set_position(m_joint[ch], new_angle);
            m_joint_position[j] = m_desired_joint_position[j];
          }
        }
      }

      float remaining_exec_time;
      remaining_exec_time = (max_delta / motor_speed) * 1000.0f + TIME_STEP;  // ms
      motor_speed *= (float)M_PI / 180.0f;

      // We want all the motions to end at the same time, so regulate the speed accordingly
      estimated_motion_time += remaining_exec_time;

      for (int s = 0; s < JOINT_COUNT; s++) {
        if (deltas[s] != 0.0f) {
          float reduction = (deltas[s] / max_delta);
          if (reduction > 1.0f)
            reduction = 1.0f;
          wb_motor_set_velocity(m_joint[s], motor_speed * reduction);
        }
      }

      log(5, "Remaining time %f\n", remaining_exec_time);

      int step = TIME_STEP;
      bool done = false;
      while (!done) {
        wb_robot_step(step);
        k = wb_keyboard_get_key();
        bool tdone = true;
        for (j = 0; j < CHANNEL_COUNT_RCB3; j++) {
          int ch = channel_indices[j];
          if (ch != -1) {
            if (deltas[ch] != 0) {
              float new_angle = m_desired_joint_position[j] * (float)M_PI / 180.0f;
              float reached_angle = wb_position_sensor_get_value(m_joint_sensor[ch]);
              float diff = (float)fabs(new_angle - reached_angle);
              if (diff > 0.03f) {
                tdone = false;
                break;
              }
            }
          }
        }

        done |= tdone;
        if (remaining_exec_time < -(step * 10))
          done = true;

        remaining_exec_time -= (float)step;

        get_ad_values();
        m_button = convert_to_remote_code(k);
        if (m_simulation_delay)
#ifdef _WIN32
          Sleep(m_simulation_delay);
#else
          usleep(m_simulation_delay * 1000);
#endif
      }

      log(2, "\n");
      pos = m->GetNextItem(default_jump);
      if (pos == NO_MOTION_INDEX)
        break;
    }

    double end_time = wb_robot_get_time();
    log(2, "Motion executed in %f s (estimated %f)\n", (float)(end_time - start_time), estimated_motion_time / 1000.0f);
  }

  m_current_motion = NO_MOTION_INDEX;
  m = NULL;
  if (m_run_current_motion_only) {
    m_run_current_motion_only = false;
    return;
  }

  // Handle Keyboard events here
  int pressed_key = convert_to_remote_code(k);
  // End of simulation
  if (pressed_key == -1)
    return;

  if (m_current_scenario != NO_MOTION_INDEX) {
    m = m_scenarios[m_current_scenario];
    unsigned int new_pos = m->GetNextItem();
    if (new_pos != NO_MOTION_INDEX) {
      RCBMotionItem &item = m->m_items[new_pos];
      m_current_scenario_position = new_pos;
      m_current_motion = item.m_params.front();
    } else {
      m_current_scenario_position = NO_MOTION_INDEX;
      m_current_scenario = NO_MOTION_INDEX;
    }
  }

  if (m_current_motion == NO_MOTION_INDEX && pressed_key != 0) {
    search_key_activated_motion(pressed_key);

    if (m_current_scenario != NO_MOTION_INDEX) {
      // We have started a new scenario
      m = m_scenarios[m_current_scenario];
      int start_pos = m->m_start;
      if (start_pos != NO_MOTION_INDEX) {
        RCBMotionItem &item = m->m_items[start_pos];
        m_current_motion = item.m_params.front();
      } else  // Invalid scenario, stop here
        m_current_scenario = NO_MOTION_INDEX;
    }
  }

  get_ad_values();
  if (m_realtime_mixing_count != 0)
    apply_mixing(true);
  wait(100);
  wb_keyboard_get_key();
}

void rcb3_simulator::wait(int ms) {
  const int nTimeSteps = ceil((double)ms / (double)TIME_STEP);
  for (int i = 0; i < nTimeSteps; i++)
    wb_robot_step(TIME_STEP);
}

int rcb3_simulator::convert_to_remote_code(int key) {
  if (key == -1)
    return 0;

  // Special case for exit.
  // Escape key
  if (key == 27)
    return -1;

  const int normal_key = key & WB_KEYBOARD_KEY;
  switch (normal_key) {
    case '1':
      return 288;  // zannen
    case '2':
      return 32;  // pushups
    case '3':
      return 96;  // happy
    case '4':
      return 256;  // punch left
    case '5':
      return 1537;  // demo all
    case '6':
      return 64;  // punch right
    case '7':
      return 272;  // kick left
    case '8':
      return 16;  // bow
    case '9':
      return 80;  // kick right
    case 'R':
      return 1;  // walk forward
    case 'V':
      return 2;  // walk backward
    case 'C':
      return 10;  // turn left
    case 'B':
      return 6;  // turn right
    case 'E':
      return 9;  // walk diag left
    case 'T':
      return 5;  // walk diag right
    case 'D':
      return 8;  // side step left
    case 'G':
      return 4;  // side step right
    case 'Z':
      return 1025;  // stand up (face down or up)
    case 'N':
      return 1026;  // stand up (face down)
    case 'I':
      return 528;  // forward wheel
    case 'K':
      return 544;  // backward wheel
    case 'U':
      return 768;  // left wheel
    case 'O':
      return 576;  // right wheel
    case 'Q':
      log(1, "Scenario stopped: %d\n", m_simulation_delay);
      m_current_scenario = NO_MOTION_INDEX;
      break;
    case '[':
      if (m_simulation_delay > 0) {
        m_simulation_delay -= 10;
        log(1, "New simulation step delay %d ms\n", m_simulation_delay);
      }
      break;
    case ']':
      m_simulation_delay += 10;
      log(1, "New simulation step delay %d ms\n", m_simulation_delay);
      break;
    default:
      break;
  }
  return 0;
}

bool rcb3_simulator::load_motion(RCBMotion *m, unsigned int store_index) {
  if (store_index >= m_motions.size())
    return false;

  if (m_motions[store_index])
    delete m_motions[store_index];

  m_motions[store_index] = m;
  return true;
}

bool rcb3_simulator::load_scenario(RCBMotion *s, unsigned int store_index) {
  if (store_index >= m_scenarios.size())
    return false;

  if (m_scenarios[store_index])
    delete m_scenarios[store_index];

  m_scenarios[store_index] = s;

  return true;
}

bool rcb3_simulator::load_scenario(const char *filename, unsigned int store_index) {
  if (store_index >= m_scenarios.size())
    return false;

  RCBMotion *s = new RCBMotion;
  if (read_rcb_file(filename, *s))
    return load_scenario(s, store_index);

  delete s;
  return false;
}

bool rcb3_simulator::load_motion(const char *filename, unsigned int store_index) {
  if (store_index >= m_motions.size())
    return false;

  RCBMotion *m = new RCBMotion;
  if (read_rcb_file(filename, *m))
    return load_motion(m, store_index);
  else {
    delete m;
    return false;
  }
}

bool rcb3_simulator::read_rcb_file(const char *filename, RCBMotion &motion) {
  if (!filename)
    return false;

  FILE *f = fopen(filename, "r");

  if (f == NULL)
    return false;

  motion.m_name = strdup(filename);
  int status = 0;
  int link_index = 0;
  int item_index = 0;
  char buffer[512];

  RCBMotionLink link;
  RCBMotionItem item;

  item.m_name = NULL;
  while (!feof(f)) {
    memset(buffer, 0, 512);
    char *ptr = fgets(buffer, 512, f);
    if (ptr == NULL)
      break;
    if (buffer[0] == '[') {
      if (status == 2) {
        motion.m_items[item_index] = item;
        motion.m_items[item_index].m_name = item.m_name;
        item.m_name = NULL;
        item.m_params.clear();
      } else if (status == 3) {
        motion.m_links[link_index] = link;
        link.m_points.clear();
      }

      if (strstr(buffer + 1, "GraphicalEdit") != NULL)
        status = 1;
      else if (strstr(buffer + 1, "Item") != NULL) {
        status = 2;
        item_index = atoi(buffer + 5);
        item.m_index = item_index;
      } else if (strstr(buffer + 1, "Link") != NULL) {
        status = 3;
        link_index = atoi(buffer + 5);
        link.m_index = link_index;
      }

    } else {
      // Reading: GraphicalEdit
      if (status == 1) {
        if (strstr(buffer, "Type") != NULL) {
          motion.m_type = atoi(buffer + 5);
        } else if (strstr(buffer, "Items") != NULL) {
          motion.m_item_count = atoi(buffer + 6);
          motion.m_items.resize(motion.m_item_count);
        } else if (strstr(buffer, "Links") != NULL) {
          motion.m_link_count = atoi(buffer + 6);
          motion.m_links.resize(motion.m_link_count);
        } else if (strstr(buffer, "Start") != NULL) {
          motion.m_start = atoi(buffer + 6);
        } else if (strstr(buffer, "Ctrl") != NULL) {
          motion.m_control = atoi(buffer + 5);
        }

      } else if (status == 2) {
        // Reading: Item
        if (strncmp(buffer, "Width", 5) == 0)
          item.m_width = atoi(buffer + 6);
        else if (strncmp(buffer, "Height", 6) == 0)
          item.m_height = atoi(buffer + 7);
        else if (strncmp(buffer, "Left", 4) == 0)
          item.m_left = atoi(buffer + 5);
        else if (strncmp(buffer, "Top", 3) == 0)
          item.m_top = atoi(buffer + 4);
        else if (strncmp(buffer, "Color", 5) == 0)
          item.m_color = atoi(buffer + 6);
        else if (strncmp(buffer, "Type", 4) == 0)
          item.m_type = atoi(buffer + 5);
        else if (strncmp(buffer, "Name", 4) == 0)
          item.m_name = strdup(buffer + 5);
        else if (strncmp(buffer, "Prm", 3) == 0) {
          char *token = strtok(buffer + 4, ",");
          while (token) {
            int v = atoi(token);
            token = strtok(NULL, ",");
            item.m_params.push_back(v);
          }
        }

      } else if (status == 3) {
        // Reading: Link
        if (strstr(buffer, "Origin") != NULL) {
          link.m_origin = atoi(buffer + 7);
        } else if (strstr(buffer, "Point") != NULL) {
          char *token = strtok(buffer + 6, ", ");
          while (token) {
            if (isdigit(*token)) {
              int v = atoi(token);
              token = strtok(NULL, ",");
              link.m_points.push_back(v);
            } else
              token = strtok(NULL, ",");
          }
        } else if (strstr(buffer, "Final") != NULL)
          link.m_final = atoi(buffer + 6);
        else if (strstr(buffer, "Main") != NULL)
          link.m_main = atoi(buffer + 5);
      }
    }
  }

  if (status == 2) {
    motion.m_items[item_index] = item;
    motion.m_items[item_index].m_name = item.m_name;
    item.m_params.clear();
  } else if (status == 3)
    motion.m_links[link_index] = link;

  fclose(f);

  return true;
}

void rcb3_simulator::search_key_activated_motion(int key) {
  for (unsigned int i = 0; i < m_motions.size(); i++) {
    if (m_motions[i]) {
      if (m_motions[i]->m_control == key) {
        m_current_motion = i;
        return;
      }
    }
  }

  for (unsigned int i = 0; i < m_scenarios.size(); i++) {
    if (m_scenarios[i]) {
      if (m_scenarios[i]->m_control == key) {
        m_scenarios[i]->m_current_pos = m_scenarios[i]->m_start;
        m_current_scenario = i;
        break;
      }
    }
  }
}

// Information for Gyro
// range: +-300deg/sec
// standby value: 1.35v
// sensitivity: 0.67mv / deg / sec
// linearity: +-550 %FS
// answer: 0.4Hz
static double gyro_angle_to_voltage(double angle) {
  double v = angle * 180.0 / M_PI;
  v *= (0.67 / 1000.0);  // scale it
  v += 1.35;             // Add standby value
  return v;
}

void rcb3_simulator::get_ad_values() {
  const double *gyroValues = wb_gyro_get_values(m_gyroTag);
  double pitch = gyro_angle_to_voltage(gyroValues[0]);
  m_ad[0] = (int)(pitch * 1024) / 5;  // 10 bit value for a voltage between 0 and 5v.
  log(5, "new ad1 value : %d (%f)\n", m_ad[0], pitch);

  double roll = gyro_angle_to_voltage(gyroValues[1]);
  m_ad[1] = (int)(roll * 1024) / 5;  // 10 bit value for a voltage between 0 and 5v.
  log(5, "new ad2 value : %d (%f)\n", m_ad[1], roll);

  const double *accValues = wb_accelerometer_get_values(m_accelerometerTag);
  // Now convert that value into the same output from the RCB3 Analog 2 Digital converter
  m_ad[2] = (int)(accValues[1] * 55) + 380;
  log(5, "new ad3 value : %d (%f)\n", m_ad[2], accValues[1]);
}

void rcb3_simulator::fill_mixing_data(int channel, int ad, int mixing_value, bool realtime) {
  bool was_active = m_ad_mixing[channel][ad];

  bool ad_mixing = (mixing_value > 0x80);
  int ad_reverse_mixing = (mixing_value & 0x40) ? -1 : 1;
  int ad_mixing_factor = (mixing_value & 0x3F) * ad_reverse_mixing;

  if (realtime) {
    m_realtime_ad_mixing[channel][ad] = ad_mixing;
    m_realtime_ad_mixing_factor[channel][ad] = ad_mixing_factor;
  } else {
    m_ad_mixing[channel][ad] = ad_mixing;
    m_ad_mixing_factor[channel][ad] = ad_mixing_factor;
  }

  if (ad_mixing) {
    log(4, "Activating mixing on channel : %d Input %d\n", channel, ad + 1);
  }

  if (realtime) {
    if (was_active && !ad_mixing)
      m_realtime_mixing_count--;
    else if (!was_active && ad_mixing)
      m_realtime_mixing_count++;
  } else {
    if (was_active && !ad_mixing)
      m_motion_mixing_count--;
    else if (!was_active && ad_mixing)
      m_motion_mixing_count++;
  }
}

// Modify the desired position according to the mixing parameters and ad inputs.
bool rcb3_simulator::apply_mixing(bool realtime) {
  bool moved = false;

  for (int j = 0; j < CHANNEL_COUNT_RCB3; j++) {
    for (int ad = 0; ad < 3; ad++) {
      int mixing_factor;
      bool mixing;
      if (realtime) {
        mixing_factor = m_realtime_ad_mixing_factor[j][ad];
        mixing = m_realtime_ad_mixing[j][ad];
      } else {
        mixing_factor = m_ad_mixing_factor[j][ad];
        mixing = m_ad_mixing[j][ad];
      }

      if (mixing) {
        int delta = (m_ad[ad] - m_ad_origin[ad]);
        if (delta != 0) {
          moved = true;
          int mod = mixing_factor * (m_ad[ad] - m_ad_origin[ad]);
          m_desired_joint_position[j] += mod;

          log(4, "Mixing: channel %d Adding %d\n", j, mod);
        }
      }
    }
  }
  return moved;
}

RCBMotion::RCBMotion() {
  m_name = NULL;
  m_type = 0;
  m_start = 0;
  m_current_pos = 0;
  m_control = 0;
  m_item_count = 0;
  m_link_count = 0;
}

unsigned int RCBMotion::GetNextItem(bool default_jump) {
  bool found = false;
  for (int l = 0; l < m_link_count; l++) {
    const RCBMotionLink &link = m_links[l];
    if (link.m_origin == m_current_pos) {
      if (default_jump && (link.m_main == 0)) {
        log(3, "Link %d\n", link.m_index);
        m_current_pos = link.m_final;
        found = true;
        break;
      }
      if (!default_jump && (link.m_main != 0)) {
        log(3, "Link %d\n", link.m_index);
        m_current_pos = link.m_final;
        found = true;
        break;
      }
    }
  }
  if (!found) {
    // Reset it to point at the start.
    m_current_pos = 0;
    return NO_MOTION_INDEX;
  }

  return m_current_pos;
}

int rcb3_simulator::get_next_scenario_slot() const {
  for (int i = 0; i < (int)m_scenarios.size(); i++) {
    if (m_scenarios[i] == NULL)
      return i;
  }
  return -1;
}

int rcb3_simulator::get_next_motion_slot() const {
  for (int i = 0; i < (int)m_motions.size(); i++) {
    if (m_motions[i] == NULL)
      return i;
  }
  return -1;
}
