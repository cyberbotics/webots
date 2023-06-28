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

#include <webots/types.h>

#include <vector>
#include "KHR2_Data.h"

class KHR2_Interface;

#define TIME_STEP 32

class rcb3_simulator {
public:
  rcb3_simulator();
  ~rcb3_simulator();

  void init();

  void play_motion(unsigned int store_index);
  void play_scenario(unsigned int store_index);
  void run();
  static void wait(int ms);

  bool load_motion(const char *filename, unsigned int store_index);
  bool load_scenario(const char *filename, unsigned int store_index);

  bool load_motion(RCBMotion *m, unsigned int store_index);
  bool load_scenario(RCBMotion *s, unsigned int store_index);

  int get_next_scenario_slot() const;
  int get_next_motion_slot() const;

private:
  // RCB3
  bool apply_mixing(bool realtime);
  void fill_mixing_data(int channel, int ad, int mixing_value, bool realtime);
  void get_ad_values();
  static bool read_rcb_file(const char *filename, RCBMotion &motion);
  int convert_to_remote_code(int key);
  void search_key_activated_motion(int key);

  WbDeviceTag m_joint[JOINT_COUNT];        /* all the motors */
  WbDeviceTag m_joint_sensor[JOINT_COUNT]; /* associated position sensor */
  WbDeviceTag m_accelerometerTag;
  WbDeviceTag m_gyroTag;

  float m_joint_position[CHANNEL_COUNT_RCB3];
  float m_desired_joint_position[CHANNEL_COUNT_RCB3];

  std::vector<RCBMotion *> m_motions;
  std::vector<RCBMotion *> m_scenarios;

  // RCB3 Internal data
  unsigned int m_current_scenario;
  unsigned int m_current_scenario_position;
  unsigned int m_current_motion;
  int m_loop_counter;
  int m_compare_register;
  int m_ad[AD_INPUT_COUNT];
  int m_button;
  bool m_run_current_motion_only;

  bool m_ad_mixing[CHANNEL_COUNT_RCB3][AD_INPUT_COUNT];
  int m_ad_mixing_factor[CHANNEL_COUNT_RCB3][AD_INPUT_COUNT];
  int m_motion_mixing_count;

  int m_ad_origin[AD_INPUT_COUNT];

  bool m_realtime_ad_mixing[CHANNEL_COUNT_RCB3][AD_INPUT_COUNT];
  int m_realtime_ad_mixing_factor[CHANNEL_COUNT_RCB3][AD_INPUT_COUNT];
  int m_realtime_mixing_count;

  int m_simulation_delay;
};
