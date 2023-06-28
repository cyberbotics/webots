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

#ifndef __KHR2_Data_h
#define __KHR2_Data_h

#include <list>
#include <vector>

#define JOINT_COUNT 17
#define CHANNEL_COUNT_RCB3 24
#define NO_MOTION_INDEX 1000
#define AD_INPUT_COUNT 3

class RCBMotionLink {
public:
  int m_index;
  int m_origin;
  int m_final;
  int m_main;
  std::vector<int> m_points;
};

class RCBMotionItem {
public:
  int m_index;
  int m_type;
  int m_width;
  int m_height;
  int m_left;
  int m_top;
  int m_color;
  char *m_name;
  std::list<int> m_params;
};

class RCBMotion {
public:
  RCBMotion();

  int m_type;
  int m_start;
  int m_current_pos;
  int m_control;
  int m_item_count;
  int m_link_count;
  char *m_name;
  std::vector<RCBMotionItem> m_items;
  std::vector<RCBMotionLink> m_links;

  unsigned int GetNextItem(bool default_jump = true);
};

#endif  // __KHR2_Data_h
