/*
 * Copyright 1996-2018 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*!
 * (c) 2006 - 2008 EPFL, Lausanne, Switzerland
 * Thomas Lochmatter
 */

#ifndef ODOMETRY_GOTO
#define ODOMETRY_GOTO

#include "odometry.h"

struct sOdometryGoto {
  struct {
    float speed_min;
  } configuration;
  struct sOdometryTrack *track;
  struct {
    float goal_x;
    float goal_y;
    float goal_theta;
  } state;
  struct {
    int speed_left;
    int speed_right;
    int atgoal;
  } result;
};

//! Initializes this module.
void odometry_goto_init();

//! Initializes a goto structure.
void odometry_goto_start(struct sOdometryGoto *og, struct sOdometryTrack *ot);
//! Sets a new target position.
void odometry_goto_set_goal(struct sOdometryGoto *og, float goal_x, float goal_y, float goal_theta);
//! Calculates the new motor speeds.
void odometry_goto_step(struct sOdometryGoto *og);

#endif
