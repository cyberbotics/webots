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
 * Adapted by Nicolas Heiniger for the e-puck
 */

#ifndef ODOMETRY
#define ODOMETRY

struct sOdometryTrack {
  struct {
    float wheel_distance;
    float wheel_conversion_left;
    float wheel_conversion_right;
  } configuration;
  struct {
    int pos_left_prev;
    int pos_right_prev;
  } state;
  struct {
    float x;
    float y;
    float theta;
  } result;
};

//! Initializes this module.
void odometry_track_init();

//! Initializes an sOdometryTrack structure using the given left and right position.
int odometry_track_start_pos(struct sOdometryTrack *ot, int pos_left, int pos_right);
//! Updates the position using the given left and right position.
void odometry_track_step_pos(struct sOdometryTrack *ot, int pos_left, int pos_right);

#endif
