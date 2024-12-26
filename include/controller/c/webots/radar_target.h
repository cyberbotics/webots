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

/**************************************************************************************/
/* Description:  Common definition of the 'WbRadarTarget' for both the C and C++ APIs */
/**************************************************************************************/

#ifndef WB_RADAR_TARGET_H
#define WB_RADAR_TARGET_H

typedef struct {
  double distance;
  double received_power;
  double speed;
  double azimuth;
} WbRadarTarget;

#endif /* WB_RADAR_TARGET_H */
