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

/**********************************************************************************/
/* Description:  Webots C programming interface for the RangeFinder node          */
/**********************************************************************************/

#ifndef WB_RANGE_FINDER_H
#define WB_RANGE_FINDER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_range_finder_enable(WbDeviceTag tag, int sampling_period);
void wb_range_finder_disable(WbDeviceTag tag);
int wb_range_finder_get_sampling_period(WbDeviceTag tag);

const float *wb_range_finder_get_range_image(WbDeviceTag tag);
int wb_range_finder_get_width(WbDeviceTag tag);
int wb_range_finder_get_height(WbDeviceTag tag);
double wb_range_finder_get_fov(WbDeviceTag tag);
double wb_range_finder_get_min_range(WbDeviceTag tag);
double wb_range_finder_get_max_range(WbDeviceTag tag);
int wb_range_finder_save_image(WbDeviceTag tag, const char *filename, int quality);

// range finder functions
#define wb_range_finder_image_get_depth(image, width, x, y) (image[(y) * (width) + (x)])

#ifdef __cplusplus
}
#endif

#endif /* WB_RANGE_FINDER_H */
