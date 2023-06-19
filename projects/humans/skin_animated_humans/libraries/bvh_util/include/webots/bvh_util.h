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
 * Description:   BVH file format utility class to be used with 'Skin' node to animate the mesh.
 *                It provides function to read the BVH file and adapt it to the 'Skin' model mesh.
 */

#ifndef WBU_BVH_UTIL_H
#define WBU_BVH_UTIL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct WbuBvhMotionPrivate *WbuBvhMotion;

WbuBvhMotion wbu_bvh_read_file(const char *filename);
void wbu_bvh_cleanup(WbuBvhMotion motion);

const char *wbu_bvh_get_filename(WbuBvhMotion motion);
int wbu_bvh_get_joint_count(const WbuBvhMotion motion);
const char *wbu_bvh_get_joint_name(const WbuBvhMotion motion, int joint_id);

int wbu_bvh_get_frame_count(const WbuBvhMotion motion);
int wbu_bvh_get_frame_index(const WbuBvhMotion motion);
bool wbu_bvh_step(WbuBvhMotion motion);
bool wbu_bvh_goto_frame(WbuBvhMotion motion, int frame_number);
bool wbu_bvh_reset(WbuBvhMotion motion);

void wbu_bvh_set_scale(WbuBvhMotion motion, double scale);
const double *wbu_bvh_get_root_translation(const WbuBvhMotion motion);
const double *wbu_bvh_get_joint_rotation(const WbuBvhMotion motion, int joint_id);

void wbu_bvh_set_model_t_pose(const WbuBvhMotion motion, const double *axisAngle, int joint_id, bool global);

#ifdef __cplusplus
}
#endif

#endif  // WBU_BVH_UTIL_H
