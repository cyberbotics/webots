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
