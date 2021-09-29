#ifndef WR_SKELETON_H
#define WR_SKELETON_H

#ifdef __cplusplus
extern "C" {
#endif

struct WrDynamicMesh;
typedef struct WrDynamicMesh WrDynamicMesh;

struct WrSkeleton;
typedef struct WrSkeleton WrSkeleton;

struct WrSkeletonBone;
typedef struct WrSkeletonBone WrSkeletonBone;

WrSkeleton *wr_skeleton_new();

int wr_skeleton_get_bone_count(WrSkeleton *skeleton);
WrSkeletonBone *wr_skeleton_get_bone_by_index(WrSkeleton *skeleton, int index);
WrSkeletonBone *wr_skeleton_get_bone_by_name(WrSkeleton *skeleton, const char *name);

void wr_skeleton_apply_binding_pose(WrSkeleton *skeleton);

// Only suitable when skeleton is attached to another transform
void wr_skeleton_update_offset(WrSkeleton *skeleton);

float *wr_skeleton_compute_bounding_spheres(WrSkeleton *skeleton, int &count);

#ifdef __cplusplus
}
#endif

#endif  // WR_SKELETON_H
