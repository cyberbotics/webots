#ifndef WR_SKELETON_BONE_H
#define WR_SKELETON_BONE_H

#ifdef __cplusplus
extern "C" {
#endif

struct WrSkeleton;
typedef struct WrSkeleton WrSkeleton;

struct WrSkeletonBone;
typedef struct WrSkeletonBone WrSkeletonBone;

const char *wr_skeleton_bone_get_name(WrSkeletonBone *bone);

void wr_skeleton_bone_get_position(const WrSkeletonBone *bone, bool absolute, float *position);
void wr_skeleton_bone_get_orientation(const WrSkeletonBone *bone, bool absolute, float *orientation);

#ifdef __cplusplus
}
#endif

#endif  // WR_SKELETON_BONE_H
