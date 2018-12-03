#ifndef WR_CAMERA_H
#define WR_CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Inheritance diagram: WrNode <- WrCamera */
struct WrCamera;
typedef struct WrCamera WrCamera;

typedef enum WrCameraProjectionMode {
  WR_CAMERA_PROJECTION_MODE_PERSPECTIVE,
  WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC
} WrCameraProjectionMode;

/* Use wr_node_delete(WR_NODE(camera)) to delete an instance */
WrCamera *wr_camera_new();

void wr_camera_set_projection_mode(WrCamera *camera, WrCameraProjectionMode mode);
void wr_camera_set_aspect_ratio(WrCamera *camera, float ratio);
void wr_camera_set_near(WrCamera *camera, float near_distance);
void wr_camera_set_far(WrCamera *camera, float far_distance);
void wr_camera_set_fovy(WrCamera *camera, float fovy);
void wr_camera_set_height(WrCamera *camera, float height);
void wr_camera_set_position(WrCamera *camera, float *position);
void wr_camera_set_flip_y(WrCamera *camera, bool flip_y);
/* Expects a 4-component array */
void wr_camera_set_orientation(WrCamera *camera, float *angle_axis);
void wr_camera_apply_yaw(WrCamera *camera, float angle);
void wr_camera_apply_pitch(WrCamera *camera, float angle);
void wr_camera_apply_roll(WrCamera *camera, float angle);

float wr_camera_get_near(WrCamera *camera);
float wr_camera_get_far(WrCamera *camera);
float wr_camera_get_fovy(WrCamera *camera);
float wr_camera_get_height(WrCamera *camera);

#ifdef __cplusplus
}
#endif

#endif  // WR_CAMERA_H
