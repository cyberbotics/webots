#ifndef WR_TEXTURE_TRANSFORM_H
#define WR_TEXTURE_TRANSFORM_H

#ifdef __cplusplus
extern "C" {
#endif

struct WrTextureTransform;
typedef struct WrTextureTransform WrTextureTransform;

WrTextureTransform *wr_texture_transform_new();
void wr_texture_transform_delete(WrTextureTransform *transform);

void wr_texture_transform_set_center(WrTextureTransform *transform, float center_x, float center_y);
void wr_texture_transform_set_position(WrTextureTransform *transform, float position_x, float position_y);
void wr_texture_transform_set_rotation(WrTextureTransform *transform, float rotation);
void wr_texture_transform_set_scale(WrTextureTransform *transform, float scale_x, float scale_y);
/* Expects a 2-component array */
void wr_texture_transform_apply_to_uv_coordinate(WrTextureTransform *transform, float *coord);

#ifdef __cplusplus
}
#endif

#endif  // WR_TEXTURE_TRANSFORM_H
