#ifndef WR_VIEWPORT_H
#define WR_VIEWPORT_H

#ifdef __cplusplus
extern "C" {
#endif

struct WrViewport;
typedef struct WrViewport WrViewport;

struct WrCamera;
typedef struct WrCamera WrCamera;

struct WrFrameBuffer;
typedef struct WrFrameBuffer WrFrameBuffer;

struct WrOverlay;
typedef struct WrOverlay WrOverlay;

struct WrPostProcessingEffect;
typedef struct WrPostProcessingEffect WrPostProcessingEffect;

typedef enum WrViewportPolygonMode {
  WR_VIEWPORT_POLYGON_MODE_POINT = 0x1B00,  // GL_POINT
  WR_VIEWPORT_POLYGON_MODE_LINE = 0x1B01,   // GL_LINE
  WR_VIEWPORT_POLYGON_MODE_FILL = 0x1B02    // GL_FILL
} WrViewportPolygonMode;

WrViewport *wr_viewport_new();
void wr_viewport_delete(WrViewport *viewport);

/* Expects a 3-component array */
void wr_viewport_set_clear_color_rgb(WrViewport *viewport, const float *color);
/* Expects a 4-component array */
void wr_viewport_set_clear_color_rgba(WrViewport *viewport, const float *color);
void wr_viewport_set_polygon_mode(WrViewport *viewport, WrViewportPolygonMode mode);
void wr_viewport_set_visibility_mask(WrViewport *viewport, int mask);
void wr_viewport_set_size(WrViewport *viewport, int width, int height);
void wr_viewport_set_pixel_ratio(WrViewport *viewport, int ratio);
void wr_viewport_set_camera(WrViewport *viewport, WrCamera *camera);
void wr_viewport_set_frame_buffer(WrViewport *viewport, WrFrameBuffer *frame_buffer);

void wr_viewport_enable_shadows(WrViewport *viewport, bool enable);
void wr_viewport_enable_skybox(WrViewport *viewport, bool enable);
/* Allows or not the viewport to modify the camera aspect ratio when its size changes */
void wr_viewport_sync_aspect_ratio_with_camera(WrViewport *viewport, bool enable);

void wr_viewport_attach_overlay(WrViewport *viewport, WrOverlay *overlay);
void wr_viewport_detach_overlay(WrViewport *viewport, WrOverlay *overlay);
/* Use these functions only if you are not re-rendering the viewport but you want to force the overlay(s) to be repainted */
void wr_viewport_render_overlay(WrViewport *viewport, WrOverlay *overlay);
void wr_viewport_render_overlays(WrViewport *viewport);

void wr_viewport_add_post_processing_effect(WrViewport *viewport, WrPostProcessingEffect *post_processing_effect);
void wr_viewport_remove_post_processing_effect(WrViewport *viewport, WrPostProcessingEffect *post_processing_effect);

void wr_viewport_set_ambient_occlusion_effect(WrViewport *viewport, WrPostProcessingEffect *ambient_occlusion_effect);
void wr_viewport_set_anti_aliasing_effect(WrViewport *viewport, WrPostProcessingEffect *anti_aliasing_effect);

int wr_viewport_get_width(WrViewport *viewport);
int wr_viewport_get_height(WrViewport *viewport);
WrCamera *wr_viewport_get_camera(WrViewport *viewport);
WrFrameBuffer *wr_viewport_get_frame_buffer(WrViewport *viewport);

#ifdef __cplusplus
}
#endif

#endif  // WR_VIEWPORT_H
