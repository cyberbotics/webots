#ifndef WR_OVERLAY_H
#define WR_OVERLAY_H

#ifdef __cplusplus
extern "C" {
#endif

struct WrOverlay;
typedef struct WrOverlay WrOverlay;

struct WrShaderProgram;
typedef struct WrShaderProgram WrShaderProgram;

struct WrTexture;
typedef struct WrTexture WrTexture;

void wr_overlay_set_screen_ratio(float ratio);

WrOverlay *wr_overlay_new();
void wr_overlay_delete(WrOverlay *overlay);

void wr_overlay_set_order(WrOverlay *overlay, int order);
void wr_overlay_put_on_top(WrOverlay *overlay);
void wr_overlay_set_visible(WrOverlay *overlay, bool visible);
void wr_overlay_show_default_size(WrOverlay *overlay, bool visible);
void wr_overlay_set_default_size(WrOverlay *overlay, float width, float height);
void wr_overlay_set_size(WrOverlay *overlay, float width, float height);
void wr_overlay_set_position(WrOverlay *overlay, float x, float y);
void wr_overlay_set_use_premultiplied_alpha_textures(WrOverlay *overlay, bool enabled);
void wr_overlay_set_anisotropy(WrOverlay *overlay, float anisotropy);
void wr_overlay_set_translucency(WrOverlay *overlay, bool enabled);
void wr_overlay_enable_interpolation(WrOverlay *overlay, bool enabled);
void wr_overlay_enable_mip_maps(WrOverlay *overlay, bool enable);
/* Expects a 4-component array */
void wr_overlay_set_background_color(WrOverlay *overlay, const float *color);

void wr_overlay_set_border_active(WrOverlay *overlay, bool active);
/* Expects a 4-component array */
void wr_overlay_set_border_color(WrOverlay *overlay, const float *color);
void wr_overlay_set_border_size(WrOverlay *overlay, float size_horizontal, float size_vertical);

void wr_overlay_set_texture(WrOverlay *overlay, WrTexture *texture);
void wr_overlay_set_texture_flip_vertical(WrOverlay *overlay, bool flip);
void wr_overlay_set_background_texture(WrOverlay *overlay, WrTexture *texture);
void wr_overlay_set_mask_texture(WrOverlay *overlay, WrTexture *texture);
void wr_overlay_set_foreground_texture(WrOverlay *overlay, WrTexture *texture);
void wr_overlay_add_additional_texture(WrOverlay *overlay, WrTexture *texture);
void wr_overlay_set_program(WrOverlay *overlay, WrShaderProgram *program);
void wr_overlay_set_default_size_program(WrOverlay *overlay, WrShaderProgram *program);

void wr_overlay_set_max_range(WrOverlay *overlay, float max_range);

bool wr_overlay_is_visible(WrOverlay *overlay);
int wr_overlay_get_order(WrOverlay *overlay);
float wr_overlay_get_x(WrOverlay *overlay);
float wr_overlay_get_y(WrOverlay *overlay);
float wr_overlay_get_width(WrOverlay *overlay);
float wr_overlay_get_height(WrOverlay *overlay);

/* allows this overlay to live in the cache even if the scene is reset */
void wr_overlay_set_cache_persistency(WrOverlay *overlay, bool is_persistent);

#ifdef __cplusplus
}
#endif

#endif  // WR_OVERLAY_H
