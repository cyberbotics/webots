#ifndef WR_DRAWABLE_TEXTURE_H
#define WR_DRAWABLE_TEXTURE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Inheritance diagram: WrTexture2d <- WrDrawableTexture */
struct WrDrawableTexture;
typedef struct WrDrawableTexture WrDrawableTexture;

struct WrFont;
typedef struct WrFont WrFont;

WrDrawableTexture *wr_drawable_texture_new();

void wr_drawable_texture_set_font(WrDrawableTexture *texture, WrFont *font);
void wr_drawable_texture_set_color(WrDrawableTexture *texture, float *color);
void wr_drawable_texture_set_antialasing(WrDrawableTexture *texture, bool enabled);
void wr_drawable_texture_set_use_premultiplied_alpha(WrDrawableTexture *texture, bool premultipliedAlpha);

void wr_drawable_texture_draw_pixel(WrDrawableTexture *texture, int x, int y);
void wr_drawable_texture_draw_text(WrDrawableTexture *texture, const char *text, int x, int y);
void wr_drawable_texture_clear(WrDrawableTexture *texture);

#ifdef __cplusplus
}
#endif

#endif  // WR_DRAWABLE_TEXTURE_H
