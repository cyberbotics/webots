#ifndef WR_FONT_H
#define WR_FONT_H

#ifdef __cplusplus
extern "C" {
#endif

struct WrFont;
typedef struct WrFont WrFont;

typedef enum WrFontError {
  WR_FONT_ERROR_NONE,
  WR_FONT_ERROR_UNKNOWN_FILE_FORMAT,
  WR_FONT_ERROR_FONT_LOADING,
  WR_FONT_ERROR_FONT_SIZE,
  WR_FONT_ERROR_CHARACTER_LOADING,
  WR_FONT_ERROR_FREETYPE_LOADING
} WrFontError;

WrFont *wr_font_new();
void wr_font_delete(WrFont *font);

void wr_font_set_face(WrFont *font, const char *filename);
void wr_font_set_size(WrFont *font, unsigned int size);

void wr_font_get_bounding_box(WrFont *font, const char *text, int *width, int *height);

WrFontError wr_font_get_error(WrFont *font);

#ifdef __cplusplus
}
#endif

#endif  // WR_FONT_H
