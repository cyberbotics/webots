#ifndef WR_JSHELPER_H
#define WR_JSHELPER_H

#ifdef __EMSCRIPTEN__

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

float *wrjs_color_array(float r, float g, float b);
// char *wr_string(int nChar, ...);
char *wrjs_dummy_texture();
void wrjs_init_context(int width, int height);

#ifdef __cplusplus
}
#endif

#endif

#endif  // WR_VIEWPORT_H
