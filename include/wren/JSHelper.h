#ifndef WR_JSHELPER_H
#define WR_JSHELPER_H

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

float *wrjs_color_array(float r, float g, float b);
float *wrjs_array4(float element0, float element1, float element2, float element3);
int *wrjs_pointerOnInt(int nbr);
char *wrjs_pointerOnFloat(float nbr);
char *wrjs_dummy_texture();

void wrjs_init_context(int width, int height);
const char *wrjs_load_hdr_file(int *w, char *url);
const char *wrjs_load_hdr_file(int *w, char *url);
void wrjs_free_hdr_file(char *pointer);
#ifdef __cplusplus
}
#endif

#endif  // WR_VIEWPORT_H
