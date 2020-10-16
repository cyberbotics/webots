#ifndef COLOR_H
#define COLOR_H

extern "C" {
struct WrColor;
typedef struct WrColor WrColor;

WrColor *wr_color_new(double r, double g, double b);
bool wr_color_is_color(WrColor *color);
double wr_color_r(WrColor *color);
double wr_color_g(WrColor *color);
double wr_color_b(WrColor *color);
WrColor *wr_color_convertLinearToSRGB(WrColor *color);
}

#endif
