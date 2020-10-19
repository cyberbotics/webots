#include "Color.h"
#include "Color.hpp"

#include <math.h>

namespace wren {

  Color::Color(double r, double g, double b) : mR(r), mG(g), mB(b), mIsColor(true) {}
  bool Color::isColor() const { return mIsColor; }
  double Color::r() const { return mR; }
  double Color::g() const { return mG; }
  double Color::b() const { return mB; }

  double Color::linearToSRGB(double c) { return (c < 0.0031308) ? c * 12.92 : 1.055 * (pow(c, 0.41666)) - 0.055; }

  Color *Color::convertLinearToSRGB() {
    double newR = linearToSRGB(mR);
    double newG = linearToSRGB(mG);
    double newB = linearToSRGB(mB);
    return createColor(newR, newG, newB);
  }
}  // namespace wren

WrColor *wr_color_new(double r, double g, double b) {
  return reinterpret_cast<WrColor *>(wren::Color::createColor(r, g, b));
}

bool wr_color_is_color(WrColor *color) {
  return reinterpret_cast<wren::Color *>(color)->isColor();
}

double wr_color_r(WrColor *color) {
  return reinterpret_cast<wren::Color *>(color)->r();
}

double wr_color_g(WrColor *color) {
  return reinterpret_cast<wren::Color *>(color)->g();
}

double wr_color_b(WrColor *color) {
  return reinterpret_cast<wren::Color *>(color)->b();
}

WrColor *wr_color_convertLinearToSRGB(WrColor *color) {
  return reinterpret_cast<WrColor *>(reinterpret_cast<wren::Color *>(color)->convertLinearToSRGB());
}
