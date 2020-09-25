#include "Color.hpp"

#include <math.h>

#include <emscripten.h>
#include <emscripten/bind.h>
using namespace emscripten;

Color::Color(double r, double g, double b) {
  mR = r;
  mG = g;
  mB = b;
  mIsColor = true;
}
bool Color::isColor() const {
  return mIsColor;
}
double Color::r() const {
  return mR;
}
double Color::g() const {
  return mG;
}
double Color::b() const {
  return mB;
}

double Color::linearToSRGB(double c) {
  return (c < 0.0031308) ? c * 12.92 : 1.055 * (pow(c, 0.41666)) - 0.055;
}

Color Color::convertLinearToSRGB() {
  double newR = linearToSRGB(mR);
  double newG = linearToSRGB(mG);
  double newB = linearToSRGB(mB);
  return *(new Color(newR, newG, newB));
}

EMSCRIPTEN_BINDINGS(Color) {
  class_<Color>("Color")
    .constructor<double, double, double>()
    .property("r", &Color::r)
    .property("g", &Color::g)
    .property("b", &Color::b)
    .property("isColor", &Color::isColor)
    .function("convertLinearToSRGB", &Color::convertLinearToSRGB);
}
