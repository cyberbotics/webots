#include "MeshBasicMaterial.hpp"

Color::Color(double r, double g, double b) {
  mR = r;
  mG = g;
  mB = b;
  mIsColor = true;
}
bool Color::isColor() const {
  return mIsColor;
}
