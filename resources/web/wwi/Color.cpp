#include <emscripten.h>
#include <emscripten/bind.h>
using namespace emscripten;

class Color {
public:
  bool isColor = true;
  double r;
  double g;
  double b;
  Color(double r1, double g1, double b1) {
    r = r1;
    g = g1;
    b = b1;
  }
};

EMSCRIPTEN_BINDINGS(Color) {
  class_<Color>("Color")
    .constructor<double, double, double>()
    .property("r", &Color::r)
    .property("g", &Color::g)
    .property("b", &Color::b)
    .property("isColor", &Color::isColor);
}
