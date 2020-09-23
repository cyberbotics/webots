#include <emscripten.h>
#include <emscripten/bind.h>
#include <stdio.h>

using namespace emscripten;

enum MOUSE { LEFT, MIDDLE, RIGHT };

EMSCRIPTEN_BINDINGS(enumeration) { enum_<MOUSE>("MOUSE")
  .value("LEFT", LEFT)
  .value("MIDDLE", MIDDLE)
  .value("RIGHT", RIGHT)
  ;
}
