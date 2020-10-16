#ifndef MESHBASICMATERIAL_HPP
#define MESHBASICMATERIAL_HPP

#include <emscripten.h>
#include <emscripten/bind.h>
using namespace emscripten;

class MeshBasicMaterial {
public:
  MeshBasicMaterial();
  bool visible() const;
};

EMSCRIPTEN_BINDINGS(MeshBasicMaterial) {
  class_<MeshBasicMaterial>("MeshBasicMaterial").constructor<>().property("isColor", &Color::isColor)
}

#endif
