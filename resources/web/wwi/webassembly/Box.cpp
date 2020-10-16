#include <emscripten.h>
#include <emscripten/bind.h>
using namespace emscripten;

class BoxBufferGeometry {
public:
  BoxBufferGeometry(float x, float y, float z) {}
  std::string name = "BoxBufferGeometry";

private:
  float x;
  float y;
  float z;
};

EMSCRIPTEN_BINDINGS(Box) {
  class_<BoxBufferGeometry>("BoxBufferGeometry").constructor<float, float, float>().property("name", &BoxBufferGeometry::name);
}
