#include <emscripten.h>
#include <emscripten/bind.h>
#include <stdio.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace emscripten;

enum MOUSE { LEFT, MIDDLE, RIGHT };

EMSCRIPTEN_BINDINGS(enumeration) {
  enum_<MOUSE>("MOUSE").value("LEFT", LEFT).value("MIDDLE", MIDDLE).value("RIGHT", RIGHT);
}

/*
glm::mat4 lookAt(glm::vec3 eye, glm::vec3 center, glm::vec3 up) {
  return glm::lookAt(eye, center, up);
}
*/

float length(glm::vec3 vec3) {
  return glm::length(vec3);
}

EMSCRIPTEN_BINDINGS(glm) {
  class_<glm::vec2>("Vector2")
    .constructor<>()
    .constructor<double, double>()
    .property("x", &glm::vec2::x)
    .property("y", &glm::vec2::y);

  class_<glm::vec3>("Vector3")
    .constructor<>()
    .constructor<double, double, double>()
    .property("x", &glm::vec3::x)
    .property("y", &glm::vec3::y)
    .property("z", &glm::vec3::z);
  function("length", &length);
}
