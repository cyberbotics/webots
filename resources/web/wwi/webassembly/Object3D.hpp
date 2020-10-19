#ifndef OBJECT3D_HPP
#define OBJECT3D_HPP

#include <glm/glm.hpp>

#include "Object3D.h"

namespace wren {
  class Object3D {
  public:
    static Object3D *createObject3D() { return new Object3D(); }
    void add(Object3D object3D);
    void remove(Object3D object3D);
    void updateMatrixWorld(bool force);

  private:
    Object3D();
    bool isObject3D;
    bool matrixAutoUpdate;
    bool matrixWorldNeedsUpdate;
    Object3D parent;
    std::vector<Object3D> children;
    std::map<string, string> userData;
    glm::vec3 position;
    glm::vec3 scale;
    glm::mat4 matrix;
    glm::mat4 matrixWorld;
    glm::quat quaternion;

    void updateMatrix();
    void compose();
  };
}  // namespace wren
#endif
