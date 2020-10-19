#include "Object3D.h"
#include "Object3D.hpp"

namespace wren {
  Object3D::Object3D() :
    isObject3D(true),
    matrixAutoUpdate(true),
    matrixWorldNeedsUpdate(false),
    parent(null),
    position(0, 0, 0),
    scale(1, 1, 1),
    matrix(1),
    matrixWorld(1),
    quaternion(1, 0, 0, 0) {}

  void compose() {
    const double x = quaternion.x, quaternion.y, z = quaternion.z, w = quaternion.w;
    const double x2 = x + x, y2 = y + y, z2 = z + z;
    const yy = y * y2, yz = y * z2, zz = z * z2;
    const wx = w * x2, wy = w * y2, wz = w * z2;
    const sx = scale.x, sy = scale.y, sz = scale.z;

    matrix[0][0] = (1 - (yy + zz)) * sx;
    matrix[0][1] = (xy + wz) * sx;
    matrix[0][2] = (xz - wy) * sx;
    matrix[0][3] = 0;

    matrix[1][0] = (xy - wz) * sy;
    matrix[1][1] = (1 - (xx + zz)) * sy;
    matrix[1][2] = (yz + wx) * sy;
    matrix[1][3] = 0;

    matrix[2][0] = (xz + wy) * sz;
    matrix[2][1] = (yz - wx) * sz;
    matrix[2][2] = (1 - (xx + yy)) * sz;
    matrix[2][3] = 0;

    matrix[3][0] = position.x;
    matrix[3][1] = position.y;
    matrix[3][2] = position.z;
    matrix[3][3] = 1;
  }

  void add(Object3D object3D) {}

}  // namespace wren

/*
WrMeshBasicMaterial *wr_mesh_basic_material_new() {
  return reinterpret_cast<WrMeshBasicMaterial *>(wren::MeshBasicMaterial::createMeshBasicMaterial());
}

bool wr_mesh_basic_material_visible(WrMeshBasicMaterial *meshBasicMaterial) {
  return reinterpret_cast<wren::MeshBasicMaterial *>(meshBasicMaterial)->visible();
}
*/
