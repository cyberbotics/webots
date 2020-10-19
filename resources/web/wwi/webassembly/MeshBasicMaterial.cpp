#include "MeshBasicMaterial.hpp"

#include "MeshBasicMaterial.h"

namespace wren {
  MeshBasicMaterial::MeshBasicMaterial() : mIsVisible(true) {}

  bool MeshBasicMaterial::isVisible() const { return mIsVisible; }
}  // namespace wren

WrMeshBasicMaterial *wr_mesh_basic_material_new() {
  return reinterpret_cast<WrMeshBasicMaterial *>(wren::MeshBasicMaterial::createMeshBasicMaterial());
}

bool wr_mesh_basic_material_is_visible(WrMeshBasicMaterial *meshBasicMaterial) {
  return reinterpret_cast<wren::MeshBasicMaterial *>(meshBasicMaterial)->isVisible();
}
