#include "MeshBasicMaterial.hpp"

#include "MeshBasicMaterial.h"

namespace wren {
  MeshBasicMaterial::MeshBasicMaterial() { mVisible = true; }

  bool MeshBasicMaterial::visible() const { return mVisible; }
}  // namespace wren

WrMeshBasicMaterial *wr_mesh_basic_material_new() {
  return reinterpret_cast<WrMeshBasicMaterial *>(wren::MeshBasicMaterial::createMeshBasicMaterial());
}

bool wr_mesh_basic_material_visible(WrMeshBasicMaterial *meshBasicMaterial) {
  return reinterpret_cast<wren::MeshBasicMaterial *>(meshBasicMaterial)->visible();
}
