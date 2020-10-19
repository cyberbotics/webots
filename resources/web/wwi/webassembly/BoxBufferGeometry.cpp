#include "BoxBufferGeometry.hpp"

#include "BoxBufferGeometry.h"

namespace wren {
  MeshBasicMaterial::MeshBasicMaterial() { mVisible = true; }

  bool MeshBasicMaterial::visible() const { return mVisible; }
}  // namespace wren

WrBoxBufferGeometry *wr_box_buffer_geometry_new() {
  return reinterpret_cast<WrBoxBufferGeometry *>(wren::BoxBufferGeometry::createBoxBufferGeometry));
}

std::map<string, float[72]> wr_box_buffer_geometry_attributes(WrBoxBufferGeometry *boxBufferGeometry) {
  return reinterpret_cast<wren::BoxBufferGeometry *>(boxBufferGeometry)->attributes();
}

std::map<string, int> wr_box_buffer_geometry_attributes(WrBoxBufferGeometry *boxBufferGeometry) {
  return reinterpret_cast<wren::BoxBufferGeometry *>(boxBufferGeometry)->attributes();
}

void wr_box_buffer_geometry_attributes(WrBoxBufferGeometry *boxBufferGeometry) {
  return reinterpret_cast<wren::BoxBufferGeometry *>(boxBufferGeometry)->attributes();
}
