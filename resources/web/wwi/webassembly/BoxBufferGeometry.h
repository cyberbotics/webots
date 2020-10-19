#ifndef MESHBASICMATERIAL_H
#define MESHBASICMATERIAL_H

extern "C" {
struct WrMeshBasicMaterial;
typedef struct WrMeshBasicMaterial WrMeshBasicMaterial;

WrMeshBasicMaterial *wr_mesh_basic_material_new();
bool wr_mesh_basic_material_visible(WrMeshBasicMaterial *meshBasicMaterial);
}

#endif
