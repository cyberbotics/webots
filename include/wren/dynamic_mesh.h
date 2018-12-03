#ifndef WR_DYNAMIC_MESH_H
#define WR_DYNAMIC_MESH_H

#ifdef __cplusplus
extern "C" {
#endif

struct WrDynamicMesh;
typedef struct WrDynamicMesh WrDynamicMesh;

WrDynamicMesh *wr_dynamic_mesh_new(bool normals, bool texture_coordinates, bool color_per_vertex);
void wr_dynamic_mesh_delete(WrDynamicMesh *mesh);

void wr_dynamic_mesh_clear(WrDynamicMesh *mesh);
void wr_dynamic_mesh_clear_selected(WrDynamicMesh *mesh, bool vertices, bool normals, bool texture_coordinates, bool colors);
void wr_dynamic_mesh_add_vertex(WrDynamicMesh *mesh, const float *vertex);
void wr_dynamic_mesh_add_normal(WrDynamicMesh *mesh, const float *normal);
void wr_dynamic_mesh_add_texture_coordinate(WrDynamicMesh *mesh, const float *texture_coordinate);
void wr_dynamic_mesh_add_index(WrDynamicMesh *mesh, unsigned int index);
void wr_dynamic_mesh_add_color(WrDynamicMesh *mesh, const float *color);

#ifdef __cplusplus
}
#endif

#endif  // WR_MESH_H
