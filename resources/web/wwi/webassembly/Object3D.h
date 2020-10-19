#ifndef OBJECT3D_H
#define OBJECT3D_H

extern "C" {
struct WrObject3D;
typedef struct WrObject3D WrObject3D;

WrObject3D *wr_oject3d_new();
void wr_object3d_add(WrObject3D *object3D, WrObject3D object3D);
void wr_object3d_remove(WrObject3D *object3D, WrObject3D object3D);
void wr_object3d_update_world_matrix(WrObject3D *object3D, bool force);
}

#endif
