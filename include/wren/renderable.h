#ifndef WR_RENDERABLE_H
#define WR_RENDERABLE_H

#define WR_MESH(x) ((WrMesh *)(x))

#ifdef __cplusplus
extern "C" {
#endif

/* Inheritance diagram: WrNode <- WrRenderable */
struct WrRenderable;
typedef struct WrRenderable WrRenderable;

struct WrMesh;
typedef struct WrMesh WrMesh;

struct WrMaterial;
typedef struct WrMaterial WrMaterial;

typedef enum WrRenderableDrawingMode {
  WR_RENDERABLE_DRAWING_MODE_TRIANGLES = 0x4,     // GL_TRIANGLES
  WR_RENDERABLE_DRAWING_MODE_TRIANGLE_FAN = 0x6,  // GL_TRIANGLE_FAN
  WR_RENDERABLE_DRAWING_MODE_LINES = 0x1,         // GL_LINES
  WR_RENDERABLE_DRAWING_MODE_LINE_STRIP = 0x3,    // GL_LINE_STRIP
  WR_RENDERABLE_DRAWING_MODE_POINTS = 0x0         // GL_POINTS
} WrRenderableDrawingMode;

typedef enum WrRenderableDrawingOrder {
  WR_RENDERABLE_DRAWING_ORDER_MAIN,
  WR_RENDERABLE_DRAWING_ORDER_AFTER_0,
  WR_RENDERABLE_DRAWING_ORDER_AFTER_1,
  WR_RENDERABLE_DRAWING_ORDER_AFTER_2,
  WR_RENDERABLE_DRAWING_ORDER_AFTER_3,
  WR_RENDERABLE_DRAWING_ORDER_COUNT
} WrRenderableDrawingOrder;

/* Use wr_node_delete(WR_NODE(renderable)) to delete an instance */
WrRenderable *wr_renderable_new();

void wr_renderable_set_mesh(WrRenderable *renderable, WrMesh *mesh);
/* Set a material to this renderable, the 'name' parameter is optional and can be set to NULL
to set the default material */
void wr_renderable_set_material(WrRenderable *renderable, WrMaterial *material, const char *name);
void wr_renderable_set_drawing_mode(WrRenderable *renderable, WrRenderableDrawingMode drawing_mode);
void wr_renderable_set_visibility_flags(WrRenderable *renderable, int flags);
void wr_renderable_invert_front_face(WrRenderable *renderable, bool invert_front_face);
void wr_renderable_set_cast_shadows(WrRenderable *renderable, bool cast_shadows);
void wr_renderable_set_receive_shadows(WrRenderable *renderable, bool receive_shadows);
void wr_renderable_set_scene_culling(WrRenderable *renderable, bool culling);
void wr_renderable_set_face_culling(WrRenderable *renderable, bool face_culling);
void wr_renderable_set_drawing_order(WrRenderable *renderable, WrRenderableDrawingOrder order);
void wr_renderable_set_in_view_space(WrRenderable *renderable, bool in_view_space);
void wr_renderable_set_z_sorted_rendering(WrRenderable *renderable, bool z_sorted);
void wr_renderable_set_point_size(WrRenderable *renderable, float point_size);

/* Returns the material associated to the 'name' parameter. Default material is returned if 'name' is set to NULL */
WrMaterial *wr_renderable_get_material(WrRenderable *renderable, const char *name);

/* Returns the bounding sphere in world space.
   sphere[0]..[2] = center, sphere[3] = radius */
void wr_renderable_get_bounding_sphere(WrRenderable *renderable, float *sphere);

#ifdef __cplusplus
}
#endif

#endif  // WR_RENDERABLE_H
