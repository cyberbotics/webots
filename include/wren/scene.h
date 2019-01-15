#ifndef WR_SCENE_H
#define WR_SCENE_H

#ifdef __cplusplus
extern "C" {
#endif

struct WrScene;
typedef struct WrScene WrScene;

struct WrTransform;
typedef struct WrTransform WrTransform;

struct WrViewport;
typedef struct WrViewport WrViewport;

struct WrRenderable;
typedef struct WrRenderable WrRenderable;

struct WrShaderProgram;
typedef struct WrShaderProgram WrShaderProgram;

typedef enum WrSceneFogType {
  WR_SCENE_FOG_TYPE_NONE,
  WR_SCENE_FOG_TYPE_EXPONENTIAL,
  WR_SCENE_FOG_TYPE_EXPONENTIAL2,
  WR_SCENE_FOG_TYPE_LINEAR
} WrSceneFogType;

/* Defines the basis for the fog distance computation */
typedef enum WrSceneFogDepthType { WR_SCENE_FOG_DEPTH_TYPE_PLANE, WR_SCENE_FOG_DEPTH_TYPE_POINT } WrSceneFogDepthType;

WrScene *wr_scene_get_instance();
void wr_scene_destroy();

// The OpenGL context must be active when calling these functions
void wr_scene_init(WrScene *scene);
void wr_scene_reset(WrScene *scene);

// Apply pending OpenGL state changes
void wr_scene_apply_pending_updates(WrScene *scene);

/* The 'materialName' parameter is optional (can be set to NULL for default), and if set, force
the renderables to use the named material */
void wr_scene_render(WrScene *scene, const char *material_name, bool culling);
void wr_scene_render_to_viewports(WrScene *scene, int count, WrViewport **viewports, const char *material_name, bool culling);

void wr_scene_set_ambient_light(const float *ambient_light);

int wr_scene_get_active_spot_light_count(WrScene *scene);
int wr_scene_get_active_point_light_count(WrScene *scene);
int wr_scene_get_active_directional_light_count(WrScene *scene);

/* Expects a 3-component array for 'color' */
void wr_scene_set_fog(WrScene *scene, WrSceneFogType fogType, WrSceneFogDepthType depthType, const float *color, float density,
                      float start, float end);
void wr_scene_set_skybox(WrScene *scene, WrRenderable *renderable);
void wr_scene_set_hdr_clear_quad(WrScene *scene, WrRenderable *renderable);
void wr_scene_set_fog_program(WrScene *scene, WrShaderProgram *program);
void wr_scene_set_shadow_volume_program(WrScene *scene, WrShaderProgram *program);

void wr_scene_enable_skybox(WrScene *scene, bool enable);
void wr_scene_enable_hdr_clear(WrScene *scene, bool enable);
void wr_scene_enable_translucence(WrScene *scene, bool enable);
void wr_scene_enable_depth_reset(WrScene *scene, bool enable);

void wr_scene_add_frame_listener(WrScene *scene, void (*listener)());
void wr_scene_remove_frame_listener(WrScene *scene, void (*listener)());

void wr_scene_get_main_buffer(WrScene *scene, int width, int height, unsigned int format, unsigned int data_type,
                              unsigned int buffer_type, void *buffer);

void wr_scene_init_frame_capture(WrScene *scene, int pixel_buffer_count, unsigned int *pixel_buffer_ids, int frame_size);
void wr_scene_bind_pixel_buffer(WrScene *scene, int buffer);
void *wr_scene_map_pixel_buffer(WrScene *scene, unsigned int access_mode);
void wr_scene_unmap_pixel_buffer(WrScene *scene);
void wr_scene_terminate_frame_capture(WrScene *scene);

int wr_scene_compute_node_count(WrScene *scene);

WrTransform *wr_scene_get_root(WrScene *scene);
WrViewport *wr_scene_get_viewport(WrScene *scene);

#ifdef __cplusplus
}
#endif

#endif  // WR_SCENE_H
