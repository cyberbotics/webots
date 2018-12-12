#ifndef WR_CONFIG_H
#define WR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

struct WrShaderProgram;
typedef struct WrShaderProgram WrShaderProgram;

void wr_config_enable_shadows(bool enable);
void wr_config_set_line_scale(float line_scale);
void wr_config_set_show_bounding_spheres(bool show);
void wr_config_set_show_axis_aligned_bounding_boxes(bool show);
void wr_config_set_show_shadow_axis_aligned_bounding_boxes(bool show);
void wr_config_set_show_frustums(bool show);
void wr_config_set_bounding_volume_program(WrShaderProgram *program);
void wr_config_set_requires_flush_after_draw(bool require);
void wr_config_set_requires_depth_buffer_distortion(bool require);
void wr_config_enable_point_size(bool enable);

int wr_config_get_max_active_spot_light_count();
int wr_config_get_max_active_point_light_count();
int wr_config_get_max_active_directional_light_count();

bool wr_config_are_shadows_enabled();
float wr_config_get_line_scale();

#ifdef __cplusplus
}
#endif

#endif  // WR_CONFIG_H
