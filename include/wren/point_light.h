#ifndef WR_POINT_LIGHT_H
#define WR_POINT_LIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Inheritance diagram: WrNode <- WrPointLight */
struct WrPointLight;
typedef struct WrPointLight WrPointLight;

/* Use wr_node_delete(WR_NODE(light)) to delete an instance */
WrPointLight *wr_point_light_new();

/* Expects a 3-component array */
void wr_point_light_set_color(WrPointLight *light, const float *color);
void wr_point_light_set_intensity(WrPointLight *light, float intensity);
void wr_point_light_set_ambient_intensity(WrPointLight *light, float ambient_intensity);
void wr_point_light_set_on(WrPointLight *light, bool on);
void wr_point_light_set_cast_shadows(WrPointLight *light, bool cast_shadows);

void wr_point_light_set_position_relative(WrPointLight *light, const float *position);
void wr_point_light_set_radius(WrPointLight *light, float radius);
void wr_point_light_set_attenuation(WrPointLight *light, float attenuation_constant, float attenuation_linear,
                                    float attenuation_quadratic);

#ifdef __cplusplus
}
#endif

#endif  // WR_POINT_LIGHT_H
