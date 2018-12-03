#ifndef WR_SPOT_LIGHT_H
#define WR_SPOT_LIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Inheritance diagram: WrNode <- WrSpotLight */
struct WrSpotLight;
typedef struct WrSpotLight WrSpotLight;

/* Use wr_node_delete(WR_NODE(light)) to delete an instance */
WrSpotLight *wr_spot_light_new();

/* Expects a 3-component array */
void wr_spot_light_set_color(WrSpotLight *light, const float *color);
void wr_spot_light_set_intensity(WrSpotLight *light, float intensity);
void wr_spot_light_set_ambient_intensity(WrSpotLight *light, float ambient_intensity);
void wr_spot_light_set_on(WrSpotLight *light, bool on);
void wr_spot_light_set_cast_shadows(WrSpotLight *light, bool cast_shadows);

/* Expects a 3-component array */
void wr_spot_light_set_position_relative(WrSpotLight *light, const float *position);
void wr_spot_light_set_radius(WrSpotLight *light, float radius);
void wr_spot_light_set_attenuation(WrSpotLight *light, float attenuation_constant, float attenuation_linear,
                                   float attenuation_quadratic);

/* Expects a 3-component array */
void wr_spot_light_set_direction(WrSpotLight *light, const float *direction);
void wr_spot_light_set_beam_width(WrSpotLight *light, float beam_width);
void wr_spot_light_set_cutoff_angle(WrSpotLight *light, float cutoff_angle);

#ifdef __cplusplus
}
#endif

#endif  // WR_SPOT_LIGHT_H
