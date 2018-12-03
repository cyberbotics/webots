#ifndef WR_DIRECTIONAL_LIGHT_H
#define WR_DIRECTIONAL_LIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Inheritance diagram: WrNode <- WrDirectionalLight */
struct WrDirectionalLight;
typedef struct WrDirectionalLight WrDirectionalLight;

/* Use wr_node_delete(WR_NODE(light)) to delete an instance */
WrDirectionalLight *wr_directional_light_new();

/* Expects a 3-component array */
void wr_directional_light_set_color(WrDirectionalLight *light, const float *color);
void wr_directional_light_set_intensity(WrDirectionalLight *light, float intensity);
void wr_directional_light_set_ambient_intensity(WrDirectionalLight *light, float ambient_intensity);
void wr_directional_light_set_on(WrDirectionalLight *light, bool on);
void wr_directional_light_set_cast_shadows(WrDirectionalLight *light, bool cast_shadows);

void wr_directional_light_set_direction(WrDirectionalLight *light, float *direction);

#ifdef __cplusplus
}
#endif

#endif  // WR_DIRECTIONAL_LIGHT_H
