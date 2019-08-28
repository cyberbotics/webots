#ifndef WR_MATERIAL_H
#define WR_MATERIAL_H

#include <wren/texture.h>

#ifdef __cplusplus
extern "C" {
#endif

enum WrMaterialType { WR_MATERIAL_NONE, WR_MATERIAL_PHONG, WR_MATERIAL_PBR };

struct WrMaterial {
  WrMaterialType type;
  void *data;
};

typedef struct WrMaterial WrMaterial;

struct WrShaderProgram;
typedef struct WrShaderProgram WrShaderProgram;

struct WrTextureTransform;
typedef struct WrTextureTransform WrTextureTransform;

struct WrTextureCubeMap;
typedef struct WrTextureCubeMap WrTextureCubeMap;

void wr_material_delete(WrMaterial *material);

void wr_material_set_texture(WrMaterial *material, WrTexture *texture, int index);
void wr_material_set_texture_wrap_s(WrMaterial *material, WrTextureWrapMode mode, int index);
void wr_material_set_texture_wrap_t(WrMaterial *material, WrTextureWrapMode mode, int index);
void wr_material_set_texture_anisotropy(WrMaterial *material, float anisotropy, int index);
void wr_material_set_texture_enable_interpolation(WrMaterial *material, bool enable, int index);
void wr_material_set_texture_enable_mip_maps(WrMaterial *material, bool enable, int index);

void wr_material_set_texture_cubemap(WrMaterial *material, WrTextureCubeMap *texture, int index);
void wr_material_set_texture_cubemap_wrap_r(WrMaterial *material, WrTextureWrapMode mode, int index);
void wr_material_set_texture_cubemap_wrap_s(WrMaterial *material, WrTextureWrapMode mode, int index);
void wr_material_set_texture_cubemap_wrap_t(WrMaterial *material, WrTextureWrapMode mode, int index);
void wr_material_set_texture_cubemap_anisotropy(WrMaterial *material, float anisotropy, int index);
void wr_material_set_texture_cubemap_enable_interpolation(WrMaterial *material, bool enable, int index);
void wr_material_set_texture_cubemap_enable_mip_maps(WrMaterial *material, bool enable, int index);

void wr_material_set_texture_transform(WrMaterial *material, WrTextureTransform *transform);
void wr_material_set_default_program(WrMaterial *material, WrShaderProgram *program);
void wr_material_set_stencil_ambient_emissive_program(WrMaterial *material, WrShaderProgram *program);
void wr_material_set_stencil_diffuse_specular_program(WrMaterial *material, WrShaderProgram *program);

WrTexture *wr_material_get_texture(const WrMaterial *material, int index);

WrTextureWrapMode wr_material_get_texture_wrap_s(const WrMaterial *material, int index);
WrTextureWrapMode wr_material_get_texture_wrap_t(const WrMaterial *material, int index);
float wr_material_get_texture_anisotropy(const WrMaterial *material, int index);
bool wr_material_is_texture_interpolation_enabled(const WrMaterial *material, int index);
bool wr_material_are_texture_mip_maps_enabled(const WrMaterial *material, int index);

// Phong material-only functions
WrMaterial *wr_phong_material_new();
void wr_phong_material_clear(WrMaterial *material);
void wr_phong_material_set_color(WrMaterial *material, const float *color);
void wr_phong_material_set_ambient(WrMaterial *material, const float *ambient);
void wr_phong_material_set_diffuse(WrMaterial *material, const float *diffuse);
void wr_phong_material_set_linear_ambient(WrMaterial *material, const float *ambient);
void wr_phong_material_set_linear_diffuse(WrMaterial *material, const float *diffuse);

void wr_phong_material_set_specular(WrMaterial *material, const float *specular);
void wr_phong_material_set_emissive(WrMaterial *material, const float *emissive);
void wr_phong_material_set_shininess(WrMaterial *material, float shininess);
void wr_phong_material_set_transparency(WrMaterial *material, float transparency);
void wr_phong_material_set_all_parameters(WrMaterial *material, const float *ambient, const float *diffuse,
                                          const float *specular, const float *emissive, float shininess, float transparency);
bool wr_phong_material_is_translucent(const WrMaterial *material);
void wr_phong_material_set_color_per_vertex(WrMaterial *material, bool enabled);

// PBR material-only functions
WrMaterial *wr_pbr_material_new();
void wr_pbr_material_clear(WrMaterial *material);
void wr_pbr_material_set_transparency(WrMaterial *material, float transparency);
void wr_pbr_material_set_background_color(WrMaterial *material, const float *background_color);
void wr_pbr_material_set_base_color(WrMaterial *material, const float *base_color);
void wr_pbr_material_set_emissive_color(WrMaterial *material, const float *emissive_color);
void wr_pbr_material_set_roughness(WrMaterial *material, float roughness);
void wr_pbr_material_set_metalness(WrMaterial *material, float metalness);
void wr_pbr_material_set_ibl_strength(WrMaterial *material, float ibl_strength);
void wr_pbr_material_set_normal_map_strength(WrMaterial *material, float normal_map_factor);
void wr_pbr_material_set_occlusion_map_strength(WrMaterial *material, float occlusion_map_strength);
void wr_pbr_material_set_emissive_intensity(WrMaterial *material, float emissive_intensity);
void wr_pbr_material_set_all_parameters(WrMaterial *material, const float *background_color, const float *base_color,
                                        float transparency, float roughness, float metalness, float ibl_strength,
                                        float normal_map_factor, float occlusion_map_strength, const float *emissive_color,
                                        float emissive_intensity);
bool wr_pbr_material_is_translucent(const WrMaterial *material);

#ifdef __cplusplus
}
#endif

#endif  // WR_MATERIAL_H
