#ifndef WR_TEXTURE_CUBEMAP_BAKER_H
#define WR_TEXTURE_CUBEMAP_BAKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <wren/texture_2d.h>
#include <wren/texture_cubemap.h>
#include <wren/texture_rtt.h>

WrTextureCubeMap *wr_texture_cubemap_bake_diffuse_irradiance(WrTextureCubeMap *input_cubemap, WrShaderProgram *shader,
                                                             unsigned int size);
WrTextureCubeMap *wr_texture_cubemap_bake_specular_irradiance(WrTextureCubeMap *input_cubemap, WrShaderProgram *shader,
                                                              unsigned int size);
WrTextureRtt *wr_texture_cubemap_bake_brdf(WrShaderProgram *shader, unsigned int size);

#ifdef __cplusplus
}
#endif

#endif
