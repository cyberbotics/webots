#ifndef WR_TEXTURE_CUBEMAP_H
#define WR_TEXTURE_CUBEMAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <wren/texture.h>

/* Inheritance diagram: WrTexture <- WrTextureCubeMap */
struct WrTextureCubeMap;
typedef struct WrTextureCubeMap WrTextureCubeMap;

typedef enum WrTextureOrientation {
  WR_TEXTURE_CUBEMAP_RIGHT = 0,
  WR_TEXTURE_CUBEMAP_LEFT,
  WR_TEXTURE_CUBEMAP_TOP,
  WR_TEXTURE_CUBEMAP_BOTTOM,
  WR_TEXTURE_CUBEMAP_BACK,
  WR_TEXTURE_CUBEMAP_FRONT,
  WR_TEXTURE_CUBEMAP_COUNT
} WrTextureOrientation;

/* Use wr_texture_delete(WR_TEXTURE(texture)) to delete an instance */
WrTextureCubeMap *wr_texture_cubemap_new();

void wr_texture_cubemap_set_data(WrTextureCubeMap *texture, const char *data, WrTextureOrientation orientation);
void wr_texture_cubemap_disable_automatic_mip_map_generation(WrTextureCubeMap *texture);

#ifdef __cplusplus
}
#endif

#endif
