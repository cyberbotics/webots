#ifndef WR_TEXTURE_H
#define WR_TEXTURE_H

#define WR_TEXTURE(x) ((WrTexture *)(x))

#ifdef __cplusplus
extern "C" {
#endif

struct WrTexture;
typedef struct WrTexture WrTexture;

typedef enum WrTextureType {
  WR_TEXTURE_TYPE_2D,
  WR_TEXTURE_TYPE_RTT,
  WR_TEXTURE_TYPE_CUBEMAP,
  WR_TEXTURE_TYPE_DRAWABLE
} WrTextureType;

typedef enum WrTextureInternalFormat {
  WR_TEXTURE_INTERNAL_FORMAT_RED,
  WR_TEXTURE_INTERNAL_FORMAT_RG8,
  WR_TEXTURE_INTERNAL_FORMAT_RGB8,
  WR_TEXTURE_INTERNAL_FORMAT_RGBA8,
  WR_TEXTURE_INTERNAL_FORMAT_R16F,
  WR_TEXTURE_INTERNAL_FORMAT_RGB16F,
  WR_TEXTURE_INTERNAL_FORMAT_RGBA16F,
  WR_TEXTURE_INTERNAL_FORMAT_R32F,
  WR_TEXTURE_INTERNAL_FORMAT_RG32F,
  WR_TEXTURE_INTERNAL_FORMAT_RGB32F,
  WR_TEXTURE_INTERNAL_FORMAT_RGBA32F,
  WR_TEXTURE_INTERNAL_FORMAT_DEPTH24_STENCIL8,
  WR_TEXTURE_INTERNAL_FORMAT_DEPTH_COMPONENT32F,
  WR_TEXTURE_INTERNAL_FORMAT_COUNT
} WrTextureInternalFormat;

typedef enum WrTextureWrapMode {
  WR_TEXTURE_WRAP_MODE_REPEAT = 0x2901,          // GL_REPEAT
  WR_TEXTURE_WRAP_MODE_CLAMP_TO_EDGE = 0x812F,   // GL_CLAMP_TO_EDGE
  WR_TEXTURE_WRAP_MODE_CLAMP_TO_BORDER = 0x812D  // GL_CLAMP_TO_BORDER
} WrTextureWrapMode;

void wr_texture_delete(WrTexture *texture);

void wr_texture_set_internal_format(WrTexture *texture, WrTextureInternalFormat format);
void wr_texture_set_texture_unit(WrTexture *texture, unsigned int unit);
void wr_texture_set_size(WrTexture *texture, int width, int height);
void wr_texture_set_translucent(WrTexture *texture, bool translucent);
void wr_texture_change_data(WrTexture *texture, void *data, int x, int y, int width, int height);

void wr_texture_setup(WrTexture *texture);

int wr_texture_get_width(const WrTexture *texture);
int wr_texture_get_height(const WrTexture *texture);
bool wr_texture_is_translucent(const WrTexture *texture);
WrTextureType wr_texture_get_type(const WrTexture *texture);
unsigned int wr_texture_get_gl_name(const WrTexture *texture);

#ifdef __cplusplus
}
#endif

#endif  // WR_TEXTURE_H
