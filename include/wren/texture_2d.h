#ifndef WR_TEXTURE_2D_H
#define WR_TEXTURE_2D_H

#ifdef __cplusplus
extern "C" {
#endif

/* Inheritance diagram: WrTexture <- WrTexture2d */
struct WrTexture2d;
typedef struct WrTexture2d WrTexture2d;

/* Use wr_texture_delete(WR_TEXTURE(texture)) to delete an instance */
WrTexture2d *wr_texture_2d_new();

/* This function creates and returns a WrTexture2d instance if a texture with
   the given path exists in the cache, or NULL if none is found. */
WrTexture2d *wr_texture_2d_copy_from_cache(const char *file_path);

void wr_texture_2d_set_data(WrTexture2d *texture, const char *data);
/* The file path is used to produce a hash for the texture cache.
   If none is provided, a unique string will be generated and used instead. */
void wr_texture_2d_set_file_path(WrTexture2d *texture, const char *file_path);

/* allows this texture to live in the cache even if the scene is reset */
void wr_texture_2d_set_cache_persistency(WrTexture2d *texture, bool is_persistent);

#ifdef __cplusplus
}
#endif

#endif  // WR_TEXTURE_2D_H
