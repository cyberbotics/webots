#ifndef WR_TEXTURE_RTT_H
#define WR_TEXTURE_RTT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Inheritance diagram: WrTexture <- WrTextureRtt */
struct WrTextureRtt;
typedef struct WrTextureRtt WrTextureRtt;

WrTextureRtt *wr_texture_rtt_new();
void wr_texture_rtt_enable_initialize_data(WrTextureRtt *texture, bool enable);

#ifdef __cplusplus
}
#endif

#endif  // WR_TEXTURE_RTT_H
