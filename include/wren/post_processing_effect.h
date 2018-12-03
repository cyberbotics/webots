#ifndef WR_POST_PROCESSING_EFFECT_H
#define WR_POST_PROCESSING_EFFECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <wren/texture.h>

struct WrPostProcessingEffect;
typedef struct WrPostProcessingEffect WrPostProcessingEffect;

struct WrPostProcessingEffectPass;
typedef struct WrPostProcessingEffectPass WrPostProcessingEffectPass;

struct WrFrameBuffer;
typedef struct WrFrameBuffer WrFrameBuffer;

struct WrMesh;
typedef struct WrMesh WrMesh;

struct WrShaderProgram;
typedef struct WrShaderProgram WrShaderProgram;

struct WrTextureRtt;
typedef struct WrTextureRtt WrTextureRtt;

struct WrViewport;
typedef struct WrViewport WrViewport;

WrPostProcessingEffectPass *wr_post_processing_effect_pass_new();
void wr_post_processing_effect_pass_delete(WrPostProcessingEffectPass *pass);

void wr_post_processing_effect_pass_set_name(WrPostProcessingEffectPass *pass, const char *name);
void wr_post_processing_effect_pass_set_program(WrPostProcessingEffectPass *pass, WrShaderProgram *program);

/* This function requires a persistent pointer to the value (it is not copied from the source), otherwise WREN crashes here. */
void wr_post_processing_effect_pass_set_program_parameter(WrPostProcessingEffectPass *pass, const char *parameter_name,
                                                          const char *value);
void wr_post_processing_effect_pass_set_output_size(WrPostProcessingEffectPass *pass, int width, int height);
/* Texture counts needs to be specified before setting individual input textures or output formats */
void wr_post_processing_effect_pass_set_input_texture_count(WrPostProcessingEffectPass *pass, int count);
void wr_post_processing_effect_pass_set_output_texture_count(WrPostProcessingEffectPass *pass, int count);
void wr_post_processing_effect_pass_set_input_texture_wrap_mode(WrPostProcessingEffectPass *pass, int index,
                                                                WrTextureWrapMode mode);
void wr_post_processing_effect_pass_set_input_texture_interpolation(WrPostProcessingEffectPass *pass, int index, bool enable);
void wr_post_processing_effect_pass_set_output_texture_format(WrPostProcessingEffectPass *pass, int index,
                                                              WrTextureInternalFormat format);
void wr_post_processing_effect_pass_set_input_texture(WrPostProcessingEffectPass *pass, int index, WrTexture *texture);
/* The pass will be invoked 'count' times (1 by default) */
void wr_post_processing_effect_pass_set_iteration_count(WrPostProcessingEffectPass *pass, int count);
WrTextureRtt *wr_post_processing_effect_pass_get_output_texture(WrPostProcessingEffectPass *pass, int index);
void wr_post_processing_effect_pass_set_clear_before_draw(WrPostProcessingEffectPass *pass, bool enable);
void wr_post_processing_effect_pass_set_alpha_blending(WrPostProcessingEffectPass *pass, bool enable);

WrPostProcessingEffect *wr_post_processing_effect_new();
void wr_post_processing_effect_delete(WrPostProcessingEffect *post_processing_effect);

/* Specifying an input framebuffer is optional. If available, its first output texture will be used as
   the first input texture of the effect's first pass */
void wr_post_processing_effect_set_input_frame_buffer(WrPostProcessingEffect *post_processing_effect,
                                                      WrFrameBuffer *frame_buffer);
/* At a minimum, one pass needs to be specified */
void wr_post_processing_effect_append_pass(WrPostProcessingEffect *post_processing_effect, WrPostProcessingEffectPass *pass);
/* Sets output texture 'output_index' of pass 'from' as input texture 'input_index' of pass 'to' */
void wr_post_processing_effect_connect(WrPostProcessingEffect *post_processing_effect, WrPostProcessingEffectPass *from,
                                       int output_index, WrPostProcessingEffectPass *to, int input_index);
/* Will usually be a pass-through shader, but can be any shader with only one input */
void wr_post_processing_effect_set_result_program(WrPostProcessingEffect *post_processing_effect, WrShaderProgram *program);
/* Will hold the result of applying the result program to the first output texture of the last pass */
void wr_post_processing_effect_set_result_frame_buffer(WrPostProcessingEffect *post_processing_effect,
                                                       WrFrameBuffer *frame_buffer);
WrPostProcessingEffectPass *wr_post_processing_effect_get_first_pass(WrPostProcessingEffect *post_processing_effect);
WrPostProcessingEffectPass *wr_post_processing_effect_get_last_pass(WrPostProcessingEffect *post_processing_effect);
WrPostProcessingEffectPass *wr_post_processing_effect_get_pass(WrPostProcessingEffect *post_processing_effect,
                                                               const char *name);
void wr_post_processing_effect_set_drawing_index(WrPostProcessingEffect *post_processing_effect, unsigned int index);
void wr_post_processing_effect_setup(WrPostProcessingEffect *post_processing_effect);
void wr_post_processing_effect_apply(WrPostProcessingEffect *post_processing_effect);

#ifdef __cplusplus
}
#endif

#endif  // WR_POST_PROCESSING_EFFECT_H
