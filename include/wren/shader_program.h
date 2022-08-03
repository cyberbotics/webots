#ifndef WR_SHADER_PROGRAM_H
#define WR_SHADER_PROGRAM_H

#include <wren/glsl_layout.h>

#ifdef __cplusplus
extern "C" {
#endif

struct WrShaderProgram;
typedef struct WrShaderProgram WrShaderProgram;

typedef enum WrShaderProgramUniformType {
  WR_SHADER_PROGRAM_UNIFORM_TYPE_BOOL,
  WR_SHADER_PROGRAM_UNIFORM_TYPE_INT,
  WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
  WR_SHADER_PROGRAM_UNIFORM_TYPE_VEC2F,
  WR_SHADER_PROGRAM_UNIFORM_TYPE_VEC3F,
  WR_SHADER_PROGRAM_UNIFORM_TYPE_VEC4F,
  WR_SHADER_PROGRAM_UNIFORM_TYPE_MAT4F
} WrShaderProgramUniformType;

WrShaderProgram *wr_shader_program_new();
void wr_shader_program_delete(WrShaderProgram *program);

void wr_shader_program_set_vertex_shader_path(WrShaderProgram *program, const char *path);
void wr_shader_program_set_fragment_shader_path(WrShaderProgram *program, const char *path);
void wr_shader_program_use_uniform(WrShaderProgram *program, WrGlslLayoutUniform uniform);
void wr_shader_program_use_uniform_buffer(WrShaderProgram *program, WrGlslLayoutUniformBuffer uniform_buffer);
void wr_shader_program_create_custom_uniform(WrShaderProgram *program, const char *name, WrShaderProgramUniformType type,
                                             const char *initial_value);
void wr_shader_program_set_custom_uniform_value(WrShaderProgram *program, const char *name, WrShaderProgramUniformType type,
                                                const char *value);
unsigned int wr_shader_program_get_gl_name(WrShaderProgram *program);
bool wr_shader_program_has_vertex_shader_compilation_failed(WrShaderProgram *program);
bool wr_shader_program_has_fragment_shader_compilation_failed(WrShaderProgram *program);
const char *wr_shader_program_get_compilation_log(WrShaderProgram *program);

/* Setup must be called after all parameters have been specified */
void wr_shader_program_setup(WrShaderProgram *program);

#ifdef __cplusplus
}
#endif

#endif  // WR_SHADER_PROGRAM_H
