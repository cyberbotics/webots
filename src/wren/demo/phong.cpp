// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Create a fixed-size window using glut, initialize WREN,
 * and display several colored sphere using the Webots phong shader.
 */

#include <GL/freeglut.h>

#include <wren/camera.h>
#include <wren/directional_light.h>
#include <wren/frame_buffer.h>
#include <wren/gl_state.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/scene.h>
#include <wren/shader_program.h>
#include <wren/static_mesh.h>
#include <wren/texture.h>
#include <wren/texture_2d.h>
#include <wren/texture_rtt.h>
#include <wren/transform.h>
#include <wren/viewport.h>

#include <cstdio>

// Define the refresh rate in milliseconds (at best)
static const int refreshRate = 30;
// window initial dimension.
static const int width = 800;
static const int height = 600;

static void initMaterialToDefault(WrMaterial *material) {
  WrShaderProgram *defaultProgram = wr_shader_program_new();
  wr_shader_program_use_uniform(defaultProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE0);
  wr_shader_program_use_uniform(defaultProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE1);
  wr_shader_program_use_uniform(defaultProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE2);
  wr_shader_program_use_uniform(defaultProgram, WR_GLSL_LAYOUT_UNIFORM_MODEL_TRANSFORM);
  wr_shader_program_use_uniform(defaultProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE_TRANSFORM);
  wr_shader_program_use_uniform_buffer(defaultProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_MATERIAL_PHONG);
  wr_shader_program_use_uniform_buffer(defaultProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_CAMERA_TRANSFORMS);
  wr_shader_program_set_vertex_shader_path(defaultProgram, "../../../resources/wren/shaders/default.vert");
  wr_shader_program_set_fragment_shader_path(defaultProgram, "../../../resources/wren/shaders/default.frag");
  wr_shader_program_setup(defaultProgram);
  if (!wr_shader_program_get_gl_name(defaultProgram))
    printf("Compilation failed: %s\n", wr_shader_program_get_compilation_log(defaultProgram));
  wr_material_set_default_program(material, defaultProgram);
}

static void initMaterialToPhong(WrMaterial *material) {
  WrShaderProgram *phongProgram = wr_shader_program_new();
  wr_shader_program_use_uniform(phongProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE0);
  wr_shader_program_use_uniform(phongProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE1);
  wr_shader_program_use_uniform(phongProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE2);
  wr_shader_program_use_uniform(phongProgram, WR_GLSL_LAYOUT_UNIFORM_MODEL_TRANSFORM);
  wr_shader_program_use_uniform(phongProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE_TRANSFORM);
  wr_shader_program_use_uniform_buffer(phongProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_MATERIAL_PHONG);
  wr_shader_program_use_uniform_buffer(phongProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_LIGHTS);
  wr_shader_program_use_uniform_buffer(phongProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_CAMERA_TRANSFORMS);
  wr_shader_program_use_uniform_buffer(phongProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_FOG);
  wr_shader_program_set_vertex_shader_path(phongProgram, "../../../resources/wren/shaders/phong.vert");
  wr_shader_program_set_fragment_shader_path(phongProgram, "../../../resources/wren/shaders/phong.frag");
  wr_shader_program_setup(phongProgram);
  if (!wr_shader_program_get_gl_name(phongProgram))
    printf("Compilation failed: %s\n", wr_shader_program_get_compilation_log(phongProgram));

  WrShaderProgram *phongStencilAmbientEmissiveProgram = wr_shader_program_new();
  wr_shader_program_use_uniform(phongStencilAmbientEmissiveProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE0);
  wr_shader_program_use_uniform(phongStencilAmbientEmissiveProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE1);
  wr_shader_program_use_uniform(phongStencilAmbientEmissiveProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE2);
  wr_shader_program_use_uniform(phongStencilAmbientEmissiveProgram, WR_GLSL_LAYOUT_UNIFORM_MODEL_TRANSFORM);
  wr_shader_program_use_uniform(phongStencilAmbientEmissiveProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE_TRANSFORM);

  wr_shader_program_use_uniform_buffer(phongStencilAmbientEmissiveProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_MATERIAL_PHONG);
  wr_shader_program_use_uniform_buffer(phongStencilAmbientEmissiveProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_LIGHTS);
  wr_shader_program_use_uniform_buffer(phongStencilAmbientEmissiveProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_CAMERA_TRANSFORMS);
  wr_shader_program_set_vertex_shader_path(phongStencilAmbientEmissiveProgram,
                                           "../../../resources/wren/shaders/phong_stencil_ambient_emissive.vert");
  wr_shader_program_set_fragment_shader_path(phongStencilAmbientEmissiveProgram,
                                             "../../../resources/wren/shaders/phong_stencil_ambient_emissive.frag");
  wr_shader_program_setup(phongStencilAmbientEmissiveProgram);
  if (!wr_shader_program_get_gl_name(phongStencilAmbientEmissiveProgram))
    printf("Compilation failed: %s\n", wr_shader_program_get_compilation_log(phongStencilAmbientEmissiveProgram));

  WrShaderProgram *phongStencilDiffuseSpecularProgram = wr_shader_program_new();
  wr_shader_program_use_uniform(phongStencilDiffuseSpecularProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE0);
  wr_shader_program_use_uniform(phongStencilDiffuseSpecularProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE1);
  wr_shader_program_use_uniform(phongStencilDiffuseSpecularProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE2);
  wr_shader_program_use_uniform(phongStencilDiffuseSpecularProgram, WR_GLSL_LAYOUT_UNIFORM_MODEL_TRANSFORM);
  wr_shader_program_use_uniform(phongStencilDiffuseSpecularProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE_TRANSFORM);
  wr_shader_program_use_uniform_buffer(phongStencilDiffuseSpecularProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_MATERIAL_PHONG);
  wr_shader_program_use_uniform_buffer(phongStencilDiffuseSpecularProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_LIGHTS);
  wr_shader_program_use_uniform_buffer(phongStencilDiffuseSpecularProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_LIGHT_RENDERABLE);
  wr_shader_program_use_uniform_buffer(phongStencilDiffuseSpecularProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_CAMERA_TRANSFORMS);
  wr_shader_program_set_vertex_shader_path(phongStencilDiffuseSpecularProgram,
                                           "../../../resources/wren/shaders/phong_stencil_diffuse_specular.vert");
  wr_shader_program_set_fragment_shader_path(phongStencilDiffuseSpecularProgram,
                                             "../../../resources/wren/shaders/phong_stencil_diffuse_specular.frag");
  wr_shader_program_setup(phongStencilDiffuseSpecularProgram);
  if (!wr_shader_program_get_gl_name(phongStencilDiffuseSpecularProgram))
    printf("Compilation failed: %s\n", wr_shader_program_get_compilation_log(phongStencilDiffuseSpecularProgram));

  wr_material_set_default_program(material, phongProgram);
  wr_material_set_stencil_ambient_emissive_program(material, phongStencilAmbientEmissiveProgram);
  wr_material_set_stencil_diffuse_specular_program(material, phongStencilDiffuseSpecularProgram);
}

// Create the WREN scene.
static void create_wren_scene() {
  WrViewport *vp = wr_scene_get_viewport(wr_scene_get_instance());

  WrCamera *camera = wr_viewport_get_camera(vp);
  float camera_position[3] = {15.0f, 14.0f, 40.0f};
  wr_camera_set_position(camera, camera_position);

  float background_color[3] = {0.1f, 0.5f, 0.8f};
  wr_viewport_set_clear_color_rgb(vp, background_color);

  float rgb[3] = {0.2f, 0.2f, 0.2f};
  wr_scene_set_ambient_light(rgb);

  WrDirectionalLight *light = wr_directional_light_new();
  float direction[3] = {0.2f, 0.2f, -0.2f};
  wr_directional_light_set_direction(light, direction);

  WrTransform *root = wr_scene_get_root(wr_scene_get_instance());

  for (int i = 0; i < 100; ++i) {
    printf("\33[2K\rLoading: %d%%", i);
    fflush(stdout);

    WrRenderable *sphereRenderable = wr_renderable_new();
    WrStaticMesh *sphereMesh = wr_static_mesh_unit_sphere_new(2, true, false);
    WrMaterial *sphereMaterial = wr_phong_material_new();
    initMaterialToPhong(sphereMaterial);
    float sphere_color[3] = {0.01f * i, 0.0001f * i * i, 1.0f - 0.01f * i};  // "random" color based on i.
    wr_phong_material_set_diffuse(sphereMaterial, sphere_color);
    WrTransform *sphereTransform = wr_transform_new();

    wr_renderable_set_mesh(sphereRenderable, WR_MESH(sphereMesh));
    wr_renderable_set_material(sphereRenderable, sphereMaterial, NULL);
    wr_transform_attach_child(sphereTransform, WR_NODE(sphereRenderable));

    float sphere_position[3] = {3.0f * (i / 10), 3.0f * (i % 10), 0.0f};  // grid position based on i.
    wr_transform_set_position(sphereTransform, sphere_position);
    wr_transform_attach_child(root, WR_NODE(sphereTransform));
  }
  printf("\n");
}

// Render function.
static void render() {
  static int i = 0;
  printf("\33[2K\rrendering iteration %d", i++);
  fflush(stdout);
  wr_scene_render(wr_scene_get_instance(), NULL, true);
  glutSwapBuffers();
}

// Call the render function in loop.
static void renderLoop(int t) {
  render();
  glutTimerFunc(refreshRate, renderLoop, 0);
}

int main(int argc, char **argv) {
  // Initialize the GLUT window.
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
  glutInitContextVersion(3, 3);  // i.e. OpenGL 3.3
  glutInitWindowSize(width, height);
  glutCreateWindow("WREN test window");

  // Initialize WREN to display on this window.
  WrFrameBuffer *mainFrameBuffer = wr_frame_buffer_new();
  wr_frame_buffer_set_size(mainFrameBuffer, width, height);
  WrTextureRtt *mainFrameBufferTexture = wr_texture_rtt_new();
  wr_texture_rtt_enable_initialize_data(mainFrameBufferTexture, true);
  wr_texture_set_internal_format(WR_TEXTURE(mainFrameBufferTexture), WR_TEXTURE_INTERNAL_FORMAT_RGBA8);
  wr_frame_buffer_append_output_texture(mainFrameBuffer, mainFrameBufferTexture);
  wr_frame_buffer_enable_depth_buffer(mainFrameBuffer, true);
  wr_frame_buffer_setup(mainFrameBuffer);

  WrViewport *vp = wr_scene_get_viewport(wr_scene_get_instance());
  wr_viewport_set_frame_buffer(vp, mainFrameBuffer);
  wr_viewport_set_size(vp, width, height);

  wr_gl_state_set_context_active(true);
  wr_scene_init(wr_scene_get_instance());

  // Create the WREN scene.
  create_wren_scene();

  // Call the render function.
  glutDisplayFunc(render);
  glutTimerFunc(refreshRate, renderLoop, 0);
  glutMainLoop();

  return 0;
}
