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
 * and display a sphere using the Webots default shader (without colors and with a texture).
 */

#include <GL/freeglut.h>

#include <wren/camera.h>
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

// Create the WREN scene.
static void create_wren_scene() {
  WrViewport *vp = wr_scene_get_viewport(wr_scene_get_instance());

  WrCamera *camera = wr_viewport_get_camera(vp);
  float camera_position[3] = {0.0f, 0.0f, 10.0f};
  wr_camera_set_position(camera, camera_position);

  float background_color[3] = {0.1f, 0.5f, 0.8f};
  wr_viewport_set_clear_color_rgb(vp, background_color);

  // Generate a dummy texture.
  WrTexture2d *texture = wr_texture_2d_new();
  wr_texture_2d_set_file_path(texture, "dummy.jpg");
  wr_texture_set_size(WR_TEXTURE(texture), 256, 256);
  char *data = static_cast<char *>(malloc(256 * 256 * 4));
  for (unsigned int d = 0; d < (256 * 256); ++d) {
    data[4 * d] = (d) % 0xff;
    data[4 * d + 1] = (d + d) % 0xff;
    data[4 * d + 2] = (d * d) % 0xff;
    data[4 * d + 3] = 0xff;
  }
  wr_texture_2d_set_data(texture, data);
  wr_texture_setup(WR_TEXTURE(texture));
  free(data);

  WrShaderProgram *sphereProgram = wr_shader_program_new();
  wr_shader_program_use_uniform(sphereProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE0);
  wr_shader_program_use_uniform(sphereProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE1);
  wr_shader_program_use_uniform(sphereProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE2);
  wr_shader_program_use_uniform(sphereProgram, WR_GLSL_LAYOUT_UNIFORM_MODEL_TRANSFORM);
  wr_shader_program_use_uniform(sphereProgram, WR_GLSL_LAYOUT_UNIFORM_TEXTURE_TRANSFORM);
  wr_shader_program_use_uniform_buffer(sphereProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_MATERIAL_PHONG);
  wr_shader_program_use_uniform_buffer(sphereProgram, WR_GLSL_LAYOUT_UNIFORM_BUFFER_CAMERA_TRANSFORMS);
  wr_shader_program_set_vertex_shader_path(sphereProgram, "../../../resources/wren/shaders/default.vert");
  wr_shader_program_set_fragment_shader_path(sphereProgram, "../../../resources/wren/shaders/default.frag");
  wr_shader_program_setup(sphereProgram);
  if (!wr_shader_program_get_gl_name(sphereProgram))
    printf("Compilation failed: %s\n", wr_shader_program_get_compilation_log(sphereProgram));

  WrRenderable *sphereRenderable = wr_renderable_new();
  WrStaticMesh *sphereMesh = wr_static_mesh_unit_sphere_new(2, true, false);
  WrMaterial *sphereMaterial = wr_phong_material_new();
  wr_material_set_default_program(sphereMaterial, sphereProgram);
  wr_material_set_texture(sphereMaterial, WR_TEXTURE(texture), 0);
  WrTransform *sphereTransform = wr_transform_new();

  wr_renderable_set_mesh(sphereRenderable, WR_MESH(sphereMesh));
  wr_renderable_set_material(sphereRenderable, sphereMaterial, NULL);
  wr_transform_attach_child(sphereTransform, WR_NODE(sphereRenderable));

  WrTransform *root = wr_scene_get_root(wr_scene_get_instance());
  wr_transform_attach_child(root, WR_NODE(sphereTransform));
}

// Render function.
static void render() {
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
