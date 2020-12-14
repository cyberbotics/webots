#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

out vec2 texUv;

void main() {
  texUv = vTexCoord;

  gl_Position = vec4(vec2(-1.0) + 2.0 * vCoord.xy, 0.0, 1.0);
}
