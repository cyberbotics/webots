#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aTexCoords;

out vec2 texUv;

void main() {
  texUv = aTexCoords;
  gl_Position = vec4(aPos, 1.0);
}
