#version 330 core

precision highp float;

in vec2 texUv;

layout(location = 0) out vec4 result;

uniform sampler2D inputTextures[2];
uniform float transparency;

void main() {
  vec4 texAColor = texture(inputTextures[0], texUv);
  vec4 texBColor = texture(inputTextures[1], texUv);
  result = texAColor + texBColor * (1.0 - transparency);
}
