#version 330 core

precision highp float;

const int sceneTextureIndex = 0;

in vec2 texUv;

layout(location = 0) out float result;

uniform float resolution;

uniform sampler2D inputTextures[1];

void main() {
  result = texture(inputTextures[sceneTextureIndex], texUv).x;
  if (resolution > 0.0) {
    result = floor(result / resolution + 0.5) * resolution;
  }
}
