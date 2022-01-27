#version 330 core

precision highp float;

const int sourceTextureIndex = 0;
const int blurTextureIndex = 1;

in vec2 texUv;

layout(location = 0) out vec4 result;

uniform int iterationNumber;
uniform vec2 viewportSize;

uniform int taps;
uniform vec4 offsets;
uniform vec4 weights;

uniform sampler2D inputTextures[2];

vec4 convolveHorizontally() {
  vec4 color;
  if (iterationNumber == 0) {
    color = weights[0] * texture(inputTextures[sourceTextureIndex], texUv);
    for (int i = 1; i < taps; ++i) {
      color += weights[i] * texture(inputTextures[sourceTextureIndex], vec2(texUv.x + offsets[i] / viewportSize.x, texUv.y));
      color += weights[i] * texture(inputTextures[sourceTextureIndex], vec2(texUv.x - offsets[i] / viewportSize.x, texUv.y));
    }
  } else {
    color = weights[0] * texture(inputTextures[blurTextureIndex], texUv);
    for (int i = 1; i < taps; ++i) {
      color += weights[i] * texture(inputTextures[blurTextureIndex], vec2(texUv.x + offsets[i] / viewportSize.x, texUv.y));
      color += weights[i] * texture(inputTextures[blurTextureIndex], vec2(texUv.x - offsets[i] / viewportSize.x, texUv.y));
    }
  }

  return color;
}

vec4 convolveVertically() {
  vec4 color = weights[0] * texture(inputTextures[blurTextureIndex], texUv);
  for (int i = 1; i < taps; ++i) {
    color += weights[i] * texture(inputTextures[blurTextureIndex], vec2(texUv.x, texUv.y + offsets[i] / viewportSize.y));
    color += weights[i] * texture(inputTextures[blurTextureIndex], vec2(texUv.x, texUv.y - offsets[i] / viewportSize.y));
  }

  return color;
}

void main() {
  if (iterationNumber % 2 == 0)
    result = convolveHorizontally();
  else
    result = convolveVertically();
}
