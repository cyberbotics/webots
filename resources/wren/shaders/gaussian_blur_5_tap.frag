#version 330 core

precision highp float;

const int sourceTextureIndex = 0;
const int blurTextureIndex = 1;

// 5-tap filter using coefficients taken from pascal's triangle.
// This works because the binomial distribution is the discrete equivalent of the normal distribution.
// http://rastergrid.com/blog/2010/09/efficient-gaussian-blur-with-linear-sampling/
const int tapCount = 5;
const int weightCount = 1 + (tapCount - 1) / 2;
const float offsets[weightCount] = float[](0.0, 1.0, 2.0);
const float weights[weightCount] = float[](20.0 / 62.0, 15.0 / 62.0, 6.0 / 62.0);

// Optimization: profit from hardware bilinear interpolation to sample two texels at once
// by sampling between the texels at a location determined by their weights
const int weightCountShared = (weightCount - 1) / 2;
const float weightsShared[weightCountShared] = float[](weights[1] + weights[2]);
const float offsetsShared[weightCountShared] = float[](offsets[1] + weights[1] / weightsShared[0]);

in vec2 texUv;

layout(location = 0) out vec4 result;

uniform int iterationNumber;
uniform vec2 viewportSize;

uniform sampler2D inputTextures[2];

vec4 convolveHorizontally() {
  vec4 color;
  if (iterationNumber == 0) {
    color = weights[0] * texture(inputTextures[sourceTextureIndex], texUv);
    for (int i = 0; i < weightCountShared; ++i) {
      color += weightsShared[i] *
               texture(inputTextures[sourceTextureIndex], vec2(texUv.x + offsetsShared[i] / viewportSize.x, texUv.y));
      color += weightsShared[i] *
               texture(inputTextures[sourceTextureIndex], vec2(texUv.x - offsetsShared[i] / viewportSize.x, texUv.y));
    }
  } else {
    color = weights[0] * texture(inputTextures[blurTextureIndex], texUv);
    for (int i = 0; i < weightCountShared; ++i) {
      color +=
        weightsShared[i] * texture(inputTextures[blurTextureIndex], vec2(texUv.x + offsetsShared[i] / viewportSize.x, texUv.y));
      color +=
        weightsShared[i] * texture(inputTextures[blurTextureIndex], vec2(texUv.x - offsetsShared[i] / viewportSize.x, texUv.y));
    }
  }

  return color;
}

vec4 convolveVertically() {
  vec4 color = weights[0] * texture(inputTextures[blurTextureIndex], texUv);
  for (int i = 0; i < weightCountShared; ++i) {
    color +=
      weightsShared[i] * texture(inputTextures[blurTextureIndex], vec2(texUv.x, texUv.y + offsetsShared[i] / viewportSize.y));
    color +=
      weightsShared[i] * texture(inputTextures[blurTextureIndex], vec2(texUv.x, texUv.y - offsetsShared[i] / viewportSize.y));
  }

  return color;
}

void main() {
  if (iterationNumber % 2 == 0)
    result = convolveHorizontally();
  else
    result = convolveVertically();
}
