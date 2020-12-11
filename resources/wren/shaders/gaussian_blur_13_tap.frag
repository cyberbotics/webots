#version 330 core

precision highp float;

const int sourceTextureIndex = 0;
const int blurTextureIndex = 1;

// 13-tap filter using coefficients taken from pascal's triangle.
// This works because the binomial distribution is the discrete equivalent of the normal distribution.
// http://rastergrid.com/blog/2010/09/efficient-gaussian-blur-with-linear-sampling/
const int tapCount = 13;
const int weightCount = 1 + (tapCount - 1) / 2;
const float offsets[weightCount] = float[](0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
const float weights[weightCount] = float[](12870.0 / 65502.0, 11440.0 / 65502.0, 8008.0 / 65502.0, 4368.0 / 65502.0,
                                           1820.0 / 65502.0, 560.0 / 65502.0, 120.0 / 65502.0);

// Optimization: profit from hardware bilinear interpolation to sample two texels at once
// by sampling between the texels at a location determined by their weights
const int weightCountShared = (weightCount - 1) / 2;
const float weightsShared[weightCountShared] =
  float[](weights[1] + weights[2], weights[3] + weights[4], weights[5] + weights[6]);
const float offsetsShared[weightCountShared] =
  float[](offsets[1] + weights[1] / weightsShared[0], offsets[3] + weights[3] / weightsShared[1],
          offsets[5] + weights[5] / weightsShared[2]);

in vec2 texUv;

layout(location = 0) out vec4 result;

uniform int iterationNumber;

uniform sampler2D inputTextures[2];

vec4 convolveHorizontally(vec2 textureSizeSource, vec2 textureSizeBlur) {
  vec4 color;
  if (iterationNumber == 0) {
    color = weights[0] * texture(inputTextures[sourceTextureIndex], texUv);
    for (int i = 0; i < weightCountShared; ++i) {
      color += weightsShared[i] *
               texture(inputTextures[sourceTextureIndex], vec2(texUv.x + offsetsShared[i] / textureSizeSource.x, texUv.y));
      color += weightsShared[i] *
               texture(inputTextures[sourceTextureIndex], vec2(texUv.x - offsetsShared[i] / textureSizeSource.x, texUv.y));
    }
  } else {
    color = weights[0] * texture(inputTextures[blurTextureIndex], texUv);
    for (int i = 0; i < weightCountShared; ++i) {
      color += weightsShared[i] *
               texture(inputTextures[blurTextureIndex], vec2(texUv.x + offsetsShared[i] / textureSizeBlur.x, texUv.y));
      color += weightsShared[i] *
               texture(inputTextures[blurTextureIndex], vec2(texUv.x - offsetsShared[i] / textureSizeBlur.x, texUv.y));
    }
  }

  return color;
}

vec4 convolveVertically(vec2 textureSizeBlur) {
  vec4 color = weights[0] * texture(inputTextures[blurTextureIndex], texUv);
  for (int i = 0; i < weightCountShared; ++i) {
    color += weightsShared[i] *
             texture(inputTextures[blurTextureIndex], vec2(texUv.x, texUv.y + offsetsShared[i] / textureSizeBlur.y));
    color += weightsShared[i] *
             texture(inputTextures[blurTextureIndex], vec2(texUv.x, texUv.y - offsetsShared[i] / textureSizeBlur.y));
  }

  return color;
}

void main() {
  vec2 textureSizeSource = vec2(textureSize(inputTextures[sourceTextureIndex], 0));
  vec2 textureSizeBlur = vec2(textureSize(inputTextures[blurTextureIndex], 0));
  if (iterationNumber % 2 == 0)
    result = convolveHorizontally(textureSizeSource, textureSizeBlur);
  else
    result = convolveVertically(textureSizeBlur);
}
