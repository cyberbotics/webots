#version 330 core

precision highp float;

const int lastResultTextureIndex = 0;

in vec2 texUv;

layout(location = 0) out vec4 fragColor;

uniform sampler2D inputTextures[2];

uniform float uBias;
uniform float uScale;
uniform float uGhostDispersal;
uniform float uHaloWidth;
uniform float uDistortion;
uniform int uSamples;

vec3 textureDistorted(in sampler2D tex, in vec2 texcoord, in vec2 direction, in vec3 distortion) {
  return vec3(texture(tex, texcoord + direction * distortion.r).r, texture(tex, texcoord + direction * distortion.g).g,
              texture(tex, texcoord + direction * distortion.b).b);
}

vec3 thresholded(vec2 texCoord, vec2 direction, vec3 distortion) {
  return max(vec3(0.0), textureDistorted(inputTextures[0], texCoord, direction, distortion) + uBias) * uScale;
}

void main() {
  vec3 result = vec3(0.0);
  vec2 texCoord = -texUv + vec2(1.0);
  vec2 texelSize = 1.0 / vec2(textureSize(inputTextures[0], 0));

  // Ghosts
  vec2 ghostVec = (vec2(0.5) - texCoord) * uGhostDispersal;

  vec3 distortion = vec3(-texelSize.x * uDistortion, 0.0, texelSize.x * uDistortion);
  vec2 direction = normalize(ghostVec);

  for (int i = 0; i < uSamples; ++i) {
    vec2 offset = fract(texCoord + ghostVec * float(i));

    float weight = length(vec2(0.5) - offset) / length(vec2(0.5));
    weight = pow(1.0 - weight, 10.0);

    result += thresholded(offset, direction, distortion) * weight;
  }

  // Color modulation
  result *= texture(inputTextures[1], vec2(length(vec2(0.5) - texCoord) / length(vec2(0.5)), 0.5)).rgb;

  // Halo
  vec2 haloVec = normalize(ghostVec) * uHaloWidth;
  float weight = length(vec2(0.5) - fract(texCoord + haloVec)) / length(vec2(0.5));
  weight = pow(1.0 - weight, 5.0);
  result += thresholded(texCoord + haloVec, direction, distortion) * weight;

  fragColor = vec4(result, 1.0);
}
