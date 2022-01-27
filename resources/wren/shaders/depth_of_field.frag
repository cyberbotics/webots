#version 330 core

precision highp float;

const int sceneTextureIndex = 0;  // full resolution scene texture
const int depthTextureIndex = 1;  // full resolution depth buffer texture
const int blurTextureIndex = 2;   // downsampled and blurred scene texture

const int numTaps = 12;
const vec2 poissonCoords[numTaps] = vec2[numTaps](vec2(0.00, 0.00), vec2(0.07, -0.45), vec2(-0.15, -0.33), vec2(0.35, -0.32),
                                                  vec2(-0.39, -0.26), vec2(0.10, -0.23), vec2(0.36, -0.12), vec2(-0.31, -0.01),
                                                  vec2(-0.38, 0.22), vec2(0.36, 0.23), vec2(-0.13, 0.29), vec2(0.14, 0.41));

const vec2 maxCoC = vec2(5.0, 10.0);  // max. circle of confusion (CoC) radius and diameter in pixels

in vec2 texUv;

layout(location = 0) out vec4 fragColor;

// Depth of field parameters:
// x = near blur depth, y = focal plane depth, z = far blur depth
// w = blurriness cutoff constant for objects behind the focal plane
uniform vec4 dofParams;
uniform vec2 cameraParams;  // x: zNear, y: zFar
uniform vec2 blurTextureSize;
uniform vec2 viewportSize;
uniform sampler2D inputTextures[3];

float linearizeDepth(float z) {
  float zNdc = 2.0 * z - 1.0;
  return cameraParams.x +
         2.0 * cameraParams.x * cameraParams.y / (cameraParams.y + cameraParams.x - zNdc * (cameraParams.y - cameraParams.x));
}

float applyDofParams(float depth) {
  if (depth < dofParams.y) {
    // scale depth value between near blur distance and focal distance to [-1, 0] range
    depth = (depth - dofParams.y) / (dofParams.y - dofParams.x);
  } else {
    // scale depth value between focal distance and far blur distance to [0, 1] range
    depth = (depth - dofParams.y) / (dofParams.z - dofParams.y);
    // clamp the far blur to a maximum blurriness
    depth = clamp(depth, 0.0, dofParams.w);
  }

  // scale and bias into [0, 1] range
  return depth;
}

void main() {
  float centerDepth = applyDofParams(linearizeDepth(texture(inputTextures[depthTextureIndex], texUv).x));
  centerDepth = 0.5 * centerDepth + 0.5;

  // Convert depth of center tap into blur radius in pixels
  float radiusScale = 0.5 * (blurTextureSize.x / viewportSize.x + blurTextureSize.y / viewportSize.y);
  float discRadiusScene = abs(centerDepth * maxCoC.y - maxCoC.x);
  float discRadiusBlur = discRadiusScene * radiusScale;  // radius on blur texture

  vec2 viewportPixelSize = vec2(1.0) / viewportSize;
  vec2 blurTexturePixelSize = vec2(1.0) / blurTextureSize;
  vec4 sum = vec4(0.0);

  for (int i = 0; i < numTaps; ++i) {
    // compute texture coordinates
    vec2 coordScene = texUv + viewportPixelSize * poissonCoords[i] * discRadiusScene;
    vec2 coordBlur = texUv + blurTexturePixelSize * poissonCoords[i] * discRadiusBlur;

    // fetch taps and depth
    vec4 tapScene = texture(inputTextures[sceneTextureIndex], coordScene);
    vec4 tapBlur = texture(inputTextures[blurTextureIndex], coordBlur);
    float tapDepth = applyDofParams(linearizeDepth(texture(inputTextures[depthTextureIndex], coordScene).x));

    // mix low and high res. taps based on tap blurriness
    float blurAmount = abs(tapDepth);  // put blurriness into [0, 1]
    vec4 tap = mix(tapScene, tapBlur, blurAmount);

    // "smart" blur ignores taps that are closer than the center tap and in focus
    float factor = (tapDepth >= centerDepth) ? 1.0 : blurAmount;

    // accumulate
    sum.rgb += tap.rgb * factor;
    sum.a += factor;
  }

  fragColor = sum / sum.a;
}
