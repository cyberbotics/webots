#version 330 core

precision highp float;

// This shader does the heavy lifting for GTAO based on
// https://github.com/asylum2010/Asylum_Tutorials/blob/master/ShaderTutors/media/shadersGL/gtao.frag

#define PI 3.1415926535897932
#define PI_HALF 1.5707963267948966

uniform sampler2D inputTextures[2];
uniform sampler2D gtaoTexture;

uniform float radius;
uniform bool flipNormalY;
uniform vec4 clipInfo;
uniform vec2 viewportSize;
uniform vec4 params;

out float fragColor;

in vec2 texUv;

#define asfloat(x) intBitsToFloat(x)
#define asint(x) floatBitsToInt(x)

// Camera transforms for this frame
layout(std140) uniform CameraTransforms {
  mat4 view;
  mat4 projection;
  mat4 infiniteProjection;
}
cameraTransforms;

float GTAOFastSqrt(float x) {
  // [Drobot2014a] Low Level Optimizations for GCN
  return asfloat(0x1FBD1DF5 + (asint(x) >> 1));
}

float GTAOFastAcos(float x) {
  float res = -0.156583 * abs(x) + PI_HALF;
  res *= GTAOFastSqrt(1.0 - abs(x));
  return x >= 0.0 ? res : PI - res;
}

vec4 getViewSpacePosition(vec2 pixelLocation, mat4 inverseProjectionMatrix) {
  // Get the depth value for this pixel
  float z = textureLod(inputTextures[0], pixelLocation, 0.0).r;
  if (z == 1.0)
    return vec4(0.0);
  // Get x/w and y/w from the viewport position
  float x = pixelLocation.x * 2.0 - 1.0;
  float y = pixelLocation.y * 2.0 - 1.0;
  vec4 projectedPosition = vec4(x, y, z, 1.0f);
  // Transform by the inverse projection matrix
  vec4 vPositionVS = inverseProjectionMatrix * projectedPosition;
  vPositionVS.z = -vPositionVS.z;
  // Divide by w to get the view-space position
  return vec4(vPositionVS.xyz / vPositionVS.w, vPositionVS.w);
}

float getFalloff(float squaredDistance) {
#define FALLOFF_START2 0.16
#define FALLOFF_END2 max(radius *radius, FALLOFF_START2)
  return 2.0 * clamp((squaredDistance - FALLOFF_START2) / (FALLOFF_END2 - FALLOFF_START2), 0.0, 1.0);
}

void searchForHorizons(inout vec2 horizons, vec2 offset, vec3 viewVector, vec4 viewSpacePosition,
                       mat4 inverseProjectionMatrix) {
  // search for h1
  vec4 sampleViewspacePosition;
  vec3 sliceSampleToOrigin;
  sampleViewspacePosition = getViewSpacePosition((gl_FragCoord.xy + offset) / viewportSize, inverseProjectionMatrix);
  float squaredSampleDistance, inverseSquaredSampleDistance, horizonCosine, falloff = 0.0;
  if (sampleViewspacePosition != vec4(0.0)) {
    sliceSampleToOrigin = sampleViewspacePosition.xyz - viewSpacePosition.xyz;

    squaredSampleDistance = dot(sliceSampleToOrigin, sliceSampleToOrigin);
    inverseSquaredSampleDistance = inversesqrt(squaredSampleDistance);
    horizonCosine = inverseSquaredSampleDistance * dot(sliceSampleToOrigin, viewVector);

    falloff = getFalloff(squaredSampleDistance);
    horizons.x = max(horizons.x, horizonCosine - falloff);
  }

  // search for h2
  sampleViewspacePosition = getViewSpacePosition((gl_FragCoord.xy - offset) / viewportSize, inverseProjectionMatrix);
  if (sampleViewspacePosition != vec4(0.0)) {
    sliceSampleToOrigin = sampleViewspacePosition.xyz - viewSpacePosition.xyz;

    squaredSampleDistance = pow(length(sliceSampleToOrigin), 2.0);
    inverseSquaredSampleDistance = inversesqrt(squaredSampleDistance);
    horizonCosine = inverseSquaredSampleDistance * dot(sliceSampleToOrigin, viewVector);

    falloff = getFalloff(squaredSampleDistance);
    horizons.y = max(horizons.y, horizonCosine - falloff);
  }
}

float integrateArc(vec2 horizons, float cosN, float sinN2, float projectedNormalLength, float n) {
  return projectedNormalLength * 0.25 *
         ((horizons.x * sinN2 + cosN - cos(2.0 * horizons.x - n)) + (horizons.y * sinN2 + cosN - cos(2.0 * horizons.y - n)));
}

void main() {
  vec3 viewSpaceNormal = textureLod(inputTextures[1], texUv, 0.0).rgb;

  mat4 inverseProjectionMatrix = inverse(cameraTransforms.projection);

  // first, retrieve view-space position and normals
  vec4 viewSpacePosition = getViewSpacePosition(texUv, inverseProjectionMatrix);
  if (viewSpaceNormal == vec3(0.0) || viewSpacePosition.z > 200.0) {
    // discard any fragments from the background or those which don't have a normal
    fragColor = 1.0;
    return;
  }

  viewSpaceNormal -= 0.5;
  viewSpaceNormal *= 2.0;
  viewSpaceNormal = normalize(vec3(viewSpaceNormal.xy, -viewSpaceNormal.z));

  if (flipNormalY)
    viewSpaceNormal.y *= -1.0;

  // calculate screen-space radius
  float projectedRadius = (radius * clipInfo.z) / viewSpacePosition.z;
  float stepsize = projectedRadius / params.z;

  // fetch noises and calculate jittered slice angle
  ivec2 loc = ivec2(gl_FragCoord.xy);
  vec2 noises = texelFetch(gtaoTexture, loc % 4, 0).rg;
  float sliceAngle = (params.x + noises.x) * PI;
  float currentStep = mod(params.y + noises.y, 1.0) * (stepsize - 1.0) + 1.0;
  vec3 searchDirection = vec3(cos(sliceAngle), sin(sliceAngle), 0.0);

  // set up last couple of things to find horizon angles
  vec3 viewVector = normalize(-viewSpacePosition.xyz);
  vec2 horizons = vec2(-1.0, -1.0);

  vec2 offset;
  for (int j = 0; j < int(params.z); ++j) {
    offset = round(searchDirection.xy * currentStep);
    currentStep += stepsize;

    searchForHorizons(horizons, offset, viewVector, viewSpacePosition, inverseProjectionMatrix);
  }

  horizons = vec2(GTAOFastAcos(horizons.x), GTAOFastAcos(horizons.y));

  // calculate n (angle between plane normal and projected normal)
  vec3 bitangent = normalize(cross(searchDirection, viewVector));
  vec3 planeNormal = cross(viewVector, bitangent);
  vec3 projectedNormal = viewSpaceNormal - bitangent * dot(viewSpaceNormal, bitangent);

  float projectedNormalLength = length(projectedNormal);
  float inverseProjectedNormalLength = 1.0 / (projectedNormalLength + 1e-6);       // to avoid division with zero
  float cosXi = dot(projectedNormal, planeNormal) * inverseProjectedNormalLength;  // xi = n + PI_HALF
  float n = GTAOFastAcos(cosXi) - PI_HALF;
  float cosN = dot(projectedNormal, viewVector) * inverseProjectedNormalLength;
  float sinN2 = -2.0 * cosXi;  // cos(x + PI_HALF) = -sin(x)

  // clamp to normal hemisphere
  horizons.x = n + max(-horizons.x - n, -PI_HALF);
  horizons.y = n + min(horizons.y - n, PI_HALF);

  // distance filter - accept all values until 100m, then ramp down to 0 contrib at 200m
  float distanceFalloff = (max(min(0.01 * (200.0 - viewSpacePosition.z), 1.0), 0.0));

  fragColor = 1.0 - ((1.0 - integrateArc(horizons, cosN, sinN2, projectedNormalLength, n)) * distanceFalloff);
}
