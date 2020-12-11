#version 330 core

precision highp float;

// This shader does the spatial filtering for GTAO based on
// https://github.com/asylum2010/Asylum_Tutorials/blob/master/ShaderTutors/media/shadersGL/gtaospatialdenoise.frag

uniform sampler2D inputTextures[2];

in vec2 texUv;
out vec2 fragColor;

// Camera transforms for this frame
layout(std140) uniform CameraTransforms {
  mat4 view;
  mat4 projection;
  mat4 infiniteProjection;
}
cameraTransforms;

vec4 getViewSpacePosition(vec2 pixelLocation) {
  // Get the depth value for this pixel
  float z = texture(inputTextures[1], pixelLocation).r;
  if (z == 1.0)
    return vec4(0.0);
  // Get x/w and y/w from the viewport position
  float x = pixelLocation.x * 2.0 - 1.0;
  float y = pixelLocation.y * 2.0 - 1.0;
  vec4 projectedPosition = vec4(x, y, z, 1.0f);
  // Transform by the inverse projection matrix
  vec4 vPositionVS = inverse(cameraTransforms.projection) * projectedPosition;
  vPositionVS.z = -vPositionVS.z;
  // Divide by w to get the view-space position
  return vec4(vPositionVS.xyz / vPositionVS.w, vPositionVS.w);
}

float gatherWeightedFragment(inout float totalweight, vec2 fragCoord, float referenceDepth, vec2 texSize) {
  vec4 sampleViewSpacePosition = getViewSpacePosition(fragCoord / texSize);
  if (sampleViewSpacePosition.w == 0.0)
    return 0.0;

  float ao = textureLod(inputTextures[0], fragCoord / texSize, 0.0).r;
  float sampleDepth = sampleViewSpacePosition.z;

  float relativeDepth = abs(sampleDepth - referenceDepth) / (referenceDepth * 0.1);
  float w = max(0.0, 0.1 - relativeDepth) * 30.0;

  totalweight += w;

  return ao * w;
}

void main() {
  vec4 viewSpacePosition = getViewSpacePosition(texUv);
  if (viewSpacePosition.w == 0.0) {
    fragColor = vec2(1.0, 0.0);
    return;
  }

  vec2 realTextureSize = vec2(textureSize(inputTextures[0], 0));
  vec2 center = (texUv * realTextureSize) - 2.0;
  ivec2 loc = ivec2(center);

  float ao0 = texture(inputTextures[0], texUv).r;
  float referenceDepth = viewSpacePosition.z;
  float totalweight = 1.0;
  float totalao = ao0;
  // NOTE: textureGather requires GL 4
  totalao += gatherWeightedFragment(totalweight, center + vec2(1, 0), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(2, 0), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(3, 0), referenceDepth, realTextureSize);

  totalao += gatherWeightedFragment(totalweight, center + vec2(0, 1), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(1, 1), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(2, 1), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(3, 1), referenceDepth, realTextureSize);

  totalao += gatherWeightedFragment(totalweight, center + vec2(0, 2), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(1, 2), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(2, 2), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(3, 2), referenceDepth, realTextureSize);

  totalao += gatherWeightedFragment(totalweight, center + vec2(0, 3), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(1, 3), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(2, 3), referenceDepth, realTextureSize);
  totalao += gatherWeightedFragment(totalweight, center + vec2(3, 3), referenceDepth, realTextureSize);

  fragColor = vec2(totalao / totalweight, 0.0);
}
