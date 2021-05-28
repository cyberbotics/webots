#version 330 core

precision highp float;

// This shader does the inter-frame filtering for GTAO based on
// https://github.com/asylum2010/Asylum_Tutorials/blob/master/ShaderTutors/media/shadersGL/gtaotemporaldenoise.frag

// these textures represent: accumulated ao, current frame ao, previous depth buffer and current depth buffer
uniform sampler2D inputTextures[4];

in vec2 texUv;
out vec4 fragColor;

uniform mat4 previousInverseViewMatrix;

// Camera transforms for this frame
layout(std140) uniform CameraTransforms {
  mat4 view;
  mat4 projection;
  mat4 infiniteProjection;
}
cameraTransforms;

vec4 viewSpacePosition(vec2 pixelLocation, bool previousFrame) {
  // Get the depth value for this pixel
  float z = 0.0;
  if (previousFrame)
    z = texture(inputTextures[2], pixelLocation).r;
  else
    z = texture(inputTextures[3], pixelLocation).r;

  // Get x/w and y/w from the viewport position
  float x = pixelLocation.x * 2.0 - 1.0;
  float y = pixelLocation.y * 2.0 - 1.0;
  vec4 projectedPosition = vec4(x, y, z, 1.0f);
  // Transform by the inverse projection matrix
  vec4 vPositionVS = inverse(cameraTransforms.projection) * projectedPosition;
  // Divide by w to get the view-space position
  return vec4(vPositionVS.xyz / vPositionVS.w, 1.0);
}

vec3 previousScreenSpacePosition(vec3 worldSpacePosition) {
  vec4 viewSpacePosition = inverse(previousInverseViewMatrix) * vec4(worldSpacePosition, 1.0);
  vec4 clipSpacePosition = cameraTransforms.projection * viewSpacePosition;

  return vec3(clipSpacePosition.xy / clipSpacePosition.w, 0.0);
}

void main() {
  // reproject to previous frame screen space
  vec4 currentViewSpacePosition = viewSpacePosition(texUv, false);

  // current frame world-space
  vec4 currentWorldSpacePosition = inverse(cameraTransforms.view) * currentViewSpacePosition;

  vec3 lastScreenSpacePosition = previousScreenSpacePosition(currentWorldSpacePosition.xyz);
  vec2 lastFrameTexUv = lastScreenSpacePosition.xy * 0.5 + 0.5;

  // unproject to previous frame world space
  vec4 previousWorldSpacePosition = previousInverseViewMatrix * viewSpacePosition(lastFrameTexUv, true);

  // detect disocclusion
  float dist2 = dot(currentWorldSpacePosition.xyz - previousWorldSpacePosition.xyz,
                    currentWorldSpacePosition.xyz - previousWorldSpacePosition.xyz);

  // fetch values
  vec2 currAO = texture(inputTextures[1], texUv).rg;
  vec2 accumAO = texture(inputTextures[0], lastFrameTexUv).rg;

  float weight = max(0.0, 0.8 - sqrt(dist2) * 100.0);
  float ao = mix(currAO.x, accumAO.x, weight);

  // fragColor = vec4(currentWorldSpacePosition.xyz, 1.0);
  fragColor = vec4(vec3(ao), 1.0);
}
