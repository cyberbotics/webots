#version 330 core

#define SMAA_MAX_SEARCH_STEPS 16
#define RESOLUTION (1.0 / viewportSize)

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

uniform vec2 viewportSize;
out vec2 texUv;
out vec4 texUvOffsets[3];
out vec2 vPixcoord;

void SMAABlendingWeightCalculationVS(vec2 texcoord) {
  vPixcoord = texcoord / RESOLUTION;

  // We will use these offsets for the searches later on
  texUvOffsets[0] = texcoord.xyxy + RESOLUTION.xyxy * vec4(-0.25, 0.125, 1.25, 0.125);
  texUvOffsets[1] = texcoord.xyxy + RESOLUTION.xyxy * vec4(-0.125, 0.25, -0.125, -1.25);

  // And these for the searches, they indicate the ends of the loops:
  texUvOffsets[2] =
    vec4(texUvOffsets[0].xz, texUvOffsets[1].yw) + vec4(-2.0, 2.0, -2.0, 2.0) * RESOLUTION.xxyy * float(SMAA_MAX_SEARCH_STEPS);
}

void main() {
  texUv = vTexCoord;
  SMAABlendingWeightCalculationVS(texUv);
  gl_Position = vec4(vec2(-1.0) + 2.0 * vCoord.xy, 0.0, 1.0);
}
