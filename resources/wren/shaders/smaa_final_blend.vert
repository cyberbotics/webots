#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

uniform vec2 viewportSize;
out vec2 texUv;
out vec4 texUvOffsets[2];

void SMAANeighborhoodBlendingVS(vec2 texcoord) {
  texUvOffsets[0] =
    texcoord.xyxy + (1.0 / viewportSize).xyxy * vec4(-1.0, 0.0, 0.0, -1.0);  // WebGL port note: Changed sign in W component
  texUvOffsets[1] =
    texcoord.xyxy + (1.0 / viewportSize).xyxy * vec4(1.0, 0.0, 0.0, 1.0);  // WebGL port note: Changed sign in W component
}

void main() {
  texUv = vTexCoord;

  SMAANeighborhoodBlendingVS(texUv);

  gl_Position = vec4(vec2(-1.0) + 2.0 * vCoord.xy, 0.0, 1.0);
}
