#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

uniform vec2 viewportSize;
out vec2 texUv;
out vec4 texUvOffsets[3];

void SMAAEdgeDetection(vec2 texcoord) {
  // WebGL port note: Changed sign in W components
  texUvOffsets[0] = texcoord.xyxy + 1.0 / viewportSize.xyxy * vec4(-1.0, 0.0, 0.0, 1.0);
  texUvOffsets[1] = texcoord.xyxy + 1.0 / viewportSize.xyxy * vec4(1.0, 0.0, 0.0, -1.0);
  texUvOffsets[2] = texcoord.xyxy + 1.0 / viewportSize.xyxy * vec4(-2.0, 0.0, 0.0, 2.0);
}

void main() {
  texUv = vTexCoord;
  SMAAEdgeDetection(texUv);

  gl_Position = vec4(vec2(-1.0) + 2.0 * vCoord.xy, 0.0, 1.0);
}
