#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

uniform mat4 modelTransform;
uniform mat4 textureTransform;

out vec2 texUv;

// Camera transforms for this frame
layout(std140) uniform CameraTransforms {
  mat4 view;
  mat4 projection;
  mat4 infiniteProjection;
}
cameraTransforms;

void main() {
  mat4 modelView = cameraTransforms.view * modelTransform;

  vec4 vCoordTransformed = modelView * vec4(vCoord, 1.0f);
  gl_Position = cameraTransforms.infiniteProjection * vCoordTransformed;

  texUv = vec2(textureTransform * vec4(vTexCoord, 0.0f, 1.0f));
}
