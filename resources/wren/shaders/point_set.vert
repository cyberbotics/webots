#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 3) in vec3 vColor;

uniform mat4 modelTransform;
uniform float pointSize;

out vec3 color;

// Camera transforms for this frame
layout(std140) uniform CameraTransforms {
  mat4 view;
  mat4 projection;
  mat4 infiniteProjection;
}
cameraTransforms;

void main() {
  mat4 modelView = cameraTransforms.view * modelTransform;

  vec4 vCoordTransformed = modelView * vec4(vCoord, 1.0);

  color = vColor;
  gl_Position = cameraTransforms.infiniteProjection * vCoordTransformed;
  gl_PointSize = pointSize;
}
