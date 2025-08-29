#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 1) in vec3 vNormal;
layout(location = 3) in vec3 vColor;

uniform mat4 modelTransform;

out vec3 color;
out vec3 normalTransformed;

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
  gl_Position = cameraTransforms.infiniteProjection * vCoordTransformed;
  color = vColor;

  normalTransformed = mat3(transpose(inverse(modelView))) * vNormal;
}
