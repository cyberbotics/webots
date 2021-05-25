#version 330 core
layout(location = 0) in vec3 vCoord;

out vec3 texUv;

// Camera transforms for this frame
layout(std140) uniform CameraTransforms {
  mat4 view;
  mat4 projection;
  mat4 infiniteProjection;
}
cameraTransforms;

void main() {
  gl_Position = cameraTransforms.infiniteProjection * mat4(mat3(cameraTransforms.view)) * vec4(vCoord, 1.0);
  texUv = vCoord;
}
