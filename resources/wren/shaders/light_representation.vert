#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

uniform mat4 modelTransform;
uniform float screenScale;

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
  mat4 projection = cameraTransforms.infiniteProjection;
  mat4 view = cameraTransforms.view;

  float w = screenScale;
  // Remove perspective scaling
  if (projection[3][3] == 0.0f) {
    vec4 row = vec4(-view[0][2], -view[1][2], -view[2][2], -view[3][2]);
    w *= dot(row, modelTransform[3]);
  }
  // Remove FOV scaling
  w /= min(projection[0][0], projection[1][1]);

  // Remove rotation & scaling for billboarding
  modelView[0].xyz = vec3(1.0f, 0.0f, 0.0f);
  modelView[1].xyz = vec3(0.0f, 1.0f, 0.0f);
  modelView[2].xyz = vec3(0.0f, 0.0f, 1.0f);

  gl_Position = projection * modelView * vec4(vCoord * w, 1.0f);
  texUv = vTexCoord;
}
