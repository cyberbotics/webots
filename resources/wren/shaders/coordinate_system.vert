#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

out vec3 fragmentPosition;
out vec2 texUv;
out float depth;

uniform mat4 modelTransform;
uniform vec2 screenPosition;  // [-1; 1]
uniform float size;           // % of screen

// Camera transforms for this frame
layout(std140) uniform CameraTransforms {
  mat4 view;
  mat4 projection;
  mat4 infiniteProjection;
}
cameraTransforms;

void main() {
  mat4 projection = cameraTransforms.infiniteProjection;
  vec4 vertex = modelTransform * vec4(vCoord, 1.0f);

  // Scale to remove distortion
  float aspect = projection[1][1] / projection[0][0];
  if (aspect < 1.0f)
    vertex.x /= aspect;
  else
    vertex.y *= aspect;

  vertex.x += screenPosition.x - (size / aspect);
  vertex.y += screenPosition.y + (size * aspect);
  vertex.z = -vertex.z;
  texUv = vTexCoord;
  gl_Position = vertex;
}
