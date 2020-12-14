#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 1) in vec3 vNormal;

out vec3 fragmentPosition;
out vec3 normalTransformed;

uniform mat4 modelTransform;

uniform float screenScale;  // 2 * object size on screen / screen width (both in pixels)
uniform float depthScale;

// Camera transforms for this frame
layout(std140) uniform CameraTransforms {
  mat4 view;
  mat4 projection;
  mat4 infiniteProjection;
}
cameraTransforms;

// Material parameters for this renderable
layout(std140) uniform PhongMaterial {
  vec4 ambient;
  vec4 diffuse;
  vec4 specularAndExponent;
  vec4 emissiveAndOpacity;
  vec4 textureFlags;
}
material;

void main() {
  mat4 projection = cameraTransforms.infiniteProjection;
  mat4 view = cameraTransforms.view;

  float w = screenScale;
  if (projection[3][3] == 0.0f) {  // perspective
    vec4 row = vec4(-view[0][2], -view[1][2], -view[2][2], -view[3][2]);
    w *= dot(row, modelTransform[3]);

    // Remove scaling due to FOV
    w *= 1.0f / min(projection[0][0], projection[1][1]);
  } else {  // orthographic
    float width = 1.0f / projection[0][0];
    float halfHeight = 1.0f / projection[1][1];

    w *= max(width, halfHeight);
  }

  mat4 modelView = view * modelTransform;
  mat4 modelViewProjection = projection * modelView;

  fragmentPosition = vec3(modelViewProjection * vec4(vCoord.xyz, 1.0));

  gl_Position = modelViewProjection * vec4(vCoord.xyz * w, 1.0);

  normalTransformed = mat3(transpose(inverse(modelView))) * vNormal;
}
