#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec2 vTexCoord;
layout(location = 4) in vec2 vUnwrappedTexCoord;

out vec3 normalTransformed;
out vec2 texUv;
out vec2 penTexUv;

uniform mat4 modelTransform;
uniform mat4 textureTransform;

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
  gl_Position = cameraTransforms.infiniteProjection * cameraTransforms.view * modelTransform * vec4(vCoord, 1.0);

  mat4 modelView = cameraTransforms.view * modelTransform;
  normalTransformed = mat3(transpose(inverse(modelView))) * vNormal;

  texUv = vec2(textureTransform * vec4(vTexCoord, 0.0, 1.0));
  penTexUv = vUnwrappedTexCoord;
}
