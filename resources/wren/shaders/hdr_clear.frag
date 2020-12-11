#version 330 core

precision highp float;

in vec2 texUv;

layout(location = 0) out vec4 result;
layout(location = 1) out vec4 normal;

// Material parameters for this renderable
layout(std140) uniform PhongMaterial {
  vec4 ambient;
  vec4 diffuse;
  vec4 specularAndExponent;
  vec4 emissiveAndOpacity;
  vec4 textureFlags;  // x, y, z, w: materialTexture[0]..[3]
}
material;

void main() {
  result = material.diffuse;
  normal = vec4(0.0);
}
