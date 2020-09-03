#version 330

out vec4 fragColor;

// Material parameters for this renderable
layout (std140) uniform Material {
  vec4 ambient;
  vec4 diffuse;
  vec4 specularAndExponent;
  vec4 emissiveAndOpacity;
  vec4 textureFlags; // x, y, z, w: materialTexture[0]..[3]
} material;

void main() {
  fragColor = material.diffuse;
}
