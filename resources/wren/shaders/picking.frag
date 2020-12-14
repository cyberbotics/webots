#version 330 core

precision highp float;

out vec4 fragColor;

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
  vec4 color;
  color.bg = material.ambient.xy;
  color.ra = material.diffuse.xy;
  fragColor = color;
}
