#version 330 core

precision highp float;

in vec2 texUv;
in float depth;

out vec4 fragColor;

uniform sampler2D inputTextures[1];

const int mainTextureIndex = 0;

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
  vec4 texColor = vec4(1.0f);
  if (material.textureFlags.x > 0.0f)
    texColor = texture(inputTextures[mainTextureIndex], texUv);

  fragColor = texColor * material.ambient;
}
