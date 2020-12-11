#version 330 core

precision highp float;

in vec2 texUv;

out vec4 fragColor;

const int mainTextureIndex = 0;

uniform int channelCount;

uniform sampler2D inputTextures[1];

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
  // The fragment color is either the material ambient color or the texture color
  if (material.textureFlags.x > 0.0f) {
    fragColor = texture(inputTextures[mainTextureIndex], texUv);
    if (channelCount == 1)
      fragColor.y = fragColor.z = fragColor.x;
  } else
    fragColor = material.ambient;

  // Set transparency
  fragColor.w = material.emissiveAndOpacity.w;
}
