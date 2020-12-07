#version 300 es

precision highp float;

const int mainTextureIndex = 0;
const int penTextureIndex = 1;
const int backgroundTextureIndex = 2;

in vec2 texUv;
in vec2 penTexUv;

out vec4 fragColor;

uniform sampler2D inputTextures[3];

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
  fragColor = vec4(1.0);

  if (material.textureFlags.x > 0.0 || material.textureFlags.z > 0.0)
    fragColor.w = 0.0;

  // Background texture
  if (material.textureFlags.z > 0.0)
    fragColor = texture(inputTextures[backgroundTextureIndex], texUv);

  // Main texture
  if (material.textureFlags.x > 0.0) {
    vec4 mainColor = texture(inputTextures[mainTextureIndex], texUv);
    fragColor = vec4(mix(fragColor.xyz, mainColor.xyz, mainColor.w), fragColor.w + mainColor.w);
  }

  // Pen texture
  if (material.textureFlags.y > 0.0) {
    vec4 penColor = texture(inputTextures[penTextureIndex], penTexUv);
    fragColor = vec4(mix(fragColor.xyz, penColor.xyz, penColor.w), fragColor.w);
  }
}
