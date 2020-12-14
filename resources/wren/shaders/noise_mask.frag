#version 330 core

precision highp float;

const int lastResultTextureIndex = 0;
const int noiseTextureIndex = 1;

in vec2 texUv;

layout(location = 0) out vec4 result;

uniform vec2 textureOffset;
uniform vec2 textureFactor;

uniform sampler2D inputTextures[2];

void main() {
  vec2 noiseUv = (texUv + textureOffset) * textureFactor;
  vec4 noiseColor = texture(inputTextures[noiseTextureIndex], noiseUv);
  vec4 lastColor = texture(inputTextures[lastResultTextureIndex], texUv);

  result = lastColor;
  result.rgb = mix(lastColor.rgb, noiseColor.rgb, noiseColor.a);
}
