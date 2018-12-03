#version 330

in vec2 texUv;

out vec4 fragColor;

const int mainTextureIndex = 0;

uniform sampler2D inputTextures[1];

void main() {
  fragColor = texture(inputTextures[mainTextureIndex], texUv);
}
