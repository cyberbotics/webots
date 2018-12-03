#version 330

in vec2 texUv;

layout(location = 0) out vec4 floatResult;
layout(location = 1) out vec4 colorResult;

uniform sampler2D inputTextures[1];

void main() {
  floatResult = vec4(texture(inputTextures[0], texUv).xyz, 1.0);
  colorResult = floatResult;
}
