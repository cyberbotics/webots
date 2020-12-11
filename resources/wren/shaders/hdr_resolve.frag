#version 330 core

precision highp float;

in vec2 texUv;

layout(location = 0) out vec4 result;

uniform sampler2D inputTextures[1];

uniform float exposure;

void main() {
  const float gamma = 2.2;
  vec3 hdrColor = texture(inputTextures[0], texUv).rgb;

  // Exposure tone mapping
  vec3 mapped = vec3(1.0) - exp(-hdrColor * exposure);
  // Gamma correction
  mapped = pow(mapped, vec3(1.0 / gamma));

  result = vec4(mapped, 1.0);
}
