#version 330 core

precision highp float;

uniform sampler2D inputTextures[1];

uniform float threshold;

in vec2 texUv;
out vec4 fragColor;

float luma(vec3 color) {
  return dot(color, vec3(0.299, 0.587, 0.114));
}

void main() {
  vec4 color = texture(inputTextures[0], texUv);
  vec2 textureSize = vec2(textureSize(inputTextures[0], 0));

  float totalLuma = luma(color.rgb);

  totalLuma += luma(texture(inputTextures[0], texUv + vec2(0, 1) / textureSize).rgb);
  totalLuma += luma(texture(inputTextures[0], texUv + vec2(0, -1) / textureSize).rgb);
  totalLuma += luma(texture(inputTextures[0], texUv + vec2(-1, 0) / textureSize).rgb);
  totalLuma += luma(texture(inputTextures[0], texUv + vec2(1, 0) / textureSize).rgb);

  totalLuma += luma(texture(inputTextures[0], texUv + vec2(1, 1) / textureSize).rgb);
  totalLuma += luma(texture(inputTextures[0], texUv + vec2(1, -1) / textureSize).rgb);
  totalLuma += luma(texture(inputTextures[0], texUv + vec2(-1, 1) / textureSize).rgb);
  totalLuma += luma(texture(inputTextures[0], texUv + vec2(-1, -1) / textureSize).rgb);

  totalLuma /= 9.0;

  if (isnan(totalLuma) || totalLuma < threshold)
    fragColor = vec4(vec3(0.0), 1.0);
  else {
    fragColor.r = min(color.r, 4000.0f);
    fragColor.g = min(color.g, 4000.0f);
    fragColor.b = min(color.b, 4000.0f);
    fragColor.a = 1.0;
  }
}
