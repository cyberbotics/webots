#version 330 core

precision highp float;

uniform sampler2D inputTextures[7];

in vec2 texUv;
out vec4 fragColor;

void main() {
  vec4 color = texture(inputTextures[0], texUv);
  vec4 fullRes = texture(inputTextures[1], texUv);
  vec4 halfRes = texture(inputTextures[2], texUv);
  vec4 quarterRes = texture(inputTextures[3], texUv);
  vec4 eigthRes = texture(inputTextures[4], texUv);
  vec4 sixteenthRes = texture(inputTextures[5], texUv);
  vec4 thirtySecondRes = texture(inputTextures[6], texUv);

  fragColor = vec4(color.rgb + 0.1 * (0.1 * fullRes.rgb + 0.2 * halfRes.rgb + 0.4 * quarterRes.rgb + 0.8 * eigthRes.rgb +
                                      1.6 * sixteenthRes.rgb + 3.2 * thirtySecondRes.rgb),
                   1.0);
}
