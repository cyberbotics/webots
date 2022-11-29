#version 330 core

precision highp float;

const int sceneTextureIndex = 0;
const int lastResultTextureIndex = 1;

const float FLT_MAX = intBitsToFloat(0x7F800000);

in vec2 texUv;

layout(location = 0) out vec4 result;

uniform float intensity;
uniform bool firstRender;

uniform sampler2D inputTextures[2];

void main() {
  vec4 sceneColor = texture(inputTextures[sceneTextureIndex], texUv);
  vec4 lastColor = texture(inputTextures[lastResultTextureIndex], texUv);

  // Fix for Mesa software renderer
  if (isnan(abs(sceneColor.r)) || isinf(sceneColor.r))
    sceneColor = vec4(FLT_MAX, FLT_MAX, FLT_MAX, 1.0);
  if (isnan(abs(lastColor.r)) || isinf(lastColor.r))
    lastColor = vec4(FLT_MAX, FLT_MAX, FLT_MAX, 1.0);

  if (firstRender || sceneColor.x == FLT_MAX || lastColor.x == FLT_MAX)
    result = sceneColor;
  else
    result = mix(sceneColor, lastColor, intensity);

  result = vec4(result.rgb, 1.0);
}
