#version 330

const int sceneTextureIndex = 0;
const int lastResultTextureIndex = 1;

in vec2 texUv;

layout(location = 0) out vec4 result;

uniform float intensity;
uniform float firstRender;

uniform sampler2D inputTextures[2];

void main() {
  vec4 sceneColor = texture(inputTextures[sceneTextureIndex], texUv);
  vec4 lastColor = texture(inputTextures[lastResultTextureIndex], texUv);

  if (firstRender == 1.0)
    result = sceneColor;
  else
    result = mix(sceneColor, lastColor, intensity);

  result = vec4(result.rgb, 1.0);
}
