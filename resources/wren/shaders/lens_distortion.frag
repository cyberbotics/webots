#version 330

in vec2 texUv;

layout(location = 0) out vec4 fragColor;

uniform vec2 center;
uniform vec2 radialDistortionCoeffs;
uniform vec2 tangentialDistortionCoeffs;

uniform sampler2D inputTextures[1];

void main() {
  vec2 distortedUv;

  // based on Brown's distortion model (https://en.wikipedia.org/wiki/Distortion_%28optics%29#Software_correction)
  float r = pow(texUv.x - center.x, 2.0) + pow(texUv.y - center.y, 2.0);
  distortedUv.x = texUv.x * (1.0 + radialDistortionCoeffs.x * r + radialDistortionCoeffs.y * r * r) +
                  tangentialDistortionCoeffs.y * (r + 2.0 * texUv.x * texUv.x) +
                  2.0 * tangentialDistortionCoeffs.x * texUv.x * texUv.y;
  distortedUv.y = texUv.y * (1.0 + radialDistortionCoeffs.x * r + radialDistortionCoeffs.y * r * r) +
                  tangentialDistortionCoeffs.x * (r + 2.0 * texUv.y * texUv.y) +
                  2.0 * tangentialDistortionCoeffs.y * texUv.x * texUv.y;

  fragColor = texture(inputTextures[0], distortedUv);
}
