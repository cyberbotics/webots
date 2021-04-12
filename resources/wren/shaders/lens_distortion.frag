#version 330 core

precision highp float;

in vec2 texUv;

layout(location = 0) out vec4 fragColor;

uniform vec2 center;
uniform vec2 radialDistortionCoeffs;
uniform vec2 tangentialDistortionCoeffs;

uniform sampler2D inputTextures[1];

void main() {
  vec2 distortedUv, relativeUv;

  // based on Brown's distortion model (https://en.wikipedia.org/wiki/Distortion_%28optics%29#Software_correction)
  float r = pow(texUv.x - center.x, 2.0) + pow(texUv.y - center.y, 2.0);
  relativeUv = texUv - center;
  distortedUv.x = texUv.x + relativeUv.x * (radialDistortionCoeffs.x * r + radialDistortionCoeffs.y * r * r) +
                  tangentialDistortionCoeffs.y * (r + 2.0 * relativeUv.x * relativeUv.x) +
                  2.0 * tangentialDistortionCoeffs.x * relativeUv.x * relativeUv.y;
  distortedUv.y = texUv.y + relativeUv.y * (radialDistortionCoeffs.x * r + radialDistortionCoeffs.y * r * r) +
                  tangentialDistortionCoeffs.x * (r + 2.0 * relativeUv.y * relativeUv.y) +
                  2.0 * tangentialDistortionCoeffs.y * relativeUv.x * relativeUv.y;

  fragColor = texture(inputTextures[0], distortedUv);
}
