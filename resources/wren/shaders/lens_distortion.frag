#version 330 core

precision highp float;

in vec2 texUv;

layout(location = 0) out vec4 fragColor;

uniform vec2 center;
uniform vec2 radialDistortionCoeffs;
uniform vec2 tangentialDistortionCoeffs;

uniform sampler2D inputTextures[1];

void main() {
  // based on "Realistic Lens Distortion Rendering", Lambers M., Sommerhoff H., Kolb A. (2018)
  // implementing an inverted Brown's distortion model
  // (https://en.wikipedia.org/wiki/Distortion_%28optics%29#Software_correction)
  vec2 distortedUv, relativeUv;
  relativeUv.x = texUv.x - center.x;
  relativeUv.y = texUv.y - center.y;
  float r = relativeUv.x * relativeUv.x + relativeUv.y * relativeUv.y;
  float d1 = radialDistortionCoeffs.x * r + radialDistortionCoeffs.y * r * r;
  float d2 = 1 / (4 * radialDistortionCoeffs.x * r + 6 * radialDistortionCoeffs.y * r * r +
                  8 * tangentialDistortionCoeffs.x * relativeUv.y + 8 * tangentialDistortionCoeffs.y * relativeUv.x + 1);
  distortedUv.x = texUv.x - d2 * (d1 * relativeUv.x + 2 * tangentialDistortionCoeffs.x * relativeUv.x * relativeUv.y +
                                  tangentialDistortionCoeffs.y * (r + 2 * relativeUv.x * relativeUv.x));
  distortedUv.y = texUv.y - d2 * (d1 * relativeUv.y + 2 * tangentialDistortionCoeffs.y * relativeUv.x * relativeUv.y +
                                  tangentialDistortionCoeffs.x * (r + 2 * relativeUv.y * relativeUv.y));

  if (distortedUv.x < 0.0 || distortedUv.x > 1.0 || distortedUv.y < 0.0 || distortedUv.y > 1.0)
    fragColor = vec4(0.0, 0.0, 0.0, 1.0);
  else
    fragColor = texture(inputTextures[0], distortedUv);
}
