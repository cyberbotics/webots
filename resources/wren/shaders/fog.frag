#version 330 core

precision highp float;

in vec3 fragmentPosition;

out vec4 fragColor;

uniform sampler2D inputTextures[1];

layout(std140) uniform Fog {
  vec2 mode;    // x: fogType, y: depthType
  vec4 params;  // x: density, y: density2, z: fog end, w: inverse range
  vec4 color;
}
fog;

void main() {
  if (fog.mode.x > 0.0) {
    float fogDensity = fog.params.x;
    float fogDensity2 = fog.params.y;
    float fogEnd = fog.params.z;
    float fogInverseScale = fog.params.w;

    float z;
    if (fog.mode.y == 1.0)
      // Real point distance (length)
      z = length(fragmentPosition.xyz);
    else
      // z-depth (plane distance)
      z = -fragmentPosition.z;

    float fogFactor;
    if (fog.mode.x == 1.0)  // FOG_EXP
      fogFactor = exp2(-fogDensity * z);
    else if (fog.mode.x == 2.0)  // FOG_EXP2
      fogFactor = exp2(-fogDensity2 * z * z);
    else  // FOG_LINEAR
      fogFactor = (fogEnd - z) * fogInverseScale;
    fogFactor = clamp(fogFactor, 0.0, 1.0);

    fragColor = vec4(fog.color.xyz, pow(1.0 - fogFactor, 2.2));
  }
}
