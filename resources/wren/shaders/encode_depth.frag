#version 330 core

precision highp float;

const float FLT_MAX = intBitsToFloat(0x7F800000);

layout(location = 0) out float floatDepth;
layout(location = 1) out float outputDepth;

uniform float minRange;
uniform float maxRange;

void main() {
  if (gl_FragCoord.w == 0.0)
    floatDepth = FLT_MAX;
  else
    floatDepth = gl_FragCoord.z / gl_FragCoord.w;

  if (floatDepth < minRange)
    floatDepth = FLT_MAX;
  if (floatDepth >= maxRange)
    floatDepth = FLT_MAX;

  outputDepth = floatDepth;
}
