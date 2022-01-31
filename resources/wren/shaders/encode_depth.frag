#version 330 core

precision highp float;

const float FLT_MAX = 1.0 / 0.0;

layout(location = 0) out float floatDepth;
layout(location = 1) out float outputDepth;

uniform float minRange;
uniform float maxRange;

void main() {
  floatDepth = gl_FragCoord.z / gl_FragCoord.w;
  if (floatDepth < minRange)
    floatDepth = FLT_MAX;
  if (floatDepth >= maxRange)
    floatDepth = FLT_MAX;

  outputDepth = floatDepth;
}
