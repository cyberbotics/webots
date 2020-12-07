#version 300 es

precision highp float;

// https://stackoverflow.com/a/33961589/1983050
// From version 410 we will be able to get infinity by `1.0 / 0.0`
#define FLT_MAX 3.402823466e+38

layout(location = 0) out float floatDepth;
layout(location = 1) out float outputDepth;

uniform float minRange;
uniform float maxRange;

void main() {
  floatDepth = gl_FragCoord.z / gl_FragCoord.w;
  if (floatDepth < minRange)
    floatDepth = FLT_MAX;
  if (floatDepth > maxRange)
    floatDepth = FLT_MAX;

  outputDepth = floatDepth;
}
