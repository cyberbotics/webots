#version 330

layout(location = 0) out float floatDepth;
layout(location = 1) out float outputDepth;

uniform float minRange;
uniform float maxRange;

void main() {
  floatDepth = gl_FragCoord.z / gl_FragCoord.w;
  floatDepth = clamp(floatDepth, 0.0, maxRange);
  if (floatDepth < minRange)
    floatDepth = maxRange;

  outputDepth = floatDepth;
}
