#version 330 core

precision highp float;
#define GAMMA 2.0

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec3 fragNormal;

in vec3 texUv;

uniform samplerCube cubeTextures[1];

void main() {
  // invert z components of sample vector due to VRML default camera orientation looking towards -z
  fragNormal = vec3(0.0);
  fragColor = textureLod(cubeTextures[0], vec3(texUv.xy, -texUv.z), 0.0);
  fragColor.rgb = pow(fragColor.rgb, vec3(GAMMA));
}
