#version 330 core

precision highp float;
in vec4 fragmentPosition;

out float depth;

void main() {
  float zd = fragmentPosition.z / fragmentPosition.w;
  depth = (gl_DepthRange.far - gl_DepthRange.near) * zd + gl_DepthRange.near;
}
