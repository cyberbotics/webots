#version 330 core

precision highp float;

in vec2 fragmentPosition;

out vec4 depth;

void main() {
  depth = vec4(gl_FragCoord.z, 0.0, 0.0, 1.0);
}
