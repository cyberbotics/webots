#version 330 core

precision highp float;

in vec4 fragmentPosition;

out vec4 depth;

void main() {
  depth = vec4(abs(fragmentPosition.b)/10.0, 0.0, 0.0, 1.0);
}
