#version 330 core

precision highp float;

in vec3 fragmentPosition;

out float depth;

void main() {
  depth = fragmentPosition.z;
}
