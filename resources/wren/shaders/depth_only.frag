#version 330

in vec3 fragmentPosition;

out float depth;

void main() {
  depth = fragmentPosition.z;
}
