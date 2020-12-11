#version 330 core

precision highp float;

layout(location = 0) out vec4 result;

void main() {
  // color mask should be set to false for all channels so this value never gets written
  result = vec4(0.8, 0.4, 0.0, 0.2);
}
