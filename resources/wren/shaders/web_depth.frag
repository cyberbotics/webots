#version 330 core

precision highp float;
in vec3 fragmentPosition;

out vec4 depth;

void main() {
  depth = vec4(gl_FragCoord.z/gl_FragCoord.w, gl_FragCoord.z, gl_FragCoord.w, fragmentPosition.z);
}
