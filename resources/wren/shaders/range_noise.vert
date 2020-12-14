#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

out vec2 texUv;
out vec2 seed;

uniform float time;

// http://byteblacksmith.com/improvements-to-the-canonical-one-liner-glsl-rand-for-opengl-es-2-0/
// https://www.wolframalpha.com/input/?i=plot%28+mod%28+sin%28mod%28x*12.9898+%2B+y*78.233%2C+3.14%29%29+*+43758.5453%2C1%29x%3D0..2%2C+y%3D0..2%29
highp float rand(vec2 seed) {
  highp float a = 12.9898;
  highp float b = 78.233;
  highp float c = 43758.5453;
  highp float dt = dot(seed.xy, vec2(a, b));
  highp float sn = mod(dt, 3.14);
  return fract(sin(sn) * c);
}

void main() {
  texUv = vTexCoord;

  float random = rand(vec2(time, time));
  seed = vec2(random, rand(vec2(random, random)));

  gl_Position = vec4(vec2(-1.0) + 2.0 * vCoord.xy, 0.0, 1.0);
}
