#version 330 core

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

out vec2 texUv;
out vec2 seed1;
out vec2 seed2;
out vec2 seed3;

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

  float rand1 = rand(vec2(time, time));
  seed1 = vec2(rand1, rand(vec2(rand1, rand1)));
  float rand2 = rand(vec2(seed1.y, seed1.y));
  seed2 = vec2(rand2, rand(vec2(rand2, rand2)));
  float rand3 = rand(vec2(seed2.y, seed2.y));
  seed3 = vec2(rand3, rand(vec2(rand3, rand3)));

  gl_Position = vec4(vec2(-1.0) + 2.0 * vCoord.xy, 0.0, 1.0);
}
