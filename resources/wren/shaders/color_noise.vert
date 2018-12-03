#version 330

layout(location = 0) in vec3 vCoord;
layout(location = 2) in vec2 vTexCoord;

out vec2 texUv;
out vec2 seed1;
out vec2 seed2;
out vec2 seed3;

uniform float time;

// http://stackoverflow.com/questions/4200224/random-noise-functions-for-glsl
// http://www.wolframalpha.com/input/?i=plot%28%20mod%28%20sin%28x%2a12.9898%20%2b%20y%2a78.233%29%20%2a%2043758.5453,1%29x=0..2,%20y=0..2%29
float rand(vec2 seed) {
  return fract(sin(dot(seed.xy, vec2(12.9898, 78.233))) * 43758.5453);
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
