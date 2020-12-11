#version 330 core

precision highp float;

in vec2 texUv;
in vec2 seed1;
in vec2 seed2;
in vec2 seed3;

out vec4 fragColor;

uniform float intensity;

uniform sampler2D inputTextures[1];

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

// Box-Muller method
float gaussian(vec2 uv, vec2 seed) {
  const float PI = 3.141592653589793238462643383279;

  // generate 2 independents random numbers
  float U = rand(uv + vec2(seed.x, seed.x));
  float V = rand(uv + vec2(seed.y, seed.y));

  // make sure U is not close to 0 because log(0) is undefined
  if (U <= 0.0000001)
    U = 0.0000001;

  // generate a gaussian value with mean 0 and standard deviation 1
  float r = sqrt(-2.0 * log(U)) * cos(2.0 * PI * V);
  return r;
}

void main() {
  fragColor = texture(inputTextures[0], texUv) +
              intensity * vec4(gaussian(texUv, seed1), gaussian(texUv, seed2), gaussian(texUv, seed3), 0.0);
  fragColor = clamp(fragColor, 0.0, 1.0);
}
