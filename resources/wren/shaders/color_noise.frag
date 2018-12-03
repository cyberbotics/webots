#version 330

in vec2 texUv;
in vec2 seed1;
in vec2 seed2;
in vec2 seed3;

out vec4 fragColor;

uniform float intensity;

uniform sampler2D inputTextures[1];

// http://stackoverflow.com/questions/4200224/random-noise-functions-for-glsl
// http://www.wolframalpha.com/input/?i=plot%28%20mod%28%20sin%28x%2a12.9898%20%2b%20y%2a78.233%29%20%2a%2043758.5453,1%29x=0..2,%20y=0..2%29
float rand(vec2 seed) {
  return fract(sin(dot(seed.xy, vec2(12.9898, 78.233))) * 43758.5453);
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
