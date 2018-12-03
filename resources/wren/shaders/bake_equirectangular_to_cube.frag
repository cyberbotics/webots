#version 330 core
out vec4 fragColor;
in vec3 worldPosition;

uniform sampler2D inputTextures[1];

const vec2 invAtan = vec2(0.1591, 0.3183);
vec2 sampleSphericalMap(vec3 v) {
  vec2 uv = vec2(atan(v.z, v.x), asin(v.y));
  uv *= invAtan;
  uv += 0.5;
  return uv;
}

void main() {
  // make sure to normalize worldPosition
  vec2 uv = sampleSphericalMap(normalize(worldPosition));
  vec3 color = texture(inputTextures[0], uv).rgb;

  color = clamp(color, 0.0f, 25.0f);

  fragColor = vec4(color, 1.0);
}
