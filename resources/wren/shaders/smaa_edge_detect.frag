#version 330 core

precision highp float;

#define SMAA_THRESHOLD 0.1

layout(location = 0) out vec4 result;

uniform sampler2D inputTextures[1];

in vec2 texUv;
in vec4 texUvOffsets[3];

vec4 SMAAColorEdgeDetection(vec2 texcoord, vec4 offset[3], sampler2D colorTex) {
  vec2 threshold = vec2(SMAA_THRESHOLD, SMAA_THRESHOLD);

  // Calculate color deltas:
  vec4 delta;
  vec3 C = texture(colorTex, texcoord).rgb;

  vec3 Cleft = texture(colorTex, offset[0].xy).rgb;
  vec3 t = abs(C - Cleft);
  delta.x = max(max(t.r, t.g), t.b);

  vec3 Ctop = texture(colorTex, offset[0].zw).rgb;
  t = abs(C - Ctop);
  delta.y = max(max(t.r, t.g), t.b);

  // We do the usual threshold:
  vec2 edges = step(threshold, delta.xy);

  // Then discard if there is no edge:
  if (dot(edges, vec2(1.0, 1.0)) == 0.0)
    discard;

  // Calculate right and bottom deltas:
  vec3 Cright = texture(colorTex, offset[1].xy).rgb;
  t = abs(C - Cright);
  delta.z = max(max(t.r, t.g), t.b);

  vec3 Cbottom = texture(colorTex, offset[1].zw).rgb;
  t = abs(C - Cbottom);
  delta.w = max(max(t.r, t.g), t.b);

  // Calculate the maximum delta in the direct neighborhood:
  float maxDelta = max(max(max(delta.x, delta.y), delta.z), delta.w);

  // Calculate left-left and top-top deltas:
  vec3 Cleftleft = texture(colorTex, offset[2].xy).rgb;
  t = abs(C - Cleftleft);
  delta.z = max(max(t.r, t.g), t.b);

  vec3 Ctoptop = texture(colorTex, offset[2].zw).rgb;
  t = abs(C - Ctoptop);
  delta.w = max(max(t.r, t.g), t.b);

  // Calculate the final maximum delta:
  maxDelta = max(max(maxDelta, delta.z), delta.w);

  // Local contrast adaptation in action:
  edges.xy *= step(0.5 * maxDelta, delta.xy);

  return vec4(edges, 0.0, 1.0);
}

void main() {
  result = SMAAColorEdgeDetection(texUv, texUvOffsets, inputTextures[0]);
}
