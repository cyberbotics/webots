#version 330 core

precision highp float;

uniform sampler2D inputTextures[2];
uniform vec2 viewportSize;
in vec2 texUv;
in vec4 texUvOffsets[2];

layout(location = 0) out vec4 result;

vec4 SMAANeighborhoodBlendingPS(vec2 texcoord, vec4 offset[2], sampler2D colorTex, sampler2D blendTex) {
  // Fetch the blending weights for current pixel:
  vec4 a;
  a.xz = texture(blendTex, texcoord).xz;
  a.y = texture(blendTex, offset[1].zw).g;
  a.w = texture(blendTex, offset[1].xy).a;
  // Is there any blending weight with a value greater than 0.0?
  if (dot(a, vec4(1.0, 1.0, 1.0, 1.0)) < 1e-5)
    return texture(colorTex, texcoord, 0.0);
  // return vec4(vec3(0.0), 1.0);
  else {  // Up to 4 lines can be crossing a pixel (one through each edge). We
    // favor blending by choosing the line with the maximum weight for each
    // direction:
    vec2 offset;
    offset.x = a.a > a.b ? a.a : -a.b;
    // left vs. right
    offset.y = a.g > a.r ? -a.g : a.r;
    // WebGL port note: Changed signs

    // Then we go in the direction that has the maximum weight:
    if (abs(offset.x) > abs(offset.y)) {
      // horizontal vs. vertical
      offset.y = 0.0;
    } else {
      offset.x = 0.0;
    }
    // Fetch the opposite color and lerp by hand:
    vec4 C = texture(colorTex, texcoord);
    texcoord += sign(offset) / viewportSize;
    vec4 Cop = texture(colorTex, texcoord);
    float s = abs(offset.x) > abs(offset.y) ? abs(offset.x) : abs(offset.y);
    // WebGL port note: Added gamma correction
    C.xyz = pow(C.xyz, vec3(2.2));
    Cop.xyz = pow(Cop.xyz, vec3(2.2));
    vec4 mixed = mix(C, Cop, s);
    mixed.xyz = pow(mixed.xyz, vec3(1.0 / 2.2));
    return mixed;
    return 0.5 * mixed;
  }
}
void main() {
  result = SMAANeighborhoodBlendingPS(texUv, texUvOffsets, inputTextures[0], inputTextures[1]);
  // result = texture(inputTextures[1], texUv);
}
