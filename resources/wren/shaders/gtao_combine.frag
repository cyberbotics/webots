#version 330 core

precision highp float;

// This shader does the final blend for GTAO based on
// https://github.com/asylum2010/Asylum_Tutorials/blob/master/ShaderTutors/media/shadersGL/gtaocombine.frag

uniform sampler2D inputTextures[3];

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec4 fragAo;
layout(location = 2) out float fragDepth;

in vec2 texUv;

vec3 MultiBounce(float ao, vec3 albedo) {
  vec3 x = vec3(ao);

  vec3 a = 2.0404 * albedo - vec3(0.3324);
  vec3 b = -4.7951 * albedo + vec3(0.6417);
  vec3 c = 2.7552 * albedo + vec3(0.6903);

  return max(x, ((x * a + b) * x + c) * x);
}

void main() {
  vec4 base = textureLod(inputTextures[0], texUv, 0.0);
  vec2 aoOut = textureLod(inputTextures[1], texUv, 0.0).rg;
  float ao = aoOut.r;

  fragDepth = textureLod(inputTextures[2], texUv, 0.0).x;

  if (ao >= 1.0 || fragDepth == 1.0)
    fragColor = base;
  else {
    fragColor.rgb = base.rgb * MultiBounce(ao, base.rgb);
    // fragColor.rgb = vec3(ao);
    fragColor.a = 1.0;
  }
  fragAo = vec4(aoOut.rg, 0.0, 0.0);
}
