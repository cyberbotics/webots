#version 330 core

precision highp float;

in vec3 color;
in vec3 normalTransformed;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec4 fragNormal;

uniform bool colorPerVertex;

// Material parameters for this renderable
layout(std140) uniform PhongMaterial {
  vec4 ambient;
  vec4 diffuse;
  vec4 specularAndExponent;
  vec4 emissiveAndOpacity;
  vec4 textureFlags;  // x, y, z, w: materialTexture[0]..[3]
}
material;

void main() {
  // Use vertex color if enabled
  if (colorPerVertex)
    fragColor = vec4(color, material.emissiveAndOpacity.w);
  else
    fragColor = material.emissiveAndOpacity;

  vec3 fragmentNormal = normalize(normalTransformed);
  fragNormal = vec4(fragmentNormal, 1.0) * 0.5 + 0.5;
}
