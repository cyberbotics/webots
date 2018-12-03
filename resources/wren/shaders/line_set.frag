#version 330

in vec3 color;

out vec4 fragColor;

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
}
