#version 330

in vec3 color;

out vec4 fragColor;

uniform bool colorPerVertex;
uniform float pointSize;

// Material parameters for this renderable
layout(std140) uniform PhongMaterial {
  vec4 ambient;
  vec4 diffuse;
  vec4 specularAndExponent;
  vec4 emissiveAndOpacity;
  vec4 textureFlags;  // x, y, z, w: materialTexture[0]..[3]
}
material;

// Circle with antialiasing drawing
// (https://www.desultoryquest.com/blog/drawing-anti-aliased-circular-points-using-opengl-slash-webgl/)
void main() {
  vec2 cxy = 2.0 * gl_PointCoord - 1.0;
  float r = dot(cxy, cxy);
  float delta = fwidth(r);
  float alpha = 1.0 - smoothstep(1.0 - delta, 1.0 + delta, r);
  alpha *= material.emissiveAndOpacity.w;

  // Use vertex color if enabled
  if (colorPerVertex)
    fragColor = vec4(color, alpha);
  else
    fragColor = vec4(material.emissiveAndOpacity.xyz, alpha);
}
