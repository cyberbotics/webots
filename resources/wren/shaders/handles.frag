#version 330 core

precision highp float;

in vec3 fragmentPosition;
in vec3 normalTransformed;

out vec4 fragColor;

// Material parameters for this renderable
layout(std140) uniform PhongMaterial {
  vec4 ambient;
  vec4 diffuse;
  vec4 specularAndExponent;
  vec4 emissiveAndOpacity;
  vec4 textureFlags;  // x, y, z, w: materialTexture[0]..[3]
}
material;

float specularReflection(in vec3 lightDirection, in vec3 normal, in vec3 viewDirection) {
  vec3 halfwayDirection = normalize(lightDirection + viewDirection);
  return pow(max(dot(normal, halfwayDirection), 0.0), material.specularAndExponent.w);
}

void main() {
  vec3 ambientTotal = vec3(1.0);
  vec3 diffuseTotal = vec3(0.0);
  vec3 specularTotal = vec3(0.0);

  vec3 fragmentNormal = normalize(normalTransformed);
  vec3 viewDirection = normalize(-fragmentPosition);

  vec3 directionToLight = normalize(vec3(1.0));
  vec4 colorAndIntensity = vec4(1.0);

  float lambert = dot(directionToLight, fragmentNormal);

  if (lambert > 0.0) {
    float diffuse = lambert * colorAndIntensity.w;
    diffuseTotal += diffuse * colorAndIntensity.xyz;

    float specular = specularReflection(directionToLight, fragmentNormal, viewDirection) * colorAndIntensity.w;
    specularTotal += specular * colorAndIntensity.xyz;
  }

  ambientTotal *= material.ambient.xyz;
  diffuseTotal *= material.diffuse.xyz;
  specularTotal *= material.specularAndExponent.xyz;

  fragColor =
    vec4(clamp(clamp(material.emissiveAndOpacity.xyz + ambientTotal, 0.0, 1.0) + clamp(diffuseTotal + specularTotal, 0.0, 1.0),
               0.0, 1.0),
         material.emissiveAndOpacity.w);
}
