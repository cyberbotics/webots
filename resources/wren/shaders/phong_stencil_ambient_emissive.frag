#version 330 core

precision highp float;

// These constants must be kept in sync with the values in Constants.hpp
const int maxDirectionalLights = 48;
const int maxPointLights = 48;
const int maxSpotLights = 48;

const int mainTextureIndex = 0;
const int penTextureIndex = 1;
const int backgroundTextureIndex = 2;

in vec2 texUv;
in vec2 penTexUv;
in vec3 fragmentNormal;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec4 fragNormal;

uniform sampler2D inputTextures[3];
uniform bool reverseNormals;

struct DirectionalLight {
  vec4 colorAndIntensity;
  vec4 direction;
};

struct PointLight {
  vec4 colorAndIntensity;
  vec4 position;
  vec4 attenuationAndRadius;
};

struct SpotLight {
  vec4 colorAndIntensity;
  vec4 position;
  vec4 direction;
  vec4 attenuationAndRadius;
  vec4 spotParams;  // x: innerCutOffAngle, y: outerCutOffAngle, z: inverse range, w: unused
};

// List of active lights and ambient intensity for this frame
layout(std140) uniform Lights {
  DirectionalLight directionalLights[maxDirectionalLights];
  PointLight pointLights[maxPointLights];
  SpotLight spotLights[maxSpotLights];
  vec4 ambientLight;
  ivec3 mLightCount;
}
lights;

// Material parameters for this renderable
layout(std140) uniform PhongMaterial {
  vec4 ambient;
  vec4 diffuse;
  vec4 specularAndExponent;
  vec4 emissiveAndOpacity;
  vec4 textureFlags;  // x, y, z, w: materialTexture[0]..[3]
}
material;

vec4 SRGBtoLINEAR(vec4 srgbIn) {
  vec3 bLess = step(vec3(0.04045), srgbIn.xyz);
  vec3 linOut = mix(srgbIn.xyz / vec3(12.92), pow((srgbIn.xyz + vec3(0.055)) / vec3(1.055), vec3(2.4)), bLess);
  return vec4(linOut, srgbIn.w);
}

void main() {
  fragNormal = vec4(normalize(reverseNormals ? -fragmentNormal : fragmentNormal), 1.0) * 0.5 + 0.5;

  vec3 ambientColor = vec3(lights.ambientLight) * material.ambient.xyz;

  vec4 texColor = vec4(1.0);
  if (material.textureFlags.x > 0.0 || material.textureFlags.z > 0.0)
    texColor.w = 0.0;

  // Background texture
  if (material.textureFlags.z > 0.0) {
    texColor.rgb = texture(inputTextures[backgroundTextureIndex], texUv).rgb;
    texColor.w = 1.0;
  }

  // Main texture
  if (material.textureFlags.x > 0.0) {
    vec4 mainColor = SRGBtoLINEAR(texture(inputTextures[mainTextureIndex], texUv));
    texColor = vec4(mix(texColor.xyz, mainColor.xyz, mainColor.w), clamp(texColor.w + mainColor.w, 0.0, 1.0));
  }

  // Pen texture
  if (material.textureFlags.y > 0.0) {
    vec4 penColor = texture(inputTextures[penTextureIndex], penTexUv);
    texColor = vec4(mix(texColor.xyz, penColor.xyz, penColor.w), texColor.w);
  }

  fragColor = texColor * vec4(material.emissiveAndOpacity.xyz + ambientColor, material.emissiveAndOpacity.w);
}
