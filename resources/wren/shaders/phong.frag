#version 330 core

precision highp float;

// These constants must be kept in sync with the values in Constants.hpp
const int maxDirectionalLights = 48;
const int maxPointLights = 48;
const int maxSpotLights = 48;

const int mainTextureIndex = 0;
const int penTextureIndex = 1;
const int backgroundTextureIndex = 2;

in vec3 fragmentPosition;
in vec3 normalTransformed;
in vec2 texUv;
in vec2 penTexUv;

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

layout(std140) uniform Fog {
  vec2 mode;    // x: fogType, y: depthType
  vec4 params;  // x: density, y: density2, z: fog end, w: inverse range
  vec4 color;
}
fog;

vec4 SRGBtoLINEAR(vec4 srgbIn) {
  vec3 bLess = step(vec3(0.04045), srgbIn.xyz);
  vec3 linOut = mix(srgbIn.xyz / vec3(12.92), pow((srgbIn.xyz + vec3(0.055)) / vec3(1.055), vec3(2.4)), bLess);
  return vec4(linOut, srgbIn.w);
}

float specularReflection(in vec3 lightDirection, in vec3 normal, in vec3 viewDirection) {
  vec3 halfwayDirection = normalize(lightDirection + viewDirection);

  return pow(max(dot(normal, halfwayDirection), 0.0), material.specularAndExponent.w);
}

void main() {
  vec3 ambientTotal = vec3(lights.ambientLight);
  vec3 diffuseTotal = vec3(0.0);
  vec3 specularTotal = vec3(0.0);

  vec3 fragmentNormal = normalize(reverseNormals ? -normalTransformed : normalTransformed);
  fragNormal = vec4(fragmentNormal, 1.0) * 0.5 + 0.5;
  vec3 viewDirection = normalize(-fragmentPosition);

  // Apply active directional lights
  for (int i = 0; i < lights.mLightCount.x; ++i) {
    DirectionalLight light = lights.directionalLights[i];

    // Direction in uniform buffer is already normalized and in view space
    vec3 directionToLight = -vec3(light.direction);

    float lambert = dot(directionToLight, fragmentNormal);

    if (lambert > 0.0) {
      float diffuse = lambert * light.colorAndIntensity.w;
      diffuseTotal += diffuse * light.colorAndIntensity.xyz;

      float specular = specularReflection(directionToLight, fragmentNormal, viewDirection) * light.colorAndIntensity.w;
      ;
      specularTotal += specular * light.colorAndIntensity.xyz;
    }
  }

  // Apply active point lights
  for (int i = 0; i < lights.mLightCount.y; ++i) {
    PointLight light = lights.pointLights[i];

    vec3 directionToLight = light.position.xyz - fragmentPosition;

    float distanceToLight = length(directionToLight);
    if (distanceToLight > light.attenuationAndRadius.w)
      continue;

    directionToLight = normalize(directionToLight);

    float lambert = dot(directionToLight, fragmentNormal);

    if (lambert > 0.0) {
      vec3 attenuation = light.attenuationAndRadius.xyz;
      float attenuationFactor = 1.0 / (attenuation.x + distanceToLight * (attenuation.y + attenuation.z * distanceToLight));

      float diffuse = lambert * light.colorAndIntensity.w;
      diffuseTotal += attenuationFactor * diffuse * light.colorAndIntensity.xyz;

      float specular = specularReflection(directionToLight, fragmentNormal, viewDirection) * light.colorAndIntensity.w;
      ;
      specularTotal += attenuationFactor * specular * light.colorAndIntensity.xyz;
    }
  }

  // Apply active spot lights
  for (int i = 0; i < lights.mLightCount.z; ++i) {
    SpotLight light = lights.spotLights[i];

    vec3 lightPosition = vec3(light.position);

    // Direction from surface to light position in eye space
    vec3 directionToLight = lightPosition - fragmentPosition;

    // Distance between surface and light position
    float distanceToLight = length(directionToLight);
    if (distanceToLight > light.attenuationAndRadius.w)
      continue;  // not illuminated
    directionToLight = normalize(directionToLight);

    // Inverted spotlight direction
    vec3 spotDirection = -light.direction.xyz;
    // Angle between spotlight direction and direction from light to point
    float spotAngle = acos(dot(directionToLight, spotDirection));
    // Inner angle
    float beamWidth = light.spotParams.x;
    // Outer angle
    float cutoffAngle = light.spotParams.y;

    float attenuationFactor = 1.0;
    // See if point on surface is inside cone of illumination defined by cutoff
    if (spotAngle >= cutoffAngle)    // outside
      continue;                      // light adds no contribution
    else if (spotAngle > beamWidth)  // falloff
      attenuationFactor *= (cutoffAngle - spotAngle) * light.spotParams.z;

    vec3 attenuation = light.attenuationAndRadius.xyz;
    attenuationFactor /= (attenuation.x + distanceToLight * (attenuation.y + attenuation.z * distanceToLight));

    float lambert = dot(directionToLight, fragmentNormal);
    if (lambert < 0.0)
      continue;

    float diffuse = lambert * light.colorAndIntensity.w;
    diffuseTotal += attenuationFactor * diffuse * light.colorAndIntensity.xyz;

    float specular = specularReflection(directionToLight, fragmentNormal, viewDirection) * light.colorAndIntensity.w;
    specularTotal += attenuationFactor * specular * light.colorAndIntensity.xyz;
  }

  vec4 texColor = vec4(1.0);
  ambientTotal *= material.ambient.xyz;
  if (material.textureFlags.x > 0.0 || material.textureFlags.z > 0.0) {
    texColor.w = 0.0;
    diffuseTotal *= material.diffuse.xyz;
    specularTotal = vec3(0.0);

    // Apply background texture
    if (material.textureFlags.z > 0.0) {
      texColor.rgb = SRGBtoLINEAR(texture(inputTextures[backgroundTextureIndex], texUv)).rgb;
      texColor.w = 1.0;
    }

    // Apply main texture
    if (material.textureFlags.x > 0.0) {
      vec4 mainColor = SRGBtoLINEAR(texture(inputTextures[mainTextureIndex], texUv));
      if (material.textureFlags.z == 0.0)
        texColor = mainColor;
      else
        texColor = vec4(mix(texColor.xyz, mainColor.xyz, mainColor.w), clamp(texColor.w + mainColor.w, 0.0, 1.0));
    }

    // Mix with pen texture
    if (material.textureFlags.y > 0.0) {
      vec4 penColor = texture(inputTextures[penTextureIndex], penTexUv);
      texColor = vec4(mix(texColor.xyz, penColor.xyz, penColor.w), texColor.w);
    }
  } else {
    specularTotal *= material.specularAndExponent.xyz;

    // Mix pen texture with mesh diffuse color
    if (material.textureFlags.y > 0.0) {
      vec4 penColor = texture(inputTextures[penTextureIndex], penTexUv);
      diffuseTotal *= mix(material.diffuse.xyz, penColor.xyz, penColor.w);
    } else
      diffuseTotal *= material.diffuse.xyz;
  }

  fragColor = texColor * vec4(material.emissiveAndOpacity.xyz + ambientTotal + diffuseTotal + specularTotal,
                              material.emissiveAndOpacity.w);

  if (fog.mode.x > 0.0) {
    float fogDensity = fog.params.x;
    float fogDensity2 = fog.params.y;
    float fogEnd = fog.params.z;
    float fogInverseScale = fog.params.w;

    float z;
    if (fog.mode.y == 1.0)
      // Real point distance (length)
      z = length(fragmentPosition.xyz);
    else
      // z-depth (plane distance)
      z = -fragmentPosition.z;

    float fogFactor;
    if (fog.mode.x == 1.0)  // FOG_EXP
      fogFactor = exp2(-fogDensity * z);
    else if (fog.mode.x == 2.0)  // FOG_EXP2
      fogFactor = exp2(-fogDensity2 * z * z);
    else  // FOG_LINEAR
      fogFactor = (fogEnd - z) * fogInverseScale;
    fogFactor = clamp(fogFactor, 0.0, 1.0);

    fragColor = vec4(mix(fragColor.xyz, fog.color.xyz, pow(1.0 - fogFactor, 2.2)), fragColor.w);
  }
}
