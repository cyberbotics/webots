#version 330 core

precision highp float;

// These constants must be kept in sync with the values in Constants.hpp
const int maxDirectionalLights = 48;
const int maxPointLights = 48;
const int maxSpotLights = 48;

in vec3 fragmentPosition;
in vec3 fragmentNormal;
in vec2 texUv;
in vec2 penTexUv;

out vec4 fragColor;

uniform sampler2D inputTextures[9];
uniform bool reverseNormals;

struct PBRInfo {
  float NdotL;                // cos angle between normal and light direction
  float NdotV;                // cos angle between normal and view direction
  float NdotH;                // cos angle between normal and half vector
  float LdotH;                // cos angle between light direction and half vector
  float VdotH;                // cos angle between view direction and half vector
  float perceptualRoughness;  // roughness value, as authored by the model creator (input to shader)
  float metalness;            // metallic value at the surface
  vec3 reflectance0;          // full reflectance color (normal incidence angle)
  vec3 reflectance90;         // reflectance color at grazing angle
  float alphaRoughness;       // roughness mapped to a more linear change in the roughness
  vec3 diffuseColor;          // color contribution from diffuse lighting
  vec3 specularColor;         // color contribution from specular lighting
};

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
}
lights;

// Index of active light (< 0 if inactive)
layout(std140) uniform LightRenderable {
  ivec4 activeLights;  // x: directional, y: point, z: spot
}
lightRenderable;

// Material parameters for this renderable
layout(std140) uniform PbrMaterial {
  vec4 baseColorAndTransparency;
  vec4 roughnessMetalnessNormalMapFactorOcclusion;
  vec4 backgroundColorAndIblStrength;
  vec4 emissiveColorAndIntensity;
  vec4 baseColorRoughnessMetalnessOcclusionMapFlags;
  vec4 normalBrdfEmissiveBackgroundFlags;
  vec4 penFlags;
  vec4 cubeTextureFlags;
}
material;

const float M_PI = 3.141592653589793;
const float minRoughness = 0.04;

vec4 SRGBtoLINEAR(vec4 srgbIn) {
  vec3 bLess = step(vec3(0.04045), srgbIn.xyz);
  vec3 linOut = mix(srgbIn.xyz / vec3(12.92), pow((srgbIn.xyz + vec3(0.055)) / vec3(1.055), vec3(2.4)), bLess);
  return vec4(linOut, srgbIn.w);
}

mat3 cotangentFrame(vec3 N, vec3 p, vec2 uv) {
  // get edge vectors of the pixel triangle
  vec3 dp1 = dFdx(p);
  vec3 dp2 = dFdy(p);
  vec2 duv1 = dFdx(uv);
  vec2 duv2 = dFdy(uv);

  if (duv1 == vec2(0.0) && duv2 == vec2(0.0))
    return mat3(vec3(0.0), vec3(0.0), N);

  // solve the linear system
  vec3 dp2perp = cross(dp2, N);
  vec3 dp1perp = cross(N, dp1);
  vec3 T = dp2perp * duv1.x + dp1perp * duv2.x;
  vec3 B = dp2perp * duv1.y + dp1perp * duv2.y;

  // construct a scale-invariant frame
  float scale = max(dot(T, T), dot(B, B));
  if (scale <= 0.0)  // inversesqrt result is undefined for value <= 0
    return mat3(T, B, N);
  float invmax = inversesqrt(scale);
  return mat3(T * invmax, B * invmax, N);
}

vec3 perturbNormal(vec3 N, vec3 V) {
  vec3 map = texture(inputTextures[4], texUv).rgb * 2.0 - 1.0;
  map.xy = vec2(material.roughnessMetalnessNormalMapFactorOcclusion.z) * map.xy;
  mat3 TBN = cotangentFrame(N, -V, texUv);
  return normalize(TBN * map);
}

// Basic Lambertian diffuse
// Implementation from Lambert's Photometria https://archive.org/details/lambertsphotome00lambgoog
vec3 diffuse(PBRInfo pbrInputs) {
  return pbrInputs.diffuseColor / M_PI;
}

// The following equation models the Fresnel reflectance term of the spec equation (aka F())
vec3 specularReflection(PBRInfo pbrInputs) {
  return pbrInputs.reflectance0 +
         (pbrInputs.reflectance90 - pbrInputs.reflectance0) * pow(clamp(1.0 - pbrInputs.VdotH, 0.0, 1.0), 5.0);
}

// This calculates the specular geometric attenuation (aka G()),
// where rougher material will reflect less light back to the viewer.
float geometricOcclusion(PBRInfo pbrInputs) {
  float NdotL = pbrInputs.NdotL;
  float NdotV = pbrInputs.NdotV;
  float r = pbrInputs.alphaRoughness;

  float attenuationL = 2.0 * NdotL / (NdotL + sqrt(r * r + (1.0 - r * r) * (NdotL * NdotL)));
  float attenuationV = 2.0 * NdotV / (NdotV + sqrt(r * r + (1.0 - r * r) * (NdotV * NdotV)));
  return attenuationL * attenuationV;
}

// The following equation(s) model the distribution of microfacet normals across the area being drawn (aka D())
// Implementation from "Average Irregularity Representation of a Roughened Surface for Ray Reflection" by T. S. Trowbridge, and
// K. P. Reitz Follows the distribution function recommended in the SIGGRAPH 2013 course notes from EPIC Games [1], Equation 3.
float microfacetDistribution(PBRInfo pbrInputs) {
  float roughnessSq = pbrInputs.alphaRoughness * pbrInputs.alphaRoughness;
  float f = (pbrInputs.NdotH * roughnessSq - pbrInputs.NdotH) * pbrInputs.NdotH + 1.0;
  return roughnessSq / (M_PI * f * f);
}

vec3 PBRpass(vec3 l, vec3 n, vec3 v, vec3 h, vec4 lightColorAndIntensity, float roughness, float metalness,
             vec3 specularEnvironmentR0, vec3 specularEnvironmentR90, float alphaRoughness, vec3 diffuseColor,
             vec3 specularColor) {
  float NdotL = clamp(dot(n, l), 0.0001, 1.0);
  float NdotV = abs(dot(n, v)) + 0.0001;
  float NdotH = clamp(dot(n, h), 0.0, 1.0);
  float LdotH = clamp(dot(l, h), 0.0, 1.0);
  float VdotH = clamp(dot(v, h), 0.0, 1.0);
  vec3 color = vec3(0.0);

  PBRInfo pbrInputs = PBRInfo(NdotL, NdotV, NdotH, LdotH, VdotH, roughness, metalness, specularEnvironmentR0,
                              specularEnvironmentR90, alphaRoughness, diffuseColor, specularColor);

  vec3 F = specularReflection(pbrInputs);
  float G = geometricOcclusion(pbrInputs);
  float D = microfacetDistribution(pbrInputs);

  vec3 diffuseContrib = (1.0 - F) * diffuse(pbrInputs) * lightColorAndIntensity.w;
  vec3 specContrib = F * G * D / (4.0 * NdotL * NdotV);
  // Obtain final intensity as reflectance (BRDF) scaled by the energy of the light (cosine law)
  color = NdotL * lightColorAndIntensity.xyz * (diffuseContrib + specContrib) * lightColorAndIntensity.w;

  return color;
}

void main() {
  // sample from normal map if one exists
  vec3 viewFragmentNormal = normalize(reverseNormals ? -fragmentNormal : fragmentNormal);
  if (material.normalBrdfEmissiveBackgroundFlags.x > 0.0)
    viewFragmentNormal = perturbNormal(viewFragmentNormal, normalize(-fragmentPosition));

  // read roughness and metalness values
  float perceptualRoughness = material.roughnessMetalnessNormalMapFactorOcclusion.x;
  float metalness = material.roughnessMetalnessNormalMapFactorOcclusion.y;

  // sample roughness map if one exists
  if (material.baseColorRoughnessMetalnessOcclusionMapFlags.y > 0.0) {
    vec4 roughnessSample = texture(inputTextures[1], texUv);
    perceptualRoughness = roughnessSample.r;
  }

  // sample metalness map if one exists
  if (material.baseColorRoughnessMetalnessOcclusionMapFlags.z > 0.0) {
    vec4 metalnessSample = texture(inputTextures[2], texUv);
    metalness = metalnessSample.r;
  }

  perceptualRoughness = clamp(perceptualRoughness, minRoughness, 1.0);
  metalness = clamp(metalness, 0.0, 1.0);
  float alphaRoughness = perceptualRoughness * perceptualRoughness;

  vec4 baseColor = material.baseColorAndTransparency;
  baseColor.w = 1.0 - baseColor.w;

  // apply base color map if it exists, composite background texture if necessary
  if (material.baseColorRoughnessMetalnessOcclusionMapFlags.x > 0.0) {
    vec4 baseColorMapColor = SRGBtoLINEAR(texture(inputTextures[0], texUv));

    if (material.normalBrdfEmissiveBackgroundFlags.w > 0.0) {
      vec3 backgroundTextureColor = SRGBtoLINEAR(texture(inputTextures[7], texUv)).rgb;
      baseColor.rgb = mix(backgroundTextureColor, baseColorMapColor.xyz, baseColorMapColor.w);
    } else {
      // take base color
      baseColor = baseColorMapColor;
      // re-apply transparency from base material
      baseColor.w *= (1.0 - material.baseColorAndTransparency.w);
    }

    baseColor = vec4(baseColor.rgb * material.baseColorAndTransparency.rgb, baseColor.w);
  }

  // Mix with pen texture
  if (material.penFlags.x > 0.0) {
    vec4 penColor = texture(inputTextures[8], penTexUv);
    baseColor = vec4(mix(baseColor.xyz, penColor.xyz, penColor.w), baseColor.w);
  }

  // vec3 ambientTotal = vec3(material.iblStrengthAndZeroes.x);
  vec3 f0 = vec3(0.04);
  vec3 diffuseColor = baseColor.rgb * (vec3(1.0) - f0);
  diffuseColor *= 1.0 - metalness;
  vec3 specularColor = mix(f0, baseColor.rgb, metalness);

  // Compute reflectance.
  float reflectance = max(max(specularColor.r, specularColor.g), specularColor.b);

  // For typical incident reflectance range (between 4% to 100%) set the grazing reflectance to 100% for typical fresnel effect.
  // For very low reflectance range on highly diffuse objects (below 4%), incrementally reduce grazing reflecance to 0%.
  float reflectance90 = clamp(reflectance * 25.0, 0.0, 1.0);
  vec3 specularEnvironmentR0 = specularColor.rgb;
  vec3 specularEnvironmentR90 = vec3(1.0, 1.0, 1.0) * reflectance90;

  vec3 v = normalize(-fragmentPosition);
  vec3 l = vec3(0.0, 1.0, 0.0);
  vec3 h = vec3(0.0, 1.0, 0.0);
  vec3 reflection = -normalize(reflect(v, viewFragmentNormal));

  float NdotL = 0.0;
  float NdotV = 0.0;
  float NdotH = 0.0;
  float LdotH = 0.0;
  float VdotH = 0.0;

  vec3 color = vec3(0.0);

  // Apply directional light if active
  if (lightRenderable.activeLights.x >= 0) {
    DirectionalLight light = lights.directionalLights[lightRenderable.activeLights.x];

    // Direction in uniform buffer is already normalized and in view space
    l = -vec3(light.direction);
    h = normalize(l + v);

    color += PBRpass(l, viewFragmentNormal, v, h, light.colorAndIntensity, perceptualRoughness, metalness,
                     specularEnvironmentR0, specularEnvironmentR90, alphaRoughness, diffuseColor, specularColor);
  }

  // Apply point light if active
  if (lightRenderable.activeLights.y >= 0) {
    PointLight light = lights.pointLights[lightRenderable.activeLights.y];

    l = light.position.xyz - fragmentPosition;

    float distanceToLight = length(l);
    if (distanceToLight > light.attenuationAndRadius.w)
      discard;

    vec3 attenuation = light.attenuationAndRadius.xyz;
    float attenuationFactor = 1.0 / (attenuation.x + distanceToLight * (attenuation.y + attenuation.z * distanceToLight));
    l = normalize(l);
    h = normalize(l + v);

    color +=
      attenuationFactor * PBRpass(l, viewFragmentNormal, v, h, light.colorAndIntensity, perceptualRoughness, metalness,
                                  specularEnvironmentR0, specularEnvironmentR90, alphaRoughness, diffuseColor, specularColor);
  }

  // Apply spot light if active
  if (lightRenderable.activeLights.z >= 0) {
    SpotLight light = lights.spotLights[lightRenderable.activeLights.z];

    vec3 lightPosition = vec3(light.position);

    // Direction from surface to light position in eye space
    vec3 l = lightPosition - fragmentPosition;

    // Distance between surface and light position
    float distanceToLight = length(l);
    if (distanceToLight > light.attenuationAndRadius.w)
      discard;  // not illuminated

    l = normalize(l);

    // Inverted spotlight direction
    vec3 spotDirection = -light.direction.xyz;
    // Angle between spotlight direction and direction from light to point
    float spotAngle = acos(dot(l, spotDirection));
    // Inner angle
    float beamWidth = light.spotParams.x;
    // Outer angle
    float cutoffAngle = light.spotParams.y;

    float attenuationFactor = 1.0;

    // See if point on surface is inside cone of illumination defined by cutoff
    if (spotAngle >= cutoffAngle)  // outside
      discard;                     // light adds no contribution

    else if (spotAngle > beamWidth)  // falloff
      attenuationFactor *= (cutoffAngle - spotAngle) * light.spotParams.z;

    vec3 attenuation = light.attenuationAndRadius.xyz;
    attenuationFactor /= (attenuation.x + distanceToLight * (attenuation.y + attenuation.z * distanceToLight));

    l = normalize(l);
    h = normalize(l + v);

    color +=
      attenuationFactor * PBRpass(l, viewFragmentNormal, v, h, light.colorAndIntensity, perceptualRoughness, metalness,
                                  specularEnvironmentR0, specularEnvironmentR90, alphaRoughness, diffuseColor, specularColor);
  }

  fragColor = vec4(color, baseColor.a);
}
