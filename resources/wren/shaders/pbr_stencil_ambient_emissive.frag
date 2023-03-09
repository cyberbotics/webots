#version 330 core

precision highp float;

in vec3 fragmentPosition;
in vec3 fragmentNormal;
in vec2 texUv;
in vec2 penTexUv;
in mat3 inverseViewMatrix;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec4 fragNormal;

uniform sampler2D inputTextures[13];
uniform samplerCube cubeTextures[1];
uniform bool wireframeRendering;
uniform bool reverseNormals;

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

struct IBLInfo {
  float NdotV;                // cos angle between normal and view direction
  float perceptualRoughness;  // roughness value, as authored by the model creator (input to shader)
  vec3 diffuseColor;          // color contribution from diffuse lighting
  vec3 specularColor;         // color contribution from specular lighting
};

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

vec3 getIBLContribution(IBLInfo iblInputs, vec3 n, vec3 reflection) {
  vec3 diffuseLight = vec3(0.0);
  vec3 specularLight = vec3(0.0);
  vec2 brdf = texture(inputTextures[5], vec2(iblInputs.NdotV, iblInputs.perceptualRoughness)).rg;
  if (material.cubeTextureFlags.x > 0.0) {
    float mipCount = 7.0;
    float lod = (iblInputs.perceptualRoughness * mipCount);
    // A single irradiance map is used for the diffuse and specular reflections:
    // Thanks to the following fact: the diffuse map is close to the specular map at the 6th LOD.
    // invert z components of sample vectors due to VRML default camera orientation looking towards -z
    diffuseLight = textureLod(cubeTextures[0], vec3(n.xy, -n.z), 6.0).rgb;
    specularLight = textureLod(cubeTextures[0], vec3(reflection.xy, -reflection.z), lod).rgb;
  } else {
    diffuseLight = material.backgroundColorAndIblStrength.rgb;
    specularLight = material.backgroundColorAndIblStrength.rgb;
  }

  vec3 diffuse = diffuseLight * iblInputs.diffuseColor;
  vec3 specular = specularLight * (iblInputs.specularColor * brdf.x + brdf.y);

  // scale ambient light contribution
  diffuse *= material.backgroundColorAndIblStrength.w;
  specular *= material.backgroundColorAndIblStrength.w;

  return diffuse + specular;
}

void main() {
  vec3 viewFragmentNormal = normalize(reverseNormals ? -fragmentNormal : fragmentNormal);
  fragNormal = vec4(normalize(viewFragmentNormal), 1.0) * 0.5 + 0.5;

  // sample from normal map if one exists
  if (material.normalBrdfEmissiveBackgroundFlags.x > 0.0)
    viewFragmentNormal = perturbNormal(viewFragmentNormal, normalize(-fragmentPosition));

  vec3 worldFragmentNormal = inverseViewMatrix * viewFragmentNormal;

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

  vec3 f0 = vec3(0.04);
  vec3 diffuseColor = baseColor.rgb * (vec3(1.0) - f0);
  diffuseColor *= 1.0 - metalness;
  vec3 specularColor = mix(f0, baseColor.rgb, metalness);
  vec3 v = normalize(-fragmentPosition);
  float NdotV = dot(viewFragmentNormal, v) - 0.001;

  vec3 reflection = -normalize(reflect(inverseViewMatrix * v, worldFragmentNormal));

  IBLInfo iblInputs = IBLInfo(NdotV, perceptualRoughness, diffuseColor, specularColor);

  vec3 color = getIBLContribution(iblInputs, worldFragmentNormal, reflection);

  if (material.baseColorRoughnessMetalnessOcclusionMapFlags.w > 0.0) {
    float ao = texture(inputTextures[3], texUv).r;
    color = mix(color, color * ao, material.roughnessMetalnessNormalMapFactorOcclusion.w);
  }

  vec3 emissive = wireframeRendering ? material.baseColorAndTransparency.rgb : material.emissiveColorAndIntensity.rgb;

  if (material.normalBrdfEmissiveBackgroundFlags.z > 0.0)
    emissive = texture(inputTextures[6], texUv).rgb;

  if (wireframeRendering)
    emissive *= material.emissiveColorAndIntensity.w;
  color += emissive;

  fragColor = vec4(color, baseColor.a);
}
