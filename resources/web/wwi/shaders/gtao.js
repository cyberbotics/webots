/* global THREE */

THREE.GTAOShader = {
  uniforms: {
    'tDepth': { value: null },
    'tNormal': { value: null },
    'viewportSize': { value: new THREE.Vector2(512, 512) },

    'radius': { value: 2.0 },
    'flipNormalY': { value: 0.0 },
    'clipInfo': { value: new THREE.Vector4(0.0, 0.0, 0.0, 0.0) },
    'params': { value: new THREE.Vector4(0.0, 0.0, 0.0, 0.0) },

    'cameraProjection': { value: new THREE.Matrix4() },
    'cameraInverseProjection': { value: new THREE.Matrix4() }
  },

  vertexShader: [
    'varying vec2 texUv;',

    'void main() {',
    '  texUv = uv;',
    '  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);',
    '}'
  ].join('\n'),

  fragmentShader: [
    '#include <common>',

    'varying vec2 texUv;',

    '#define asfloat(x) float(x)',
    '#define asint(x) int(x)',

    'uniform sampler2D tDiffuse;',
    'uniform sampler2D tDepth;',
    'uniform sampler2D tNormal;',
    'uniform sampler2D tNoise;',

    'uniform vec2 viewportSize;',

    'uniform float radius;',
    'uniform float flipNormalY;',
    'uniform vec4 clipInfo;',
    'uniform vec4 params;',

    'uniform mat4 cameraProjection;',
    'uniform mat4 cameraInverseProjection;',

    'float GTAOFastSqrt(float x) {',
    '  // [Drobot2014a] Low Level Optimizations for GCN',
    '  return asfloat(0x1FBD1DF5 + (asint(x) / 2));',
    '}',

    'float GTAOFastAcos(float x) {',
    '  float res = -0.156583 * abs(x) + PI_HALF;',
    '  res *= GTAOFastSqrt(1.0 - abs(x));',
    '  return x >= 0.0 ? res : PI - res;',
    '}',

    'vec2 add = vec2(1.0, 0.0);',

    'float hash(in vec2 p) {',
    '  return fract(sin(p.x*15.32+p.y*35.78) * 43758.23);',
    '}',

    'vec2 hash2(vec2 p) {',
    '  return vec2(hash(p*.754),hash(1.5743*p.yx+4.5891))-.5;',
    '}',

    'vec2 noise2(vec2 x) {',
    '  vec2 p = floor(x);',
    '  vec2 f = fract(x);',
    '  f = f*f*(3.0-2.0*f);',
    '  vec2 res = mix(mix( hash2(p),          hash2(p + add.xy),f.x),',
    '                 mix( hash2(p + add.yx), hash2(p + add.xx),f.x),f.y);',
    '  return res;',
    '}',

    'vec4 getViewSpacePosition(vec2 pixelLocation, mat4 inverseProjectionMatrix) {',
    '  // Get the depth value for this pixel',
    '  float z = texture2D(tDepth, pixelLocation).r;',
    '  if (z == 1.0)',
    '    return vec4(0.0);',
    '  // Get x/w and y/w from the viewport position',
    '  float x = pixelLocation.x * 2.0 - 1.0;',
    '  float y = pixelLocation.y * 2.0 - 1.0;',
    '  vec4 projectedPosition = vec4(x, y, z, 1.0);',
    '  // Transform by the inverse projection matrix',
    '  vec4 vPositionVS = inverseProjectionMatrix * projectedPosition;',
    '  vPositionVS.z = -vPositionVS.z;',
    '  // Divide by w to get the view-space position',
    '  return vec4(vPositionVS.xyz / vPositionVS.w, vPositionVS.w);',
    '}',

    'float getFalloff(float squaredDistance) {',
    '#define FALLOFF_START2 0.16',
    '#define FALLOFF_END2 max(radius *radius, FALLOFF_START2)',
    '  return 2.0 * clamp((squaredDistance - FALLOFF_START2) / (FALLOFF_END2 - FALLOFF_START2), 0.0, 1.0);',
    '}',

    'void searchForHorizons(inout vec2 horizons, vec2 offset, vec3 viewVector, vec4 viewSpacePosition,',
    '                       mat4 inverseProjectionMatrix) {',
    '  // search for h1',
    '  vec4 sampleViewspacePosition;',
    '  vec3 sliceSampleToOrigin;',
    '  sampleViewspacePosition = getViewSpacePosition((gl_FragCoord.xy + offset) / viewportSize, inverseProjectionMatrix);',
    '  float squaredSampleDistance, inverseSquaredSampleDistance, horizonCosine, falloff = 0.0;',
    '  if (sampleViewspacePosition != vec4(0.0)) {',
    '    sliceSampleToOrigin = sampleViewspacePosition.xyz - viewSpacePosition.xyz;',

    '    squaredSampleDistance = dot(sliceSampleToOrigin, sliceSampleToOrigin);',
    '    inverseSquaredSampleDistance = inversesqrt(squaredSampleDistance);',
    '    horizonCosine = inverseSquaredSampleDistance * dot(sliceSampleToOrigin, viewVector);',

    '    falloff = getFalloff(squaredSampleDistance);',
    '    horizons.x = max(horizons.x, horizonCosine - falloff);',
    '  }',

    '  // search for h2',
    '  sampleViewspacePosition = getViewSpacePosition((gl_FragCoord.xy - offset) / viewportSize, inverseProjectionMatrix);',
    '  if (sampleViewspacePosition != vec4(0.0)) {',
    '    sliceSampleToOrigin = sampleViewspacePosition.xyz - viewSpacePosition.xyz;',

    '    squaredSampleDistance = pow(length(sliceSampleToOrigin), 2.0);',
    '    inverseSquaredSampleDistance = inversesqrt(squaredSampleDistance);',
    '    horizonCosine = inverseSquaredSampleDistance * dot(sliceSampleToOrigin, viewVector);',

    '    falloff = getFalloff(squaredSampleDistance);',
    '    horizons.y = max(horizons.y, horizonCosine - falloff);',
    '  }',
    '}',

    'float integrateArc(vec2 horizons, float cosN, float sinN2, float projectedNormalLength, float n) {',
    '  return projectedNormalLength * 0.25 *',
    '         ((horizons.x * sinN2 + cosN - cos(2.0 * horizons.x - n)) + (horizons.y * sinN2 + cosN - cos(2.0 * horizons.y - n)));',
    '}',

    'void main() {',
    '  vec3 viewSpaceNormal = texture2D(tNormal, texUv).rgb;',

    '  // first, retrieve view-space position and normals',
    '  vec4 viewSpacePosition = getViewSpacePosition(texUv, cameraInverseProjection);',
    '  if (viewSpaceNormal == vec3(0.0) || viewSpacePosition.z > 200.0) {',
    '    // discard any fragments from the background or those which do not have a normal',
    '    gl_FragColor = vec4(1.0);',
    '    return;',
    '  }',

    '  viewSpaceNormal -= 0.5;',
    '  viewSpaceNormal *= 2.0;',
    '  viewSpaceNormal = normalize(vec3(viewSpaceNormal.xy, -viewSpaceNormal.z));',

    '  if (flipNormalY > 0.0)',
    '    viewSpaceNormal.y *= -1.0;',

    '  // calculate screen-space radius',
    '  float projectedRadius = (radius * clipInfo.z) / viewSpacePosition.z;',
    '  float stepsize = projectedRadius / params.z;',

    '  // fetch noises and calculate jittered slice angle',
    '  vec2 noises = noise2(gl_FragCoord.xy);',
    '  float sliceAngle = (params.x + noises.x) * PI;',
    '  float currentStep = mod(params.y + noises.y, 1.0) * (stepsize - 1.0) + 1.0;',
    '  vec3 searchDirection = vec3(cos(sliceAngle), sin(sliceAngle), 0.0);',

    '  // set up last couple of things to find horizon angles',
    '  vec3 viewVector = normalize(-viewSpacePosition.xyz);',
    '  vec2 horizons = vec2(-1.0, -1.0);',

    '  vec2 offset;',
    '  for (int j = 0; j < 3; ++j) {', // TODO: 3 => params.z
    '    offset = floor(searchDirection.xy * currentStep + 0.5);',
    '    currentStep += stepsize;',

    '    searchForHorizons(horizons, offset, viewVector, viewSpacePosition, cameraInverseProjection);',
    '  }',

    '  horizons = vec2(GTAOFastAcos(horizons.x), GTAOFastAcos(horizons.y));',

    '  // calculate n (angle between plane normal and projected normal)',
    '  vec3 bitangent = normalize(cross(searchDirection, viewVector));',
    '  vec3 planeNormal = cross(viewVector, bitangent);',
    '  vec3 projectedNormal = viewSpaceNormal - bitangent * dot(viewSpaceNormal, bitangent);',

    '  float projectedNormalLength = length(projectedNormal);',
    '  float inverseProjectedNormalLength = 1.0 / (projectedNormalLength + 1e-6);       // to avoid division with zero',
    '  float cosXi = dot(projectedNormal, planeNormal) * inverseProjectedNormalLength;  // xi = n + PI_HALF',
    '  float n = GTAOFastAcos(cosXi) - PI_HALF;',
    '  float cosN = dot(projectedNormal, viewVector) * inverseProjectedNormalLength;',
    '  float sinN2 = -2.0 * cosXi;  // cos(x + PI_HALF) = -sin(x)',

    '  // clamp to normal hemisphere',
    '  horizons.x = n + max(-horizons.x - n, -PI_HALF);',
    '  horizons.y = n + min(horizons.y - n, PI_HALF);',

    '  // distance filter - accept all values until 100m, then ramp down to 0 contrib at 200m',
    '  float distanceFalloff = (max(min(0.01 * (200.0 - viewSpacePosition.z), 1.0), 0.0));',

    '  gl_FragColor = vec4(1.0 - ((1.0 - integrateArc(horizons, cosN, sinN2, projectedNormalLength, n)) * distanceFalloff));',
    '}'
  ].join('\n')
};
