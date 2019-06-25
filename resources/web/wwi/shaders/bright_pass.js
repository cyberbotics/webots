/* global THREE */

// equivalent of: WEBOTS_HOME/resources/wren/shaders/bright_pass.frag

THREE.brightPassShader = {

  uniforms: {
    'tDiffuse': { value: null },
    'threshold': { value: 21.0 },
    'textureSize': { value: new THREE.Vector2(800, 600)}
  },

  vertexShader: [
    'varying vec2 texUv;',

    'void main() {',
    '  texUv = uv;',
    '  gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );',
    '}'
  ].join('\n'),

  fragmentShader: [
    'uniform sampler2D tDiffuse;',
    'uniform float threshold;',
    'uniform vec2 textureSize;',

    'varying vec2 texUv;',

    'float luma(vec3 color) {',
    '  return dot(color, vec3(0.299, 0.587, 0.114));',
    '}',

    'bool isnan(float val) {',
    '  return (val <= 0.0 || 0.0 <= val) ? false : true;',
    '}',

    'void main() {',
    '  vec4 color = texture2D( tDiffuse, texUv );',
    '  float totalLuma = luma( color.xyz );',

    '  totalLuma += luma(texture2D(tDiffuse, texUv + vec2(0, 1) / textureSize).rgb);',
    '  totalLuma += luma(texture2D(tDiffuse, texUv + vec2(0, -1) / textureSize).rgb);',
    '  totalLuma += luma(texture2D(tDiffuse, texUv + vec2(-1, 0) / textureSize).rgb);',
    '  totalLuma += luma(texture2D(tDiffuse, texUv + vec2(1, 0) / textureSize).rgb);',
    '  totalLuma += luma(texture2D(tDiffuse, texUv + vec2(1, 1) / textureSize).rgb);',
    '  totalLuma += luma(texture2D(tDiffuse, texUv + vec2(1, -1) / textureSize).rgb);',
    '  totalLuma += luma(texture2D(tDiffuse, texUv + vec2(-1, 1) / textureSize).rgb);',
    '  totalLuma += luma(texture2D(tDiffuse, texUv + vec2(-1, -1) / textureSize).rgb);',
    '  totalLuma /= 9.0;',

    ' if (totalLuma < threshold)',
    '   gl_FragColor = vec4(vec3(0.0), 1.0);',
    ' else {',
    '   gl_FragColor.r = min(color.r, 4000.0);',
    '   gl_FragColor.g = min(color.g, 4000.0);',
    '   gl_FragColor.b = min(color.b, 4000.0);',
    '   gl_FragColor.a = 1.0;',
    ' }',
    '}'
  ].join('\n')
};
