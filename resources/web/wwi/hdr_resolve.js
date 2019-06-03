/* global THREE */

THREE.HDRResolveShader = {
  uniforms: {
    'tDiffuse': { value: null },
    'gamma': { value: 2.2 }, // TODO: this should be fix.
    'exposure': { value: 1.0 }
  },

  vertexShader: [
    'varying vec2 vUv;',

    'void main() {',
    '  vUv = uv;',
    '  gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );',
    '}'
  ].join('\n'),

  fragmentShader: [
    'uniform sampler2D tDiffuse;',
    'uniform float gamma;',
    'uniform float exposure;',
    'varying vec2 vUv;',
    'void main() {',
    '  vec4 tex = texture2D( tDiffuse, vec2( vUv.x, vUv.y ) );',
    '  vec3 mapped = vec3(1.0) - exp(-tex.xyz * exposure);',
    '  mapped = pow(mapped, vec3(1.0 / gamma));',
    '  gl_FragColor = vec4(mapped, 1.0);',
    '}'
  ].join('\n')
};
