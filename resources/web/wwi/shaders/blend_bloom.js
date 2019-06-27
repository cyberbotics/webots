/* global THREE */

// equivalent of: WEBOTS_HOME/resources/wren/shaders/blend_bloom.frag

THREE.blendBloomShader = {

  uniforms: {
    'blurTexture1': { value: null },
    'blurTexture2': { value: null },
    'blurTexture3': { value: null },
    'blurTexture4': { value: null },
    'blurTexture5': { value: null },
    'blurTexture6': { value: null }
  },

  vertexShader: [
    'varying vec2 vUv;',
    'void main() {',
    '  vUv = uv;',
    '  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);',
    '}'
  ].join('\n'),

  fragmentShader: [
    'varying vec2 vUv;',
    'uniform sampler2D blurTexture1;',
    'uniform sampler2D blurTexture2;',
    'uniform sampler2D blurTexture3;',
    'uniform sampler2D blurTexture4;',
    'uniform sampler2D blurTexture5;',
    'uniform sampler2D blurTexture6;',

    'void main() {',
    '  gl_FragColor = vec4(0.1 * vec3(',
    '    0.1 * texture2D(blurTexture1, vUv).rgb + ',
    '    0.2 * texture2D(blurTexture2, vUv).rgb + ',
    '    0.4 * texture2D(blurTexture3, vUv).rgb + ',
    '    0.8 * texture2D(blurTexture4, vUv).rgb + ',
    '    1.6 * texture2D(blurTexture5, vUv).rgb + ',
    '    3.2 * texture2D(blurTexture6, vUv).rgb',
    '  ), 1.0);',
    '}'
  ].join('\n')
};
