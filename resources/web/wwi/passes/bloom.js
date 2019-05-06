/* global THREE */

THREE.Bloom = class Bloom extends THREE.Pass {
  constructor(resolution = new THREE.Vector2(256, 256), threshold = 21.0) {
    super();

    this.threshold = threshold;
    this.resolution = resolution.clone();

    // create color only once here, reuse it later inside the render function
    this.clearColor = new THREE.Color(0, 0, 0);

    // render targets
    var pars = { minFilter: THREE.LinearFilter, magFilter: THREE.LinearFilter, format: THREE.RGBAFormat };
    this.renderTargetsHorizontal = [];
    this.renderTargetsVertical = [];
    var resx = Math.round(this.resolution.x / 2);
    var resy = Math.round(this.resolution.y / 2);

    this.renderTargetBright = new THREE.WebGLRenderTarget(resx, resy, pars);
    this.renderTargetBright.texture.name = 'Bloom.bright';
    this.renderTargetBright.texture.generateMipmaps = false;

    for (let i = 0; i < 6; i++) {
      var renderTargetHorizonal = new THREE.WebGLRenderTarget(resx, resy, pars);

      renderTargetHorizonal.texture.name = 'Bloom.h' + i;
      renderTargetHorizonal.texture.generateMipmaps = false;

      this.renderTargetsHorizontal.push(renderTargetHorizonal);

      var renderTargetVertical = new THREE.WebGLRenderTarget(resx, resy, pars);

      renderTargetVertical.texture.name = 'Bloom.v' + i;
      renderTargetVertical.texture.generateMipmaps = false;

      this.renderTargetsVertical.push(renderTargetVertical);

      resx = Math.round(resx / 2);
      resy = Math.round(resy / 2);
    }

    // luminosity high pass material

    if (THREE.brightPassShader === undefined)
      console.error('Bloom relies on THREE.brightPassShader');

    var brightPassShader = THREE.brightPassShader;
    this.brightPassUniforms = THREE.UniformsUtils.clone(brightPassShader.uniforms);
    this.brightPassUniforms[ 'threshold' ].value = threshold;
    this.brightPassUniforms[ 'textureSize' ].value = resolution;

    this.materialBrightPass = new THREE.ShaderMaterial({
      uniforms: this.brightPassUniforms,
      vertexShader: brightPassShader.vertexShader,
      fragmentShader: brightPassShader.fragmentShader,
      defines: {}
    });

    // Gaussian Blur Materials
    this.separableBlurMaterials = [];
    var kernelSizeArray = [ 3, 5, 7, 9, 11, 13 ];
    resx = Math.round(this.resolution.x / 2);
    resy = Math.round(this.resolution.y / 2);

    for (let i = 0; i < 6; i++) {
      this.separableBlurMaterials.push(this.getSeperableBlurMaterial(kernelSizeArray[ i ]));

      this.separableBlurMaterials[ i ].uniforms[ 'texSize' ].value = new THREE.Vector2(resx, resy);

      resx = Math.round(resx / 2);
      resy = Math.round(resy / 2);
    }

    // Composite material
    if (THREE.brightPassShader === undefined)
      console.error('Bloom relies on THREE.blendBloomShader');
    this.compositeMaterial = new THREE.ShaderMaterial(THREE.blendBloomShader);
    this.compositeMaterial.uniforms[ 'blurTexture1' ].value = this.renderTargetsVertical[ 0 ].texture;
    this.compositeMaterial.uniforms[ 'blurTexture2' ].value = this.renderTargetsVertical[ 1 ].texture;
    this.compositeMaterial.uniforms[ 'blurTexture3' ].value = this.renderTargetsVertical[ 2 ].texture;
    this.compositeMaterial.uniforms[ 'blurTexture4' ].value = this.renderTargetsVertical[ 3 ].texture;
    this.compositeMaterial.uniforms[ 'blurTexture5' ].value = this.renderTargetsVertical[ 4 ].texture;
    this.compositeMaterial.uniforms[ 'blurTexture6' ].value = this.renderTargetsVertical[ 5 ].texture;
    this.compositeMaterial.needsUpdate = true;

    // copy material
    if (THREE.CopyShader === undefined)
      console.error('THREE.BloomPass relies on THREE.CopyShader');

    var copyShader = THREE.CopyShader;

    this.copyUniforms = THREE.UniformsUtils.clone(copyShader.uniforms);
    this.copyUniforms[ 'opacity' ].value = 1.0;

    this.materialCopy = new THREE.ShaderMaterial({
      uniforms: this.copyUniforms,
      vertexShader: copyShader.vertexShader,
      fragmentShader: copyShader.fragmentShader,
      blending: THREE.AdditiveBlending,
      depthTest: false,
      depthWrite: false,
      transparent: true
    });

    this.enabled = true;
    this.needsSwap = false;

    this.oldClearColor = new THREE.Color();
    this.oldClearAlpha = 1;

    this.basic = new THREE.MeshBasicMaterial();

    this.fsQuad = new THREE.Pass.FullScreenQuad(null);
  }

  dispose() {
    for (let i = 0; i < this.renderTargetsHorizontal.length; i++)
      this.renderTargetsHorizontal[ i ].dispose();

    for (let i = 0; i < this.renderTargetsVertical.length; i++)
      this.renderTargetsVertical[ i ].dispose();

    this.renderTargetBright.dispose();
  }

  setSize(width, height) {
    var resx = Math.round(width / 2);
    var resy = Math.round(height / 2);

    this.renderTargetBright.setSize(resx, resy);
    this.brightPassUniforms[ 'textureSize' ].value = new THREE.Vector2(width, height);

    for (let i = 0; i < 6; i++) {
      this.renderTargetsHorizontal[ i ].setSize(resx, resy);
      this.renderTargetsVertical[ i ].setSize(resx, resy);

      this.separableBlurMaterials[ i ].uniforms[ 'texSize' ].value = new THREE.Vector2(resx, resy);

      resx = Math.round(resx / 2);
      resy = Math.round(resy / 2);
    }
  }

  render(renderer, writeBuffer, readBuffer, deltaTime, maskActive) {
    this.oldClearColor.copy(renderer.getClearColor());
    this.oldClearAlpha = renderer.getClearAlpha();
    var oldAutoClear = renderer.autoClear;
    renderer.autoClear = false;

    renderer.setClearColor(this.clearColor, 0);

    if (maskActive)
      renderer.context.disable(renderer.context.STENCIL_TEST);

    // Render input to screen
    if (this.renderToScreen) {
      this.fsQuad.material = this.basic;
      this.basic.map = readBuffer.texture;

      renderer.setRenderTarget(null);
      renderer.clear();
      this.fsQuad.render(renderer);
    }

    // 1. Extract Bright Areas

    this.brightPassUniforms[ 'tDiffuse' ].value = readBuffer.texture;
    this.brightPassUniforms[ 'threshold' ].value = this.threshold;
    this.fsQuad.material = this.materialBrightPass;

    renderer.setRenderTarget(this.renderTargetBright);
    renderer.clear();
    this.fsQuad.render(renderer);

    // 2. Blur All the mips progressively

    var inputRenderTarget = this.renderTargetBright;
    for (let i = 0; i < 6; i++) {
      this.fsQuad.material = this.separableBlurMaterials[ i ];

      this.separableBlurMaterials[ i ].uniforms[ 'colorTexture' ].value = inputRenderTarget.texture;
      this.separableBlurMaterials[ i ].uniforms[ 'direction' ].value = new THREE.Vector2(1.0, 0.0);
      renderer.setRenderTarget(this.renderTargetsHorizontal[ i ]);
      renderer.clear();
      this.fsQuad.render(renderer);

      this.separableBlurMaterials[ i ].uniforms[ 'colorTexture' ].value = this.renderTargetsHorizontal[ i ].texture;
      this.separableBlurMaterials[ i ].uniforms[ 'direction' ].value = new THREE.Vector2(0.0, 1.0);
      renderer.setRenderTarget(this.renderTargetsVertical[ i ]);
      renderer.clear();
      this.fsQuad.render(renderer);

      inputRenderTarget = this.renderTargetsVertical[ i ];
    }

    // Composite All the mips

    this.fsQuad.material = this.compositeMaterial;
    renderer.setRenderTarget(this.renderTargetsHorizontal[ 0 ]);
    renderer.clear();
    this.fsQuad.render(renderer);

    // Blend it additively over the input texture

    this.fsQuad.material = this.materialCopy;
    this.copyUniforms[ 'tDiffuse' ].value = this.renderTargetsHorizontal[ 0 ].texture;

    if (maskActive)
      renderer.context.enable(renderer.context.STENCIL_TEST);

    if (this.renderToScreen) {
      renderer.setRenderTarget(null);
      this.fsQuad.render(renderer);
    } else {
      renderer.setRenderTarget(readBuffer);
      this.fsQuad.render(renderer);
    }

    // Restore renderer settings

    renderer.setClearColor(this.oldClearColor, this.oldClearAlpha);
    renderer.autoClear = oldAutoClear;
  }

  getSeperableBlurMaterial(kernelRadius) {
    return new THREE.ShaderMaterial({

      defines: {
        'KERNEL_RADIUS': kernelRadius,
        'SIGMA': kernelRadius
      },

      uniforms: {
        'colorTexture': { value: null },
        'texSize': { value: new THREE.Vector2(0.5, 0.5) },
        'direction': { value: new THREE.Vector2(0.5, 0.5) }
      },

      vertexShader: [
        'varying vec2 vUv;',
        'void main() {',
        '  vUv = uv;',
        '  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);',
        '}'
      ].join('\n'),
      fragmentShader: [
        '#include <common>',
        'varying vec2 vUv;',
        'uniform sampler2D colorTexture;',
        'uniform vec2 texSize;',
        'uniform vec2 direction;',

        'float gaussianPdf(in float x, in float sigma) {',
        '  return 0.39894 * exp(-0.5 * x * x/(sigma * sigma))/sigma;',
        '}',

        'void main() {',
        '  vec2 invSize = 1.0 / texSize;',
        '  float fSigma = float(SIGMA);',
        '  float weightSum = gaussianPdf(0.0, fSigma);',
        '  vec3 diffuseSum = texture2D(colorTexture, vUv).rgb * weightSum;',
        '  for(int i = 1; i < KERNEL_RADIUS; i ++) {',
        '    float x = float(i);',
        '    float w = gaussianPdf(x, fSigma);',
        '    vec2 uvOffset = direction * invSize * x;',
        '    vec3 sample1 = texture2D(colorTexture, vUv + uvOffset).rgb;',
        '    vec3 sample2 = texture2D(colorTexture, vUv - uvOffset).rgb;',
        '    diffuseSum += (sample1 + sample2) * w;',
        '    weightSum += 2.0 * w;',
        '  }',
        '  gl_FragColor = vec4(diffuseSum/weightSum, 1.0);',
        '}'
      ].join('\n')
    });
  }
};
