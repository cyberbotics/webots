/* global THREE */

THREE.GTAOPass = function(scene, camera, depthTexture, useNormals, resolution) {
  THREE.Pass.call(this);

  this.scene = scene;
  this.camera = camera;

  this.clear = true;
  this.needsSwap = false;

  this.supportsDepthTextureExtension = (depthTexture !== undefined) ? depthTexture : false;
  this.supportsNormalTexture = (useNormals !== undefined) ? useNormals : false;

  this.originalClearColor = new THREE.Color();
  this.oldClearColor = new THREE.Color();
  this.oldClearAlpha = 1;

  this.params = {
    output: 0,
    gtaoBias: 0.5,
    gtaoIntensity: 0.18,
    gtaoScale: 1,
    gtaoKernelRadius: 100,
    gtaoMinResolution: 0,
    gtaoBlur: true,
    gtaoBlurRadius: 8,
    gtaoBlurStdDev: 4,
    gtaoBlurDepthCutoff: 0.01
  };

  this.qualityLevel = 4.0;

  this.frameCounter = 0;
  this.rotations = [60.0, 300.0, 180.0, 240.0, 120.0, 0.0];
  this.offsets = [0.0, 0.5, 0.25, 0.75];

  this.resolution = (resolution !== undefined) ? new THREE.Vector2(resolution.x, resolution.y) : new THREE.Vector2(256, 256);

  this.gtaoRenderTarget = new THREE.WebGLRenderTarget(this.resolution.x, this.resolution.y, {
    minFilter: THREE.LinearFilter,
    magFilter: THREE.LinearFilter,
    format: THREE.RGBAFormat
  });
  this.blurIntermediateRenderTarget = this.gtaoRenderTarget.clone();
  this.beautyRenderTarget = this.gtaoRenderTarget.clone();

  this.normalRenderTarget = new THREE.WebGLRenderTarget(this.resolution.x, this.resolution.y, {
    minFilter: THREE.NearestFilter,
    magFilter: THREE.NearestFilter,
    format: THREE.RGBAFormat
  });
  this.depthRenderTarget = this.normalRenderTarget.clone();

  if (this.supportsDepthTextureExtension) {
    var depthTexture = new THREE.DepthTexture();
    depthTexture.type = THREE.UnsignedShortType;
    depthTexture.minFilter = THREE.NearestFilter;
    depthTexture.maxFilter = THREE.NearestFilter;

    this.beautyRenderTarget.depthTexture = depthTexture;
    this.beautyRenderTarget.depthBuffer = true;
  }

  this.depthMaterial = new THREE.MeshDepthMaterial();
  this.depthMaterial.depthPacking = THREE.RGBADepthPacking;
  this.depthMaterial.blending = THREE.NoBlending;

  this.normalMaterial = new THREE.MeshNormalMaterial();
  this.normalMaterial.blending = THREE.NoBlending;

  if (THREE.GTAOShader === undefined)
    console.error('THREE.GTAOPass relies on THREE.GTAOShader');

  this.gtaoMaterial = new THREE.ShaderMaterial({
    defines: Object.assign({}, THREE.GTAOShader.defines),
    fragmentShader: THREE.GTAOShader.fragmentShader,
    vertexShader: THREE.GTAOShader.vertexShader,
    uniforms: THREE.UniformsUtils.clone(THREE.GTAOShader.uniforms)
  });
  this.gtaoMaterial.extensions.derivatives = true;
  this.gtaoMaterial.defines[ 'DEPTH_PACKING' ] = this.supportsDepthTextureExtension ? 0 : 1;
  this.gtaoMaterial.defines[ 'NORMAL_TEXTURE' ] = this.supportsNormalTexture ? 1 : 0;
  this.gtaoMaterial.defines[ 'PERSPECTIVE_CAMERA' ] = this.camera.isPerspectiveCamera ? 1 : 0;
  this.gtaoMaterial.uniforms[ 'tDepth' ].value = (this.supportsDepthTextureExtension) ? depthTexture : this.depthRenderTarget.texture;
  this.gtaoMaterial.uniforms[ 'tNormal' ].value = this.normalRenderTarget.texture;
  this.gtaoMaterial.uniforms[ 'viewportSize' ].value.set(this.resolution.x, this.resolution.y);
  this.gtaoMaterial.uniforms[ 'cameraInverseProjection' ].value.getInverse(this.camera.projectionMatrix);
  this.gtaoMaterial.uniforms[ 'cameraProjection' ].value = this.camera.projectionMatrix;
  this.gtaoMaterial.blending = THREE.NoBlending;

  if (THREE.DepthLimitedBlurShader === undefined)
    console.error('THREE.GTAOPass relies on THREE.DepthLimitedBlurShader');

  this.vBlurMaterial = new THREE.ShaderMaterial({
    uniforms: THREE.UniformsUtils.clone(THREE.DepthLimitedBlurShader.uniforms),
    defines: Object.assign({}, THREE.DepthLimitedBlurShader.defines),
    vertexShader: THREE.DepthLimitedBlurShader.vertexShader,
    fragmentShader: THREE.DepthLimitedBlurShader.fragmentShader
  });
  this.vBlurMaterial.defines[ 'DEPTH_PACKING' ] = this.supportsDepthTextureExtension ? 0 : 1;
  this.vBlurMaterial.defines[ 'PERSPECTIVE_CAMERA' ] = this.camera.isPerspectiveCamera ? 1 : 0;
  this.vBlurMaterial.uniforms[ 'tDiffuse' ].value = this.gtaoRenderTarget.texture;
  this.vBlurMaterial.uniforms[ 'tDepth' ].value = (this.supportsDepthTextureExtension) ? depthTexture : this.depthRenderTarget.texture;
  this.vBlurMaterial.uniforms[ 'size' ].value.set(this.resolution.x, this.resolution.y);
  this.vBlurMaterial.blending = THREE.NoBlending;

  this.hBlurMaterial = new THREE.ShaderMaterial({
    uniforms: THREE.UniformsUtils.clone(THREE.DepthLimitedBlurShader.uniforms),
    defines: Object.assign({}, THREE.DepthLimitedBlurShader.defines),
    vertexShader: THREE.DepthLimitedBlurShader.vertexShader,
    fragmentShader: THREE.DepthLimitedBlurShader.fragmentShader
  });
  this.hBlurMaterial.defines[ 'DEPTH_PACKING' ] = this.supportsDepthTextureExtension ? 0 : 1;
  this.hBlurMaterial.defines[ 'PERSPECTIVE_CAMERA' ] = this.camera.isPerspectiveCamera ? 1 : 0;
  this.hBlurMaterial.uniforms[ 'tDiffuse' ].value = this.blurIntermediateRenderTarget.texture;
  this.hBlurMaterial.uniforms[ 'tDepth' ].value = (this.supportsDepthTextureExtension) ? depthTexture : this.depthRenderTarget.texture;
  this.hBlurMaterial.uniforms[ 'size' ].value.set(this.resolution.x, this.resolution.y);
  this.hBlurMaterial.blending = THREE.NoBlending;

  if (THREE.CopyShader === undefined)
    console.error('THREE.GTAOPass relies on THREE.CopyShader');

  this.materialCopy = new THREE.ShaderMaterial({
    uniforms: THREE.UniformsUtils.clone(THREE.CopyShader.uniforms),
    vertexShader: THREE.CopyShader.vertexShader,
    fragmentShader: THREE.CopyShader.fragmentShader,
    blending: THREE.NoBlending
  });
  this.materialCopy.transparent = true;
  this.materialCopy.depthTest = false;
  this.materialCopy.depthWrite = false;
  this.materialCopy.blending = THREE.CustomBlending;
  this.materialCopy.blendSrc = THREE.DstColorFactor;
  this.materialCopy.blendDst = THREE.ZeroFactor;
  this.materialCopy.blendEquation = THREE.AddEquation;
  this.materialCopy.blendSrcAlpha = THREE.DstAlphaFactor;
  this.materialCopy.blendDstAlpha = THREE.ZeroFactor;
  this.materialCopy.blendEquationAlpha = THREE.AddEquation;

  if (THREE.CopyShader === undefined)
    console.error('THREE.GTAOPass relies on THREE.UnpackDepthRGBAShader');

  this.depthCopy = new THREE.ShaderMaterial({
    uniforms: THREE.UniformsUtils.clone(THREE.UnpackDepthRGBAShader.uniforms),
    vertexShader: THREE.UnpackDepthRGBAShader.vertexShader,
    fragmentShader: THREE.UnpackDepthRGBAShader.fragmentShader,
    blending: THREE.NoBlending
  });

  this.fsQuad = new THREE.Pass.FullScreenQuad(null);
};

THREE.GTAOPass.OUTPUT = {
  'Beauty': 1,
  'Default': 0,
  'GTAO': 2,
  'Depth': 3,
  'Normal': 4
};

THREE.GTAOPass.prototype = Object.assign(Object.create(THREE.Pass.prototype), {
  constructor: THREE.GTAOPass,

  render: function(renderer, writeBuffer, readBuffer, deltaTime, maskActive) {
    // Rendering readBuffer first when rendering to screen
    if (this.renderToScreen) {
      this.materialCopy.blending = THREE.NoBlending;
      this.materialCopy.uniforms[ 'tDiffuse' ].value = readBuffer.texture;
      this.materialCopy.needsUpdate = true;
      this.renderPass(renderer, this.materialCopy, null);
    }

    if (this.params.output === 1)
      return;

    this.frameCounter++;

    this.oldClearColor.copy(renderer.getClearColor());
    this.oldClearAlpha = renderer.getClearAlpha();
    var oldAutoClear = renderer.autoClear;
    renderer.autoClear = false;

    renderer.setRenderTarget(this.depthRenderTarget);
    renderer.clear();

    // this.gtaoMaterial.uniforms['bias'].value = this.params.gtaoBias;
    // this.gtaoMaterial.uniforms['intensity'].value = this.params.gtaoIntensity;
    // this.gtaoMaterial.uniforms['scale'].value = this.params.gtaoScale;
    // this.gtaoMaterial.uniforms['kernelRadius'].value = this.params.gtaoKernelRadius;
    // this.gtaoMaterial.uniforms['minResolution'].value = this.params.gtaoMinResolution;
    // this.gtaoMaterial.uniforms['cameraNear'].value = this.camera.near;
    // this.gtaoMaterial.uniforms['cameraFar'].value = this.camera.far;
    // this.gtaoMaterial.uniforms['randomSeed'].value = Math.random();

    this.gtaoMaterial.uniforms['clipInfo'].value.x = this.camera.near;
    this.gtaoMaterial.uniforms['clipInfo'].value.y = this.camera.far ? this.camera.far : 1000000.0;
    this.gtaoMaterial.uniforms['clipInfo'].value.z = 0.5 * (this.resolution.y / (2.0 * Math.tan(this.camera.fovX * 0.5)));

    this.gtaoMaterial.uniforms['params'].value.x = this.rotations[this.frameCounter % 6] / 360;
    this.gtaoMaterial.uniforms['params'].value.y = this.offsets[Math.floor(this.frameCounter / 6) % 4];
    this.gtaoMaterial.uniforms['params'].value.z = 2 << (this.qualityLevel - 1);

    this.gtaoMaterial.uniforms[ 'cameraInverseProjection' ].value.getInverse(this.camera.projectionMatrix);
    this.gtaoMaterial.uniforms[ 'cameraProjection' ].value = this.camera.projectionMatrix;

    var depthCutoff = this.params.gtaoBlurDepthCutoff * (this.camera.far - this.camera.near);
    this.vBlurMaterial.uniforms[ 'depthCutoff' ].value = depthCutoff;
    this.hBlurMaterial.uniforms[ 'depthCutoff' ].value = depthCutoff;

    this.vBlurMaterial.uniforms[ 'cameraNear' ].value = this.camera.near;
    this.vBlurMaterial.uniforms[ 'cameraFar' ].value = this.camera.far;
    this.hBlurMaterial.uniforms[ 'cameraNear' ].value = this.camera.near;
    this.hBlurMaterial.uniforms[ 'cameraFar' ].value = this.camera.far;

    this.params.gtaoBlurRadius = Math.floor(this.params.gtaoBlurRadius);
    if ((this.prevStdDev !== this.params.gtaoBlurStdDev) || (this.prevNumSamples !== this.params.gtaoBlurRadius)) {
      THREE.BlurShaderUtils.configure(this.vBlurMaterial, this.params.gtaoBlurRadius, this.params.gtaoBlurStdDev, new THREE.Vector2(0, 1));
      THREE.BlurShaderUtils.configure(this.hBlurMaterial, this.params.gtaoBlurRadius, this.params.gtaoBlurStdDev, new THREE.Vector2(1, 0));
      this.prevStdDev = this.params.gtaoBlurStdDev;
      this.prevNumSamples = this.params.gtaoBlurRadius;
    }

    // Rendering scene to depth texture
    renderer.setClearColor(0x000000);
    renderer.setRenderTarget(this.beautyRenderTarget);
    renderer.clear();
    renderer.render(this.scene, this.camera);

    // Re-render scene if depth texture extension is not supported
    if (!this.supportsDepthTextureExtension) {
      // Clear rule : far clipping plane in both RGBA and Basic encoding
      this.renderOverride(renderer, this.depthMaterial, this.depthRenderTarget, 0x000000, 1.0);
    }

    if (this.supportsNormalTexture) {
      // Clear rule : default normal is facing the camera
      this.renderOverride(renderer, this.normalMaterial, this.normalRenderTarget, 0x7777ff, 1.0);
    }

    // Rendering GTAO texture
    this.renderPass(renderer, this.gtaoMaterial, this.gtaoRenderTarget, 0xffffff, 1.0);

    // Blurring GTAO texture
    if (this.params.gtaoBlur) {
      this.renderPass(renderer, this.vBlurMaterial, this.blurIntermediateRenderTarget, 0xffffff, 1.0);
      this.renderPass(renderer, this.hBlurMaterial, this.gtaoRenderTarget, 0xffffff, 1.0);
    }

    var outputMaterial = this.materialCopy;
    // Setting up GTAO rendering
    if (this.params.output === 3) {
      if (this.supportsDepthTextureExtension) {
        this.materialCopy.uniforms[ 'tDiffuse' ].value = this.beautyRenderTarget.depthTexture;
        this.materialCopy.needsUpdate = true;
      } else {
        this.depthCopy.uniforms[ 'tDiffuse' ].value = this.depthRenderTarget.texture;
        this.depthCopy.needsUpdate = true;
        outputMaterial = this.depthCopy;
      }
    } else if (this.params.output === 4) {
      this.materialCopy.uniforms[ 'tDiffuse' ].value = this.normalRenderTarget.texture;
      this.materialCopy.needsUpdate = true;
    } else {
      this.materialCopy.uniforms[ 'tDiffuse' ].value = this.gtaoRenderTarget.texture;
      this.materialCopy.needsUpdate = true;
    }

    // Blending depends on output, only want a CustomBlending when showing GTAO
    if (this.params.output === 0)
      outputMaterial.blending = THREE.CustomBlending;
    else
      outputMaterial.blending = THREE.NoBlending;

    // Rendering GTAOPass result on top of previous pass
    this.renderPass(renderer, outputMaterial, this.renderToScreen ? null : readBuffer);

    renderer.setClearColor(this.oldClearColor, this.oldClearAlpha);
    renderer.autoClear = oldAutoClear;
  },

  renderPass: function(renderer, passMaterial, renderTarget, clearColor, clearAlpha) {
    // save original state
    this.originalClearColor.copy(renderer.getClearColor());
    var originalClearAlpha = renderer.getClearAlpha();
    var originalAutoClear = renderer.autoClear;

    renderer.setRenderTarget(renderTarget);

    // setup pass state
    renderer.autoClear = false;
    if ((clearColor !== undefined) && (clearColor !== null)) {
      renderer.setClearColor(clearColor);
      renderer.setClearAlpha(clearAlpha || 0.0);
      renderer.clear();
    }

    this.fsQuad.material = passMaterial;
    this.fsQuad.render(renderer);

    // restore original state
    renderer.autoClear = originalAutoClear;
    renderer.setClearColor(this.originalClearColor);
    renderer.setClearAlpha(originalClearAlpha);
  },

  renderOverride: function(renderer, overrideMaterial, renderTarget, clearColor, clearAlpha) {
    this.originalClearColor.copy(renderer.getClearColor());
    var originalClearAlpha = renderer.getClearAlpha();
    var originalAutoClear = renderer.autoClear;

    renderer.setRenderTarget(renderTarget);
    renderer.autoClear = false;

    clearColor = overrideMaterial.clearColor || clearColor;
    clearAlpha = overrideMaterial.clearAlpha || clearAlpha;
    if ((clearColor !== undefined) && (clearColor !== null)) {
      renderer.setClearColor(clearColor);
      renderer.setClearAlpha(clearAlpha || 0.0);
      renderer.clear();
    }

    this.scene.overrideMaterial = overrideMaterial;
    renderer.render(this.scene, this.camera);
    this.scene.overrideMaterial = null;

    // restore original state
    renderer.autoClear = originalAutoClear;
    renderer.setClearColor(this.originalClearColor);
    renderer.setClearAlpha(originalClearAlpha);
  },

  setSize: function(width, height) {
    this.beautyRenderTarget.setSize(width, height);
    this.gtaoRenderTarget.setSize(width, height);
    this.blurIntermediateRenderTarget.setSize(width, height);
    this.normalRenderTarget.setSize(width, height);
    this.depthRenderTarget.setSize(width, height);

    this.gtaoMaterial.uniforms[ 'viewportSize' ].value.set(width, height);
    this.gtaoMaterial.uniforms[ 'cameraInverseProjection' ].value.getInverse(this.camera.projectionMatrix);
    this.gtaoMaterial.uniforms[ 'cameraProjection' ].value = this.camera.projectionMatrix;
    this.gtaoMaterial.needsUpdate = true;

    this.vBlurMaterial.uniforms[ 'size' ].value.set(width, height);
    this.vBlurMaterial.needsUpdate = true;

    this.hBlurMaterial.uniforms[ 'size' ].value.set(width, height);
    this.hBlurMaterial.needsUpdate = true;
  }
});
