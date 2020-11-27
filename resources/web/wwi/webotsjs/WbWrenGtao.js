import {WbWrenAbstractPostProcessingEffect} from "./WbWrenAbstractPostProcessingEffect.js";
import {WbWrenPostProcessingEffects} from "./WbWrenPostProcessingEffects.js";
import {WbWrenShaders} from "./WbWrenShaders.js";
import {arrayXPointer} from "./WbUtils.js";

class WbWrenGtao extends WbWrenAbstractPostProcessingEffect {
  constructor(){
    super();
    this.halfResolution = false;

    this.gtaoPass = null;
    this.temporalPass = null
    this.near = 0.0;
    this.far = 0.0
    this.fov = 0.78
    this.radius = 2.0;
    this.flipNormalY = 0.0
    this.frameCounter = 0;

    this.clipInfo = [0, 0, 0, 0];
    this.params = [0.0, 0.0, 0.0, 0.0];
    this.rotations = [60.0, 300.0, 180.0, 240.0, 120.0, 0.0];
    this.offsets = [0.0, 0.5, 0.25, 0.75];
    this.previousInverseViewMatrix = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
  }

  setHalfResolution(halfResolution) {
    this.halfResolution = halfResolution;
  }

  setFov(fov) {
    this.fov = fov;
    this.applyParametersToWren();
  }

  setRadius(radius) {
    this.radius = radius;
    this.applyParametersToWren();
  }

  setQualityLevel(qualityLevel) {
    this.params[2] = 2 << (qualityLevel - 1);
    this.applyParametersToWren();
  }

  applyOldInverseViewMatrixToWren() {
    if (!this.wrenPostProcessingEffect)
      return;
    let previousInverseViewMatrixPointer
    if(typeof this.previousInverseViewMatrix !== 'number')
      previousInverseViewMatrixPointer = arrayXPointer(this.previousInverseViewMatrix);
    else
      previousInverseViewMatrixPointer = this.previousInverseViewMatrix;

    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [this.temporalPass, "previousInverseViewMatrix", previousInverseViewMatrixPointer]);
    if(typeof this.previousInverseViewMatrix !== 'number')
      _free(previousInverseViewMatrixPointer);
  }

  copyNewInverseViewMatrix(inverseViewMatrix) {
    this.previousInverseViewMatrix = inverseViewMatrix;
  }

  detachFromViewport() {
    if (this.wrenViewport) {
      _wr_viewport_set_ambient_occlusion_effect(this.wrenViewport, null);
      _wr_post_processing_effect_delete(this.wrenPostProcessingEffect);
      this.wrenPostProcessingEffect = null;
      this.wrenViewport = null;
      this.hasBeenSetup = false;
    }
  }

  setup(viewport) {
    if (this.wrenPostProcessingEffect) {
      // In case we want to update the viewport, the old postProcessingEffect has to be removed first
      if (this.wrenViewport == viewport)
        _wr_viewport_remove_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);

      _wr_post_processing_effect_delete(this.wrenPostProcessingEffect);
    }

    this.wrenViewport = viewport;

    let width = _wr_viewport_get_width(this.wrenViewport);
    let height = _wr_viewport_get_height(this.wrenViewport);

    if (this.halfResolution) {
      width = width <= 1.0 ? 2.0 : width;
      height = height <= 1.0 ? 2.0 : height;
    }

    let viewportFramebuffer = _wr_viewport_get_frame_buffer(this.wrenViewport);

    let depthTexture = _wr_frame_buffer_get_depth_texture(viewportFramebuffer);
    let normalTexture = _wr_frame_buffer_get_output_texture(viewportFramebuffer, 1);

    this.wrenPostProcessingEffect = WbWrenPostProcessingEffects.gtao(width, height, ENUM.WR_TEXTURE_INTERNAL_FORMAT_RGBA16F, depthTexture, normalTexture, this.halfResolution);

    this.gtaoPass = Module.ccall('wr_post_processing_effect_get_pass', 'number', ['number', 'string'], [this.wrenPostProcessingEffect, "gtaoForwardPass"]);
    this.temporalPass = Module.ccall('wr_post_processing_effect_get_pass', 'number', ['number', 'string'], [this.wrenPostProcessingEffect, "temporalDenoise"]);
    this.applyParametersToWren();

    _wr_viewport_set_ambient_occlusion_effect(this.wrenViewport, this.wrenPostProcessingEffect);
    _wr_post_processing_effect_setup(this.wrenPostProcessingEffect);

    this.hasBeenSetup = true;
  }

  applyParametersToWren() {
    if (!this.wrenPostProcessingEffect)
      return;

    this.clipInfo[0] = this.near;
    this.clipInfo[1] = this.far ? this.far : 1000000.0;
    this.clipInfo[2] = 0.5 * (_wr_viewport_get_height(this.wrenViewport) / (2.0 * Math.tan(this.fov * 0.5)));

    let array4 = _wrjs_array4(this.clipInfo[0], this.clipInfo[1], this.clipInfo[2], this.clipInfo[3]);
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [this.gtaoPass, "clipInfo", array4]);

    this.params[0] = this.rotations[this.frameCounter % 6] / 360.0;
    this.params[1] = this.offsets[Math.floor(this.frameCounter / 6) %4];

    let paramsPointer = arrayXPointer(this.params);
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [this.gtaoPass, "params", paramsPointer]);
    let radiusPointer = _wrjs_pointerOnFloat(this.radius);
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [this.gtaoPass, "radius", radiusPointer]);
    let flipNormalYPointer = _wrjs_pointerOnFloat(this.flipNormalY);
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [this.gtaoPass, "flipNormalY", flipNormalYPointer]);
    _free(paramsPointer);
    ++this.frameCounter;
  }
}

export {WbWrenGtao}
