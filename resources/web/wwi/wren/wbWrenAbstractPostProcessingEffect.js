class WbWrenAbstractPostProcessingEffect {
  constructor() {
    this.hasBeenSetup = false;
  }

  delete() {
    if (typeof this.wrenViewport !== 'undefined')
      _wr_viewport_remove_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);

    _wr_post_processing_effect_delete(this.wrenPostProcessingEffect);
  }

  detachFromViewport() {
    if (this.wrenViewport) {
      _wr_viewport_remove_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);
      _wr_post_processing_effect_delete(this.wrenPostProcessingEffect);
      this.wrenPostProcessingEffect = undefined;
      this.wrenViewport = undefined;
      this.hasBeenSetup = false;
    }
  }
}

export {WbWrenAbstractPostProcessingEffect};
