import {WbWrenAbstractPostProcessingEffect} from './wbWrenAbstractPostProcessingEffect.js';
import {WbWrenPostProcessingEffects} from './wbWrenPostProcessingEffects.js';

class WbWrenSmaa extends WbWrenAbstractPostProcessingEffect {
  setup(viewport) {
    if (typeof this.wrenPostProcessingEffect === 'undefined') {
      // In case we want to update the viewport, the old postProcessingEffect has to be removed first
      if (this.wrenViewport === viewport)
        _wr_viewport_remove_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);

      _wr_post_processing_effect_delete(this.wrenPostProcessingEffect);
    }

    this.wrenViewport = viewport;

    const width = _wr_viewport_get_width(this.wrenViewport);
    const height = _wr_viewport_get_height(this.wrenViewport);

    this.wrenPostProcessingEffect = WbWrenPostProcessingEffects.smaa(width, height, ENUM.WR_TEXTURE_INTERNAL_FORMAT_RGBA8);

    _wr_viewport_set_anti_aliasing_effect(this.wrenViewport, this.wrenPostProcessingEffect);
    _wr_post_processing_effect_setup(this.wrenPostProcessingEffect);

    this.hasBeenSetup = true;
  }
}

export {WbWrenSmaa};
