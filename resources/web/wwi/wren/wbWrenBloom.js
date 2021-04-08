// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import {pointerOnFloat} from './../nodes/wbUtils.js';
import {WbWrenAbstractPostProcessingEffect} from './wbWrenAbstractPostProcessingEffect.js';
import {WbWrenPostProcessingEffects} from './wbWrenPostProcessingEffects.js';

class WbWrenBloom extends WbWrenAbstractPostProcessingEffect {
  constructor() {
    super();
    this.threshold = 10.0;
    this.thresholdPointer;
  }

  setup(viewport) {
    if (this.wrenPostProcessingEffect) {
      // In case we want to update the viewport, the old postProcessingEffect has to be removed first
      if (this.wrenViewport === viewport)
        _wr_viewport_remove_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);

      _wr_post_processing_effect_delete(this.wrenPostProcessingEffect);
      this.wrenPostProcessingEffect = undefined;
    }

    this.wrenViewport = viewport;

    const width = _wr_viewport_get_width(this.wrenViewport);
    const height = _wr_viewport_get_height(this.wrenViewport);

    // can't use the effect on resolutions smaller than this, it requires 6 passes dividing the viewport each time, so resolutions
    // smaller than 2^6 in width or height preculde the use of this effect
    if (Math.min(width, height) <= 64.0)
      return;

    this.wrenPostProcessingEffect = WbWrenPostProcessingEffects.bloom(width, height, ENUM.WR_TEXTURE_INTERNAL_FORMAT_RGBA16F);

    this.applyParametersToWren();

    _wr_viewport_add_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);
    _wr_post_processing_effect_setup(this.wrenPostProcessingEffect);

    this.hasBeenSetup = true;
  }

  setThreshold(threshold) {
    this.threshold = threshold;

    this.applyParametersToWren();
  }

  applyParametersToWren() {
    if (!this.wrenPostProcessingEffect)
      return;
    const pass = Module.ccall('wr_post_processing_effect_get_pass', 'number', ['number', 'string'], [this.wrenPostProcessingEffect, 'brightPassFilter']);
    _free(this.thresholdPointer);
    this.thresholdPointer = pointerOnFloat(this.threshold);
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [pass, 'threshold', this.thresholdPointer]);
  }
}

export {WbWrenBloom};
