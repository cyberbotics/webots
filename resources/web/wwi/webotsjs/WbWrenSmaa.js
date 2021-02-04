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

import {WbWrenAbstractPostProcessingEffect} from "./WbWrenAbstractPostProcessingEffect.js"
import {WbWrenPostProcessingEffects} from "./WbWrenPostProcessingEffects.js";

class WbWrenSmaa extends WbWrenAbstractPostProcessingEffect {
  constructor() {
    super();
  }

  setup(viewport) {
    if (this.wrenPostProcessingEffect) {
      // In case we want to update the viewport, the old postProcessingEffect has to be removed first
      if (this.wrenViewport == viewport)
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

export {WbWrenSmaa}
