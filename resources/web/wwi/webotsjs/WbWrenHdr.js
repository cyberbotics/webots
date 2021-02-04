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

import {WbWrenAbstractPostProcessingEffect} from "./WbWrenAbstractPostProcessingEffect.js";
import {WbWrenShaders} from "./WbWrenShaders.js";
import {pointerOnFloat} from "./WbUtils.js";

class WbWrenHdr extends WbWrenAbstractPostProcessingEffect{
  constructor(){
    super();
    this.exposure = 1.0;

    this.exposurePointer = undefined;
  }

  setup(viewport){
    if (this.wrenPostProcessingEffect) {
      // In case we want to update the viewport, the old postProcessingEffect has to be removed first
      if (this.wrenViewport == viewport)
        _wr_viewport_remove_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);

      _wr_post_processing_effect_delete(this.wrenPostProcessingEffect);
    }

    this.wrenViewport = viewport;

    let width = _wr_viewport_get_width(this.wrenViewport);
    let height = _wr_viewport_get_height(this.wrenViewport);

    this.wrenPostProcessingEffect = this.hdrResolve(width, height);

    this.applyParametersToWren();

    _wr_viewport_add_post_processing_effect(this.wrenViewport, this.wrenPostProcessingEffect);
    _wr_post_processing_effect_setup(this.wrenPostProcessingEffect);

    this.hasBeenSetup = true;
  }

  setExposure(exposure){
    this.exposure = exposure;

    this.applyParametersToWren();
  }

  applyParametersToWren() {
    if (!this.wrenPostProcessingEffect)
      return;
    let firstPass = _wr_post_processing_effect_get_first_pass(this.wrenPostProcessingEffect)
    _free(this.exposurePointer);
    this.exposurePointer = pointerOnFloat(this.exposure);
    Module.ccall('wr_post_processing_effect_pass_set_program_parameter', null, ['number', 'string', 'number'], [firstPass, "exposure", this.exposurePointer]);
  }

  hdrResolve(width, height) {
    let hdrResolveEffect = _wr_post_processing_effect_new();
    _wr_post_processing_effect_set_drawing_index(hdrResolveEffect, 7); //enum

    let hdrPass = _wr_post_processing_effect_pass_new();
    Module.ccall('wr_post_processing_effect_pass_set_name', null, ['number', 'string'], [hdrPass, "hdrResolve"]);
    _wr_post_processing_effect_pass_set_program(hdrPass, WbWrenShaders.hdrResolveShader());
    _wr_post_processing_effect_pass_set_output_size(hdrPass, width, height);
    _wr_post_processing_effect_pass_set_alpha_blending(hdrPass, false);
    _wr_post_processing_effect_pass_set_input_texture_count(hdrPass, 1);
    _wr_post_processing_effect_pass_set_output_texture_count(hdrPass, 1);
    _wr_post_processing_effect_pass_set_output_texture_format(hdrPass, 0, ENUM.WR_TEXTURE_INTERNAL_FORMAT_RGB8); //enum: try with rgba
    _wr_post_processing_effect_append_pass(hdrResolveEffect, hdrPass);

    _wr_post_processing_effect_set_result_program(hdrResolveEffect, WbWrenShaders.passThroughShader());

     return hdrResolveEffect
  }
}

export {WbWrenHdr}
