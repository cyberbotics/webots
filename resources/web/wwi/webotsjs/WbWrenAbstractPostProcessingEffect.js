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

export {WbWrenAbstractPostProcessingEffect}

class WbWrenAbstractPostProcessingEffect {
  constructor() {
    this.wrenPostProcessingEffect = undefined;
    this.wrenViewport = undefined;

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
