// Copyright 1996-2020 Cyberbotics Ltd.
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

import {WbMatrix3} from "./WbMatrix3.js";
import {WbVector4} from "./WbVector4.js";

class WbMatrix4 {
  constructor(){
    this.m = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
  }

  extracted3x3Matrix()  {
    return new WbMatrix3(this.m[0], this.m[1], this.m[2], this.m[4], this.m[5], this.m[6], this.m[8], this.m[9], this.m[10]);
  }

  mulByVec4(v) {
    return new WbVector4(this.m[0] * v.x + this.m[1] * v.y + this.m[2] * v.z + this.m[3] * v.w,
      this.m[4] * v.x + this.m[5] * v.y + this.m[6] * v.z + this.m[7] * v.w,
      this.m[8] * v.x + this.m[9] * v.y + this.m[10] * v.z + this.m[11] * v.w,
      this.m[12] * v.x + this.m[13] * v.y + this.m[14] * v.z + this.m[15] * v.w);
  }
}
export {WbMatrix4}
