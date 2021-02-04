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

import {WbVector3} from "./WbVector3.js";

class WbMatrix3 {
  constructor(m0 = 1.0, m1 = 1.0, m2 = 1.0, m3 = 1.0, m4 = 1.0, m5 = 1.0, m6 = 1.0, m7 = 1.0, m8 = 1.0){
    this.m = [9];
    this.m[0] = m0;
    this.m[1] = m1;
    this.m[2] = m2;
    this.m[3] = m3;
    this.m[4] = m4;
    this.m[5] = m5;
    this.m[6] = m6;
    this.m[7] = m7;
    this.m[8] = m8;
  }

  mulByVec3(v){
    return new WbVector3(this.m[0] * v.x + this.m[1] * v.y + this.m[2] * v.z,
      this.m[3] * v.x + this.m[4] * v.y + this.m[5] * v.z,
      this.m[6] * v.x + this.m[7] * v.y + this.m[8] * v.z);
  }
}
export {WbMatrix3}
