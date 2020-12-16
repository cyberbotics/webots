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

import{M_PI} from "./../WbConstants.js"

class WbVector3 {
  constructor(x = 0.0, y = 0.0, z = 0.0){
    this.x = x;
    this.y = y;
    this.z = z;
  }

  add(vec){
    return new WbVector3(this.x + vec.x, this.y + vec.y, this.z + vec.z);
  }

  sub(vec) {
    return new WbVector3(this.x - vec.x, this.y - vec.y, this.z -  vec.z);
  }

  mul(number) {
    return new WbVector3(this.x * number, this.y * number, this.z * number);
  }

  div(number) {
    return new WbVector3(this.x / number, this.y / number, this.z / number);
  }
  // cross product
  cross(v) {
    return new WbVector3(this.y * v.z - this.z * v.y, this.z * v.x - this.x * v.z, this.x * v.y - this.y * v.x);
  }

  almostEquals(v, tolerance) {
    return Math.abs(this.x - v.x) < tolerance && Math.abs(this.y - v.y) < tolerance && Math.abs(this.z - v.z) < tolerance;
  }

  equal(v) {
    return this.x === v.x && this.y === v.y && this.z === v.z;
  }

  normalize() {
    let result = this.div(this.length());
    this.x = result.x;
    this.y = result.y;
    this.z = result.z;
  }

  normalized() {
    return this.div(this.length());
  }

  length() {
     return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
  }

  length2() {
    return this.x * this.x + this.y * this.y + this.z * this.z; }
  // dot product
  dot(v) {
     return this.x * v.x + this.y * v.y + this.z * v.z;
   }

   // null test
  isNull() {
    return this.x == 0.0 && this.y == 0.0 && this.z == 0.0;
  }

  // angle between two vectors (in radians)
  angle(v) {
    let s = this.dot(v) / Math.sqrt(this.length2() * v.length2());
    assert(Math.abs(s) < 1.0000000001);
    return (s >= 1.0) ? 0 : (s <= -1.0) ? M_PI : Math.acos(s);
  }

  get(index) {
    if(index === 0)
      return this.x;
    else if(index === 1)
      return this.y;
    else if (index === 2)
      return this.z;
  }
}
export {WbVector3}
