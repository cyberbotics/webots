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

class WbVector2 {
  constructor(x = 0.0, y = 0.0){
    this.x = x;
    this.y = y;
  }

  add(vec){
    return new WbVector2(this.x + vec.x, this.y + vec.y);
  }

  sub(vec) {
    return new WbVector2(this.x - vec.x, this.y - vec.y);
  }

  mul(number) {
    return new WbVector2(this.x * number, this.y * number);
  }

  div(number) {
    return new WbVector2(this.x / number, this.y / number);
  }

  equal(v) {
    return this.x === v.x && this.y === v.y;
  }

  normalize() {
    let result = this.div(this.length());
    this.x = result.x;
    this.y = result.y;
  }

  normalized() {
    return this.div(this.length());
  }

  length() {
     return Math.sqrt(this.x * this.x + this.y * this.y);
  }

  length2() {
    return this.x * this.x + this.y * this.y;
  }

  // dot product
  dot(v) {
     return this.x * v.x + this.y * v.y;
   }

   // null test
  isNull() {
    return this.x == 0.0 && this.y == 0.0;
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
  }
}
export {WbVector2}
