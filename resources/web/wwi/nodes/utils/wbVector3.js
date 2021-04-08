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

class WbVector3 {
  constructor(x = 0.0, y = 0.0, z = 0.0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  add(vector) {
    return new WbVector3(this.x + vector.x, this.y + vector.y, this.z + vector.z);
  }

  sub(vector) {
    return new WbVector3(this.x - vector.x, this.y - vector.y, this.z - vector.z);
  }

  mul(number) {
    return new WbVector3(this.x * number, this.y * number, this.z * number);
  }

  div(number) {
    return new WbVector3(this.x / number, this.y / number, this.z / number);
  }

  cross(vector) {
    return new WbVector3(this.y * vector.z - this.z * vector.y, this.z * vector.x - this.x * vector.z, this.x * vector.y - this.y * vector.x);
  }

  almostEquals(vector, tolerance) {
    return Math.abs(this.x - vector.x) < tolerance && Math.abs(this.y - vector.y) < tolerance && Math.abs(this.z - vector.z) < tolerance;
  }

  equal(vector) {
    return this.x === vector.x && this.y === vector.y && this.z === vector.z;
  }

  normalize() {
    const result = this.div(this.length());
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
    return this.x * this.x + this.y * this.y + this.z * this.z;
  }

  // dot product
  dot(vector) {
    return this.x * vector.x + this.y * vector.y + this.z * vector.z;
  }

  // null test
  isNull() {
    return this.x === 0.0 && this.y === 0.0 && this.z === 0.0;
  }

  // angle between two vectors (in radians)
  angle(vector) {
    const s = this.dot(vector) / Math.sqrt(this.length2() * vector.length2());
    return (s >= 1.0) ? 0 : (s <= -1.0) ? Math.PI : Math.acos(s);
  }

  get(index) {
    if (index === 0)
      return this.x;
    else if (index === 1)
      return this.y;
    else if (index === 2)
      return this.z;
  }

  setXyz(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }
}

export {WbVector3};
