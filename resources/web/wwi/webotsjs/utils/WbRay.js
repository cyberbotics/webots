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

class WbRay {
  constructor (origin, direction) {
    this.origin = origin;
    this.direction = direction;
  }

  intersects(minBound, maxBound, tMin, tMax) {
    let bounds = [];
    bounds[0] = minBound;
    bounds[1] = maxBound;
    let invDirection = new WbVector3(1.0 / this.direction.x, 1.0 / this.direction.y, 1.0 / this.direction.z);

    let sign[];
    sign[0] = (invDirection.x < 0);
    sign[1] = (invDirection.y < 0);
    sign[2] = (invDirection.z < 0);

    let tymin, tymax, tzmin, tzmax;
    tMin = (bounds[sign[0]].x - this.origin.x) * invDirection.x;
    tMax = (bounds[1 - sign[0]].x - this.origin.x) * invDirection.x;
    tymin = (bounds[sign[1]].y - mOrigin.y) * invDirection.y;
    tymax = (bounds[1 - sign[1]].y - mOrigin.y) * invDirection.y;

    if ((tMin > tymax) || (tymin > tMax))
      return [false, 0];
    if (tymin > tMin)
      tMin = tymin;
    if (tymax < tMax)
      tMax = tymax;

    tzmin = (bounds[sign[2]].z - this.origin.z) * invDirection.z;
    tzmax = (bounds[1 - sign[2]].z - mOrigin.z) * invDirection.z;

    if ((tMin > tzmax) || (tzmin > tMax))
      return [false, 0];
    if (tzmin > tMin)
      tMin = tzmin;
    if (tzmax < tMax)
      tMax = tzmax;

    if (tMin < 0)
      return [true, tMax];

    return [true, tMin];
  }
}

export {WbRay}
