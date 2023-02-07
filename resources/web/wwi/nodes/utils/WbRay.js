import WbVector3 from './WbVector3.js';

export default class WbRay {
  constructor(origin, direction) {
    this.origin = origin;
    this.direction = direction;
  }

  intersects(minBound, maxBound, tMin, tMax) {
    const bounds = [];
    bounds[0] = minBound;
    bounds[1] = maxBound;
    const invDirection = new WbVector3(1.0 / this.direction.x, 1.0 / this.direction.y, 1.0 / this.direction.z);

    const sign = [];
    sign[0] = (invDirection.x < 0);
    sign[1] = (invDirection.y < 0);
    sign[2] = (invDirection.z < 0);

    let tymin, tymax, tzmin, tzmax;
    tMin = (bounds[+sign[0]].x - this.origin.x) * invDirection.x;
    tMax = (bounds[1 - +sign[0]].x - this.origin.x) * invDirection.x;
    tymin = (bounds[+sign[1]].y - this.origin.y) * invDirection.y;
    tymax = (bounds[1 - +sign[1]].y - this.origin.y) * invDirection.y;

    if ((tMin > tymax) || (tymin > tMax))
      return [false, 0];
    if (tymin > tMin)
      tMin = tymin;
    if (tymax < tMax)
      tMax = tymax;

    tzmin = (bounds[+sign[2]].z - this.origin.z) * invDirection.z;
    tzmax = (bounds[1 - +sign[2]].z - this.origin.z) * invDirection.z;

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
