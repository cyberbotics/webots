/*
 * content: utility functions for vector2 variables
 * assumptions: a vector 2 is an object with x and y keys
 *              e.g. v = {x: 1, y: 2}
 */

export function testFunction() { // TODO: to remove
  return 'WBVECTOR2 WORKS';
};

export function equal(vA, vB) {
  return vA.x === vB.x && vA.y === vB.y;
};

export function add(vA, vB) {
  return {x: vA.x + vB.x, y: vA.y + vB.y};
};

export function minus(vA, vB) {
  return {x: vA.x - vB.x, y: vA.y - vB.y};
};

export function multiply(v, s) {
  return {x: s * v.x, y: s * v.y};
};

export function norm(v) {
  return Math.sqrt(v.x * v.x + v.y * v.y);
};

export function atan2(v) {
  return Math.atan2(v.x, v.y);
};

export function distance(vA, vB) {
  return norm(minus(vA, vB));
}

export function angle(vA, vB) {
  return atan2(minus(vA, vB));
};

export function cross(vA, vB) {
  return vA.x * vB.y - vA.y * vB.x;
};

export function dot(vA, vB) {
  return vA.x * vB.x + vA.y * vB.y;
};

export function normalize(v) {
  const n = norm(v);

  if (n === 0)
    error(); // use global property?
  else
    return {x: v.x / n, y: v.y / n };
};

// return the intersection point between segment 1 (p1->p2)
// and segment 2 (p3->p4), a point is an object with x and y keys
// if no intersections are found return nil
export function intersection(p1, p2, p3, p4) {
  // check that the interval exists
  if (Math.max(p1.x, p2.x) < Math.min(p3.x, p4.x))
    return null;

  // check that point 1 and 2 are not equal
  if (p1.x === p2.x && p1.y === p2.y)
    return null;

  // check that point 3 and 4 are not equal
  if (p3.x === p4.x && p3.y === p4.y)
    return null;

  // check for parallel segments
  if ((p1.x === p2.x && p3.x === p4.x) || (p1.y === p2.y && p3.y === p4.y))
    return null;

  let Xa = 0;
  let Ya = 0;
  // compute the intersection of the two lines
  if (p1.x !== p2.x && p3.x !== p4.x) {
    const A1 = (p1.y - p2.y) / (p1.x - p2.x);
    const A2 = (p3.y - p4.y) / (p3.x - p4.x);
    const b1 = p1.y - A1 * p1.x;
    const b2 = p3.y - A2 * p3.x;

    if (A1 === A2)
      return null; // parallel segments

    Xa = (b2 - b1) / (A1 - A2);
    Ya = A2 * Xa + b2;
  } else {
    const A1 = (p1.x - p2.x) / (p1.y - p2.y);
    const A2 = (p3.x - p4.x) / (p3.y - p4.y);
    const b1 = p1.x - A1 * p1.y;
    const b2 = p3.x - A2 * p3.y;

    if (A1 === A2)
      return null; // parallel segments

    Ya = (b2 - b1) / (A1 - A2);
    Xa = A2 * Ya + b2;
  }

  if (Xa < Math.max(Math.min(p1.x, p2.x), Math.min(p3.x, p4.x)) || Xa > Math.min(Math.max(p1.x, p2.x), Math.max(p3.x, p4.x)))
    return null; // intersection is out of bound
  else
    return {x: Xa, y: Ya};
};
