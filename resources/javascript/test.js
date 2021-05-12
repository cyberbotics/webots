import * as wbrotation from 'modules/webots/wbrotation.js';

export function main() {
  let a = {x: 1, y: 0, z: 0, a: 1.2};
  let b = {x: 1, y: 0, z: 0, a: 1.2};

  let q = {w: 1, x: 0, y: 0, z: 1.2};

  // return wbrotation.equal(a, b);
  return wbrotation.testFunction();
};
