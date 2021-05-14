import * as wbrotation from 'modules/webots/wbrotation.js';

function render(text) {
  return text
}

export function main() {
  let a = {x: 1, y: 0, z: 0, a: 1.2};
  let b = {x: 1, y: 0, z: 0, a: 1.2};

  let q = {w: 1, x: 0, y: 0, z: 1.2};

  let result = '';
  // return wbrotation.equal(a, b);
  let x = wbrotation.testFunction() + q.w;
  result += eval("x");

  return result;
};
