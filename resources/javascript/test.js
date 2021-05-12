import * as wbrotation from 'modules/wbrotation.mjs';

export function main(){
  //return wbrotation.testFunction();
  var a = {x: 1, y: 0, z: 0, a: 1.2};
  var b = {x: 1, y: 0, z: 0, a: 1.2};

  var q = {w: 1, x: 0, y: 0, z: 1.2};

  return wbrotation.equal(a,b);
};
