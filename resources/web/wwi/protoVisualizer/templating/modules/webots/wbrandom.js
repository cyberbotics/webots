/*
 * content: random number generation function
 */

import * as wbutility from './wbutility.js';

let seedValue = 0;

export function seed(s) {
  seedValue = s;
};

// Linear congruential generator (LCG) algorithm.
// With the coefficients used in Microsoft Visual Basic.
// https://en.wikipedia.org/wiki/Linear_congruential_generator
function updateSeed() {
  seedValue = (1140671485 * seedValue + 12820163) % Math.pow(2, 24);
};

export function real(min, max) {
  if (typeof min === 'undefined') {
    min = 0;
    max = 1;
  } else if (typeof max === 'undefined') {
    max = min;
    min = 0;
  }

  wbutility.assert(wbutility.isScalar(min) && wbutility.isScalar(max), 'Expected parameters to be numbers in wbrandom.real.');

  updateSeed();

  return min + seedValue * (max - min) / (Math.pow(2, 24) - 1);
}

export function integer(min, max) {
  if (typeof min === 'undefined') {
    min = 0;
    max = Math.pow(2, 24) - 1;
  } else if (typeof max === 'undefined') {
    max = min;
    min = 1;
  }

  wbutility.assert(wbutility.isScalar(min) && wbutility.isScalar(max), 'Expected parameters to be numbers in wbrandom.integer.');

  updateSeed();

  return min + seedValue % (1 + max - min);
}
