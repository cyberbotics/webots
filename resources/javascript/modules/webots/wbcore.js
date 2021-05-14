/*
 * content: core functions
 */

export function testFunction() { // TODO: to remove
  return 'WBCORE WORKS';
};

// copy by value any variable
export function deepCopy(orig) {
  return JSON.parse(JSON.stringify(orig));
};
