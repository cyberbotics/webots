/*
 * content: core functions
 */

// copy by value any variable
export function deepCopy(orig) {
  return JSON.parse(JSON.stringify(orig));
};
