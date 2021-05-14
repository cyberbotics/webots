/*
 * content: utility functions to handle files
 */

export function testFunction() { // TODO: to remove
  return 'WBFILE WORKS';
};

// remove the filename and extension from the file path
export function getPathWithoutFilename(filePath) {
  // make sure to use '/' as separator
  filePath = filePath.replace('\\', '/');
  const r = /[^\/]*$/;

  return filePath.replace(r, '');
}
