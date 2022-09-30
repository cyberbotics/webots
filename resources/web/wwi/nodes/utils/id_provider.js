// Negative IDs are assigned to nodes provided by Webots without IDs. Begins at -2 because -1 means 'nothing' in Selector.
let undefinedID = -2;

function getAnId() {
  return 'n' + undefinedID--;
}

export {getAnId};
