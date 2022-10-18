// Negative IDs are assigned to nodes provided by Webots without IDs. Begins at -2 because -1 means 'nothing' in Selector.
let nodeUndefinedID = -2;
let parameterUndefinedID = -2;

function getAnId() {
  return 'n' + nodeUndefinedID--;
}

function getAParameterId() {
  return 'p' + parameterUndefinedID--;
}

export {getAnId, getAParameterId};
