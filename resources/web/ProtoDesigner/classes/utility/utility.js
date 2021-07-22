let parameterId = 0; // used to uniquely keep track of proto parameters
let protoId = 0; // used to uniquely keep track of protos

function generateParameterId() {
  return 'p' + parameterId++;
};

function generateProtoId() {
  return protoId++;
};

export {generateParameterId, generateProtoId};
