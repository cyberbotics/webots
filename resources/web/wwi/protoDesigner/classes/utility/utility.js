let parameterId = 0; // used to uniquely keep track of proto parameters
let protoId = 0; // used to uniquely keep track of protos

function generateParameterId() {
  return 'p' + parameterId++;
};

function generateProtoId() {
  return protoId++;
};

/*
export const VRML = {
  MFBool: 0,
  SFBool: 1,
  MFColor: 2,
  SFColor: 3,
  MFFloat: 4,
  SFFloat: 5,
  MFInt32: 6,
  SFInt32: 7,
  MFNode: 8,
  SFNode: 9,
  MFRotation: 10,
  SFRotation: 11,
  MFString: 12,
  SFString: 13,
  MFVec2f: 14,
  SFVec2f: 15,
  MFVec3f: 16,
  SFVec3f: 17
};
*/

function vrmlTypeAsString(type) {
  switch (type) {
    case VRML.MFBool: return 'MFBool';
    case VRML.SFBool: return 'SFBool';
    case VRML.MFColor: return 'MFColor';
    case VRML.SFColor: return 'SFColor';
    case VRML.MFFloat: return 'MFFloat';
    case VRML.SFFloat: return 'SFFloat';
    case VRML.MFInt32: return 'MFInt32';
    case VRML.SFInt32: return 'SFInt32';
    case VRML.MFNode: return 'MFNode';
    case VRML.SFNode: return 'SFNode';
    case VRML.MFRotation: return 'MFRotation';
    case VRML.SFRotation: return 'SFRotation';
    case VRML.MFString: return 'MFString';
    case VRML.SFString: return 'SFString';
    case VRML.MFVec2f: return 'MFVec2f';
    case VRML.SFVec2f: return 'SFVec2f';
    case VRML.MFVec3f: return 'MFVec3f';
    case VRML.SFVec3f: return 'SFVec3f';
    default:
      throw new Error('unknown type');
  }
}

export {vrmlTypeAsString, generateParameterId, generateProtoId};
