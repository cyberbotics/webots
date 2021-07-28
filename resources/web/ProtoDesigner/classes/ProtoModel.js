import {VRML} from './utility/utility.js';

export const ProtoModel = {
  'ProtoCylinder': {
    'supported': {'myRadiusRadius': VRML.SFFloat, 'myHeightHeight': VRML.SFFloat},
    'unsupported': {}
  },
  'TinkerbotsPlasticAppearance': {
    'supported': {'color': VRML.SFString, 'occlusionMapUrl': VRML.MFString},
    'unsupported': {}
  }
};
