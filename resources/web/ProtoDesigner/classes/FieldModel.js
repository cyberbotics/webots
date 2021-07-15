export const VRML_TYPE = {
  MF_BOOL: 0,
  SF_BOOL: 1,
  MF_COLOR: 2,
  SF_COLOR: 3,
  MF_FLOAT: 4,
  SF_FLOAT: 5,
  MF_INT32: 6,
  SF_INT32: 7,
  MF_NODE: 8,
  SF_NODE: 9,
  MF_ROTATION: 10,
  SF_ROTATION: 11,
  MF_STRING: 12,
  SF_STRING: 13,
  MF_VECT2F: 14,
  SF_VECT2F: 15,
  MF_VECT3F: 16,
  SF_VECT3F: 17
};

export const FieldModel = {
  'Box': {'size': VRML_TYPE.SF_VECT3F},
  'Sphere': {'radius': VRML_TYPE.SF_FLOAT, 'ico': VRML_TYPE.SF_BOOL, 'subdivision': VRML_TYPE.SF_INT32},
  'Shape': {'castShadow': VRML_TYPE.SF_BOOL, 'isPickable': VRML_TYPE.SF_BOOL, 'geometry': VRML_TYPE.SF_NODE, 'appearance': VRML_TYPE.SF_NODE},
  'PBRAppearance': {'baseColor': VRML_TYPE.SF_VECT3F}
};
