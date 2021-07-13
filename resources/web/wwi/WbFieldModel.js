export const FIELD_TYPES = {
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

export const WbFieldModel = {
  'Box': {'size': FIELD_TYPES.SF_VECT3F},
  'Sphere': {'radius': FIELD_TYPES.SF_FLOAT, 'ico': FIELD_TYPES.SF_BOOL, 'subdivision': FIELD_TYPES.SF_INT32},
  'Shape': {'castShadow': FIELD_TYPES.SF_BOOL, 'isPickable': FIELD_TYPES.SF_BOOL, 'geometry': FIELD_TYPES.SF_NODE, 'appearance': FIELD_TYPES.SF_NODE},
  'PBRAppearance': {'baseColor': FIELD_TYPES.SF_VECT3F}
};
