export const FIELD_TYPES = {
  SF_BOOL: 0,
  SF_VECT2F: 1,
  SF_VECT3F: 2,
  SF_NODE: 3
};

export const WbFieldModel = {
  'Box': {'size': FIELD_TYPES.SF_VECT3F},
  'Shape': {'castShadow': FIELD_TYPES.SF_BOOL, 'isPickable': FIELD_TYPES.SF_BOOL, 'geometry': FIELD_TYPES.SF_NODE, 'appearance': FIELD_TYPES.SF_NODE},
  'PBRAppearance': {'baseColor': FIELD_TYPES.SF_VECT3F}
};
