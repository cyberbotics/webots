export const VRML = {
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
  'Box': {
    'supported': {'size': VRML.SF_VECT3F},
    'unsupported': {}
  },
  'Capsule': {
    'supported': {'bottom': VRML.SF_BOOL, 'height': VRML.SF_FLOAT, 'radius': VRML.SF_FLOAT, 'side': VRML.SF_BOOL, 'top': VRML.SF_BOOL, 'subdivision': VRML.SF_INT32},
    'unsupported': {}
  },
  'Cone': {
    'supported': {'bottomRadius': VRML.SF_FLOAT, 'height': VRML.SF_FLOAT, 'side': VRML.SF_BOOL, 'bottom': VRML.SF_BOOL, 'subdivision': VRML.SF_INT32},
    'unsupported': {}
  },
  'Cylinder': {
    'supported': {'bottom': VRML.SF_BOOL, 'height': VRML.SF_FLOAT, 'radius': VRML.SF_FLOAT, 'side': VRML.SF_BOOL, 'top': VRML.SF_BOOL, 'subdivision': VRML.SF_INT32},
    'unsupported': {}
  },
  'ElevationGrid': {
    'supported': {'height': VRML.MF_FLOAT, 'xDimension': VRML.SF_INT32, 'xSpacing': VRML.SF_FLOAT, 'zDimension': VRML.SF_INT32, 'zSpacing': VRML.SF_FLOAT, 'thickness': VRML.SF_FLOAT},
    'unsupported': {}
  },
  'IndexedFaceSet': {
    'supported': {'coord': VRML.SF_NODE, 'normal': VRML.SF_NODE, 'height': VRML.MF_FLOAT, 'texCoord': VRML.SF_NODE, 'ccw': VRML.SF_BOOL, 'coordIndex': VRML.MF_INT32, 'normalIndex': VRML.MF_INT32, 'texCoordIndex': VRML.MF_INT32},
    'unsupported': {'solid': VRML.SF_BOOL, 'convex': VRML.SF_BOOL, 'normalPerVertex': VRML.SF_BOOL, 'creaseAngle': VRML.SF_FLOAT}
  },
  'Mesh': {
    'supported': {},
    'unsupported': {'url': VRML.MF_STRING}
  },
  'Plane': {
    'supported': {'size': VRML.SF_VECT2F},
    'unsupported': {}
  },
  'Sphere': {
    'supported': {'radius': VRML.SF_FLOAT, 'subdivision': VRML.SF_INT32, 'ico': VRML.SF_BOOL},
    'unsupported': {}
  },
  'Shape': {'castShadow': VRML.SF_BOOL, 'isPickable': VRML.SF_BOOL, 'geometry': VRML.SF_NODE, 'appearance': VRML.SF_NODE},
  'PBRAppearance': {
    'supported': {'baseColor': VRML.SF_COLOR, 'baseColorMap': VRML.SF_NODE, 'transparency': VRML.SF_FLOAT, 'roughness': VRML.SF_FLOAT, 'roughnessMap': VRML.SF_NODE, 'metalness': VRML.SF_FLOAT, 'metalnessMap': VRML.SF_NODE, 'IBLStrength': VRML.SF_FLOAT, 'normalMap': VRML.SF_NODE, 'normalMapFactor': VRML.SF_FLOAT, 'occlusionMap': VRML.SF_NODE, 'occlusionMapStrength': VRML.SF_FLOAT, 'emissiveColor': VRML.SF_COLOR, 'emissiveColorMap': VRML.SF_NODE, 'emissiveIntensity': VRML.SF_FLOAT},
    'unsupported': {}
  },
  'Transform': {
    'supported': {'translation': VRML.SF_VECT3F, 'rotation': VRML.SF_ROTATION, 'scale': VRML.SF_VECT3F, 'children': VRML.MF_NODE},
    'unsupported': {'translationStep': VRML.SF_FLOAT, 'rotationStep': VRML.SF_FLOAT}
  },
  'NodeName': {
    'supported': {},
    'unsupported': {}
  }
};


this.emissiveIntensity = emissiveIntensity;
