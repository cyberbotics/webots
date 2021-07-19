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

export const FieldModel = {
  'Box': {
    'supported': {'size': VRML.SFVec3f},
    'unsupported': {}
  },
  'Capsule': {
    'supported': {'bottom': VRML.SFBool, 'height': VRML.SFFloat, 'radius': VRML.SFFloat, 'side': VRML.SFBool, 'top': VRML.SFBool, 'subdivision': VRML.SFInt32},
    'unsupported': {}
  },
  'Cone': {
    'supported': {'bottomRadius': VRML.SFFloat, 'height': VRML.SFFloat, 'side': VRML.SFBool, 'bottom': VRML.SFBool, 'subdivision': VRML.SFInt32},
    'unsupported': {}
  },
  'Cylinder': {
    'supported': {'bottom': VRML.SFBool, 'height': VRML.SFFloat, 'radius': VRML.SFFloat, 'side': VRML.SFBool, 'top': VRML.SFBool, 'subdivision': VRML.SFInt32},
    'unsupported': {}
  },
  'ElevationGrid': {
    'supported': {'height': VRML.MFFloat, 'xDimension': VRML.SFInt32, 'xSpacing': VRML.SFFloat, 'zDimension': VRML.SFInt32, 'zSpacing': VRML.SFFloat, 'thickness': VRML.SFFloat},
    'unsupported': {}
  },
  'IndexedFaceSet': {
    'supported': {'coord': VRML.SFNode, 'normal': VRML.SFNode, 'height': VRML.MFFloat, 'texCoord': VRML.SFNode, 'ccw': VRML.SFBool, 'coordIndex': VRML.MFInt32, 'normalIndex': VRML.MFInt32, 'texCoordIndex': VRML.MFInt32}, 'unsupported': {'solid': VRML.SFBool, 'convex': VRML.SFBool, 'normalPerVertex': VRML.SFBool, 'creaseAngle': VRML.SFFloat}
  },
  'Mesh': {
    'supported': {},
    'unsupported': {'url': VRML.MFString}
  },
  'Plane': {
    'supported': {'size': VRML.SFVec2f},
    'unsupported': {}
  },
  'Sphere': {
    'supported': {'radius': VRML.SFFloat, 'subdivision': VRML.SFInt32, 'ico': VRML.SFBool},
    'unsupported': {}
  },
  'Shape': {
    'supported': {'castShadow': VRML.SFBool, 'isPickable': VRML.SFBool, 'geometry': VRML.SFNode, 'appearance': VRML.SFNode},
    'unsupported': {}
  },
  'PBRAppearance': {
    'supported': {'baseColor': VRML.SFColor, 'baseColorMap': VRML.SFNode, 'transparency': VRML.SFFloat, 'roughness': VRML.SFFloat, 'roughnessMap': VRML.SFNode, 'metalness': VRML.SFFloat, 'metalnessMap': VRML.SFNode, 'IBLStrength': VRML.SFFloat, 'normalMap': VRML.SFNode, 'normalMapFactor': VRML.SFFloat, 'occlusionMap': VRML.SFNode, 'occlusionMapStrength': VRML.SFFloat, 'emissiveColor': VRML.SFColor, 'emissiveColorMap': VRML.SFNode, 'emissiveIntensity': VRML.SFFloat, 'textureTransform': VRML.SFNode},
    'unsupported': {'name': VRML.SFString}
  },
  'Transform': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode},
    'unsupported': {'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat}
  },
  'NodeName': {
    'supported': {},
    'unsupported': {}
  }
};


this.emissiveIntensity = emissiveIntensity;
