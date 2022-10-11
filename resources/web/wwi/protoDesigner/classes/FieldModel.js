import {VRML} from './utility/utility.js';

export const FieldModel = {
  'Appearance': {
    'supported': {'material': VRML.SFNode, 'texture': VRML.SFNode, 'textureTransform': VRML.SFNode},
    'unsupported': {'name': VRML.SFString}
  },
  'Background': {
    'supported': {},
    'unsupported': {}
  },
  'Billboard': {
    'supported': {'children': VRML.MFNode},
    'unsupported': {}
  },
  'Box': {
    'supported': {'size': VRML.SFVec3f},
    'unsupported': {}
  },
  'Capsule': {
    'supported': {'bottom': VRML.SFBool, 'height': VRML.SFFloat, 'radius': VRML.SFFloat, 'side': VRML.SFBool, 'top': VRML.SFBool, 'subdivision': VRML.SFInt32},
    'unsupported': {}
  },
  'Color': {
    'supported': {'color': VRML.MFColor},
    'unsupported': {}
  },
  'Cone': {
    'supported': {'bottomRadius': VRML.SFFloat, 'height': VRML.SFFloat, 'side': VRML.SFBool, 'bottom': VRML.SFBool, 'subdivision': VRML.SFInt32},
    'unsupported': {}
  },
  'Coordinate': {
    'supported': {'point': VRML.MFVec3f},
    'unsupported': {}
  },
  'Cylinder': {
    'supported': {'bottom': VRML.SFBool, 'height': VRML.SFFloat, 'radius': VRML.SFFloat, 'side': VRML.SFBool, 'top': VRML.SFBool, 'subdivision': VRML.SFInt32},
    'unsupported': {'castLensFlares': VRML.SFBool}
  },
  'DirectionalLight': {
    'supported': {'ambientIntensity': VRML.SFFloat, 'color': VRML.SFColor, 'direction': VRML.SFVec3f, 'intensity': VRML.SFFloat, 'on': VRML.SFBool, 'castShadows': VRML.SFBool},
    'unsupported': {}
  },
  'DistanceSensor': {
    'supported': {},
    'unsupported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'boundingObject': VRML.SFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'lookupTable': VRML.MFVec3f, 'type': VRML.SFString, 'numberOfRays': VRML.SFInt32, 'aperture': VRML.SFFloat, 'gaussianWidth': VRML.SFFloat, 'resolution': VRML.SFFloat, 'redColorSensitivity': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f}
  },
  'ElevationGrid': {
    'supported': {'height': VRML.MFFloat, 'xDimension': VRML.SFInt32, 'xSpacing': VRML.SFFloat, 'yDimension': VRML.SFInt32, 'ySpacing': VRML.SFFloat, 'thickness': VRML.SFFloat},
    'unsupported': {}
  },
  'Fog': {
    'supported': {'color': VRML.SFColor, 'fogType': VRML.SFString, 'visibilityRange': VRML.SFFloat},
    'unsupported': {}
  },
  'Group': {
    'supported': {'children': VRML.MFNode},
    'unsupported': {}
  },
  'HingeJoint': {
    'supported': {'endPoint': VRML.SFNode},
    'unsupported': {'jointParameters': VRML.SFNode, 'device': VRML.MFNode, 'position': VRML.SFFloat}
  },
  'HingeJointParameters': {
    'supported': {},
    'unsupported': {'position': VRML.SFFloat, 'axis': VRML.SFVec3f, 'anchor': VRML.SFVec3f, 'minStop': VRML.SFFloat, 'maxStop': VRML.SFFloat, 'springConstant': VRML.SFFloat, 'dampingConstant': VRML.SFFloat, 'staticFriction': VRML.SFFloat, 'suspensionSpringConstant': VRML.SFFloat, 'suspensionDampingConstant': VRML.SFFloat, 'suspensionAxis': VRML.SFVec3f, 'stopCFM': VRML.SFFloat, 'stopERP': VRML.SFFloat}
  },
  'ImageTexture': {
    'supported': {'url': VRML.MFString, 'repeatS': VRML.SFBool, 'repeatT': VRML.SFBool, 'filtering': VRML.SFInt32},
    'unsupported': {}
  },
  'IndexedFaceSet': {
    'supported': {'coord': VRML.SFNode, 'normal': VRML.SFNode, 'height': VRML.MFFloat, 'texCoord': VRML.SFNode, 'ccw': VRML.SFBool, 'coordIndex': VRML.MFInt32, 'normalIndex': VRML.MFInt32, 'texCoordIndex': VRML.MFInt32, 'normalPerVertex': VRML.SFBool, 'creaseAngle': VRML.SFFloat},
    'unsupported': {'solid': VRML.SFBool, 'convex': VRML.SFBool}
  },
  'IndexedLineSet': {
    'supported': {'coord': VRML.SFNode, 'coordIndex': VRML.MFInt32},
    'unsupported': {}
  },
  'JointParameters': {
    'supported': {'position': VRML.SFFloat, 'axis': VRML.SFVec3f, 'minStop': VRML.SFFloat, 'maxStop': VRML.SFFloat, 'springConstant': VRML.SFFloat, 'dampingConstant': VRML.SFFloat, 'staticFriction': VRML.SFFloat},
    'unsupported': {}
  },
  'LED': {
    'supported': {},
    'unsupported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'boundingObject': VRML.SFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'color': VRML.MFColor, 'gradual': VRML.SFBool, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f}
  },
  'LightSensor': {
    'supported': {},
    'unsupported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'boundingObject': VRML.SFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'lookupTable': VRML.MFVec3f, 'colorFilter': VRML.SFColor, 'occlusion': VRML.SFBool, 'resolution': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f}
  },
  'Material': {
    'supported': {'ambientIntensity': VRML.SFFloat, 'diffuseColor': VRML.SFColor, 'emissiveColor': VRML.SFColor, 'shininess': VRML.SFFloat, 'specularColor': VRML.SFColor, 'transparency': VRML.SFFloat},
    'unsupported': {}
  },
  'Mesh': {
    'supported': {'url': VRML.MFString, 'ccw': VRML.SFBool, 'name': VRML.SFString, 'materialIndex': VRML.SFInt32},
    'unsupported': {}
  },
  'Normal': {
    'supported': {'vector': VRML.MFVec3f},
    'unsupported': {}
  },
  'PBRAppearance': {
    'supported': {'baseColor': VRML.SFColor, 'baseColorMap': VRML.SFNode, 'transparency': VRML.SFFloat, 'roughness': VRML.SFFloat, 'roughnessMap': VRML.SFNode, 'metalness': VRML.SFFloat, 'metalnessMap': VRML.SFNode, 'IBLStrength': VRML.SFFloat, 'normalMap': VRML.SFNode, 'normalMapFactor': VRML.SFFloat, 'occlusionMap': VRML.SFNode, 'occlusionMapStrength': VRML.SFFloat, 'emissiveColor': VRML.SFColor, 'emissiveColorMap': VRML.SFNode, 'emissiveIntensity': VRML.SFFloat, 'textureTransform': VRML.SFNode},
    'unsupported': {'name': VRML.SFString}
  },
  'Plane': {
    'supported': {'size': VRML.SFVec2f},
    'unsupported': {}
  },
  'PointLight': {
    'supported': {'ambientIntensity': VRML.SFFloat, 'attenuation': VRML.SFVec3f, 'color': VRML.SFColor, 'intensity': VRML.SFFloat, 'location': VRML.SFVec3f, 'on': VRML.SFBool, 'radius': VRML.SFFloat, 'castShadows': VRML.SFBool},
    'unsupported': {}
  },
  'PointSet': {
    'supported': {'color': VRML.SFNode, 'coord': VRML.SFNode},
    'unsupported': {}
  },
  'PositionSensor': {
    'supported': {},
    'unsupported': {'name': VRML.SFString, 'noise': VRML.SFFloat, 'resolution': VRML.SFFloat}
  },
  'Robot': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode},
    'unsupported': {'name': VRML.SFString, 'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'boundingObject': VRML.SFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f}
  },
  'RotationalMotor': {
    'supported': {},
    'unsupported': {'name': VRML.SFString, 'acceleration': VRML.SFFloat, 'consumptionFactor': VRML.SFFloat, 'controlPID': VRML.SFVec3f, 'maxVelocity': VRML.SFFloat, 'minPosition': VRML.SFFloat, 'maxPosition': VRML.SFFloat, 'maxTorque': VRML.SFFloat, 'multiplier': VRML.SFFloat, 'sound': VRML.SFString, 'muscles': VRML.MFNode}
  },
  'Shape': {
    'supported': {'appearance': VRML.SFNode, 'geometry': VRML.SFNode, 'castShadows': VRML.SFBool, 'isPickable': VRML.SFBool},
    'unsupported': {}
  },
  'SliderJoint': {
    'supported': {'jointParameters': VRML.SFNode, 'device': VRML.MFNode, 'endPoint': VRML.SFNode, 'position': VRML.SFFloat},
    'unsupported': {}
  },
  'Slot': {
    'supported': {'type': VRML.SFString, 'endPoint': VRML.SFNode},
    'unsupported': {}
  },
  'Solid': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode},
    'unsupported': {'name': VRML.SFString, 'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'boundingObject': VRML.SFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'controller': VRML.SFString, 'controllerArgs': VRML.MFString, 'customData': VRML.SFString, 'supervisor': VRML.SFBool, 'synchronization': VRML.SFBool, 'battery': VRML.MFFloat, 'cpuConsumption': VRML.SFFloat, 'selfCollision': VRML.SFBool, 'showWindow': VRML.SFBool, 'window': VRML.SFString, 'remoteControl': VRML.SFString}
  },
  'Sphere': {
    'supported': {'radius': VRML.SFFloat, 'subdivision': VRML.SFInt32, 'ico': VRML.SFBool},
    'unsupported': {}
  },
  'SpotLight': {
    'supported': {'ambientIntensity': VRML.SFFloat, 'attenuation': VRML.SFVec3f, 'beamWidth': VRML.SFFloat, 'color': VRML.SFColor, 'cutOffAngle': VRML.SFFloat, 'direction': VRML.SFVec3f, 'intensity': VRML.SFFloat, 'location': VRML.SFVec3f, 'on': VRML.SFBool, 'radius': VRML.SFFloat, 'castShadows': VRML.SFBool},
    'unsupported': {}
  },
  'TextureCoordinate': {
    'supported': {'point': VRML.MFVec2f},
    'unsupported': {}
  },
  'TextureTransform': {
    'supported': {'center': VRML.SFVec2f, 'rotation': VRML.SFFloat, 'scale': VRML.SFVec2f, 'translation': VRML.SFVec2f},
    'unsupported': {}
  },
  'Transform': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode},
    'unsupported': {'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat}
  },
  'Viewpoint': {
    'supported': {},
    'unsupported': {}
  },
  'World': {
    'supported': {},
    'unsupported': {}
  }
};
