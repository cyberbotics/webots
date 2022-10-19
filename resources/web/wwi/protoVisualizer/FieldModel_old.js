import {VRML} from './constants.js';

export const FieldModel = {
  'Appearance': {
    'supported': {
      'material': {'type': VRML.SFNode, 'defaultValue': undefined},
      'texture': {'type': VRML.SFNode, 'defaultValue': undefined},
      'textureTransform': {'type': VRML.SFNode, 'defaultValue': undefined},
    },
    'unsupported': {
      'name': {'type': VRML.SFString, 'defaultValue': 'appearance'}
    }
  },
  'Accelerometer': {
    'supported': {
      'translation': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'rotation': {'type': VRML.SFRotation, 'defaultValue': {'x': 0, 'y': 0, z: 1, 'a': 0}},
      'scale': {'type': VRML.SFVec3f, 'defaultValue': {'x': 1, 'y': 1, 'z': 1}},
      'children': {'type': VRML.MFNode, 'defaultValue': []},
      'name': {'type': VRML.SFString, 'defaultValue': 'accelerometer'},
      'boundingObject': {'type': VRML.SFNode, 'defaultValue': undefined},
    },
    'unsupported': {
      'model': {'type': VRML.SFString, 'defaultValue': ''},
      'description': {'type': VRML.SFString, 'defaultValue': ''},
      'contactMaterial': {'type': VRML.SFString, 'defaultValue': 'default'},
      'immersionProperties': {'type': VRML.MFNode, 'defaultValue': []},
      'physics': {'type': VRML.SFNode, 'defaultValue': undefined},
      'locked': {'type': VRML.SFBool, 'defaultValue': false},
      'radarCrossSection': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'recognitionColors': {'type': VRML.MFColor, 'defaultValue': []},
      'translationStep': {'type': VRML.SFFloat, 'defaultValue': 0.01},
      'rotationStep': {'type': VRML.SFFloat, 'defaultValue': 0.261799387},
      'linearVelocity': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'angularVelocity': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'lookupTable': {'type': VRML.MFVec3f, 'defaultValue': []},
      'xAxis': {'type': VRML.SFBool, 'defaultValue': true},
      'yAxis': {'type': VRML.SFBool, 'defaultValue': true},
      'zAxis': {'type': VRML.SFBool, 'defaultValue': true},
      'resolution': {'type': VRML.SFFloat, 'defaultValue': -1}
    }
  },
  'Altimeter': {
    'supported': {
      'translation': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'rotation': {'type': VRML.SFRotation, 'defaultValue': {'x': 0, 'y': 0, z: 1, 'a': 0}},
      'scale': {'type': VRML.SFVec3f, 'defaultValue': {'x': 1, 'y': 1, 'z': 1}},
      'children': {'type': VRML.MFNode, 'defaultValue': []},
      'name': {'type': VRML.SFString, 'defaultValue': 'altimeter'},
      'boundingObject': {'type': VRML.SFNode, 'defaultValue': undefined},
    },
    'unsupported': {
      'model': {'type': VRML.SFString, 'defaultValue': ''},
      'description': {'type': VRML.SFString, 'defaultValue': ''},
      'contactMaterial': {'type': VRML.SFString, 'defaultValue': 'default'},
      'immersionProperties': {'type': VRML.MFNode, 'defaultValue': []},
      'physics': {'type': VRML.SFNode, 'defaultValue': undefined},
      'locked': {'type': VRML.SFBool, 'defaultValue': false},
      'radarCrossSection': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'recognitionColors': {'type': VRML.MFColor, 'defaultValue': []},
      'translationStep': {'type': VRML.SFFloat, 'defaultValue': 0.01},
      'rotationStep': {'type': VRML.SFFloat, 'defaultValue': 0.261799387},
      'linearVelocity': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'angularVelocity': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'accuracy': {'type': VRML.SFFloat, 'defaultValue': 0},
      'resolution': {'type': VRML.SFFloat, 'defaultValue': -1}
    }
  },
  'Background': {
    'supported': {
      'skyColor': {'type': VRML.MFColor, 'defaultValue': [{'r': 0, 'g': 0, 'b': 0}]},
      'backUrl': {'type': VRML.MFString, 'defaultValue': []},
      'bottomUrl': {'type': VRML.MFString, 'defaultValue': []},
      'frontUrl': {'type': VRML.MFString, 'defaultValue': []},
      'leftUrl': {'type': VRML.MFString, 'defaultValue': []},
      'rightUrl': {'type': VRML.MFString, 'defaultValue': []},
      'topUrl': {'type': VRML.MFString, 'defaultValue': []},
      'backIrradianceUrl': {'type': VRML.MFString, 'defaultValue': []},
      'bottomIrradianceUrl': {'type': VRML.MFString, 'defaultValue': []},
      'frontIrradianceUrl': {'type': VRML.MFString, 'defaultValue': []},
      'leftIrradianceUrl': {'type': VRML.MFString, 'defaultValue': []},
      'rightIrradianceUrl': {'type': VRML.MFString, 'defaultValue': []},
      'topIrradianceUrl': {'type': VRML.MFString, 'defaultValue': []},
      'luminosity': {'type': VRML.SFFloat, 'defaultValue': 1}
    },
    'unsupported': {
    }
  },
  'BallJoint': {
    'supported': {
      'endPoint': {'type': VRML.SFNode, 'defaultValue': undefined},
      'jointParameters': {'type': VRML.SFNode, 'defaultValue': undefined},
      'jointParameters2': {'type': VRML.SFNode, 'defaultValue': undefined},
      'jointParameters3': {'type': VRML.SFNode, 'defaultValue': undefined},
      'device': {'type': VRML.MFNode, 'defaultValue': []},
      'device2': {'type': VRML.MFNode, 'defaultValue': []},
      'device3': {'type': VRML.MFNode, 'defaultValue': []}
    },
    'unsupported': {
      'position': {'type': VRML.SFFloat, 'defaultValue': 0},
      'position2': {'type': VRML.SFFloat, 'defaultValue': 0},
      'position3': {'type': VRML.SFFloat, 'defaultValue': 0}
    }
  },
  'BallJointParameters': {
    'supported': {
      'position': {'type': VRML.SFFloat, 'defaultValue': 0},
      'anchor': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'minStop': {'type': VRML.SFFloat, 'defaultValue': 0},
      'maxStop': {'type': VRML.SFFloat, 'defaultValue': 0}
    },
    'unsupported': {
      'springConstant': {'type': VRML.SFFloat, 'defaultValue': 0},
      'dampingConstant': {'type': VRML.SFFloat, 'defaultValue': 0},
      'staticFriction': {'type': VRML.SFFloat, 'defaultValue': 0},
      'springConstant': {'type': VRML.SFFloat, 'defaultValue': 0},
      'dampingConstant': {'type': VRML.SFFloat, 'defaultValue': 0}
    }
  },
  'Billboard': {
    'supported': {
      'children': {'type': VRML.MFNode, 'defaultValue': []}
    },
    'unsupported': {
    }
  },
  'Box': {
    'supported': {
      'size': {'type': VRML.SFVec3f, 'defaultValue': {'x': 2, 'y': 2, 'z': 2}}
    },
    'unsupported': {
    }
  },
  'Brake': {
    'supported': {
      'name': {'type': VRML.SFString, 'defaultValue': 'brake'}
    },
    'unsupported': {
    }
  },
  'CadShape': {
    'supported': {
      'url': {'type': VRML.MFString, 'defaultValue': []},
      'ccw': {'type': VRML.SFBool, 'defaultValue': true},
      'castShadows': {'type': VRML.SFBool, 'defaultValue': true},
      'isPickable': {'type': VRML.SFBool, 'defaultValue': true}
    },
    'unsupported': {
    }
  },
  'Camera': {
    'supported': {
      'translation': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'rotation': {'type': VRML.SFRotation, 'defaultValue': {'x': 0, 'y': 0, z: 1, 'a': 0}},
      'scale': {'type': VRML.SFVec3f, 'defaultValue': {'x': 1, 'y': 1, 'z': 1}},
      'children': {'type': VRML.MFNode, 'defaultValue': []},
      'name': {'type': VRML.SFString, 'defaultValue': 'accelerometer'},
      'boundingObject': {'type': VRML.SFNode, 'defaultValue': undefined},
    },
    'unsupported': {
      'model': {'type': VRML.SFString, 'defaultValue': ''},
      'description': {'type': VRML.SFString, 'defaultValue': ''},
      'contactMaterial': {'type': VRML.SFString, 'defaultValue': 'default'},
      'immersionProperties': {'type': VRML.MFNode, 'defaultValue': []},
      'physics': {'type': VRML.SFNode, 'defaultValue': undefined},
      'locked': {'type': VRML.SFBool, 'defaultValue': false},
      'translationStep': {'type': VRML.SFFloat, 'defaultValue': 0.01},
      'rotationStep': {'type': VRML.SFFloat, 'defaultValue': 0.261799387},
      'radarCrossSection': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'recognitionColors': {'type': VRML.MFColor, 'defaultValue': []},
      'linearVelocity': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'angularVelocity': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'spherical': {'type': VRML.SFBool, 'defaultValue': false},
      'fieldOfView': {'type': VRML.SFFloat, 'defaultValue': 0.785398},
      'width': {'type': VRML.SFInt32, 'defaultValue': 64},
      'height': {'type': VRML.SFInt32, 'defaultValue': 64},
      'projection': {'type': VRML.SFString, 'defaultValue': 'planar'},
      'near': {'type': VRML.SFFloat, 'defaultValue': 0.01},
      'far': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'exposure': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'antiAliasing': {'type': VRML.SFBool, 'defaultValue': false},
      'ambientOcclusionRadius': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'bloomThreshold': {'type': VRML.SFFloat, 'defaultValue': -1.0},
      'motionBlur': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'noise': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'noiseMaskUrl': {'type': VRML.SFString, 'defaultValue': ''},
      'lens': {'type': VRML.SFNode, 'defaultValue': undefined},
      'focus': {'type': VRML.SFNode, 'defaultValue': undefined},
      'zoom': {'type': VRML.SFNode, 'defaultValue': undefined},
      'recognition': {'type': VRML.SFNode, 'defaultValue': undefined},
      'lensFlare': {'type': VRML.SFNode, 'defaultValue': undefined}
    }
  },
  'Capsule': {
    'supported': {
      'bottom': {'type': VRML.SFBool, 'defaultValue': true},
      'height': {'type': VRML.SFFloat, 'defaultValue': 2},
      'radius': {'type': VRML.SFFloat, 'defaultValue': 1},
      'side': {'type': VRML.SFBool, 'defaultValue': true},
      'top':  {'type': VRML.SFBool, 'defaultValue': true},
      'subdivision': {'type': VRML.SFInt32, 'defaultValue': 12}
    },
    'unsupported': {
    }
  },
  'Charger': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'battery': VRML.MFFloat, 'radius': VRML.SFFloat, 'emissiveColor': VRML.SFColor, 'gradual': VRML.SFBool}
  },
  'Color': {
    'supported': {'color': VRML.MFColor},
    'unsupported': {}
  },
  'Compass': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'lookupTable': VRML.MFVec3f, 'xAxis': VRML.SFBool, 'yAxis': VRML.SFBool, 'zAxis': VRML.SFBool, 'resolution': VRML.SFFloat}
  },
  'Cone': {
    'supported': {'bottomRadius': VRML.SFFloat, 'height': VRML.SFFloat, 'side': VRML.SFBool, 'bottom': VRML.SFBool, 'subdivision': VRML.SFInt32},
    'unsupported': {}
  },
  'Connector': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'type': VRML.SFString, 'isLocked': VRML.SFBool, 'autoLock': VRML.SFBool, 'unilateralLock': VRML.SFBool, 'unilateralUnlock': VRML.SFBool, 'distanceTolerance': VRML.SFFloat, 'axisTolerance': VRML.SFFloat, 'rotationTolerance': VRML.SFFloat, 'numberOfRotations': VRML.SFInt32, 'snap': VRML.SFBool, 'tensileStrength': VRML.SFFloat, 'shearStrength': VRML.SFFloat}
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
  'Display': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'width': VRML.SFInt32, 'height': VRML.SFInt32}
  },
  'DistanceSensor': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'lookupTable': VRML.MFVec3f, 'type': VRML.SFString, 'numberOfRays': VRML.SFInt32, 'aperture': VRML.SFFloat, 'gaussianWidth': VRML.SFFloat, 'resolution': VRML.SFFloat, 'redColorSensitivity': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f}
  },
  'ElevationGrid': {
    'supported': {'height': VRML.MFFloat, 'xDimension': VRML.SFInt32, 'xSpacing': VRML.SFFloat, 'yDimension': VRML.SFInt32, 'ySpacing': VRML.SFFloat, 'thickness': VRML.SFFloat},
    'unsupported': {}
  },
  'Emitter': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'type': VRML.SFString, 'range': VRML.SFFloat, 'maxRange': VRML.SFFloat, 'aperture': VRML.SFFloat, 'channel': VRML.SFInt32, 'baudRate': VRML.SFInt32, 'byteSize': VRML.SFInt32, 'bufferSize': VRML.SFInt32, 'allowedChannels': VRML.MFInt32}
  },
  'Fog': {
    'supported': {'color': VRML.SFColor, 'fogType': VRML.SFString, 'visibilityRange': VRML.SFFloat},
    'unsupported': {}
  },
  'GPS': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'type': VRML.SFString, 'accuracy': VRML.SFFloat, 'noiseCorrelation': VRML.SFFloat, 'resolution': VRML.SFFloat, 'speedNoise': VRML.SFFloat, 'speedResolution': VRML.SFFloat}
  },
  'Group': {
    'supported': {'children': VRML.MFNode},
    'unsupported': {}
  },
  'Gyro': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'lookupTable': VRML.MFVec3f, 'xAxis': VRML.SFBool, 'yAxis': VRML.SFBool, 'zAxis': VRML.SFBool, 'resolution': VRML.SFFloat}
  },
  'HingeJoint': {
    'supported': {'endPoint': VRML.SFNode, 'jointParameters': VRML.SFNode, 'device': VRML.MFNode},
    'unsupported': {}
  },
  'Hinge2Joint': {
    'supported': {'endPoint': VRML.SFNode, 'jointParameters': VRML.SFNode, 'jointParameters2': VRML.SFNode, 'device': VRML.MFNode, 'device2': VRML.MFNode},
    'unsupported': {}
  },
  'HingeJointParameters': {
    'supported': {'position': VRML.SFFloat, 'axis': VRML.SFVec3f, 'anchor': VRML.SFVec3f, 'minStop': VRML.SFFloat, 'maxStop': VRML.SFFloat},
    'unsupported': {'springConstant': VRML.SFFloat, 'dampingConstant': VRML.SFFloat, 'staticFriction': VRML.SFFloat, 'suspensionSpringConstant': VRML.SFFloat, 'suspensionDampingConstant': VRML.SFFloat, 'suspensionAxis': VRML.SFVec3f, 'stopCFM': VRML.SFFloat, 'stopERP': VRML.SFFloat}
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
  'InertialUnit': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'noise': VRML.SFFloat, 'xAxis': VRML.SFBool, 'yAxis': VRML.SFBool, 'zAxis': VRML.SFBool, 'resolution': VRML.SFFloat}
  },
  'JointParameters': {
    'supported': {'position': VRML.SFFloat, 'axis': VRML.SFVec3f, 'minStop': VRML.SFFloat, 'maxStop': VRML.SFFloat, 'springConstant': VRML.SFFloat, 'dampingConstant': VRML.SFFloat, 'staticFriction': VRML.SFFloat},
    'unsupported': {}
  },
  'LED': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'color': VRML.MFColor, 'gradual': VRML.SFBool, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f}
  },
  'Lidar': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'tiltAngle': VRML.SFFloat, 'horizontalResolution': VRML.SFInt32, 'fieldOfView': VRML.SFFloat, 'verticalFieldOfView': VRML.SFFloat, 'numberOfLayers': VRML.SFInt32, 'near': VRML.SFFloat, 'minRange': VRML.SFFloat, 'maxRange': VRML.SFFloat, 'spherical': VRML.SFBool, 'type': VRML.SFString, 'noise': VRML.SFFloat, 'resolution': VRML.SFFloat, 'defaultFrequency': VRML.SFFloat, 'minFrequency': VRML.SFFloat, 'maxFrequency': VRML.SFFloat, 'rotatingHead': VRML.SFNode}
  },
  'LightSensor': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'lookupTable': VRML.MFVec3f, 'colorFilter': VRML.SFColor, 'occlusion': VRML.SFBool, 'resolution': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f}
  },
  'LinearMotor': {
    'supported': {'name': VRML.SFString, 'acceleration': VRML.SFFloat, 'consumptionFactor': VRML.SFFloat, 'controlPID': VRML.SFVec3f, 'maxVelocity': VRML.SFFloat, 'minPosition': VRML.SFFloat, 'maxPosition': VRML.SFFloat, 'maxForce': VRML.SFFloat, 'multiplier': VRML.SFFloat, 'sound': VRML.SFString, 'muscles': VRML.MFNode},
    'unsupported': {}
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
  'Pen': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'inkColor': VRML.SFColor, 'inkDensity': VRML.SFFloat, 'leadSize': VRML.SFFloat, 'maxDistance': VRML.SFFloat, 'write': VRML.SFBool}
  },
  'Physics': {
    'supporter': {},
    'unsupported': {'density': VRML.SFFloat, 'mass': VRML.SFFloat, 'centerOfMass': VRML.MFVec3f, 'inertiaMatrix': VRML.MFVec3f, 'damping': VRML.SFNode},
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
    'supported': {'name': VRML.SFString},
    'unsupported': {'noise': VRML.SFFloat, 'resolution': VRML.SFFloat}
  },
  'Propeller': {
    'supported': {'shaftAxis': VRML.SFVec3f, 'centerOfThrust': VRML.SFVec3f, 'thrustConstants': VRML.SFVec2f, 'torqueConstants': VRML.SFVec2f, 'fastHelixThreshold': VRML.SFFloat, 'device': VRML.SFNode, 'fastHelix': VRML.SFNode, 'slowHelix': VRML.SFNode},
    'unsupported': {}
  },
  'Radar': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'minRange': VRML.SFFloat, 'maxRange': VRML.SFFloat, 'horizontalFieldOfView': VRML.SFFloat, 'verticalFieldOfView': VRML.SFFloat, 'minAbsoluteRadialSpeed': VRML.SFFloat, 'minRadialSpeed': VRML.SFFloat, 'maxRadialSpeed': VRML.SFFloat, 'cellDistance': VRML.SFFloat, 'cellSpeed': VRML.SFFloat, 'rangeNoise': VRML.SFFloat, 'speedNoise': VRML.SFFloat, 'angularNoise': VRML.SFFloat, 'antennaGain': VRML.SFFloat, 'frequency': VRML.SFFloat, 'transmittedPower': VRML.SFFloat, 'minDetectableSignal': VRML.SFFloat, 'occlusion': VRML.SFBool}
  },
  'RangeFinder': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'fieldOfView': VRML.SFFloat, 'width': VRML.SFInt32, 'height': VRML.SFInt32, 'spherical': VRML.SFBool, 'near': VRML.SFFloat, 'minRange': VRML.SFFloat, 'maxRange': VRML.SFFloat, 'motionBlur': VRML.SFFloat, 'noise': VRML.SFFloat, 'resolution': VRML.SFFloat, 'lense': VRML.SFNode}
  },
  'Receiver': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'type': VRML.SFString, 'aperture': VRML.SFFloat, 'channel': VRML.SFInt32, 'baudRate': VRML.SFInt32, 'byteSize': VRML.SFInt32, 'bufferSize': VRML.SFInt32, 'signalStrengthNoise': VRML.SFFloat, 'directionNoise': VRML.SFFloat, 'allowedChannels': VRML.MFInt32}
  },
  'Robot': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode,'name': VRML.SFString},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'controller': VRML.SFString, 'controllerArgs': VRML.MFString, 'customData': VRML.SFString, 'supervisor': VRML.SFBool, 'synchronization': VRML.SFBool, 'battery': VRML.MFFloat, 'cpuConsumption': VRML.SFFloat, 'selfCollision': VRML.SFBool, 'showWindow': VRML.SFBool, 'window': VRML.SFString, 'remoteControl': VRML.SFString, 'boundingObject': VRML.SFNode}
  },
  'RotationalMotor': {
    'supported': {'name': VRML.SFString, 'minPosition': VRML.SFFloat, 'maxPosition': VRML.SFFloat},
    'unsupported': {'acceleration': VRML.SFFloat, 'consumptionFactor': VRML.SFFloat, 'controlPID': VRML.SFVec3f, 'maxVelocity': VRML.SFFloat, 'maxTorque': VRML.SFFloat, 'multiplier': VRML.SFFloat, 'sound': VRML.SFString, 'muscles': VRML.MFNode}
  },
  'Shape': {
    'supported': {'appearance': VRML.SFNode, 'geometry': VRML.SFNode, 'castShadows': VRML.SFBool, 'isPickable': VRML.SFBool},
    'unsupported': {}
  },
  'SliderJoint': {
    'supported': {'jointParameters': VRML.SFNode, 'device': VRML.MFNode, 'endPoint': VRML.SFNode},
    'unsupported': {}
  },
  'Slot': {
    'supported': {'type': VRML.SFString, 'endPoint': VRML.SFNode},
    'unsupported': {}
  },
  'Solid': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'boundingObject': VRML.SFNode}
  },
  'Sphere': {
    'supported': {'radius': VRML.SFFloat, 'subdivision': VRML.SFInt32, 'ico': VRML.SFBool},
    'unsupported': {}
  },
  'Speaker': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f}
  },
  'SpotLight': {
    'supported': {
      'ambientIntensity': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'attenuation': {'type': VRML.SFVec3f, 'defaultValue': {'x': 1, 'y': 0, 'z': 0}},
      'beamWidth': {'type': VRML.SFFloat, 'defaultValue': 1.570796},
      'color': {'type': VRML.SFColor, 'defaultValue': {'r': 1, 'g': 1, 'b': 1}},
      'cutOffAngle': {'type': VRML.SFFloat, 'defaultValue': 0.785398},
      'direction': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': -1}},
      'intensity': {'type': VRML.SFFloat, 'defaultValue': 1.0},
      'location': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'on': {'type': VRML.SFBool, 'defaultValue': true},
      'radius': {'type': VRML.SFFloat, 'defaultValue': 100},
      'castShadows': {'type': VRML.SFBool, 'defaultValue': false}
    },
    'unsupported': {
    }
  },
  'TextureCoordinate': {
    'supported': {
      'point': {'type': VRML.MFVec2f, 'defaultValue': []}
    },
    'unsupported': {
    }
  },
  'TextureTransform': {
    'supported': {
      'center': {'type': VRML.SFVec2f, 'defaultValue': {'x': 0, 'y': 0}},
      'rotation': {'type': VRML.SFFloat, 'defaultValue': 0},
      'scale': {'type': VRML.SFVec2f, 'defaultValue': {'x': 1, 'y': 1}},
      'translation': {'type': VRML.SFVec2f, 'defaultValue': {'x': 0, 'y': 0}}
    },
    'unsupported': {
    }
  },
  'TouchSensor': {
    'supported': {
      'translation': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'rotation': {'type': VRML.SFRotation, 'defaultValue': {'x': 0, 'y': 0, z: 1, 'a': 0}},
      'scale': {'type': VRML.SFVec3f, 'defaultValue': {'x': 1, 'y': 1, 'z': 1}},
      'children': {'type': VRML.MFNode, 'defaultValue': []},
      'name': {'type': VRML.SFString, 'defaultValue': 'accelerometer'},
      'boundingObject': {'type': VRML.SFNode, 'defaultValue': undefined},
    },
    'unsupported': {
      'model': {'type': VRML.SFString, 'defaultValue': ''},
      'description': {'type': VRML.SFString, 'defaultValue': ''},
      'contactMaterial': {'type': VRML.SFString, 'defaultValue': 'default'},
      'immersionProperties': {'type': VRML.MFNode, 'defaultValue': []},
      'physics': {'type': VRML.SFNode, 'defaultValue': undefined},
      'locked': {'type': VRML.SFBool, 'defaultValue': false},
      'radarCrossSection': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'recognitionColors': {'type': VRML.MFColor, 'defaultValue': []},
      'translationStep': {'type': VRML.SFFloat, 'defaultValue': 0.01},
      'rotationStep': {'type': VRML.SFFloat, 'defaultValue': 0.261799387},
      'linearVelocity': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'angularVelocity': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'type': {'type': VRML.SFString, 'defaultValue': 'bumper'},
      'lookupTable': {'type': VRML.MFVec3f, 'defaultValue': [{'x': 0, 'y': 0, 'z': 0}, {'x': 5000, 'y': 50000, 'z': 0}]},
      'resolution': {'type': VRML.SFFloat, 'defaultValue': -1}
    }
  },
  'Track': {
    'supported': {
      'translation': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'rotation': {'type': VRML.SFRotation, 'defaultValue': {'x': 0, 'y': 0, z: 1, 'a': 0}},
      'scale': {'type': VRML.SFVec3f, 'defaultValue': {'x': 1, 'y': 1, 'z': 1}},
      'children': {'type': VRML.MFNode, 'defaultValue': []},
      'name': {'type': VRML.SFString, 'defaultValue': 'accelerometer'},
      'boundingObject': {'type': VRML.SFNode, 'defaultValue': undefined},
      'device': {'type': VRML.MFNode, 'defaultValue': []},
      'animatedGeometry': {'type': VRML.SFNode, 'defaultValue': undefined},
      'geometriesCount': {'type': VRML.SFInt32, 'defaultValue': 10}
    },
    'unsupported': {
      'model': {'type': VRML.SFString, 'defaultValue': ''},
      'description': {'type': VRML.SFString, 'defaultValue': ''},
      'contactMaterial': {'type': VRML.SFString, 'defaultValue': 'default'},
      'immersionProperties': {'type': VRML.MFNode, 'defaultValue': []},
      'physics': {'type': VRML.SFNode, 'defaultValue': undefined},
      'locked': {'type': VRML.SFBool, 'defaultValue': false},
      'radarCrossSection': {'type': VRML.SFFloat, 'defaultValue': 0.0},
      'recognitionColors': {'type': VRML.MFColor, 'defaultValue': []},
      'translationStep': {'type': VRML.SFFloat, 'defaultValue': 0.01},
      'rotationStep': {'type': VRML.SFFloat, 'defaultValue': 0.261799387},
      'linearVelocity': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'angularVelocity': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'trackAnimation': {'type': VRML.SFVec2f, 'defaultValue': {'x': 0, 'y': 0}}
    }
  },
  'TrackWheel': {
    'supported': {
      'children': {'type': VRML.MFNode, 'defaultValue': []},
      'position': {'type': VRML.SFVec2f, 'defaultValue': {'x': 0, 'y': 0}},
      'rotation': {'type': VRML.SFRotation, 'defaultValue': {'x': 1, 'y': 0, z: 0, 'a': 1.5708}},
      'radius': {'type': VRML.SFFloat, 'defaultValue': 0.1},
      'inner': {'type': VRML.SFBool, 'defaultValue': true}
    },
    'unsupported': {
    }
  },
  'Transform': {
    'supported': {
      'translation': {'type': VRML.SFVec3f, 'defaultValue': {'x': 0, 'y': 0, 'z': 0}},
      'rotation': {'type': VRML.SFRotation, 'defaultValue': {'x': 0, 'y': 0, z: 1, 'a': 0}},
      'scale': {'type': VRML.SFVec3f, 'defaultValue': {'x': 1, 'y': 1, 'z': 1}},
      'children': {'type': VRML.MFNode, 'defaultValue': []}
    },
    'unsupported': {
      'translationStep': {'type': VRML.SFFloat, 'defaultValue': 0.01},
      'rotationStep': {'type': VRML.SFFloat, 'defaultValue': 0.261799387},
    }
  },
  'Viewpoint': {
    'supported': {},
    'unsupported': {}
  },
  'WorldInfo': {
    'supported': {
    },
    'unsupported': {}
  },
  'Zoom': {
    'supported': {
    },
    'unsupported': {
      'maxFieldOfView': {'type': VRML.SFFloat, 'defaultValue': 1.5},
      'minFieldOfView': {'type': VRML.SFFloat, 'defaultValue': 0.5}
    }
  }
};
