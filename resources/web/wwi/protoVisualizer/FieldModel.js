import {VRML} from './vrml_type.js';

export const FieldModel = {
  'Gyro': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'gyro'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'lookupTable': {
      'type': VRML.MFVec3f,
      'defaultValue': []
    },
    'xAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'yAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'zAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'Mesh': {
    'url': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'ccw': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'materialIndex': {
      'type': VRML.SFInt32,
      'defaultValue': -1
    }
  },
  'Camera': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'camera'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'fieldOfView': {
      'type': VRML.SFFloat,
      'defaultValue': 0.785398
    },
    'width': {
      'type': VRML.SFInt32,
      'defaultValue': 64
    },
    'height': {
      'type': VRML.SFInt32,
      'defaultValue': 64
    },
    'near': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'far': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'exposure': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'antiAliasing': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'ambientOcclusionRadius': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'bloomThreshold': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'motionBlur': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'noise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'noiseMaskUrl': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'lens': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'focus': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'zoom': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'recognition': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'lensFlare': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'spherical': {
      'type': VRML.SFBool,
      'defaultValue': false
    }
  },
  'Lens': {
    'center': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 0.5,
        'y': 0.5
      }
    },
    'radialCoefficients': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0
      }
    },
    'tangentialCoefficients': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0
      }
    }
  },
  'TextureCoordinate': {
    'point': {
      'type': VRML.MFVec2f,
      'defaultValue': []
    }
  },
  'Charger': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'charger'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'battery': {
      'type': VRML.MFFloat,
      'defaultValue': []
    },
    'radius': {
      'type': VRML.SFFloat,
      'defaultValue': 0.04
    },
    'emissiveColor': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 0.0,
        'g': 1.0,
        'b': 0.0
      }
    },
    'gradual': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'PointLight': {
    'ambientIntensity': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'attenuation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'color': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 1.0,
        'g': 1.0,
        'b': 1.0
      }
    },
    'intensity': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'location': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'on': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'radius': {
      'type': VRML.SFFloat,
      'defaultValue': 100.0
    },
    'castShadows': {
      'type': VRML.SFBool,
      'defaultValue': false
    }
  },
  'Recognition': {
    'maxRange': {
      'type': VRML.SFFloat,
      'defaultValue': 100.0
    },
    'maxObjects': {
      'type': VRML.SFInt32,
      'defaultValue': -1
    },
    'occlusion': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'frameColor': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 1.0,
        'g': 0.0,
        'b': 0.0
      }
    },
    'frameThickness': {
      'type': VRML.SFInt32,
      'defaultValue': 1
    },
    'segmentation': {
      'type': VRML.SFBool,
      'defaultValue': false
    }
  },
  'Accelerometer': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'accelerometer'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'lookupTable': {
      'type': VRML.MFVec3f,
      'defaultValue': []
    },
    'xAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'yAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'zAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'PBRAppearance': {
    'baseColor': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 1.0,
        'g': 1.0,
        'b': 1.0
      }
    },
    'baseColorMap': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'transparency': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'roughness': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'roughnessMap': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'metalness': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'metalnessMap': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'IBLStrength': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'normalMap': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'normalMapFactor': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'occlusionMap': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'occlusionMapStrength': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'emissiveColor': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 0.0,
        'g': 0.0,
        'b': 0.0
      }
    },
    'emissiveColorMap': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'emissiveIntensity': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'textureTransform': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'PBRAppearance'
    }
  },
  'Connector': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'connector'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'isLocked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'autoLock': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'unilateralLock': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'unilateralUnlock': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'distanceTolerance': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'axisTolerance': {
      'type': VRML.SFFloat,
      'defaultValue': 0.2
    },
    'rotationTolerance': {
      'type': VRML.SFFloat,
      'defaultValue': 0.2
    },
    'numberOfRotations': {
      'type': VRML.SFInt32,
      'defaultValue': 4
    },
    'snap': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'tensileStrength': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'shearStrength': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'Speaker': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'speaker'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'LinearMotor': {
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'linear motor'
    },
    'acceleration': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'consumptionFactor': {
      'type': VRML.SFFloat,
      'defaultValue': 10.0
    },
    'controlPID': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 10.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'maxVelocity': {
      'type': VRML.SFFloat,
      'defaultValue': 10.0
    },
    'minPosition': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'maxPosition': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'maxForce': {
      'type': VRML.SFFloat,
      'defaultValue': 10.0
    },
    'multiplier': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'sound': {
      'type': VRML.SFString,
      'defaultValue': 'webots://projects/default/worlds/sounds/linear_motor.wav'
    },
    'muscles': {
      'type': VRML.MFNode,
      'defaultValue': []
    }
  },
  'TouchSensor': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'touch sensor'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'lookupTable': {
      'type': VRML.MFVec3f,
      'defaultValue': [
        {
          'x': 0.0,
          'y': 0.0,
          'z': 0.0
        },
        {
          'x': 5000.0,
          'y': 50000.0,
          'z': 0.0
        }
      ]
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'Lidar': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'lidar'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'tiltAngle': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'horizontalResolution': {
      'type': VRML.SFInt32,
      'defaultValue': 512
    },
    'fieldOfView': {
      'type': VRML.SFFloat,
      'defaultValue': 1.5708
    },
    'verticalFieldOfView': {
      'type': VRML.SFFloat,
      'defaultValue': 0.2
    },
    'numberOfLayers': {
      'type': VRML.SFInt32,
      'defaultValue': 4
    },
    'near': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'minRange': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'maxRange': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'noise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'defaultFrequency': {
      'type': VRML.SFFloat,
      'defaultValue': 10.0
    },
    'minFrequency': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'maxFrequency': {
      'type': VRML.SFFloat,
      'defaultValue': 25.0
    },
    'rotatingHead': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'spherical': {
      'type': VRML.SFBool,
      'defaultValue': true
    }
  },
  'Group': {
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    }
  },
  'Sphere': {
    'radius': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'subdivision': {
      'type': VRML.SFInt32,
      'defaultValue': 1
    },
    'ico': {
      'type': VRML.SFBool,
      'defaultValue': true
    }
  },
  'Slot': {
    'type': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'endPoint': {
      'type': VRML.SFNode,
      'defaultValue': null
    }
  },
  'Propeller': {
    'shaftAxis': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'centerOfThrust': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'thrustConstants': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 1.0,
        'y': 0.0
      }
    },
    'torqueConstants': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 1.0,
        'y': 0.0
      }
    },
    'fastHelixThreshold': {
      'type': VRML.SFFloat,
      'defaultValue': 75.4
    },
    'device': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'fastHelix': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'slowHelix': {
      'type': VRML.SFNode,
      'defaultValue': null
    }
  },
  'Physics': {
    'density': {
      'type': VRML.SFFloat,
      'defaultValue': 1000.0
    },
    'mass': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'centerOfMass': {
      'type': VRML.MFVec3f,
      'defaultValue': []
    },
    'inertiaMatrix': {
      'type': VRML.MFVec3f,
      'defaultValue': []
    },
    'damping': {
      'type': VRML.SFNode,
      'defaultValue': null
    }
  },
  'Plane': {
    'size': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0
      }
    }
  },
  'Skin': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'skin'
    },
    'modelUrl': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'appearance': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'bones': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'castShadows': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    }
  },
  'Zoom': {
    'maxFieldOfView': {
      'type': VRML.SFFloat,
      'defaultValue': 1.5
    },
    'minFieldOfView': {
      'type': VRML.SFFloat,
      'defaultValue': 0.5
    }
  },
  'ImageTexture': {
    'url': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'repeatS': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'repeatT': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'filtering': {
      'type': VRML.SFInt32,
      'defaultValue': 4
    }
  },
  'Microphone': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'microphone'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'aperture': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'sensitivity': {
      'type': VRML.SFFloat,
      'defaultValue': -50.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'BallJointParameters': {
    'position': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'anchor': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'minStop': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'maxStop': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'springConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'dampingConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'staticFriction': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'Altimeter': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'altimeter'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'accuracy': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'WorldInfo': {
    'info': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'title': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'window': {
      'type': VRML.SFString,
      'defaultValue': '<none>'
    },
    'gravity': {
      'type': VRML.SFFloat,
      'defaultValue': 9.81
    },
    'CFM': {
      'type': VRML.SFFloat,
      'defaultValue': 1e-05
    },
    'ERP': {
      'type': VRML.SFFloat,
      'defaultValue': 0.2
    },
    'physics': {
      'type': VRML.SFString,
      'defaultValue': '<none>'
    },
    'basicTimeStep': {
      'type': VRML.SFFloat,
      'defaultValue': 32.0
    },
    'FPS': {
      'type': VRML.SFFloat,
      'defaultValue': 60.0
    },
    'optimalThreadCount': {
      'type': VRML.SFInt32,
      'defaultValue': 1
    },
    'physicsDisableTime': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'physicsDisableLinearThreshold': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'physicsDisableAngularThreshold': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'defaultDamping': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'inkEvaporation': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'gpsReference': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'lineScale': {
      'type': VRML.SFFloat,
      'defaultValue': 0.1
    },
    'dragForceScale': {
      'type': VRML.SFFloat,
      'defaultValue': 30.0
    },
    'dragTorqueScale': {
      'type': VRML.SFFloat,
      'defaultValue': 5.0
    },
    'randomSeed': {
      'type': VRML.SFInt32,
      'defaultValue': 0
    },
    'contactProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'northDirection': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 1.0,
        'z': 0.0
      }
    },
    'fast2d': {
      'type': VRML.SFString,
      'defaultValue': ''
    }
  },
  'Hinge2JointParameters': {
    'position': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'axis': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'anchor': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'minStop': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'maxStop': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'springConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'dampingConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'staticFriction': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'suspensionSpringConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'suspensionDampingConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'IndexedFaceSet': {
    'coord': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'normal': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'texCoord': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'solid': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'ccw': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'convex': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'normalPerVertex': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'coordIndex': {
      'type': VRML.MFInt32,
      'defaultValue': []
    },
    'normalIndex': {
      'type': VRML.MFInt32,
      'defaultValue': []
    },
    'texCoordIndex': {
      'type': VRML.MFInt32,
      'defaultValue': []
    },
    'creaseAngle': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'Background': {
    'skyColor': {
      'type': VRML.MFColor,
      'defaultValue': [
        {
          'r': 0.0,
          'g': 0.0,
          'b': 0.0
        }
      ]
    },
    'backUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'bottomUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'frontUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'leftUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'rightUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'topUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'backIrradianceUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'bottomIrradianceUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'frontIrradianceUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'leftIrradianceUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'rightIrradianceUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'topIrradianceUrl': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'luminosity': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    }
  },
  'SolidReference': {
    'solidName': {
      'type': VRML.SFString,
      'defaultValue': ''
    }
  },
  'Muscle': {
    'volume': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'startOffset': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'endOffset': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'color': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'castShadows': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'visible': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'maxRadius': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'PointSet': {
    'color': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'coord': {
      'type': VRML.SFNode,
      'defaultValue': null
    }
  },
  'DirectionalLight': {
    'ambientIntensity': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'color': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 1.0,
        'g': 1.0,
        'b': 1.0
      }
    },
    'direction': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': -1.0
      }
    },
    'intensity': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'on': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'castShadows': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'castLensFlares': {
      'type': VRML.SFBool,
      'defaultValue': false
    }
  },
  'Color': {
    'color': {
      'type': VRML.MFColor,
      'defaultValue': []
    }
  },
  'InertialUnit': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'inertial unit'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'noise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'xAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'yAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'zAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'lookupTable': {
      'type': VRML.MFVec3f,
      'defaultValue': []
    }
  },
  'Transform': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    }
  },
  'Solid': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'solid'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'CadShape': {
    'url': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'ccw': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'castShadows': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'isPickable': {
      'type': VRML.SFBool,
      'defaultValue': true
    }
  },
  'Pen': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'pen'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'inkColor': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 0.0,
        'g': 0.0,
        'b': 0.0
      }
    },
    'inkDensity': {
      'type': VRML.SFFloat,
      'defaultValue': 0.5
    },
    'leadSize': {
      'type': VRML.SFFloat,
      'defaultValue': 0.002
    },
    'maxDistance': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'write': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'RotationalMotor': {
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'rotational motor'
    },
    'acceleration': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'consumptionFactor': {
      'type': VRML.SFFloat,
      'defaultValue': 10.0
    },
    'controlPID': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 10.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'maxVelocity': {
      'type': VRML.SFFloat,
      'defaultValue': 10.0
    },
    'minPosition': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'maxPosition': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'maxTorque': {
      'type': VRML.SFFloat,
      'defaultValue': 10.0
    },
    'multiplier': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'sound': {
      'type': VRML.SFString,
      'defaultValue': 'webots://projects/default/worlds/sounds/rotational_motor.wav'
    },
    'muscles': {
      'type': VRML.MFNode,
      'defaultValue': []
    }
  },
  'JointParameters': {
    'position': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'axis': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0
      }
    },
    'minStop': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'maxStop': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'springConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'dampingConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'staticFriction': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'SliderJoint': {
    'jointParameters': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'device': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'endPoint': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'position': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'Material': {
    'ambientIntensity': {
      'type': VRML.SFFloat,
      'defaultValue': 0.2
    },
    'diffuseColor': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 0.8,
        'g': 0.8,
        'b': 0.8
      }
    },
    'emissiveColor': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 0.0,
        'g': 0.0,
        'b': 0.0
      }
    },
    'shininess': {
      'type': VRML.SFFloat,
      'defaultValue': 0.2
    },
    'specularColor': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 0.0,
        'g': 0.0,
        'b': 0.0
      }
    },
    'transparency': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'DistanceSensor': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'distance sensor'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'lookupTable': {
      'type': VRML.MFVec3f,
      'defaultValue': [
        {
          'x': 0.0,
          'y': 0.0,
          'z': 0.0
        },
        {
          'x': 0.1,
          'y': 1000.0,
          'z': 0.0
        }
      ]
    },
    'numberOfRays': {
      'type': VRML.SFInt32,
      'defaultValue': 1
    },
    'aperture': {
      'type': VRML.SFFloat,
      'defaultValue': 1.5708
    },
    'gaussianWidth': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'redColorSensitivity': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'LED': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'led'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'color': {
      'type': VRML.MFColor,
      'defaultValue': [
        {
          'r': 1.0,
          'g': 0.0,
          'b': 0.0
        }
      ]
    },
    'gradual': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'Cylinder': {
    'bottom': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'height': {
      'type': VRML.SFFloat,
      'defaultValue': 2.0
    },
    'radius': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'side': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'top': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'subdivision': {
      'type': VRML.SFInt32,
      'defaultValue': 36
    }
  },
  'PositionSensor': {
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'position sensor'
    },
    'noise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    }
  },
  'ImmersionProperties': {
    'fluidName': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'referenceArea': {
      'type': VRML.SFString,
      'defaultValue': 'immersed area'
    },
    'dragForceCoefficients': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'dragTorqueCoefficients': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'viscousResistanceForceCoefficient': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'viscousResistanceTorqueCoefficient': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'TrackWheel': {
    'position': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 1.0,
        'y': 0.0,
        'z': 0.0,
        'a': 1.5708
      }
    },
    'radius': {
      'type': VRML.SFFloat,
      'defaultValue': 0.1
    },
    'inner': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    }
  },
  'IndexedLineSet': {
    'coord': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'coordIndex': {
      'type': VRML.MFInt32,
      'defaultValue': []
    }
  },
  'Damping': {
    'linear': {
      'type': VRML.SFFloat,
      'defaultValue': 0.2
    },
    'angular': {
      'type': VRML.SFFloat,
      'defaultValue': 0.2
    }
  },
  'Focus': {
    'focalDistance': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'focalLength': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'maxFocalDistance': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'minFocalDistance': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'Radar': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'radar'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'minRange': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'maxRange': {
      'type': VRML.SFFloat,
      'defaultValue': 50.0
    },
    'horizontalFieldOfView': {
      'type': VRML.SFFloat,
      'defaultValue': 0.78
    },
    'verticalFieldOfView': {
      'type': VRML.SFFloat,
      'defaultValue': 0.1
    },
    'minAbsoluteRadialSpeed': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'minRadialSpeed': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'maxRadialSpeed': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'cellDistance': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'cellSpeed': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'rangeNoise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'speedNoise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'angularNoise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'antennaGain': {
      'type': VRML.SFFloat,
      'defaultValue': 20.0
    },
    'frequency': {
      'type': VRML.SFFloat,
      'defaultValue': 24.0
    },
    'transmittedPower': {
      'type': VRML.SFFloat,
      'defaultValue': 20.0
    },
    'minDetectableSignal': {
      'type': VRML.SFFloat,
      'defaultValue': -100.0
    },
    'occlusion': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'Appearance': {
    'material': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'texture': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'textureTransform': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'appearance'
    }
  },
  'Display': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'display'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'width': {
      'type': VRML.SFInt32,
      'defaultValue': 64
    },
    'height': {
      'type': VRML.SFInt32,
      'defaultValue': 64
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'windowPosition': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0
      }
    },
    'pixelSize': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    }
  },
  'Compass': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'compass'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'lookupTable': {
      'type': VRML.MFVec3f,
      'defaultValue': []
    },
    'xAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'yAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'zAxis': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'Fluid': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'fluid'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'density': {
      'type': VRML.SFFloat,
      'defaultValue': 1000.0
    },
    'viscosity': {
      'type': VRML.SFFloat,
      'defaultValue': 0.001
    },
    'streamVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    }
  },
  'Shape': {
    'appearance': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'geometry': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'castShadows': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'isPickable': {
      'type': VRML.SFBool,
      'defaultValue': true
    }
  },
  'Normal': {
    'vector': {
      'type': VRML.MFVec3f,
      'defaultValue': []
    }
  },
  'TextureTransform': {
    'center': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'scale': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0
      }
    },
    'translation': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0
      }
    }
  },
  'BallJoint': {
    'jointParameters': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'jointParameters2': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'jointParameters3': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'device': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'device2': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'device3': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'endPoint': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'position': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'position2': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'position3': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'HingeJoint': {
    'jointParameters': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'device': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'endPoint': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'position': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'Fog': {
    'color': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 1.0,
        'g': 1.0,
        'b': 1.0
      }
    },
    'visibilityRange': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'RangeFinder': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'range-finder'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'fieldOfView': {
      'type': VRML.SFFloat,
      'defaultValue': 0.785398
    },
    'width': {
      'type': VRML.SFInt32,
      'defaultValue': 64
    },
    'height': {
      'type': VRML.SFInt32,
      'defaultValue': 64
    },
    'near': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'minRange': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'maxRange': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'motionBlur': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'noise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'lens': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'spherical': {
      'type': VRML.SFBool,
      'defaultValue': false
    }
  },
  'SpotLight': {
    'ambientIntensity': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'attenuation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'beamWidth': {
      'type': VRML.SFFloat,
      'defaultValue': 1.570796
    },
    'color': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 1.0,
        'g': 1.0,
        'b': 1.0
      }
    },
    'cutOffAngle': {
      'type': VRML.SFFloat,
      'defaultValue': 0.785398
    },
    'direction': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': -1.0
      }
    },
    'intensity': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'location': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'on': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'radius': {
      'type': VRML.SFFloat,
      'defaultValue': 100.0
    },
    'castShadows': {
      'type': VRML.SFBool,
      'defaultValue': false
    }
  },
  'LensFlare': {
    'transparency': {
      'type': VRML.SFFloat,
      'defaultValue': 0.4
    },
    'scale': {
      'type': VRML.SFFloat,
      'defaultValue': 1.5
    },
    'bias': {
      'type': VRML.SFFloat,
      'defaultValue': -0.9
    },
    'dispersal': {
      'type': VRML.SFFloat,
      'defaultValue': 0.6
    },
    'samples': {
      'type': VRML.SFInt32,
      'defaultValue': 4
    },
    'haloWidth': {
      'type': VRML.SFFloat,
      'defaultValue': 0.4
    },
    'chromaDistortion': {
      'type': VRML.SFFloat,
      'defaultValue': 2.0
    },
    'blurIterations': {
      'type': VRML.SFInt32,
      'defaultValue': 2
    }
  },
  'Radio': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'radio'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'protocol': {
      'type': VRML.SFString,
      'defaultValue': '802.11b'
    },
    'txPowerMin': {
      'type': VRML.SFFloat,
      'defaultValue': -10.0
    },
    'txPowerMax': {
      'type': VRML.SFFloat,
      'defaultValue': 10.0
    },
    'address': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'rxSensitivity': {
      'type': VRML.SFFloat,
      'defaultValue': -100.0
    },
    'txPower': {
      'type': VRML.SFFloat,
      'defaultValue': 5.0
    },
    'frequency': {
      'type': VRML.SFFloat,
      'defaultValue': 2400000000.0
    },
    'channel': {
      'type': VRML.SFInt32,
      'defaultValue': 0
    },
    'bitrate': {
      'type': VRML.SFInt32,
      'defaultValue': 1000000
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'Billboard': {
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    }
  },
  'Capsule': {
    'bottom': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'height': {
      'type': VRML.SFFloat,
      'defaultValue': 2.0
    },
    'radius': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'side': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'top': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'subdivision': {
      'type': VRML.SFInt32,
      'defaultValue': 12
    }
  },
  'Cone': {
    'bottomRadius': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'height': {
      'type': VRML.SFFloat,
      'defaultValue': 2.0
    },
    'side': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'bottom': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'subdivision': {
      'type': VRML.SFInt32,
      'defaultValue': 12
    }
  },
  'Box': {
    'size': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 2.0,
        'y': 2.0,
        'z': 2.0
      }
    }
  },
  'Coordinate': {
    'point': {
      'type': VRML.MFVec3f,
      'defaultValue': []
    }
  },
  'Robot': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'robot'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'controller': {
      'type': VRML.SFString,
      'defaultValue': '<generic>'
    },
    'controllerArgs': {
      'type': VRML.MFString,
      'defaultValue': []
    },
    'customData': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'supervisor': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'synchronization': {
      'type': VRML.SFBool,
      'defaultValue': true
    },
    'battery': {
      'type': VRML.MFFloat,
      'defaultValue': []
    },
    'cpuConsumption': {
      'type': VRML.SFFloat,
      'defaultValue': 10.0
    },
    'selfCollision': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'window': {
      'type': VRML.SFString,
      'defaultValue': '<generic>'
    },
    'remoteControl': {
      'type': VRML.SFString,
      'defaultValue': '<none>'
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'data': {
      'type': VRML.SFString,
      'defaultValue': ''
    }
  },
  'Emitter': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'emitter'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'range': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'maxRange': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'aperture': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'channel': {
      'type': VRML.SFInt32,
      'defaultValue': 0
    },
    'baudRate': {
      'type': VRML.SFInt32,
      'defaultValue': -1
    },
    'byteSize': {
      'type': VRML.SFInt32,
      'defaultValue': 8
    },
    'bufferSize': {
      'type': VRML.SFInt32,
      'defaultValue': -1
    },
    'allowedChannels': {
      'type': VRML.MFInt32,
      'defaultValue': []
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'Receiver': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'receiver'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'aperture': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'channel': {
      'type': VRML.SFInt32,
      'defaultValue': 0
    },
    'baudRate': {
      'type': VRML.SFInt32,
      'defaultValue': -1
    },
    'byteSize': {
      'type': VRML.SFInt32,
      'defaultValue': 8
    },
    'bufferSize': {
      'type': VRML.SFInt32,
      'defaultValue': -1
    },
    'signalStrengthNoise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'directionNoise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'allowedChannels': {
      'type': VRML.MFInt32,
      'defaultValue': []
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'Brake': {
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'brake'
    }
  },
  'Viewpoint': {
    'fieldOfView': {
      'type': VRML.SFFloat,
      'defaultValue': 0.785398
    },
    'orientation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'position': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': -10.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'near': {
      'type': VRML.SFFloat,
      'defaultValue': 0.05
    },
    'far': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'exposure': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'follow': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'followSmoothness': {
      'type': VRML.SFFloat,
      'defaultValue': 0.5
    },
    'lensFlare': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'ambientOcclusionRadius': {
      'type': VRML.SFFloat,
      'defaultValue': 2.0
    },
    'bloomThreshold': {
      'type': VRML.SFFloat,
      'defaultValue': 21.0
    },
    'followOrientation': {
      'type': VRML.SFBool,
      'defaultValue': false
    }
  },
  'ElevationGrid': {
    'height': {
      'type': VRML.MFFloat,
      'defaultValue': []
    },
    'xDimension': {
      'type': VRML.SFInt32,
      'defaultValue': 0
    },
    'xSpacing': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'yDimension': {
      'type': VRML.SFInt32,
      'defaultValue': 0
    },
    'ySpacing': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    },
    'thickness': {
      'type': VRML.SFFloat,
      'defaultValue': 1.0
    }
  },
  'Track': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'track'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'device': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'textureAnimation': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0
      }
    },
    'animatedGeometry': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'geometriesCount': {
      'type': VRML.SFInt32,
      'defaultValue': 10
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'GPS': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'gps'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'accuracy': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'noiseCorrelation': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'speedNoise': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'speedResolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  },
  'Hinge2Joint': {
    'jointParameters': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'jointParameters2': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'device': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'device2': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'endPoint': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'position': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'position2': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    }
  },
  'HingeJointParameters': {
    'position': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'axis': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'anchor': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'minStop': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'maxStop': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'springConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'dampingConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'staticFriction': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'suspensionSpringConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'suspensionDampingConstant': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'suspensionAxis': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'stopERP': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'stopCFM': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    }
  },
  'ContactProperties': {
    'material1': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'material2': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'coulombFriction': {
      'type': VRML.MFFloat,
      'defaultValue': [
        1.0
      ]
    },
    'frictionRotation': {
      'type': VRML.SFVec2f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0
      }
    },
    'rollingFriction': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'bounce': {
      'type': VRML.SFFloat,
      'defaultValue': 0.5
    },
    'bounceVelocity': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'forceDependentSlip': {
      'type': VRML.MFFloat,
      'defaultValue': [
        0.0
      ]
    },
    'softERP': {
      'type': VRML.SFFloat,
      'defaultValue': 0.2
    },
    'softCFM': {
      'type': VRML.SFFloat,
      'defaultValue': 0.001
    },
    'bumpSound': {
      'type': VRML.SFString,
      'defaultValue': 'webots://projects/default/worlds/sounds/bump.wav'
    },
    'rollSound': {
      'type': VRML.SFString,
      'defaultValue': 'webots://projects/default/worlds/sounds/roll.wav'
    },
    'slideSound': {
      'type': VRML.SFString,
      'defaultValue': 'webots://projects/default/worlds/sounds/slide.wav'
    },
    'maxContactJoints': {
      'type': VRML.SFInt32,
      'defaultValue': 10
    }
  },
  'LightSensor': {
    'translation': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'rotation': {
      'type': VRML.SFRotation,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 1.0,
        'a': 0.0
      }
    },
    'scale': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 1.0,
        'y': 1.0,
        'z': 1.0
      }
    },
    'children': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'name': {
      'type': VRML.SFString,
      'defaultValue': 'light sensor'
    },
    'model': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'description': {
      'type': VRML.SFString,
      'defaultValue': ''
    },
    'contactMaterial': {
      'type': VRML.SFString,
      'defaultValue': 'default'
    },
    'immersionProperties': {
      'type': VRML.MFNode,
      'defaultValue': []
    },
    'boundingObject': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'physics': {
      'type': VRML.SFNode,
      'defaultValue': null
    },
    'locked': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'translationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.01
    },
    'rotationStep': {
      'type': VRML.SFFloat,
      'defaultValue': 0.261799387
    },
    'radarCrossSection': {
      'type': VRML.SFFloat,
      'defaultValue': 0.0
    },
    'recognitionColors': {
      'type': VRML.MFColor,
      'defaultValue': []
    },
    'lookupTable': {
      'type': VRML.MFVec3f,
      'defaultValue': [
        {
          'x': 0.0,
          'y': 0.0,
          'z': 0.0
        },
        {
          'x': 1.0,
          'y': 1000.0,
          'z': 0.0
        }
      ]
    },
    'colorFilter': {
      'type': VRML.SFColor,
      'defaultValue': {
        'r': 1.0,
        'g': 1.0,
        'b': 1.0
      }
    },
    'occlusion': {
      'type': VRML.SFBool,
      'defaultValue': false
    },
    'resolution': {
      'type': VRML.SFFloat,
      'defaultValue': -1.0
    },
    'linearVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    },
    'angularVelocity': {
      'type': VRML.SFVec3f,
      'defaultValue': {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0
      }
    }
  }
};
