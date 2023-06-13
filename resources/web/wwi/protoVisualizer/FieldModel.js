import {VRML} from './vrml_type.js';

export const FieldModel = {
  'Accelerometer': {
    'description': 'An Accelerometer node is used to measure the acceleration in a physics-based\\nsimulation. The acceleration is measured along the 3 axes (X, Y and Z) and is\\nexpressed in m/s^2. It is mostly used to measure the direction of the\\ngravity, but can be used for many other purposes.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Accelerometer.png',
    'fields': {
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
    }
  },
  'Altimeter': {
    'description': 'The Altimeter node can be used to determine the global altitude of a robot or of a robot part.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Altimeter.png',
    'fields': {
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
    }
  },
  'Appearance': {
    'description': 'The Appearance node specifies the visual properties of geometry\\nby defining the Material and texture nodes.\\n\\nNote: you should use the PBRAppearance node instead of the Appearance node\\nfor modern simulations with PBR (Physically Based Rendering).\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Appearance.png',
    'fields': {
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
    }
  },
  'Background': {
    'description': 'The Background node defines the background used for rendering the 3D world.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Background.png',
    'fields': {
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
    }
  },
  'BallJoint': {
    'description': 'A BallJoint node can be used to simulate a rotating motion with 3 DOF (ball and socket).\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/BallJoint.png',
    'fields': {
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
    }
  },
  'BallJointParameters': {
    'description': 'The BallJointParamaters node defines the parameters of a BallJoint node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/BallJointParameters.png',
    'fields': {
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
    }
  },
  'Billboard': {
    'description': 'A Billboard node contains children nodes that rotate and translate automatically to face the viewpoint.\\nIt is otherwise similar to a Group node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Billboard.png',
    'fields': {
      'children': {
        'type': VRML.MFNode,
        'defaultValue': []
      }
    }
  },
  'Box': {
    'description': 'The Box node specifies a rectangular parallelepiped box centred at (0,0,0) in\\nthe local coordinate system and aligned with the local coordinate axes.\\nBy default, the box measures 2 meters in each dimension, from -1 to +1.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Box.png',
    'fields': {
      'size': {
        'type': VRML.SFVec3f,
        'defaultValue': {
          'x': 2.0,
          'y': 2.0,
          'z': 2.0
        }
      }
    }
  },
  'Brake': {
    'description': 'A Brake node can be added in the \'device\' field of any joint to allow the\\nchange of the dampingConstant coefficient of this joint from a controller program.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Brake.png',
    'fields': {
      'name': {
        'type': VRML.SFString,
        'defaultValue': 'brake'
      }
    }
  },
  'CadShape': {
    'description': 'The CadShape node defines a geometry and appearance from a \'.dae\' or \'.obj\' file.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/CadShape.png',
    'fields': {
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
    }
  },
  'Camera': {
    'description': 'The Camera node is used to model an on-board camera.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Camera.png',
    'fields': {
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
    }
  },
  'Capsule': {
    'description': 'A Capsule is like a normal cylinder except it has half-sphere caps at its ends.\\nThis primitive features particularly fast and accurate collision detection.\\nThe cylinder\'s height, not counting the caps, is given by the height field.\\nThe Capsule is aligned along the local z-axis.\\nThe radius of the caps and of the cylinder itself is given by the radius field.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Capsule.png',
    'fields': {
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
    }
  },
  'Charger': {
    'description': 'The Charger node is used to model a special kind of battery charger for the robots.\\nWhen a robot gets close to a Charger, the robot\'s battery gets recharged.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Charger.png',
    'fields': {
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
    }
  },
  'Color': {
    'description': 'The Color node defines a set of RGB colors to be used by a parent node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Color.png',
    'fields': {
      'color': {
        'type': VRML.MFColor,
        'defaultValue': []
      }
    }
  },
  'Compass': {
    'description': 'A Compass node can be used to simulate 1, 2 and 3-axis digital compasses.\\nIt indicates the direction of the simulated magnetic north which is specified in the WorldInfo node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Compass.png',
    'fields': {
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
    }
  },
  'Cone': {
    'description': 'The Cone node specifies a cone which is centered in the local\\ncoordinate system and whose central axis is aligned with the local z-axis.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Cone.png',
    'fields': {
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
    }
  },
  'Connector': {
    'description': 'Connector nodes are used to simulate mechanical docking systems, or any other type of device that\\ncan dynamically create a rigid link with a similar device.\\nThe physical connection between two Connectors can be created and destroyed at run time by the robot controller program.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Connector.png',
    'fields': {
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
    }
  },
  'ContactProperties': {
    'description': 'The ContactProperties node specifies the properties for a contact between two specified materials.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/ContactProperties.png',
    'fields': {
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
    }
  },
  'Coordinate': {
    'description': 'The Coordinate node defines a set of 3D coordinates to be used in the \'coord\' field\\nof vertex-based geometry nodes including IndexedFaceSet and IndexedLineSet nodes.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Coordinate.png',
    'fields': {
      'point': {
        'type': VRML.MFVec3f,
        'defaultValue': []
      }
    }
  },
  'Cylinder': {
    'description': 'The Cylinder node specifies a cylinder centered at (0,0,0) in the local coordinate\\nsystem and with a central axis oriented along the local z-axis.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Cylinder.png',
    'fields': {
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
    }
  },
  'Damping': {
    'description': 'A Damping node can be used to add linear and rotational damping to a Solid node.\\nThe damping will slow down the motion of Solid nodes and allow them to come to rest.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Damping.png',
    'fields': {
      'linear': {
        'type': VRML.SFFloat,
        'defaultValue': 0.2
      },
      'angular': {
        'type': VRML.SFFloat,
        'defaultValue': 0.2
      }
    }
  },
  'DirectionalLight': {
    'description': 'The DirectionalLight node defines a light source that illuminates along rays parallel to a given 3-dimensional vector.\\nA DirectionalLight node is well-suited to model sunlight, for example.\\nIf you want to model a light bulb you should rather use a PointLight or a SpotLight node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/DirectionalLight.png',
    'fields': {
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
    }
  },
  'Display': {
    'description': 'The Display node allows to handle a 2D pixel array using simple\\nAPI functions, and render it into a 2D overlay on the 3D view,\\nor into a 2D texture of any Shape node, or both.\\nIt can model an embedded screen or it can display any graphical\\ninformation such as graphs, text, robot trajectory, filtered camera\\nimages and so on.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Display.png',
    'fields': {
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
    }
  },
  'DistanceSensor': {
    'description': 'The DistanceSensor node can be used to model an ultrasound sonar, an infra-red sensor,\\na single-ray laser or any type of device that measures the distance to objects.\\nTo model a Lidar sensor, you should rather use a Lidar node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/DistanceSensor.png',
    'fields': {
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
    }
  },
  'ElevationGrid': {
    'description': 'The ElevationGrid node specifies a uniform rectangular grid of varying height in the Y=0 plane of the local coordinate system.\\nThe geometry is described by a scalar array of height values that specify the height of a surface above each point of the grid.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/ElevationGrid.png',
    'fields': {
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
    }
  },
  'Emitter': {
    'description': 'The Emitter node is used to model a radio, or infra-red emitter.\\nIt can be used to send data packets to Receiver nodes (onboard other robots).\\nAn Emitter cannot receive data: bidirectional communication requires two Emitter/Receiver pairs.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Emitter.png',
    'fields': {
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
    }
  },
  'Fluid': {
    'description': 'A Fluid node can be used to represent a collection of fluid volumes where hydrostatic and hydrodynamic forces apply.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Fluid.png',
    'fields': {
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
    }
  },
  'Focus': {
    'description': 'A Focus node allows to specify a controllable focus device for a Camera node.\\nIt should be added in the focus field of a Camera node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Focus.png',
    'fields': {
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
    }
  },
  'Fog': {
    'description': 'The Fog node provides a way to simulate atmospheric effects by blending objects with\\nthe color specified by the color field based on the distances of the various objects from the camera.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Fog.png',
    'fields': {
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
    }
  },
  'GPS': {
    'description': 'The GPS node can be used to determine the global position of a robot or of a robot part.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/GPS.png',
    'fields': {
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
    }
  },
  'Group': {
    'description': 'A Group node contains children nodes without introducing a new transformation.\\nIt is equivalent to a Transform node without the \'translation\' and \'rotation\' fields.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Group.png',
    'fields': {
      'children': {
        'type': VRML.MFNode,
        'defaultValue': []
      }
    }
  },
  'Gyro': {
    'description': 'A Gyro node measures the angular velocity about 3 orthogonal axes (X, Y and Z).\\nThe output is in rad/s. The Gyro node is mostly used for balance control.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Gyro.png',
    'fields': {
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
    }
  },
  'Hinge2Joint': {
    'description': 'A Hinge2Joint can be used to simulate a combination of two rotating motions along axes which intersect.\\nIt is equivalent to two HingeJoint nodes but it spares the creation of an intermediate solid and is therefore more stable.\\nSpring and damping behavior can be specified.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Hinge2Joint.png',
    'fields': {
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
    }
  },
  'Hinge2JointParameters': {
    'description': 'DEPRECATED\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Hinge2JointParameters.png',
    'fields': {
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
    }
  },
  'HingeJoint': {
    'description': 'A HingeJoint can be used to simulate a rotating motion.\\nSpring and damping behavior can be specified.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/HingeJoint.png',
    'fields': {
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
    }
  },
  'HingeJointParameters': {
    'description': 'A HingeJointParameters defines the parameters of a HingeJoint node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/HingeJointParameters.png',
    'fields': {
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
    }
  },
  'ImageTexture': {
    'description': 'The ImageTexture node defines a texture map by specifying an image file and general parameters for mapping to geometry.\\nThe supported file formats are PNG and JPEG.\\nThe image dimensions (width and height) should be a power of two, e.g., 256 x 1024 pixels.\\nAll the geometry primitive nodes can be textured.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/ImageTexture.png',
    'fields': {
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
    }
  },
  'ImmersionProperties': {
    'description': 'The ImmersionProperties node specifies the properties for a partially or fully immersed Solid node in a Fluid node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/ImmersionProperties.png',
    'fields': {
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
    }
  },
  'IndexedFaceSet': {
    'description': 'The IndexedFaceSet node represents a 3D shape formed by constructing faces (polygons) from vertices listed in the coord field.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/IndexedFaceSet.png',
    'fields': {
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
    }
  },
  'IndexedLineSet': {
    'description': 'The IndexedLineSet node represents a 3D geometry formed by constructing polylines from 3D vertices.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/IndexedLineSet.png',
    'fields': {
      'coord': {
        'type': VRML.SFNode,
        'defaultValue': null
      },
      'coordIndex': {
        'type': VRML.MFInt32,
        'defaultValue': []
      }
    }
  },
  'InertialUnit': {
    'description': 'The InertialUnit node simulates an Inertial Measurement Unit (IMU).\\nThe InertialUnit node computes and returns the roll, pitch and yaw angles of the\\nrobot with respect to the global coordinate system defined in the WorldInfo node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/InertialUnit.png',
    'fields': {
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
    }
  },
  'JointParameters': {
    'description': 'The JointParameters node defines the parameters of a mechanical joint.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/JointParameters.png',
    'fields': {
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
    }
  },
  'LED': {
    'description': 'The LED node can be used to model a light emitting diode (LED) that can be controlled by the robot.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/LED.png',
    'fields': {
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
    }
  },
  'Lens': {
    'description': 'A Lens node allows to specify image distortion due to the camera lens\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Lens.png',
    'fields': {
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
    }
  },
  'LensFlare': {
    'description': 'The LensFlare node can be inserted in the \'lensFlare\' field of any Viewpoint or Camera node\\nIt defines the properties of the lens flare created by every light whose \'castLensFlares\' field is TRUE.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/LensFlare.png',
    'fields': {
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
    }
  },
  'Lidar': {
    'description': 'The Lidar node is used to model an on-board lidar.\\nA lidar is used to measure the distance to obstacles.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Lidar.png',
    'fields': {
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
    }
  },
  'LightSensor': {
    'description': 'A LightSensor node can be used to model a phototransistor, a photodiode or any type\\nof device that measures the irradiance of light on its surface.\\nA LightSensor node detects the light emitted by PointLight, SpotLight and DirectionalLight nodes.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/LightSensor.png',
    'fields': {
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
  },
  'LinearMotor': {
    'description': 'A LinearMotor node can be used to generate a translation motion along the axis of a Slider node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/LinearMotor.png',
    'fields': {
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
    }
  },
  'Material': {
    'description': 'The Material node specifies the surface material properties of the associated geometry nodes.\\nThis is essentially used for the visual appearance of objects, but in addition,\\nCamera and infra-red DistanceSensor nodes are sensitive to the color of objects.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Material.png',
    'fields': {
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
    }
  },
  'Mesh': {
    'description': 'The Mesh node represents a 3D geometry defined in an external file.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Mesh.png',
    'fields': {
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
    }
  },
  'Microphone': {
    'description': '',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Microphone.png',
    'fields': {
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
    }
  },
  'Muscle': {
    'description': 'The Muscle node graphically represents an artificial muscle.\\nThis is a child node of LinearMotor and RotationalMotor nodes.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Muscle.png',
    'fields': {
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
    }
  },
  'Normal': {
    'description': 'The Normal node specifies a set of 3D vectors used by vertex-based geometry nodes to map normal to vertices or faces.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Normal.png',
    'fields': {
      'vector': {
        'type': VRML.MFVec3f,
        'defaultValue': []
      }
    }
  },
  'PBRAppearance': {
    'description': 'The PBRAppearance node specifies the visual properties of geometry using a\\nphysically based shading model and several textures to define material\\nproperties.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/PBRAppearance.png',
    'fields': {
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
    }
  },
  'Pen': {
    'description': 'A Pen node can be used to model a pen attached to a mobile robot.\\nIt can draw the trajectory of the robot on a textured ground.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Pen.png',
    'fields': {
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
    }
  },
  'Physics': {
    'description': 'A Physics node allows to specify the physical properties (density, mass, friction coefficient, etc.) of the solid object that contains it.\\nWhen a Physics node is added to a solid object, this indicates that the dynamics (forces, gravity, friction, inertia, etc.) of the object\\nmust be simulated, otherwise only the kinematics is simulated.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Physics.png',
    'fields': {
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
    }
  },
  'Plane': {
    'description': 'A Plane node defines an infinite 3D plane at y=0 in the local coordinate system.\\nThe Plane can be used in a boundingObject to simulate boundaries, e.g., the floor or infinite walls.\\nThe size field is used only for the visual representation, when collision detection is concerned the Plane is infinite.\\nThe Plane is a strictly static object, it cannot be placed in the boundingObject of a dynamic (Physics-based) object.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Plane.png',
    'fields': {
      'size': {
        'type': VRML.SFVec2f,
        'defaultValue': {
          'x': 1.0,
          'y': 1.0
        }
      }
    }
  },
  'PointLight': {
    'description': 'The PointLight node specifies a light source that emits light equally in all directions.\\nThe emitted light can be detected by a LightSensor node.\\nPutting a PointLight onboard a robot allows the PointLight to move with the robot.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/PointLight.png',
    'fields': {
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
    }
  },
  'PointSet': {
    'description': 'The PointSet node represents a 3D geometry formed a set of points\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/PointSet.png',
    'fields': {
      'color': {
        'type': VRML.SFNode,
        'defaultValue': null
      },
      'coord': {
        'type': VRML.SFNode,
        'defaultValue': null
      }
    }
  },
  'Pose': {
    'description': 'The Pose node is a grouping node that defines a coordinate system for its children that is\\nrelative to the coordinate system of its parent.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Pose.png',
    'fields': {
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
    }
  },
  'PositionSensor': {
    'description': 'A PositionSensor allows a robot controller to read the position of a joint with respect to its main axis.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/PositionSensor.png',
    'fields': {
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
    }
  },
  'Propeller': {
    'description': 'The Propeller node is used to model a motorized helix propeller.\\nIt can be used to propel underwater robots, flying robots, floating robots or even wheeled robots.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Propeller.png',
    'fields': {
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
    }
  },
  'Radar': {
    'description': 'The Radar node is used to model a radar device, commonly found in automobiles.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Radar.png',
    'fields': {
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
    }
  },
  'Radio': {
    'description': 'Experimental Radio node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Radio.png',
    'fields': {
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
    }
  },
  'RangeFinder': {
    'description': 'The RangeFinder node is used to model an on-board range finder.\\nA range finder is used to measure the distance to obstacles.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/RangeFinder.png',
    'fields': {
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
    }
  },
  'Receiver': {
    'description': 'A Receiver node models a radio or infra-red receiver.\\nIt can be used to receive data packets emitted by Emitter nodes (onboard other robots).\\nA Receiver cannot emit data: bidirectional communication requires two Emitter/Receiver pairs.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Receiver.png',
    'fields': {
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
    }
  },
  'Recognition': {
    'description': 'A Recognition node allows to add object recognition capability to a Camera node.\\nIt should be added in the recognition field of a Camera node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Recognition.png',
    'fields': {
      'maxRange': {
        'type': VRML.SFFloat,
        'defaultValue': 100.0
      },
      'maxObjects': {
        'type': VRML.SFInt32,
        'defaultValue': -1
      },
      'occlusion': {
        'type': VRML.SFInt32,
        'defaultValue': 1
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
    }
  },
  'Robot': {
    'description': 'The Robot node is a generic type of robot.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Robot.png',
    'fields': {
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
    }
  },
  'RotationalMotor': {
    'description': 'The RotationalMotor node can be used by a robot controller program to generate a rotational motion around its hinge axis.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/RotationalMotor.png',
    'fields': {
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
    }
  },
  'Shape': {
    'description': 'A Shape node is a visual objects that includes both an \'appearance\' and a \'geometry\'.\\nA Shape can also be used in a \'boundingObject\', in this case, only its \'geometry\' field is taken into account.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Shape.png',
    'fields': {
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
    }
  },
  'Skin': {
    'description': 'The Skin node is used in order to add graphical skin animation to a robot.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Skin.png',
    'fields': {
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
    }
  },
  'SliderJoint': {
    'description': 'A SliderJoint can be used to simulate a translation motion.\\nSpring and damping behavior can be specified.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/SliderJoint.png',
    'fields': {
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
    }
  },
  'Slot': {
    'description': 'A Slot can be connected to another slot if the fields \'type\' are the same.\\nIt is possible to set the gender of the Slot by ending the type string with a \'+\' or \'-\'\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Slot.png',
    'fields': {
      'type': {
        'type': VRML.SFString,
        'defaultValue': ''
      },
      'endPoint': {
        'type': VRML.SFNode,
        'defaultValue': null
      }
    }
  },
  'Solid': {
    'description': 'A Solid node can be used to represent objects in the simulated environment (e.g. obstacles, walls, ground, robot parts, etc.).\\nSolid nodes can be collision detected (boundingObject) and therefore can prevent objects from intersecting.\\nIn addition, Solid nodes can have an optional Physics node that allow them to be simulated with the physics engine.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Solid.png',
    'fields': {
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
    }
  },
  'SolidReference': {
    'description': 'A SolidReference node is a reference to an existing Solid defined earlier.\\nIt can be used inside the \'endPoint\' field of a Joint to close a mechanical loop.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/SolidReference.png',
    'fields': {
      'solidName': {
        'type': VRML.SFString,
        'defaultValue': ''
      }
    }
  },
  'Speaker': {
    'description': 'The Speaker node is used to model a loudspeaker device.\\nIt can be used to playback wav sound files as well as text-to-speech.\\nThe sounds are localized in the 3D space and rendered at the main viewpoint in stereo.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Speaker.png',
    'fields': {
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
    }
  },
  'Sphere': {
    'description': 'The Sphere node specifies a sphere primitive centered at (0,0,0) in the local coordinate system.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Sphere.png',
    'fields': {
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
    }
  },
  'SpotLight': {
    'description': 'The SpotLight node defines a light source that emits light from a specific point along a specific direction vector and constrained within a specific angle.\\nThe light emitted by a SpotLight (or other light sources) can be detected by a LightSensor node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/SpotLight.png',
    'fields': {
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
    }
  },
  'TextureCoordinate': {
    'description': 'The TextureCoordinate node specifies a set of 2D texture coordinates used by vertex-based geometry\\nnodes to map textures to vertices.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/TextureCoordinate.png',
    'fields': {
      'point': {
        'type': VRML.MFVec2f,
        'defaultValue': []
      }
    }
  },
  'TextureTransform': {
    'description': 'The TextureTransform node defines a 2D transformation that is applied to texture coordinates.\\nThis node affects the way textures coordinates are applied to the geometric surface.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/TextureTransform.png',
    'fields': {
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
    }
  },
  'TouchSensor': {
    'description': 'A TouchSensor can be used to measure contact force (\'force\', \'force-3d\') or simply detect collisions (\'bumper\').\\nIt is critical that \'boundingObject\' of a TouchSensor is defined and placed appropriately.\\nRefer to the Webots reference manual for more information on this.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/TouchSensor.png',
    'fields': {
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
    }
  },
  'Track': {
    'description': 'The Track node can be used to simulate tracks of tank robots or conveyor belts.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Track.png',
    'fields': {
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
    }
  },
  'TrackWheel': {
    'description': 'A TrackWheel node can be used to simulate a wheel that it is part of a track system.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/TrackWheel.png',
    'fields': {
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
    }
  },
  'Transform': {
    'description': 'The Transform node is a grouping node that defines a coordinate system for its children that is\\nrelative to the coordinate system of its parent.\\nThe \'scale\' field of a Transform node can be adjusted only in a graphical context and not in a \'boundingObject\' context.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Transform.png',
    'fields': {
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
    }
  },
  'VacuumGripper': {
    'description': 'VacuumGripper nodes are used to simulate vacuum suction systems.\\nThe physical connection with a Solid can be created and destroyed at run time by the robot controller program.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/VacuumGripper.png',
    'fields': {
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
      'children': {
        'type': VRML.MFNode,
        'defaultValue': []
      },
      'name': {
        'type': VRML.SFString,
        'defaultValue': 'vacuum gripper'
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
      'isOn': {
        'type': VRML.SFBool,
        'defaultValue': false
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
    }
  },
  'Viewpoint': {
    'description': 'The Viewpoint node defines the viewing parameters for the main 3D view of Webots.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Viewpoint.png',
    'fields': {
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
    }
  },
  'WorldInfo': {
    'description': 'The WorldInfo node provides general information about the simulated world,\\ne.g., gravity, the physics time step, ODE global parameters, plugins, etc.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/WorldInfo.png',
    'fields': {
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
    }
  },
  'Zoom': {
    'description': 'A Zoom node allows to specify a controllable zoom device for a Camera node.\\nIt should be added in the zoom field of a Camera node.\\n',
    'icon': 'https://raw.githubusercontent.com/cyberbotics/webots/released/resources/nodes/icons/Zoom.png',
    'fields': {
      'maxFieldOfView': {
        'type': VRML.SFFloat,
        'defaultValue': 1.5
      },
      'minFieldOfView': {
        'type': VRML.SFFloat,
        'defaultValue': 0.5
      }
    }
  }
};
