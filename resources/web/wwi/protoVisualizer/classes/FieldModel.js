import {VRML} from './constants.js';

export const FieldModel = {
  'Appearance': {
    'supported': {'material': VRML.SFNode, 'texture': VRML.SFNode, 'textureTransform': VRML.SFNode},
    'unsupported': {'name': VRML.SFString}
  },
  'Accelerometer': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'lookupTable': VRML.MFVec3f, 'xAxis': VRML.SFBool, 'yAxis': VRML.SFBool, 'zAxis': VRML.SFBool, 'resolution': VRML.SFFloat}
  },
  'Altimeter': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'accuracy': VRML.SFFloat, 'resolution': VRML.SFFloat}
  },
  'Background': {
    'supported': {},
    'unsupported': {}
  },
  'Billboard': {
    'supported': {'children': VRML.MFNode},
    'unsupported': {}
  },
  'BallJoint': {
    'supported': {'endPoint': VRML.SFNode, 'jointParameters': VRML.SFNode, 'jointParameters2': VRML.SFNode, 'jointParameters3': VRML.SFNode, 'device': VRML.MFNode, 'device2': VRML.MFNode, 'device3': VRML.MFNode},
    'unsupported': {}
  },
  'BallJointParameters': {
    'supported': {'position': VRML.SFFloat, 'axis': VRML.SFVec3f, 'anchor': VRML.SFVec3f, 'minStop': VRML.SFFloat, 'maxStop': VRML.SFFloat},
    'unsupported': {'springConstant': VRML.SFFloat, 'dampingConstant': VRML.SFFloat, 'staticFriction': VRML.SFFloat, 'suspensionSpringConstant': VRML.SFFloat, 'suspensionDampingConstant': VRML.SFFloat, 'suspensionAxis': VRML.SFVec3f, 'stopCFM': VRML.SFFloat, 'stopERP': VRML.SFFloat}  },
  'Box': {
    'supported': {'size': VRML.SFVec3f},
    'unsupported': {}
  },
  'Brake': {
    'supported': {'name': VRML.SFString},
    'unsupported': {}
  },
  'CadShape': {
    'supported': {'url': VRML.MFString, 'ccw': VRML.SFBool, 'castShadows': VRML.SFBool, 'isPickable': VRML.SFBool},
    'unsupported': {}
  },
  'Camera': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'fieldOfView': VRML.SFFloat, 'width': VRML.SFInt32, 'height': VRML.SFInt32, 'spherical': VRML.SFBool, 'near': VRML.SFFloat, 'far': VRML.SFFloat, 'exposure': VRML.SFFloat, 'antiAliasing': VRML.SFBool, 'ambientOcclustionRadius': VRML.SFFloat, 'bloomThreshold': VRML.SFFloat, 'motionBlur': VRML.SFFloat, 'noise': VRML.SFFloat, 'noiseMaskURL': VRML.SFString, 'lens': VRML.SFNode, 'focus': VRML.SFNode, 'zoom': VRML.SFNode, 'recognition': VRML.SFNode, 'lensFlare': VRML.SFNode}
  },
  'Capsule': {
    'supported': {'bottom': VRML.SFBool, 'height': VRML.SFFloat, 'radius': VRML.SFFloat, 'side': VRML.SFBool, 'top': VRML.SFBool, 'subdivision': VRML.SFInt32},
    'unsupported': {}
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
  'TouchSensor': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode},
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'type': VRML.SFString, 'lookupTable': VRML.MFVec3f, 'resolution': VRML.SFFloat}
  },
  'Track': {
    'supported': {'translation': VRML.SFVec3f, 'rotation': VRML.SFRotation, 'scale': VRML.SFVec3f, 'children': VRML.MFNode, 'name': VRML.SFString, 'boundingObject': VRML.SFNode, 'device': VRML.MFNode, 'animatedGeometry': VRML.SFNode, 'geometriesCount': VRML.SFInt32 },
    'unsupported': {'model': VRML.SFString, 'description': VRML.SFString, 'contactMaterial': VRML.SFString, 'immersionProperties': VRML.MFNode, 'physics': VRML.SFNode, 'locked': VRML.SFBool, 'radarCrossSection': VRML.SFFloat, 'recognitionColors': VRML.MFColor, 'translationStep': VRML.SFFloat, 'rotationStep': VRML.SFFloat, 'linearVelocity': VRML.SFVec3f, 'angularVelocity': VRML.SFVec3f, 'textureAnimation': VRML.SFVec2f}
  },
  'TrackWheel': {
    'supported': {'children': VRML.MFNode, 'position': VRML.SFVec2f, 'rotation': VRML.SFRotation, 'radius': VRML.SFFloat, 'inner': VRML.SFBool},
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
