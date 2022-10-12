import {VRML} from './utility/utility.js';

export const ProtoList = {
  "PandaHand": {
    "url": "this-is-a-url",
    "base-type": "Solid",
    "need-robot-ancestor": false,
    "parameters": {
      "translation": {
        "type": VRML.SFVec3f,
        "defaultValue": {"x": 0, "y": 0, "z": 1}
      },
      "rotation": {
        "type": VRML.SFRotation,
        "defaultValue": {"x": 0, "y": 0, "z": 1, "a": 0}
      },
      "name": {
        "type": VRML.SFString,
        "defaultValue": "panda hand"
      },
    }
  },
  "Panda": {
    "url": "this-is-another-url",
    "base-type": "Solid",
    "need-robot-ancestor": false,
    "parameters": {
      "translation": {
        "type": VRML.SFVec3f,
        "defaultValue": {"x": 0, "y": 0, "z": 1}
      },
      "rotation": {
        "type": VRML.SFRotation,
        "defaultValue": {"x": 0, "y": 0, "z": 1, "a": 0}
      },
      "name": {
        "type": VRML.SFString,
        "defaultValue": "panda"
      },
      "controller": {
        "type": VRML.SFString,
        "defaultValue": "<generic>"
      },
      "controllerArgs": {
        "type": VRML.MFString,
        "defaultValue": []
      },
      "window": {
        "type": VRML.SFString,
        "defaultValue": "<generic>"
      },
      "customData": {
        "type": VRML.SFString,
        "defaultValue": ""
      },
      "supervisor": {
        "type": VRML.SFBool,
        "defaultValue": false
      },
      "synchronization": {
        "type": VRML.SFBool,
        "defaultValue": true
      },
      "selfCollision": {
        "type": VRML.SFBool,
        "defaultValue": false
      },
      "endEffectorSlot": {
        "type": VRML.SFNode,
        "defaultValue": [{"node_name": "PandaHand", "fields": {}}]
      }
    }
  }
};