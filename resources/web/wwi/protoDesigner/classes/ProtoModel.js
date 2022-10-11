import {VRML} from './utility/utility.js';

export const ProtoModel = {
  "PandaHand": {
    "url": "this-is-a-url",
    "base-type": "Solid",
    "need-robot-ancestor": true,
    "parameters": {
      "translation": {
        "type": VRML.SFVec3f,
        "defaultValue": "0 0 0"
      },
      "rotation": {
        "type": VRML.SFRotation,
        "defaultValue": "0 0 1 0"
      },
      "name": {
        "type": VRML.SFString,
        "defaultValue": "panda hand"
      },
    }
  }
};