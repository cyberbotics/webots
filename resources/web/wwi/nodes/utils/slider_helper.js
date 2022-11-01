import WbSolid from '../WbSolid.js';
import WbTransform from '../WbTransform.js';
import WbWorld from './../WbWorld.js';
import WbJoint from './../WbJoint.js';

export function listJoints() {
  const nodes = WbWorld.nodes;
  for (let i = 0; i < nodes.length; i++) {
    if (nodes[i] instanceof WbJoint)
      console.log('Joint found');
  }
}
