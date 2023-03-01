import WbSolid from '../WbSolid.js';
import WbTransform from '../WbTransform.js';
import WbWorld from '../WbWorld.js';
import {WbNodeType} from '../wb_node_type.js';

export function findUpperTransform(node) {
  if (typeof node === 'undefined')
    return undefined;

  let n = WbWorld.instance.nodes.get(node.parent);
  while (typeof n !== 'undefined') {
    if (n instanceof WbTransform)
      return n;

    n = WbWorld.instance.nodes.get(n.parent);
  }

  return undefined;
}

export function findUpperShape(node) {
  if (typeof node === 'undefined')
    return undefined;

  let n = WbWorld.instance.nodes.get(node.parent);
  while (typeof n !== 'undefined') {
    if (n.nodeType === WbNodeType.WB_NODE_SHAPE)
      return n;

    n = WbWorld.instance.nodes.get(n.parent);
  }

  return undefined;
}

export function nodeIsInBoundingObject(node) {
  if (typeof node === 'undefined' || typeof node.parent === 'undefined')
    return false;

  const parent = WbWorld.instance.nodes.get(node.parent);
  if (typeof parent !== 'undefined') {
    if (parent instanceof WbSolid && typeof parent.boundingObject !== 'undefined')
      return parent.boundingObject === node;
    else if (typeof parent.parent !== 'undefined')
      return nodeIsInBoundingObject(parent);
  }

  return false;
}
