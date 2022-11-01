import WbBillboard from '../WbBillboard.js';
import WbSolid from '../WbSolid.js';
import WbTransform from '../WbTransform.js';
import WbWorld from '../WbWorld.js';

export function findUpperTransform(node) {
  if (typeof node === 'undefined')
    return undefined;

  let n = WbWorld.instance.nodes.get(node.parent);
  while (typeof n !== 'undefined') {
    if (n instanceof WbTransform)
      return n;
    else
      n = n.parent;
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

export function isDescendantOfBillboard(node) {
  while (typeof node !== 'undefined') {
    if (node instanceof WbBillboard)
      return true;

    node = WbWorld.instance.nodes.get(node.parent);
  }

  return false;
}
