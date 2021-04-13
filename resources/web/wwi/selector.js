import {getAncestor} from './nodes/utils/wbUtils.js';
import {WbTransform} from './nodes/wbTransform.js';
import {WbWorld} from './nodes/wbWorld.js';

class Selector {
  static select(id) {
    Selector.previousId = Selector.selectedId;
    Selector.selectedId = 'n-1'; // in case we select nothing

    const node = WbWorld.instance.nodes.get('n' + id);
    if (typeof node === 'undefined') {
      Selector.preciseId = 'n' + id;
      Selector.previousAncestor = 'n-1';
      return;
    }

    if (Selector.previousAncestor === getAncestor(node).id && (!Selector.local || Selector.preciseId !== 'n' + id)) {
      Selector.selectedId = Selector.firstSolidId(node);
      Selector.local = true;
    } else {
      Selector.selectedId = getAncestor(node).id;
      Selector.previousAncestor = Selector.selectedId;
      Selector.local = false;
    }

    Selector.preciseId = 'n' + id;
  }

  static checkIfParentisSelected(node) {
    const parent = WbWorld.instance.nodes.get(node.parent);
    if (typeof parent !== 'undefined') {
      if (Selector.selectedId === parent.id)
        return true;
      else if (typeof parent.parent !== 'undefined')
        return Selector.checkIfParentisSelected(parent);
    }

    return false;
  }

  static firstSolidId(node) {
    if (typeof node !== 'undefined') {
      if (node instanceof WbTransform && node.isSolid)
        return node.id;
      else if (typeof node.parent !== 'undefined' && typeof WbWorld.instance.nodes.get(node.parent) !== 'undefined')
        return Selector.firstSolidId(WbWorld.instance.nodes.get(node.parent));
    }
    return -1;
  }

  static reset() {
    Selector.selectedId = 'n-1';
    Selector.previousId = 'n-1';
    Selector.previousAncestor = 'n-1';
    Selector.local = false;
  }
}

Selector.selectedId = 'n-1';
Selector.previousId = 'n-1';
Selector.previousAncestor = 'n-1';
Selector.local = false;
export {Selector};
