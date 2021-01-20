// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import {World} from "./World.js"
import {getAncestor} from "./WbUtils.js"
import {WbTransform} from "./WbTransform.js"

class Selector {
  static select(id) {
    Selector.previousId = Selector.selectedId;
    Selector.selectedId = "n-1"; //in case we select nothing

    let node = World.instance.nodes.get('n' + id)
    if (typeof node === 'undefined'){
      Selector.preciseId = 'n' + id;
      return;
    }

    if (Selector.previousAncestor === getAncestor(node).id && (!Selector.local || Selector.preciseId !== 'n' + id)) {
      Selector.selectedId = Selector.firstSolidId(node);
      Selector.local = true;
    }
    else {
      Selector.selectedId = getAncestor(node).id;
      Selector.previousAncestor = Selector.selectedId;
      Selector.local = false;
    }

    Selector.preciseId = 'n' + id;
  }

  static checkIfParentisSelected(node) {
    let parent = World.instance.nodes.get(node.parent);
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
      if(node instanceof WbTransform && node.isSolid)
        return node.id;
      else if (typeof node.parent !== 'undefined' && typeof World.instance.nodes.get(node.parent) !== 'undefined')
        return Selector.firstSolidId(World.instance.nodes.get(node.parent));
    }
    return -1;
  }
}

Selector.selectedId = "n-1"
Selector.previousId = "n-1"
Selector.previousAncestor = "n-1"
Selector.local = false;
export {Selector}
