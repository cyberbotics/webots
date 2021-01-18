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
    let node = World.instance.nodes.get('n' + id)

    if (typeof node === 'undefined')
      return;

    Selector.previousId = Selector.selectedId;

    if ('n' + id === Selector.preciseId)
      Selector.selectedId = Selector.firstSolidId(node)
    else
      Selector.selectedId = getAncestor(node).id;


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
Selector.preciseId = "n-1"
export {Selector}
