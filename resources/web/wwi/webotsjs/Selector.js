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

class Selector {
  static select(id) {
    Selector.previousId = Selector.selectedId;
    if ('n' + id === Selector.selectedId) {
        if (typeof World.instance.nodes.get(Selector.selectedId) !== 'undefined')
        Selector.selectedId = getAncestor(World.instance.nodes.get(Selector.selectedId)).id;
    }
    else
      Selector.selectedId = 'n' + id;
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
}

Selector.selectedId = "n-1"
Selector.previousId = "n-1"
export {Selector}
