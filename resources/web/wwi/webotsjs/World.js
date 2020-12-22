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

class World {
  constructor () {
    //We can keep the viewpoint outside of the children list because
    //it is independant from Webots during the simulation so we don't need to retrieve it to apply update
    this.viewpoint = undefined;
    //Only the top level nodes are represented here
    this.sceneTree = [];

    //All the nodes are included here so it is easier to retrieve them for update
    //dict from id to node
    this.nodes = new Map();
    this.defUse = {}
    World.instance = this;
  }
}

World.instance = undefined;
export {World}
