// Copyright 1996-2021 Cyberbotics Ltd.
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

class WbWorld {
  constructor() {
    this.hasFog = false;
    this.basicTimeStep = 32;
    this.coordinateSystem = 'NUE';
    this.upVector = glm.vec3(0,1,0);
    // Only the top level nodes are represented here
    this.sceneTree = [];
    // All the nodes are included here so it is easier to retrieve them for update
    // map from id to node
    this.nodes = new Map();
  }

  static init() {
    WbWorld.instance = new WbWorld();
  }

  static computeUpVector() {
    WbWorld.instance.upVector = glm.vec3(WbWorld.instance.coordinateSystem[0] === 'U' ? 1 : 0, WbWorld.instance.coordinateSystem[1] === 'U' ? 1 : 0, WbWorld.instance.coordinateSystem[2] === 'U' ? 1 : 0);
  }
}

WbWorld.instance;
export {WbWorld};
