import WbWorld from './WbWorld.js';
import { findUpperTransform } from './utils/node_utilities.js';

export default class WbBaseNode {
  #nodeType;
  #upperTransformFirstTimeSearch;
  #upperTransform;
  constructor(id) {
    this.id = id;

    this.wrenObjectsCreatedCalled = false;
    this.isPreFinalizedCalled = false;
    this.isPostFinalizedCalled = false;

    this.useList = [];

    this.#upperTransformFirstTimeSearch = true;
  }

  get nodeType() { return undefined; }

  get upperTransform() {
    if (this.#upperTransformFirstTimeSearch) {
      this.#upperTransform = findUpperTransform(this);
      if (this.wrenObjectsCreatedCalled)
        this.#upperTransformFirstTimeSearch = false;
    }

    return this.#upperTransform;
  }

  createWrenObjects() {
    this.wrenObjectsCreatedCalled = true;

    if (typeof this.parent !== 'undefined')
      this.wrenNode = WbWorld.instance.nodes.get(this.parent).wrenNode;
    else
      this.wrenNode = _wr_scene_get_root(_wr_scene_get_instance());
  }

  delete() {
    WbWorld.instance.nodes.delete(this.id);
  }

  unfinalize() {
    this.isPreFinalizedCalled = false;
    this.wrenObjectsCreatedCalled = false;
    this.isPostFinalizedCalled = false;
    for (const useId of this.useList) { // notify USE nodes to do the same
      const useNode = WbWorld.instance.nodes.get(useId);
      useNode?.unfinalize();
    }
  }

  finalize() {
    if (!this.isPreFinalizedCalled)
      this.preFinalize();

    if (!this.wrenObjectsCreatedCalled)
      this.createWrenObjects();

    if (!this.isPostFinalizedCalled)
      this.postFinalize();

    for (const useId of this.useList) {
      const useNode = WbWorld.instance.nodes.get(useId);
      useNode.finalize();
    }
  }

  preFinalize() {
    this.isPreFinalizedCalled = true;
  }

  postFinalize() {
    this.isPostFinalizedCalled = true;
  }

  boundingSphere() {}
}
