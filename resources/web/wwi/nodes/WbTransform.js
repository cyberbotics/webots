import WbGroup from './WbGroup.js';
import WbWorld from './WbWorld.js';

import {getAnId} from './utils/id_provider.js';
import {findUpperTransform} from './utils/node_utilities.js';
import WbMatrix4 from './utils/WbMatrix4.js';

export default class WbTransform extends WbGroup {
  #absoluteScale;
  #absoluteScaleNeedUpdate;
  #matrix;
  #matrixNeedUpdate;
  #previousTranslation;
  #rotation;
  #scale;
  #translation;
  #upperTransformFirstTimeSearch;
  #vrmlMatrix;
  #vrmlMatrixNeedUpdate;
  constructor(id, translation, scale, rotation) {
    super(id);
    this.#translation = translation;
    this.#scale = scale;
    this.#rotation = rotation;
    this.#rotation.normalizeRotation();

    this.#absoluteScaleNeedUpdate = true;
    this.#upperTransformFirstTimeSearch = true;
    this.#vrmlMatrix = new WbMatrix4();
    this.#vrmlMatrixNeedUpdate = true;
    this.#matrixNeedUpdate = true;
  }

  get translation() {
    return this.#translation;
  }

  set translation(newTranslation) {
    this.#previousTranslation = this.#translation;
    this.#translation = newTranslation;

    this.#updateTranslation();
  }

  get scale() {
    return this.#scale;
  }

  set scale(newScale) {
    this.#scale = newScale;

    this.#updateScale();
  }

  get rotation() {
    return this.#rotation;
  }

  set rotation(newRotation) {
    this.#rotation = newRotation;
    this.#rotation.normalizeRotation();

    this.#updateRotation();
  }

  absoluteScale() {
    if (this.#absoluteScaleNeedUpdate)
      this.#updateAbsoluteScale();

    return this.#absoluteScale;
  }

  #updateAbsoluteScale() {
    this.#absoluteScale = this.#scale;
    // multiply with upper transform scale if any
    const ut = this.#upperTransform();
    if (typeof ut !== 'undefined')
      this.#absoluteScale = this.#absoluteScale.mulByVector(ut.absoluteScale());

    this.#absoluteScaleNeedUpdate = false;
  }

  clone(customID) {
    const transform = new WbTransform(customID, this.#translation, this.#scale, this.#rotation);

    const length = this.children.length;
    for (let i = 0; i < length; i++) {
      const cloned = this.children[i].clone(getAnId());
      cloned.parent = customID;
      WbWorld.instance.nodes.set(cloned.id, cloned);
      transform.children.push(cloned);
    }

    this.useList.push(customID);
    return transform;
  }

  createWrenObjects() {
    if (!this.wrenObjectsCreatedCalled) {
      super.createWrenObjects(true);
      const transform = _wr_transform_new();

      _wr_transform_attach_child(this.wrenNode, transform);
      this.wrenNode = transform;
    }

    this.children.forEach(child => {
      child.createWrenObjects();
    });

    this.#applyTranslationToWren();
    this.#applyRotationToWren();
    this.#applyScaleToWren();
  }

  delete(isBoundingObject) {
    if (this.wrenObjectsCreatedCalled)
      _wr_node_delete(this.wrenNode);

    super.delete(isBoundingObject);
  }

  matrix() {
    if (typeof this.#matrix === 'undefined') {
      this.#matrix = new WbMatrix4();
      this.#updateMatrix();
    } else if (this.#matrixNeedUpdate)
      this.#updateMatrix();

    return this.#matrix;
  }

  recomputeBoundingSphere() {
    super.recomputeBoundingSphere();
    this._boundingSphere.transformOwner = true;
  }

  vrmlMatrix() {
    if (this.#vrmlMatrixNeedUpdate) {
      this.#vrmlMatrix.fromVrml(this.#translation, this.#rotation, this.#scale);
      this.#vrmlMatrixNeedUpdate = false;
    }

    return this.#vrmlMatrix;
  }

  #applyRotationToWren() {
    const rotation = _wrjs_array4(this.#rotation.w, this.#rotation.x, this.#rotation.y, this.#rotation.z);
    _wr_transform_set_orientation(this.wrenNode, rotation);
  }

  #applyScaleToWren() {
    const scale = _wrjs_array3(this.#scale.x, this.#scale.y, this.#scale.z);
    _wr_transform_set_scale(this.wrenNode, scale);
  }

  #applyTranslationToWren() {
    const translation = _wrjs_array3(this.#translation.x, this.#translation.y, this.#translation.z);
    _wr_transform_set_position(this.wrenNode, translation);
  }

  #updateMatrix() {
    this.#matrix.fromVrml(this.#translation, this.#rotation, this.#scale);

    // multiply with upper matrix if any
    const transform = this.#upperTransform();
    if (typeof transform !== 'undefined')
      this.#matrix = transform.matrix().mul(this.#matrix);
    this.#matrixNeedUpdate = false;
  }

  #updateTranslation() {
    if (typeof WbWorld.instance.viewpoint.followedId !== 'undefined' &&
      WbWorld.instance.viewpoint.followedId === this.id)
      WbWorld.instance.viewpoint.setFollowedObjectDeltaPosition(this.#translation, this.#previousTranslation);

    if (this.wrenObjectsCreatedCalled)
      this.#applyTranslationToWren();

    this.#matrixNeedUpdate = true;
  }

  #updateRotation() {
    if (this.wrenObjectsCreatedCalled)
      this.#applyRotationToWren();

    this.#matrixNeedUpdate = true;
  }

  #updateScale() {
    if (this.wrenObjectsCreatedCalled)
      this.#applyScaleToWren();

    this.#matrixNeedUpdate = true;
  }

  #upperTransform() {
    if (this.#upperTransformFirstTimeSearch) {
      this.upperTransform = findUpperTransform(this);
      if (this.wrenObjectsCreatedCalled)
        this.#upperTransformFirstTimeSearch = false;
    }

    return this.upperTransform;
  }
}
