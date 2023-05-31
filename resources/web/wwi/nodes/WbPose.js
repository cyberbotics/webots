import WbGroup from './WbGroup.js';
import WbWorld from './WbWorld.js';

import { getAnId } from './utils/id_provider.js';
import { findUpperPose } from './utils/node_utilities.js';
import WbMatrix4 from './utils/WbMatrix4.js';
import { WbNodeType } from './wb_node_type.js';
import WbVector3 from './utils/WbVector3.js';

export default class WbPose extends WbGroup {
  #matrix;
  #previousTranslation;
  #rotation;
  #translation;
  #upperPoseFirstTimeSearch;
  _vrmlMatrix;
  _vrmlMatrixNeedUpdate;
  constructor(id, translation, rotation) {
    super(id);
    this.#translation = translation;
    this.#rotation = rotation;
    this.#rotation.normalizeRotation();

    this.#upperPoseFirstTimeSearch = true;
    this._vrmlMatrix = new WbMatrix4();
    this._vrmlMatrixNeedUpdate = true;
    this._matrixNeedUpdate = true;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_POSE;
  }

  get translation() {
    return this.#translation;
  }

  set translation(newTranslation) {
    this.#previousTranslation = this.#translation;
    this.#translation = newTranslation;

    this.#updateTranslation();
  }

  get rotation() {
    return this.#rotation;
  }

  set rotation(newRotation) {
    this.#rotation = newRotation;
    this.#rotation.normalizeRotation();

    this.#updateRotation();
  }

  clone(customID) {
    const pose = new WbPose(customID, this.#translation, this.#rotation);

    const length = this.children.length;
    for (let i = 0; i < length; i++) {
      const cloned = this.children[i].clone(getAnId());
      cloned.parent = customID;
      WbWorld.instance.nodes.set(cloned.id, cloned);
      pose.children.push(cloned);
    }

    this.useList.push(customID);
    return pose;
  }

  createWrenObjects() {
    super.createWrenObjects(true);
    const transform = _wr_transform_new();

    _wr_transform_attach_child(this.wrenNode, transform);
    this.wrenNode = transform;

    this.children.forEach(child => {
      this._updateProgress('Create WREN object');
      child.createWrenObjects();
    });

    this.applyTranslationToWren();
    this.applyRotationToWren();
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
    } else if (this._matrixNeedUpdate)
      this.#updateMatrix();

    return this.#matrix;
  }

  recomputeBoundingSphere() {
    super.recomputeBoundingSphere();
    this._boundingSphere.poseOwner = true;
  }

  vrmlMatrix() {
    if (this._vrmlMatrixNeedUpdate) {
      this._vrmlMatrix.fromVrml(this.#translation, this.#rotation, new WbVector3(1.0, 1.0, 1.0));
      this._vrmlMatrixNeedUpdate = false;
    }

    return this._vrmlMatrix;
  }

  geometry() {
    if (this.children.length === 0)
      return;

    const firstChild = this.children[0];
    if (firstChild.nodeType === WbNodeType.WB_NODE_SHAPE)
      return firstChild.geometry;

    return firstChild;
  }

  applyRotationToWren() {
    const rotation = _wrjs_array4(this.#rotation.w, this.#rotation.x, this.#rotation.y, this.#rotation.z);
    _wr_transform_set_orientation(this.wrenNode, rotation);
  }

  applyTranslationToWren() {
    const translation = _wrjs_array3(this.#translation.x, this.#translation.y, this.#translation.z);
    _wr_transform_set_position(this.wrenNode, translation);
  }

  #updateMatrix() {
    this.#matrix.fromVrml(this.#translation, this.#rotation, new WbVector3(1.0, 1.0, 1.0));

    // multiply with upper matrix if any
    const pose = this.#upperPose();
    if (typeof pose !== 'undefined')
      this.#matrix = pose.matrix().mul(this.#matrix);
    this._matrixNeedUpdate = false;
  }

  #updateTranslation() {
    if (typeof WbWorld.instance.viewpoint.followedId !== 'undefined' &&
      WbWorld.instance.viewpoint.followedId === this.id)
      WbWorld.instance.viewpoint.setFollowedObjectDeltaPosition(this.#translation, this.#previousTranslation);

    if (this.wrenObjectsCreatedCalled)
      this.applyTranslationToWren();

    this._matrixNeedUpdate = true;
  }

  #updateRotation() {
    if (this.wrenObjectsCreatedCalled)
      this.applyRotationToWren();

    this._matrixNeedUpdate = true;
  }

  #upperPose() {
    if (this.#upperPoseFirstTimeSearch) {
      this.upperPose = findUpperPose(this);
      if (this.wrenObjectsCreatedCalled)
        this.#upperPoseFirstTimeSearch = false;
    }

    return this.upperPose;
  }
}
