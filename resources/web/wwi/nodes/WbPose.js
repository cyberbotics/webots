import WbGroup from './WbGroup.js';
import WbWorld from './WbWorld.js';

import { getAnId } from './utils/id_provider.js';
import { findUpperPose } from './utils/node_utilities.js';
import WbMatrix4 from './utils/WbMatrix4.js';
import { WbNodeType } from './wb_node_type.js';
import WbVector3 from './utils/WbVector3.js';

export default class WbPose extends WbGroup {
  #absoluteScale;
  #absoluteScaleNeedUpdate;
  #matrix;
  #matrixNeedUpdate;
  #previousTranslation;
  #rotation;
  #translation;
  #upperPoseFirstTimeSearch;
  #vrmlMatrix;
  #vrmlMatrixNeedUpdate;
  constructor(id, translation, rotation) {
    super(id);
    this.#translation = translation;
    this.#rotation = rotation;
    this.#rotation.normalizeRotation();

    this.#absoluteScaleNeedUpdate = true;
    this.#upperPoseFirstTimeSearch = true;
    this.#vrmlMatrix = new WbMatrix4();
    this.#vrmlMatrixNeedUpdate = true;
    this.#matrixNeedUpdate = true;
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

  absoluteScale() {
    if (this.#absoluteScaleNeedUpdate)
      this.#updateAbsoluteScale();

    return this.#absoluteScale;
  }

  #updateAbsoluteScale() {
    this.#absoluteScale = new WbVector3(1.0, 1.0, 1.0);
    // multiply with upper pose scale if any
    const up = this.#upperPose();
    if (typeof up !== 'undefined')
      this.#absoluteScale = this.#absoluteScale.mulByVector(up.absoluteScale());

    this.#absoluteScaleNeedUpdate = false;
  }

  clone(customID) {
    const transform = new WbPose(customID, this.#translation, this.#rotation);

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
    super.createWrenObjects(true);
    const transform = _wr_transform_new();

    _wr_transform_attach_child(this.wrenNode, transform);
    this.wrenNode = transform;

    this.children.forEach(child => {
      child.createWrenObjects();
    });

    this.#applyTranslationToWren();
    this.#applyRotationToWren();
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
    this._boundingSphere.poseOwner = true;
  }

  vrmlMatrix() {
    if (this.#vrmlMatrixNeedUpdate) {
      this.#vrmlMatrix.fromVrml(this.#translation, this.#rotation, WbVector3(1.0, 1.0, 1.0));
      this.#vrmlMatrixNeedUpdate = false;
    }

    return this.#vrmlMatrix;
  }

  geometry() {
    if (this.children.length === 0)
      return;

    const firstChild = this.children[0];
    if (firstChild.nodeType === WbNodeType.WB_NODE_SHAPE)
      return firstChild.geometry;

    return firstChild;
  }

  #applyRotationToWren() {
    const rotation = _wrjs_array4(this.#rotation.w, this.#rotation.x, this.#rotation.y, this.#rotation.z);
    _wr_transform_set_orientation(this.wrenNode, rotation);
  }

  #applyTranslationToWren() {
    const translation = _wrjs_array3(this.#translation.x, this.#translation.y, this.#translation.z);
    _wr_transform_set_position(this.wrenNode, translation);
  }

  #updateMatrix() {
    this.#matrix.fromVrml(this.#translation, this.#rotation, WbVector3(1.0, 1.0, 1.0));

    // multiply with upper matrix if any
    const pose = this.#upperPose();
    if (typeof pose !== 'undefined')
      this.#matrix = pose.matrix().mul(this.#matrix);
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

  #upperPose() {
    if (this.#upperPoseFirstTimeSearch) {
      this.upperPose = findUpperPose(this);
      if (this.wrenObjectsCreatedCalled)
        this.#upperPoseFirstTimeSearch = false;
    }

    return this.upperPose;
  }
}
