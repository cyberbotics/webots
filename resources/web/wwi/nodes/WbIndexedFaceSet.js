import WbTriangleMeshGeometry from './WbTriangleMeshGeometry.js';

import {getAnId} from './utils/id_provider.js';
import {resetIfNegative} from './utils/WbFieldChecker.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbIndexedFaceSet extends WbTriangleMeshGeometry {
  #ccw;
  #coord;
  #coordIndex;
  #creaseAngle;
  #normal;
  #normalIndex;
  #normalPerVertex;
  #texCoord;
  #texCoordIndex;
  constructor(id, coordIndex, normalIndex, texCoordIndex, coord, texCoord, normal, ccw, creaseAngle, normalPerVertex) {
    super(id);

    this.#coordIndex = coordIndex;
    this.#normalIndex = normalIndex;
    this.#texCoordIndex = texCoordIndex;

    this.#coord = coord;
    this.#texCoord = texCoord;
    this.#normal = normal;

    this.#ccw = ccw;
    this.#creaseAngle = creaseAngle;
    this.#normalPerVertex = normalPerVertex;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_INDEXED_FACE_SET;
  }

  get ccw() {
    return this.#ccw;
  }

  set ccw(newCcw) {
    this.#ccw = newCcw;

    if (this.wrenObjectsCreatedCalled)
      this.#update();
  }

  get coordIndex() {
    return this.#coordIndex;
  }

  set coordIndex(newCoordIndex) {
    this.#coordIndex = newCoordIndex;

    if (this.wrenObjectsCreatedCalled)
      this.#update();
  }

  get creaseAngle() {
    return this.#creaseAngle;
  }

  set creaseAngle(newCreaseAngle) {
    this.#creaseAngle = newCreaseAngle;

    if (this.wrenObjectsCreatedCalled)
      this.#updateCreaseAngle();
  }

  get normalIndex() {
    return this.#normalIndex;
  }

  set normalIndex(newNormalIndex) {
    this.#normalIndex = newNormalIndex;

    if (this.wrenObjectsCreatedCalled)
      this.#update();
  }

  get normalPerVertex() {
    return this.#normalPerVertex();
  }

  set normalPerVertex(newNormalPerVertex) {
    this.#normalPerVertex = newNormalPerVertex;

    if (this.wrenObjectsCreatedCalled)
      this.#update();
  }

  get texCoordIndex() {
    return this.#texCoordIndex;
  }

  set texCoordIndex(newTexCoordIndex) {
    this.#texCoordIndex = newTexCoordIndex;

    if (this.wrenObjectsCreatedCalled)
      this.#update();
  }

  clone(customID) {
    this.useList.push(customID);
    const newCoord = this.#coord?.clone(getAnId());
    const newNormal = this.#normal?.clone(getAnId());
    const newTexCoord = this.#texCoord?.clone(getAnId());
    return new WbIndexedFaceSet(customID, this.#coordIndex, this.#normalIndex, this.#texCoordIndex, newCoord, newTexCoord,
      newNormal, this.#ccw, this.#creaseAngle, this.#normalPerVertex);
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.#coord?.createWrenObjects();
    this.#normal?.createWrenObjects();
    this.#texCoord?.createWrenObjects();
  }

  delete() {
    this.#coord?.delete();
    this.#normal?.delete();
    this.#texCoord?.delete();

    super.delete();
  }

  preFinalize() {
    super.preFinalize();

    this.#coord?.preFinalize();
    this.#normal?.preFinalize();
    this.#texCoord?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.#coord?.postFinalize();
    this.#normal?.postFinalize();
    this.#texCoord?.postFinalize();

    if (typeof this.#coord !== 'undefined') {
      this.#coord.onChange = () => {
        this._buildWrenMesh(true);
        if (typeof this.onRecreated === 'function')
          this.onRecreated();
      };
    }

    if (typeof this.#normal !== 'undefined') {
      this.#normal.onChange = () => {
        this._buildWrenMesh(true);
        if (typeof this.onRecreated === 'function')
          this.onRecreated();
      };
    }

    if (typeof this.#texCoord !== 'undefined') {
      this.#texCoord.onChange = () => {
        this._buildWrenMesh(true);
        if (typeof this.onRecreated === 'function')
          this.onRecreated();
      };
    }
  }

  _updateTriangleMesh() {
    this._triangleMesh.init(this.#coord?.point, this.#coordIndex, this.#normal?.vector, this.#normalIndex,
      this.#texCoord?.point, this.#texCoordIndex, this.#creaseAngle, this.#normalPerVertex);
  }

  #update() {
    this._buildWrenMesh(true);

    if (typeof this.onRecreated === 'function')
      this.onRecreated();
  }

  #updateCreaseAngle() {
    const newCreaseAngle = resetIfNegative(this.#creaseAngle, 0);
    if (newCreaseAngle !== false)
      this.#creaseAngle = newCreaseAngle;

    this.#update();
  }
}
