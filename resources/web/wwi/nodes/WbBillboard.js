import WbGroup from './WbGroup.js';
import WbWorld from './WbWorld.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbBillboard extends WbGroup {
  constructor(id) {
    super(id);
    WbWorld.instance.billboards.push(id);
  }

  get nodeType() {
    return WbNodeType.WB_NODE_BILLBOARD;
  }

  createWrenObjects() {
    super.createWrenObjects(true);

    const transform = _wr_transform_new();
    _wr_transform_attach_child(this.wrenNode, transform);
    this.wrenNode = transform;

    this.children.forEach(child => {
      child.createWrenObjects();
    });

    this.updatePosition();
  }

  delete() {
    if (this.wrenObjectsCreatedCalled)
      _wr_node_delete(this.wrenNode);

    const index = WbWorld.instance.billboards.indexOf(this.id);
    if (index > -1)
      WbWorld.instance.billboards.splice(index, 1);
  }

  updatePosition() {
    this.#applyRotationToWren();
    this.#applyTranslationToWren();
  }

  #applyRotationToWren() {
    let orientation = WbWorld.instance.viewpoint.orientation;
    let orientationPointer = _wrjs_array4(orientation.w, orientation.x, orientation.y, orientation.z);
    _wr_transform_set_orientation(this.wrenNode, orientationPointer);
  }

  #applyTranslationToWren() {
    let position = WbWorld.instance.viewpoint.position;
    let positionPointer = _wrjs_array3(position.x, position.y, position.z);
    _wr_transform_set_position(this.wrenNode, positionPointer);
  }

  static isDescendantOfBillboard(node) {
    while (typeof node !== 'undefined') {
      if (node.nodeType === WbNodeType.WB_NODE_BILLBOARD)
        return true;

      node = WbWorld.instance.nodes.get(node.parent);
    }

    return false;
  }
}
