import WbDevice from './WbDevice.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import {arrayXPointerFloat} from './utils/utils.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbLightSensor extends WbDevice {
  #transform;
  #material;
  #mesh;
  #renderable;

  get nodeType() {
    return WbNodeType.WB_NODE_LIGHT_SENSOR;
  }

  createWrenObjects() {
    this.#transform = _wr_transform_new();
    this.#material = _wr_phong_material_new();
    _wr_material_set_default_program(this.#material, WbWrenShaders.lineSetShader());

    const vertices = [0, 0, 0, 1, 0, 0];
    const verticesPointer = arrayXPointerFloat(vertices);
    this.#mesh = _wr_static_mesh_line_set_new(2, verticesPointer, undefined);
    _free(verticesPointer);
    this.#renderable = _wr_renderable_new();
    _wr_renderable_set_cast_shadows(this.#renderable, false);
    _wr_renderable_set_receive_shadows(this.#renderable, false);
    _wr_renderable_set_visibility_flags(this.#renderable, WbWrenRenderingContext.VM_REGULAR);
    _wr_renderable_set_drawing_mode(this.#renderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);
    _wr_renderable_set_mesh(this.#renderable, this.#mesh);
    _wr_renderable_set_material(this.#renderable, this.#material, undefined);

    super.createWrenObjects();

    _wr_transform_attach_child(this.#transform, this.#renderable);
    _wr_transform_attach_child(this.wrenNode, this.#transform);

    _wr_node_set_visible(this.#transform, false);

    const yellowColor = _wrjs_array3(1, 1, 0);
    _wr_phong_material_set_emissive(this.#material, yellowColor);

    this.#applyOptionalRenderingToWren();
  }

  delete() {
    if (this.wrenObjectsCreatedCalled) {
      _wr_node_delete(this.#renderable);
      _wr_node_delete(this.#transform);
      _wr_material_delete(this.#material);
      _wr_static_mesh_delete(this.#mesh);
    }

    super.delete();
  }

  #applyOptionalRenderingToWren() {
    if (!this.wrenObjectsCreatedCalled)
      return;

    const lineScale = _wr_config_get_line_scale();
    const scalePointer = _wrjs_array3(lineScale, lineScale, lineScale);
    _wr_transform_set_scale(this.#transform, scalePointer);
  }

  applyOptionalRendering(enable) {
    _wr_node_set_visible(this.#transform, enable);
  }
}
