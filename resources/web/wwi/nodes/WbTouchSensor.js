import WbSolid from './WbSolid.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import {arrayXPointerFloat} from './utils/utils.js';
import {findUpperTransform} from './utils/node_utilities.js';

// This class is used to retrieve the type of device
export default class WbTouchSensor extends WbSolid {
  #axisMesh;
  #axisRenderable;
  #axesTransform;
  #material;
  #transform;

  delete() {
    _wr_node_delete(this.#transform);
    _wr_node_delete(this.#axesTransform);
    _wr_node_delete(this.#axisRenderable[0]);
    _wr_node_delete(this.#axisRenderable[1]);
    _wr_node_delete(this.#axisRenderable[2]);
    _wr_static_mesh_delete(this.#axisMesh[0]);
    _wr_static_mesh_delete(this.#axisMesh[1]);
    _wr_static_mesh_delete(this.#axisMesh[2]);

    for (let i = 0; i < 3; ++i)
      _wr_material_delete(this.#material[i]);
  }

  applyOptionalRendering(enable) {
    _wr_node_set_visible(this.#transform, enable);
  }

  postFinalize() {
    super.postFinalize();

    let ancestor = findUpperTransform(this);
    if (typeof ancestor === 'undefined')
      ancestor = this;

    this.#applyOptionalRenderingToWren();

    console.log(this._boundingSphere.radius);
  }

  #applyOptionalRenderingToWren() {
    this.#transform = _wr_transform_new();
    this.#axesTransform = _wr_transform_new();

    _wr_node_set_visible(this.#transform, false);

    const colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    this.#material = [];
    this.#axisMesh = [];
    this.#axisRenderable = [];
    for (let i = 0; i < 3; ++i) {
      this.#material[i] = _wr_phong_material_new();
      const colorPointer = _wrjs_array3(colors[i][0], colors[i][1], colors[i][2]);
      _wr_phong_material_set_color(this.#material[i], colorPointer);
      _wr_material_set_default_program(this.#material[i], WbWrenShaders.lineSetShader());
    }

    // Axes (X & Z only)
    const axesCoordinates = [[0, 0, 0, 0.5, 0, 0], [0, 0, 0, 0, 0.5, 0], [0, 0, 0, 0, 0, 0.5]];

    for (let i = 0; i < 3; ++i) {
      const axesCoordinatesPointer = arrayXPointerFloat(axesCoordinates[i]);
      this.#axisMesh[i] = _wr_static_mesh_line_set_new(2, axesCoordinatesPointer, undefined);
      _free(axesCoordinatesPointer);

      this.#axisRenderable[i] = _wr_renderable_new();
      _wr_renderable_set_cast_shadows(this.#axisRenderable[i], false);
      _wr_renderable_set_receive_shadows(this.#axisRenderable[i], false);
      _wr_renderable_set_visibility_flags(this.#axisRenderable[i], WbWrenRenderingContext.VM_REGULAR);
      _wr_renderable_set_drawing_order(this.#axisRenderable[i], Enum.WR_RENDERABLE_DRAWING_ORDER_AFTER_2);
      _wr_renderable_set_drawing_mode(this.#axisRenderable[i], Enum.WR_RENDERABLE_DRAWING_MODE_LINES);
      _wr_renderable_set_mesh(this.#axisRenderable[i], this.#axisMesh[i]);
      _wr_renderable_set_material(this.#axisRenderable[i], this.#material[i], undefined);

      _wr_transform_attach_child(this.#axesTransform, this.#axisRenderable[i]);
    }
    _wr_transform_attach_child(this.#transform, this.#axesTransform);

    super.createWrenObjects();

    _wr_transform_attach_child(this.wrenNode, this.#transform);
  }
}
