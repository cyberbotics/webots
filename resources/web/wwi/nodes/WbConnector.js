import WbDevice from './WbDevice.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import {arrayXPointerFloat} from './utils/utils.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbConnector extends WbDevice {
  #axisMesh;
  #axisRenderable;
  #axesTransform;
  #material;
  #numberOfRotations;
  #rotationsMesh;
  #rotationsRenderable;
  #rotationsTransform;
  #transform;
  constructor(id, translation, rotation, name, numberOfRotations) {
    super(id, translation, rotation, name);
    this.#numberOfRotations = numberOfRotations;
    this.#material = [];
    this.#axisMesh = [];
    this.#axisRenderable = [];
  }

  get nodeType() {
    return WbNodeType.WB_NODE_CONNECTOR;
  }

  get numberOfRotations() {
    return this.#numberOfRotations;
  }

  set numberOfRotations(newNumberOfRotations) {
    if (newNumberOfRotations < 0)
      newNumberOfRotations = 0;

    this.#numberOfRotations = newNumberOfRotations;
    this.#applyOptionalRenderingToWren();
  }

  createWrenObjects() {
    this.#transform = _wr_transform_new();
    this.#axesTransform = _wr_transform_new();
    this.#rotationsTransform = _wr_transform_new();

    _wr_node_set_visible(this.#transform, false);

    // Connector axes: X = red, Z = blue, Y = black
    const colors = [[1, 0, 0], [0, 0, 1], [0, 0, 0]];
    for (let i = 0; i < 3; ++i) {
      this.#material[i] = _wr_phong_material_new();
      const colorPointer = _wrjs_array3(colors[i][0], colors[i][1], colors[i][2]);
      _wr_phong_material_set_color(this.#material[i], colorPointer);
      _wr_material_set_default_program(this.#material[i], WbWrenShaders.lineSetShader());
    }

    // Axes (X & Z only)
    const axesCoordinates = [[0, 0, 0, 0.5, 0, 0], [0, 0, 0, 0, 0, 0.5]];

    for (let i = 0; i < 2; ++i) {
      const axesCoordinatesPointer = arrayXPointerFloat(axesCoordinates[i]);
      this.#axisMesh[i] = _wr_static_mesh_line_set_new(2, axesCoordinatesPointer, undefined);
      _free(axesCoordinatesPointer);

      this.#axisRenderable[i] = _wr_renderable_new();
      _wr_renderable_set_cast_shadows(this.#axisRenderable[i], false);
      _wr_renderable_set_receive_shadows(this.#axisRenderable[i], false);
      _wr_renderable_set_visibility_flags(this.#axisRenderable[i], WbWrenRenderingContext.VM_REGULAR);
      _wr_renderable_set_drawing_mode(this.#axisRenderable[i], Enum.WR_RENDERABLE_DRAWING_MODE_LINES);
      _wr_renderable_set_mesh(this.#axisRenderable[i], this.#axisMesh[i]);
      _wr_renderable_set_material(this.#axisRenderable[i], this.#material[i], undefined);

      _wr_transform_attach_child(this.#axesTransform, this.#axisRenderable[i]);
    }
    _wr_transform_attach_child(this.#transform, this.#axesTransform);

    // Rotation alignements (mesh is constructed in 'applyOptionalRenderingToWren')
    this.#rotationsRenderable = _wr_renderable_new();
    _wr_renderable_set_cast_shadows(this.#rotationsRenderable, false);
    _wr_renderable_set_receive_shadows(this.#rotationsRenderable, false);
    _wr_renderable_set_visibility_flags(this.#rotationsRenderable, WbWrenRenderingContext.VM_REGULAR);
    _wr_renderable_set_drawing_mode(this.#rotationsRenderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);
    _wr_renderable_set_material(this.#rotationsRenderable, this.#material[2], undefined);
    this.#rotationsMesh = undefined;

    _wr_transform_attach_child(this.#rotationsTransform, this.#rotationsRenderable);
    _wr_transform_attach_child(this.#transform, this.#rotationsTransform);

    super.createWrenObjects();

    _wr_transform_attach_child(this.wrenNode, this.#transform);
    this.#applyOptionalRenderingToWren();
  }

  delete() {
    _wr_node_delete(this.#transform);
    _wr_node_delete(this.#axesTransform);
    _wr_node_delete(this.#axisRenderable[0]);
    _wr_node_delete(this.#axisRenderable[1]);
    _wr_node_delete(this.#rotationsTransform);
    _wr_node_delete(this.#rotationsRenderable);
    _wr_static_mesh_delete(this.#axisMesh[0]);
    _wr_static_mesh_delete(this.#axisMesh[1]);
    _wr_static_mesh_delete(this.#rotationsMesh);

    for (let i = 0; i < 3; ++i)
      _wr_material_delete(this.#material[i]);

    super.delete();
  }

  applyOptionalRendering(enable) {
    _wr_node_set_visible(this.#transform, enable);
  }

  #applyOptionalRenderingToWren() {
    if (!this.wrenObjectsCreatedCalled)
      return;

    _wr_static_mesh_delete(this.#rotationsMesh);
    this.#rotationsMesh = undefined;

    // draw rotational alignments in black the first aligmnent is the z-axis so we skip it
    const n = this.#numberOfRotations;
    if (n > 1) {
      const vertices = [];
      const angleStep = 2 * Math.PI / n;
      for (let i = 1; i < n; ++i) {
        const angle = angleStep * i;
        const y = 0.4 * Math.sin(angle);
        const z = 0.4 * Math.cos(angle);

        const idx = (i - 1) * 6;
        // Segment origin
        vertices[idx] = 0;
        vertices[idx + 1] = 0;
        vertices[idx + 2] = 0;

        // Segment orientation
        vertices[idx + 3] = 0;
        vertices[idx + 4] = y;
        vertices[idx + 5] = z;
      }

      const verticesPointer = arrayXPointerFloat(vertices);
      this.#rotationsMesh = _wr_static_mesh_line_set_new((n - 1) * 2, verticesPointer, undefined);
      _free(verticesPointer);
      _wr_renderable_set_mesh(this.#rotationsRenderable, this.#rotationsMesh);
      _wr_node_set_visible(this.#rotationsTransform, true);
    } else
      _wr_node_set_visible(this.#rotationsTransform, false);

    this.#updateLineScale();
  }

  #updateLineScale() {
    if (this.wrenObjectsCreatedCalled) {
      const ls = _wr_config_get_line_scale();
      const scale = _wrjs_array3(ls, ls, ls);
      _wr_transform_set_scale(this.#transform, scale);
    }
  }
}
