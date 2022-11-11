import WbSolid from './WbSolid.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import {arrayXPointerFloat} from './utils/utils.js';

// This class is used to retrieve the type of device
export default class WbAbstractCamera extends WbSolid {
  #fieldOfView;
  #height;
  #width;
  constructor(id, translation, scale, rotation, name, height, width, fieldOfView) {
    super(id, translation, scale, rotation, name);

    this.#fieldOfView = fieldOfView;
    this.#height = height;
    this.#width = width;

    this._isRangeFinder = false;
    this._isFrustumEnabled = true;
    this._charType = '';
  }

  get fieldOfView() {
    return this.#fieldOfView;
  }

  get height() {
    return this.#height;
  }

  get width() {
    return this.#width;
  }

  createWrenObjects() {
    super.createWrenObjects();
    // Frustum
    this._material = _wr_phong_material_new();
    _wr_material_set_default_program(this._material, WbWrenShaders.lineSetShader());
    _wr_phong_material_set_color_per_vertex(this._material, true);

    this._renderable = _wr_renderable_new();
    _wr_renderable_set_cast_shadows(this._renderable, false);
    _wr_renderable_set_receive_shadows(this._renderable, false);
    _wr_renderable_set_material(this._renderable, this._material, null);
    _wr_renderable_set_drawing_mode(this._renderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);

    this._transform = _wr_transform_new();
    _wr_transform_attach_child(this._transform, this._renderable);
    _wr_transform_attach_child(this.wrenNode, this._transform);

    this._applyFrustumToWren();

    // Frustum display
    this._frustumDisplayTransform = _wr_transform_new();
    this._frustumDisplayRenderable = _wr_renderable_new();
    this._frustumDisplayMesh = _wr_static_mesh_unit_rectangle_new(false);
    this._frustumDisplayMaterial = _wr_phong_material_new();

    _wr_material_set_default_program(this._frustumDisplayMaterial, WbWrenShaders.simpleShader());
    _wr_phong_material_set_transparency(this._frustumDisplayMaterial, 0.5);
    _wr_renderable_set_material(this._frustumDisplayRenderable, this._frustumDisplayMaterial, null);
    _wr_renderable_set_mesh(this._frustumDisplayRenderable, this._frustumDisplayMesh);
    _wr_renderable_set_face_culling(this._frustumDisplayRenderable, false);

    if (this._isRangeFinder)
      _wr_renderable_set_visibility_flags(this.frustumDisplayRenderable, WbWrenRenderingContext.VF_RANGE_FINDER_FRUSTUMS);
    else
      _wr_renderable_set_visibility_flags(this._frustumDisplayRenderable, WbWrenRenderingContext.VF_CAMERA_FRUSTUMS);

    _wr_node_set_visible(this._frustumDisplayTransform, false);
    _wr_transform_attach_child(this._frustumDisplayTransform, this._frustumDisplayRenderable);
    _wr_transform_attach_child(this.wrenNode, this._frustumDisplayTransform);

    this.updateFrustumDisplay();
  }

  _applyFrustumToWren() {
    console.error('salut');
    _wr_node_set_visible(this._transform, false);

    _wr_static_mesh_delete(this._mesh);
    this._mesh = undefined;

    if (!this._isFrustumEnabled)
      return;

    const frustumColor = [0.5, 0.5, 0.5];
    const frustumColorRgb = _wrjs_array3(frustumColor[0], frustumColor[1], frustumColor[2]);

    _wr_phong_material_set_color(this._material, frustumColorRgb);

    let drawFarPlane;
    let f;
    const n = this.minRange();
    // if the far is set to 0 it means the far clipping plane is set to infinity
    // so, the far distance of the colored frustum should be set arbitrarily

    if (this._charType === 'c' && this.maxRange() === 0.0) {
      f = n + 2 * _wr_config_get_line_scale();
      drawFarPlane = false;
    } else {
      f = this.maxRange();
      drawFarPlane = true;
    }

    const w = this.#width;
    const h = this.#height;
    const fovX = this.#fieldOfView;
    const t = Math.tan(fovX / 2);
    const dw1 = n * t;
    const dh1 = dw1 * h / w;
    const n1 = n;
    const dw2 = f * t;
    const dh2 = dw2 * h / w;
    const n2 = f;

    let vertices = [];
    let colors = [];
    let vertex = [0.0, 0.0, 0.0];
    this.#addVertex(vertices, colors, vertex, frustumColor);
    vertex[0] = n;
    this.#addVertex(vertices, colors, vertex, frustumColor);

    const pos = [[n1, dw1, dh1], [n1, dw1, -dh1], [n1, -dw1, -dh1], [n1, -dw1, dh1]];
    this.#drawRectangle(vertices, colors, pos, frustumColor);

    // Creation of the far plane
    // if the camera is not of the range-finder type, the far is set to infinity
    // so, the far rectangle of the colored frustum shouldn't be set
    if (drawFarPlane) {
      const pos = [[n2, dw2, dh2], [n2, dw2, -dh2], [n2, -dw2, -dh2], [n2, -dw2, dh2]];
      this.#drawRectangle(vertices, colors, pos, frustumColor);
    }
    // Creation of the external outline of the frustum (4 lines)
    const frustumOutline = [[n1, dw1, dh1], [n2, dw2, dh2], [n1, -dw1, dh1], [n2, -dw2, dh2], [n1, dw1, -dh1], [n2, dw2, -dh2],
      [n1, -dw1, -dh1], [n2, -dw2, -dh2]];
    for (let i = 0; i < 8; i++)
      this.#addVertex(vertices, colors, frustumOutline[i], frustumColor);

    const verticesPointer = arrayXPointerFloat(vertices);
    const colorsPointer = arrayXPointerFloat(colors);
    this._mesh = _wr_static_mesh_line_set_new(vertices.length / 3, verticesPointer, colorsPointer);
    _wr_renderable_set_mesh(this._renderable, this._mesh);
    // _free(verticesPointer);
    // _free(colorsPointer);

    if (this._isRangeFinder)
      _wr_renderable_set_visibility_flags(this._renderable, WbWrenRenderingContext.VF_RANGE_FINDER_FRUSTUMS);
    else
      _wr_renderable_set_visibility_flags(this._renderable, WbWrenRenderingContext.VF_CAMERA_FRUSTUMS);

    _wr_node_set_visible(this._transform, true);
  }

  updateFrustumDisplay() {
    _wr_node_set_visible(this._frustumDisplayTransform, false);

    if (!this._isFrustumEnabled)
      return;

    const n = this.minRange();
    const quadWidth = 2 * n * Math.tan(this.#fieldOfView / 2);
    const translation = [n, 0, 0];
    // Axis-angle for roll(pi/2), pitch(-pi/2), and yaw(0)
    const orientation = [Math.PI * 2 / 3, Math.sqrt(3) / 3, -Math.sqrt(3) / 3, -Math.sqrt(3) / 3];
    const scale = [quadWidth, (quadWidth * this.#height) / this.#width, 1];

    _wr_transform_set_position(this._frustumDisplayTransform, translation);
    _wr_transform_set_orientation(this._frustumDisplayTransform, orientation);
    _wr_transform_set_scale(this._frustumDisplayTransform, scale);
    // _wr_material_set_texture(this._frustumDisplayMaterial, mWrenCamera->getWrenTexture(), 0);
    console.log('update');
    _wr_node_set_visible(this._frustumDisplayTransform, true);
  }

  minRange() {
  }
  maxRange() {
    return 1.0;
  }

  #addVertex(vertices, colors, vertex, color) {
    vertices.push(vertex[0]);
    vertices.push(vertex[1]);
    vertices.push(vertex[2]);

    colors.push(color[0]);
    colors.push(color[1]);
    colors.push(color[2]);
  }

  #drawRectangle(vertices, colors, v, color) {
    for (let i = 0; i < 4; ++i) {
      this.#addVertex(vertices, colors, v[i], color);
      this.#addVertex(vertices, colors, v[(i + 1) % 4], color);
    }
  }
}
