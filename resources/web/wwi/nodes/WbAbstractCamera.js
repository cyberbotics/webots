import WbSolid from './WbSolid.js';
import WbWrenCamera from '../wren/WbWrenCamera.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';

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
    this._isFrustumEnabled = false;
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

    this._transform = wr_transform_new();
    _wr_transform_attach_child(this._transform, this._renderable);
    _wr_transform_attach_child(this.wrenNode, this._transform);

    this._applyFrustumToWren();

    // Frustum display
    this._frustumDisplayTransform = _wr_transform_new();
    this._frustumDisplayRenderable = _wr_renderable_new();
    this._frustumDisplayMesh = _wr_static_mesh_unit_rectangle_new(false);
    this._frustumDisplayMaterial = _wr_phong_material_new();

    _wr_material_set_default_program(this._frustumDisplayMaterial, WbWrenShaders.simpleShader());
    _wr_phong_material_set_transparency(this._FrustumDisplayMaterial, 0.5);
    _wr_renderable_set_material(this._frustumDisplayRenderable, this._frustumDisplayMaterial, null);
    _wr_renderable_set_mesh(this._frustumDisplayRenderable, this._frustumDisplayMesh);
    _wr_renderable_set_face_culling(this._frustumDisplayRenderable, false);

    if (this._isRangeFinder)
      _wr_renderable_set_visibility_flags(this.frustumDisplayRenderable, WbWrenRenderingContext.VF_RANGE_FINDER_FRUSTUMS);
    else
      wr_renderable_set_visibility_flags(this._frustumDisplayRenderable, WbWrenRenderingContext.VF_CAMERA_FRUSTUMS);

    _wr_node_set_visible(this._frustumDisplayTransform, false);
    _wr_transform_attach_child(this._frustumDisplayTransform, this._frustumDisplayRenderable);
    _wr_transform_attach_child(this.wrenNode, this._frustumDisplayTransform);
  }

  applyFrustumToWren() {
    _wr_node_set_visible(this._transform, false);

    _wr_static_mesh_delete(this._mesh);
    this._mesh = undefined;

    if (!this._isFrustumEnabled)
        return;

    
    const frustumColorRgb = _wrjs_array3(0.5, 0.5, 0.5);

    _wr_phong_material_set_color(this.material, frustumColorRgb);

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
    const fovY = WbWrenCamera.computeFieldOfViewY(fovX, w / h);  // fovX -> fovY
    const t = Math.tan(fovX / 2);
    const dw1 = n * t;
    const dh1 = dw1 * h / w;
    const n1 = n;
    const dw2 = f * t;
    const dh2 = dw2 * h / w;
    const n2 = f;

    let vertices;
    let colors;
    let vertex = [0.0, 0.0, 0.0];
    addVertex(vertices, colors, vertex, frustumColor);
    vertex[0] = n;
    addVertex(vertices, colors, vertex, frustumColor);

    // creation of the near plane
    if (hasBeenSetup() && !mWrenCamera->isPlanarProjection()) {
        const float cubeColor[3] = {0.0f, 0.0f, 0.0f};
        drawCube(vertices, colors, n, cubeColor);

        const float n95 = 0.95f * n;
        if (mWrenCamera->isSubCameraActive(WbWrenCamera::CAMERA_ORIENTATION_BACK)) {
        const float pos[4][3] = {{n95, n95, -n}, {n95, -n95, -n}, {-n95, -n95, -n}, {-n95, n95, -n}};
        drawRectangle(vertices, colors, pos, frustumColor);
        }
        if (mWrenCamera->isSubCameraActive(WbWrenCamera::CAMERA_ORIENTATION_DOWN)) {
        const float pos0[4][3] = {{n95, -n, n95}, {n95, -n, -n95}, {-n95, -n, -n95}, {-n95, -n, n95}};
        const float pos1[4][3] = {{n95, n, n95}, {n95, n, -n95}, {-n95, n, -n95}, {-n95, n, n95}};
        drawRectangle(vertices, colors, pos0, frustumColor);
        drawRectangle(vertices, colors, pos1, frustumColor);
        }
        if (mWrenCamera->isSubCameraActive(WbWrenCamera::CAMERA_ORIENTATION_LEFT)) {
        const float pos0[4][3] = {{-n, n95, n95}, {-n, n95, -n95}, {-n, -n95, -n95}, {-n, -n95, n95}};
        const float pos1[4][3] = {{n, n95, n95}, {n, n95, -n95}, {n, -n95, -n95}, {n, -n95, n95}};
        drawRectangle(vertices, colors, pos0, frustumColor);
        drawRectangle(vertices, colors, pos1, frustumColor);
        }
        if (mWrenCamera->isSubCameraActive(WbWrenCamera::CAMERA_ORIENTATION_FRONT)) {
        const float pos[4][3] = {{n95, n95, n}, {n95, -n95, n}, {-n95, -n95, n}, {-n95, n95, n}};
        drawRectangle(vertices, colors, pos, frustumColor);
        }
    } else {
        const float pos[4][3] = {{n1, dw1, dh1}, {n1, dw1, -dh1}, {n1, -dw1, -dh1}, {n1, -dw1, dh1}};
        drawRectangle(vertices, colors, pos, frustumColor);
    }

    // Creation of the far plane
    // if the camera is not of the range-finder type, the far is set to infinity
    // so, the far rectangle of the colored frustum shouldn't be set
    if (drawFarPlane && isPlanarProjection()) {
        const float pos[4][3] = {{n2, dw2, dh2}, {n2, dw2, -dh2}, {n2, -dw2, -dh2}, {n2, -dw2, dh2}};
        drawRectangle(vertices, colors, pos, frustumColor);
    }

    const float zero[3] = {0.0f, 0.0f, 0.0f};
    // Creation of the external outline of the frustum (4 lines)
    if (!isPlanarProjection()) {
        const float angleY[4] = {-fovY / 2.0f, -fovY / 2.0f, fovY / 2.0f, fovY / 2.0f};
        const float angleX[4] = {fovX / 2.0f, -fovX / 2.0f, -fovX / 2.0f, fovX / 2.0f};
        for (int k = 0; k < 4; ++k) {
        const float helper = cosf(angleY[k]);
        // get x, y and z from the spherical coordinates
        float y = 0.0f;
        if (angleY[k] > M_PI_4 || angleY[k] < -M_PI_4)
            y = f * cosf(angleY[k] + M_PI_2) * sinf(angleX[k]);
        else
            y = f * helper * sinf(angleX[k]);
        const float z = f * sinf(angleY[k]);
        const float x = f * helper * cosf(angleX[k]);
        addVertex(vertices, colors, zero, frustumColor);
        const float outlineVertex[3] = {x, y, z};
        addVertex(vertices, colors, outlineVertex, frustumColor);
        }
    } else {
        const float frustumOutline[8][3] = {{n1, dw1, dh1},  {n2, dw2, dh2},  {n1, -dw1, dh1},  {n2, -dw2, dh2},
                                            {n1, dw1, -dh1}, {n2, dw2, -dh2}, {n1, -dw1, -dh1}, {n2, -dw2, -dh2}};
        for (int i = 0; i < 8; ++i)
        addVertex(vertices, colors, frustumOutline[i], frustumColor);
    }

    mMesh = wr_static_mesh_line_set_new(vertices.size() / 3, &vertices[0], &colors[0]);
    wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));

    if (isRangeFinder())
        wr_renderable_set_visibility_flags(mRenderable, WbWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS);
    else
        wr_renderable_set_visibility_flags(mRenderable, WbWrenRenderingContext::VF_CAMERA_FRUSTUMS);

    wr_node_set_visible(WR_NODE(mTransform), true);
  }

  minRange() {}
  maxRange() { 
    return 1.0;
 }

 static addVertex(vertices, colors, vertex, color) {
  vertices.append(vertex[0]);
  vertices.append(vertex[1]);
  vertices.append(vertex[2]);

  colors.append(color[0]);
  colors.append(color[1]);
  colors.append(color[2]);
}
}
