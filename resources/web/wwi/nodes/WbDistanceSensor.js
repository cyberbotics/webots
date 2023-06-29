import WbLookupTable from './WbLookupTable.js';
import WbDevice from './WbDevice.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import { arrayXPointerFloat } from './utils/utils.js';
import WbVector3 from './utils/WbVector3.js';
import { WbNodeType } from './wb_node_type.js';

export default class WbDistanceSensor extends WbDevice {
  #aperture;
  #lookupTable;
  #lut;
  #material;
  #mesh;
  #nRays;
  #NUM_PREDEFINED;
  #numberOfRays;
  #POLAR;
  #rays;
  #renderable;
  #transform;
  constructor(id, translation, rotation, name, numberOfRays, aperture, lookupTable) {
    super(id, translation, rotation, name);
    this.#aperture = aperture;
    this.#lookupTable = lookupTable;
    this.#numberOfRays = numberOfRays;

    this.#NUM_PREDEFINED = 10;
    const HALF = Math.PI;
    const THIRD = 2 * Math.PI / 3;
    const QUARTER = Math.PI / 2;
    const FIFTH = 2 * Math.PI / 5;
    const SIXTH = Math.PI / 3;
    const SEVENTH = 2 * Math.PI / 7;
    this.#POLAR = [
      [[0, 0]],
      [[QUARTER, 1], [-QUARTER, 1]],
      [[0, 1], [THIRD, 1], [-THIRD, 1]],
      [[0, 0], [0, 1], [THIRD, 1], [-THIRD, 1]],
      [[0, 0], [0, 1], [QUARTER, 1], [HALF, 1], [-QUARTER, 1]],
      [[0, 0], [0, 1], [FIFTH, 1], [2 * FIFTH, 1], [3 * FIFTH, 1], [4 * FIFTH, 1]],
      [[0, 0], [0, 1], [SIXTH, 1], [2 * SIXTH, 1], [3 * SIXTH, 1], [4 * SIXTH, 1], [5 * SIXTH, 1]],
      [[0, 0], [0, 1], [SEVENTH, 1], [2 * SEVENTH, 1], [3 * SEVENTH, 1], [4 * SEVENTH, 1], [5 * SEVENTH, 1], [6 * SEVENTH, 1]],
      [[0, 0.3], [THIRD, 0.3], [-THIRD, 0.3], [0, 1], [SIXTH, 1], [2 * SIXTH, 1], [3 * SIXTH, 1], [4 * SIXTH, 1],
        [5 * SIXTH, 1]],
      [[0, 0], [0, 0.5], [THIRD, 0.5], [-THIRD, 0.5], [0, 1], [SIXTH, 1], [2 * SIXTH, 1], [3 * SIXTH, 1], [4 * SIXTH, 1],
        [5 * SIXTH, 1]]];
  }

  get nodeType() {
    return WbNodeType.WB_NODE_DISTANCE_SENSOR;
  }

  get aperture() {
    return this.#aperture;
  }

  set aperture(newAperture) {
    if (newAperture < 0)
      newAperture = -newAperture;
    this.#aperture = newAperture;

    this.#updateRaySetup();
  }

  get lookupTable() {
    return this.#lookupTable;
  }

  set lookupTable(newLookupTable) {
    this.#lookupTable = newLookupTable;
    this.#updateRaySetup();
  }

  get numberOfRays() {
    return this.#numberOfRays;
  }

  set numberOfRays(newNumberOfRays) {
    if (newNumberOfRays < 1)
      newNumberOfRays = 1;

    this.#numberOfRays = newNumberOfRays;
    this.#updateRaySetup();
  }

  applyOptionalRendering(enable) {
    _wr_node_set_visible(this.#transform, enable);
  }

  createWrenObjects() {
    // Optional rendering
    this.#material = _wr_phong_material_new();
    _wr_phong_material_set_color_per_vertex(this.#material, true);
    _wr_material_set_default_program(this.#material, WbWrenShaders.lineSetShader());
    this.#mesh = _wr_dynamic_mesh_new(false, false, true);
    this.#renderable = _wr_renderable_new();
    _wr_renderable_set_cast_shadows(this.#renderable, false);
    _wr_renderable_set_receive_shadows(this.#renderable, false);
    _wr_renderable_set_mesh(this.#renderable, this.#mesh);
    _wr_renderable_set_visibility_flags(this.#renderable, WbWrenRenderingContext.VM_REGULAR);
    _wr_renderable_set_drawing_mode(this.#renderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);
    _wr_renderable_set_material(this.#renderable, this.#material, undefined);
    this.#transform = _wr_transform_new();
    _wr_transform_attach_child(this.#transform, this.#renderable);

    _wr_node_set_visible(this.#transform, false);

    super.createWrenObjects();
    _wr_transform_attach_child(this.wrenNode, this.#transform);

    this.#applyOptionalRenderingToWren();
  }

  delete() {
    if (this.wrenObjectsCreatedCalled) {
      _wr_node_delete(this.#renderable);
      _wr_node_delete(this.#transform);
      _wr_material_delete(this.#material);
      _wr_dynamic_mesh_delete(this.#mesh);
    }

    super.delete();
  }

  preFinalize() {
    super.preFinalize();
    this.#updateRaySetup();
  }

  #applyOptionalRenderingToWren() {
    if (!this.wrenObjectsCreatedCalled)
      return;

    _wr_dynamic_mesh_clear(this.#mesh);

    if (this.#rays) {
      const minValue = this.#lut.minMetricsRange();
      const maxValue = this.#lut.maxMetricsRange();

      const redColor = [1, 0, 0];
      const colorPointer = arrayXPointerFloat(redColor);

      let vertexIndex = 0;
      for (let i = 0; i < this.#nRays; ++i) {
        const direction = this.#rays[i];
        let vertex = direction.mul(minValue);
        let vertexPointer = _wrjs_array3(vertex.x, vertex.y, vertex.z);
        _wr_dynamic_mesh_add_vertex(this.#mesh, vertexPointer);
        _wr_dynamic_mesh_add_index(this.#mesh, vertexIndex++);
        _wr_dynamic_mesh_add_color(this.#mesh, colorPointer);

        vertex = direction.mul(maxValue);
        vertexPointer = _wrjs_array3(vertex.x, vertex.y, vertex.z);
        _wr_dynamic_mesh_add_vertex(this.#mesh, vertexPointer);
        _wr_dynamic_mesh_add_index(this.#mesh, vertexIndex++);
        _wr_dynamic_mesh_add_color(this.#mesh, colorPointer);
      }
      _free(colorPointer);
    }
  }

  #updateRaySetup() {
    // rebuild the lookup table
    this.#lut = new WbLookupTable(this.#lookupTable);

    // if a 0 aperture is specified together with multiple rays then improve performance by casting one single ray only
    this.#nRays = (this.#numberOfRays > 1 && this.#aperture === 0) ? 1 : this.#numberOfRays;

    this.#rays = [];

    this.#setupRayDirs();

    if (this.wrenObjectsCreatedCalled)
      this.#applyOptionalRenderingToWren();
  }

  #setupRayDirs() {
    if (this.#nRays === 1)
      this.#rays[0] = new WbVector3(1, 0, 0);
    else {
      if (this.#nRays > this.#NUM_PREDEFINED) {
        // Not a predefined configuration: arrange rays in a 3d cone oriented towards x. The cone is further divided in a
        // number of thinner cones that will accomodate some rays.
        let left = this.#nRays; // number of rays left to be assigned
        let ncone; // number of orbits
        let s = 0; // number of so far assigned rays

        const alphas = [];
        const thetas = [];

        // as long as there are rays left to be assigned
        for (ncone = 0; left > 0; ncone++) {
          const capacity = 3 * ncone + 1;
          const m = (left > capacity) ? capacity : left;
          const twoPiOverM = 2.0 * Math.PI / m;

          // within a cone arrange the rays in a circle
          for (let i = 0; i < m; i++) {
            alphas[s] = s * twoPiOverM;
            thetas[s] = ncone;
            s++;
          }

          left -= capacity;
        }

        for (let i = 0; i < this.#nRays; i++) {
          // rescale the cones to fit all rays
          thetas[i] /= (ncone - 1);

          // convert from polar to 3d coordinate system
          this.#polarTo3d(alphas[i], thetas[i], i);
        }
      } else {
        for (let i = 0; i < this.#nRays; i++)
          // convert from polar to 3d coordinate system
          this.#polarTo3d(this.#POLAR[this.#nRays - 1][i][0], this.#POLAR[this.#nRays - 1][i][1], i);
      }
    }
  }

  #polarTo3d(alpha, theta, i) {
    // rotate the cone so that its initial angle looks upwards, that ways we obtain a left/right symmetry
    alpha += Math.PI / 2;

    // rescale the cone tip angle to fit all rays in the user-defined aperture angle
    theta *= this.#aperture / 2;

    // first rotate around x-axis which is the sensors central ray axis
    const x = Math.cos(theta);
    let y = -Math.sin(theta);

    // then rotate around z-axis
    const z = -y * Math.sin(alpha);
    y *= Math.cos(alpha);

    this.#rays[i] = new WbVector3(x, y, z);
  }
}
