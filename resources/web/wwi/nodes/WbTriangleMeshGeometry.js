import {arrayXPointerFloat, arrayXPointerInt} from './utils/utils.js';
import WbGeometry from './WbGeometry.js';
import WbMatrix4 from './utils/WbMatrix4.js';
import WbTriangleMesh from './utils/WbTriangleMesh.js';
import WbVector3 from './utils/WbVector3.js';
import WbWrenMeshBuffers from './utils/WbWrenMeshBuffers.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';

export default class WbTriangleMeshGeometry extends WbGeometry {
  #normalsMaterial;
  #normalsRenderable;
  createWrenObjects() {
    if (this.wrenObjectsCreatedCalled)
      return;

    super.createWrenObjects();

    this._buildWrenMesh(false);
  }

  delete() {
    super.delete();

    _wr_static_mesh_delete(this._wrenMesh);

    this._deleteWrenRenderable();
  }

  preFinalize() {
    if (this.isPreFinalizedCalled)
      return;

    super.preFinalize();

    this.#createTriangleMesh();
  }

  recomputeBoundingSphere() {
    this._boundingSphere.empty();
    if (typeof this._triangleMesh === 'undefined' || this._triangleMesh.numberOfTriangles === 0)
      return;

    // Ritter's bounding sphere approximation:
    // 1. Pick a point x from P, search a point y in P, which has the largest distance from x;
    // 2. Search a point z in P, which has the largest distance from y. set up an
    //    initial sphere B, with its centre as the midpoint of y and z, the radius as
    //    half of the distance between y and z;
    // 3. If all points in P are within sphere B, then we get a bounding sphere.
    //    Otherwise, let p be a point outside the sphere, construct a new sphere covering
    //    both point p and previous sphere. Repeat this step until all points are covered.
    // Note that steps 1. and 2. help in computing a better fitting (smaller) sphere by
    // estimating the center of the final sphere and thus reducing the bias due to the enclosed
    // vertices order.
    const nbTriangles = this._triangleMesh.numberOfTriangles;
    let p2 = new WbVector3(this._triangleMesh.vertex(0, 0, 0), this._triangleMesh.vertex(0, 0, 1),
      this._triangleMesh.vertex(0, 0, 2));
    let p1;
    let maxDistance; // squared distance
    for (let i = 0; i < 2; ++i) {
      maxDistance = 0.0;
      p1 = p2;
      for (let t = 0; t < nbTriangles; ++t) {
        for (let v = 0; v < 3; ++v) {
          const point = new WbVector3(this._triangleMesh.vertex(t, v, 0), this._triangleMesh.vertex(t, v, 1),
            this._triangleMesh.vertex(t, v, 2));
          const d = p1.distance2(point);
          if (d > maxDistance) {
            maxDistance = d;
            p2 = point;
          }
        }
      }
    }
    this._boundingSphere.set(p2.add(p1).mul(0.5), Math.sqrt(maxDistance) * 0.5);

    for (let t = 0; t < nbTriangles; ++t) {
      for (let v = 0; v < 3; ++v) {
        const point = new WbVector3(this._triangleMesh.vertex(t, v, 0), this._triangleMesh.vertex(t, v, 1),
          this._triangleMesh.vertex(t, v, 2));
        this._boundingSphere.enclose(point);
      }
    }
  }

  // Private functions

  #buildGeomIntoBuffers(buffers, m, generateUserTexCoords) {
    if (!this._triangleMesh.isValid || typeof buffers === 'undefined')
      return;

    const rm = m.extracted3x3Matrix();
    const n = this._triangleMesh.numberOfTriangles;

    let start = buffers.vertexIndex / 3;
    const vBuf = buffers.vertexBuffer;
    if (typeof vBuf !== 'undefined') {
      let i = buffers.vertexIndex;
      for (let t = 0; t < n; ++t) { // foreach triangle
        for (let v = 0; v < 3; ++v) { // foreach vertex
          WbWrenMeshBuffers.writeCoordinates(this._triangleMesh.vertex(t, v, 0), this._triangleMesh.vertex(t, v, 1),
            this._triangleMesh.vertex(t, v, 2), m, vBuf, i);
          i += 3;
        }
      }
    }

    const nBuf = buffers.normalBuffer;
    if (typeof nBuf !== 'undefined') {
      let i = buffers.vertexIndex;
      for (let t = 0; t < n; ++t) { // foreach triangle
        for (let v = 0; v < 3; ++v) { // foreach vertex
          WbWrenMeshBuffers.writeNormal(this._triangleMesh.normal(t, v, 0), this._triangleMesh.normal(t, v, 1),
            this._triangleMesh.normal(t, v, 2), rm, nBuf, i);
          i += 3;
        }
      }
    }

    const tBuf = buffers.texCoordBuffer;
    const utBuf = buffers.unwrappedTexCoordsBuffer;
    if (typeof tBuf !== 'undefined') {
      let i = start * buffers.texCoordSetsCount * 2;
      for (let t = 0; t < n; ++t) { // foreach triangle
        for (let v = 0; v < 3; ++v) { // foreach vertex
          tBuf[i] = this._triangleMesh.textureCoordinate(t, v, 0);
          tBuf[i + 1] = this._triangleMesh.textureCoordinate(t, v, 1);
          if (generateUserTexCoords) {
            utBuf[i] = this._triangleMesh.nonRecursiveTextureCoordinate(t, v, 0);
            utBuf[i + 1] = this._triangleMesh.nonRecursiveTextureCoordinate(t, v, 1);
          } else {
            utBuf[i] = this._triangleMesh.textureCoordinate(t, v, 0);
            utBuf[i + 1] = this._triangleMesh.textureCoordinate(t, v, 1);
          }

          i += 2;
        }
      }
    }

    const iBuf = buffers.indexBuffer;
    if (typeof iBuf !== 'undefined') {
      start = buffers.vertexIndex / 3;
      let i = buffers.index;
      for (let t = 0; t < n; ++t) { // foreach triangle
        for (let v = 0; v < 3; ++v) // foreach vertex
          iBuf[i++] = start + this._triangleMesh.index(t, v);
      }
      buffers.index = i;
    }

    buffers.vertexIndex = buffers.vertexIndex + this.#estimateVertexCount() * 3;
  }

  _buildWrenMesh(updateTriangleMesh) {
    if (updateTriangleMesh)
      this.#createTriangleMesh();

    this._deleteWrenRenderable();

    if (typeof this._wrenMesh !== 'undefined') {
      _wr_static_mesh_delete(this._wrenMesh);
      this._wrenMesh = undefined;
    }

    if (!this._triangleMesh.isValid)
      return;

    const createOutlineMesh = this.isInBoundingObject();

    this._computeWrenRenderable();

    if (!this.ccw)
      _wr_renderable_invert_front_face(this._wrenRenderable, true);

    // normals representation
    this.#normalsMaterial = _wr_phong_material_new();
    _wr_material_set_default_program(this.#normalsMaterial, WbWrenShaders.lineSetShader());
    _wr_phong_material_set_color_per_vertex(this.#normalsMaterial, true);
    _wr_phong_material_set_transparency(this.#normalsMaterial, 0.4);

    this.#normalsRenderable = _wr_renderable_new();
    _wr_renderable_set_cast_shadows(this.#normalsRenderable, false);
    _wr_renderable_set_receive_shadows(this.#normalsRenderable, false);
    _wr_renderable_set_material(this.#normalsRenderable, this.#normalsMaterial, null);
    _wr_renderable_set_visibility_flags(this.#normalsRenderable, WbWrenRenderingContext.VF_NORMALS);
    _wr_renderable_set_drawing_mode(this.#normalsRenderable, Enum.WR_RENDERABLE_DRAWING_MODE_LINES);
    _wr_transform_attach_child(this.wrenNode, this.#normalsRenderable);

    // Restore pickable state
    super.setPickable(this.isPickable);

    const buffers = super._createMeshBuffers(this.#estimateVertexCount(), this.#estimateIndexCount());
    if (typeof buffers !== 'undefined') {
      this.#buildGeomIntoBuffers(buffers, new WbMatrix4(), !this._triangleMesh.areTextureCoordinatesValid);
      const vertexBufferPointer = arrayXPointerFloat(buffers.vertexBuffer);
      const normalBufferPointer = arrayXPointerFloat(buffers.normalBuffer);
      const texCoordBufferPointer = arrayXPointerFloat(buffers.texCoordBuffer);
      const unwrappedTexCoordsBufferPointer = arrayXPointerFloat(buffers.unwrappedTexCoordsBuffer);
      const indexBufferPointer = arrayXPointerInt(buffers.indexBuffer);
      this._wrenMesh = _wr_static_mesh_new(buffers.verticesCount, buffers.indicesCount, vertexBufferPointer,
        normalBufferPointer, texCoordBufferPointer, unwrappedTexCoordsBufferPointer, indexBufferPointer, createOutlineMesh);

      _free(vertexBufferPointer);
      _free(normalBufferPointer);
      _free(texCoordBufferPointer);
      _free(unwrappedTexCoordsBufferPointer);
      _free(indexBufferPointer);

      buffers.clear();
    }

    _wr_renderable_set_mesh(this._wrenRenderable, this._wrenMesh);
  }

  #createTriangleMesh() {
    this._triangleMesh = new WbTriangleMesh();
    this._updateTriangleMesh();
  }

  _deleteWrenRenderable() {
    if (typeof this.#normalsMaterial !== 'undefined') {
      _wr_material_delete(this.#normalsMaterial);
      this.#normalsMaterial = undefined;
    }

    if (typeof this.#normalsRenderable !== 'undefined') {
      _wr_node_delete(this.#normalsRenderable);
      this.#normalsRenderable = undefined;
    }

    super._deleteWrenRenderable();
  }

  #estimateIndexCount() {
    if (!this._triangleMesh.isValid)
      return;

    return 3 * this._triangleMesh.numberOfTriangles;
  }

  #estimateVertexCount() {
    if (!this._triangleMesh.isValid)
      return;

    return 3 * this._triangleMesh.numberOfTriangles;
  }

  _updateTriangleMesh() {}
}
