import {WbGeometry} from "./WbGeometry.js"
import {WbTriangleMesh} from "./WbTriangleMesh.js"
import {WbWrenShaders} from "./WbWrenShaders.js"


class WbTriangleMeshGeometry extends WbGeometry {
  constructor(id) {
    super(id);
    this.triangleMesh = undefined;
    this.normalsMaterial = undefined;
    this.normalsRenderable = undefined;
  }

  createWrenObjects() {
    //if (!mTriangleMeshError.isEmpty())
      //console.error(mTriangleMeshError);

    super.createWrenObjects();

    buildWrenMesh(false);
  }

  deleteWrenRenderable() {
    if (this.normalsMaterial)
      _wr_material_delete(this.normalsMaterial);
    this.normalsMaterial = undefined;
    if (this.normalsRenderable)
      _wr_node_delete(this.normalsRenderable);
    this.normalsRenderable = undefined;

    super.deleteWrenRenderable();
  }
  buildWrenMesh(updateCache) {
    /*if (updateCache) {
      WbTriangleMeshCache::releaseTriangleMesh(this);
      mMeshKey.set(this);
      WbTriangleMeshCache::useTriangleMesh(this);
    }*/

    this.deleteWrenRenderable();

    _wr_static_mesh_delete(this.wrenMesh);
    this.wrenMesh = undefined;

    if (!this.triangleMesh.isValid)
      return;

    //const createOutlineMesh = isInBoundingObject();

    this.computeWrenRenderable();

    // normals representation
    this.normalsMaterial = _wr_phong_material_new();
    _wr_material_set_default_program(this.normalsMaterial, WbWrenShaders.lineSetShader());
    _wr_phong_material_set_color_per_vertex(this.normalsMaterial, true);
    _wr_phong_material_set_transparency(this.normalsMaterial, 0.4);

    this.normalsRenderable = _wr_renderable_new();
    _wr_renderable_set_cast_shadows(this.normalsRenderable, false);
    _wr_renderable_set_receive_shadows(this.normalsRenderable, false);
    _wr_renderable_set_material(this.normalsRenderable, this.normalsMaterial, null);
    _wr_renderable_set_visibility_flags(this.normalsRenderable, 0x00040000);//ENUM : VF_NORMALS
    _wr_renderable_set_drawing_mode(this.normalsRenderable, ENUM.WR_RENDERABLE_DRAWING_MODE_LINES);
    _wr_transform_attach_child(this.wrenNode, this.normalsRenderable);

    // Restore pickable state
    //setPickable(isPickable());

    let buffers = super.createMeshBuffers(estimateVertexCount(), estimateIndexCount());
    /*this.buildGeomIntoBuffers(buffers, WbMatrix4(), !mTriangleMesh->areTextureCoordinatesValid());

    mWrenMesh = wr_static_mesh_new(buffers->verticesCount(), buffers->indicesCount(), buffers->vertexBuffer(),
                                   buffers->normalBuffer(), buffers->texCoordBuffer(), buffers->unwrappedTexCoordBuffer(),
                                   buffers->indexBuffer(), createOutlineMesh);

    delete buffers;

    wr_renderable_set_mesh(mWrenRenderable, WR_MESH(mWrenMesh));
    updateNormalsRepresentation();*/
  }
}

export {WbTriangleMeshGeometry}
