import WbBaseNode from './WbBaseNode.js';
import {arrayXPointerFloat, arrayXPointerInt} from './utils/utils.js';
import WbMatrix4 from './utils/WbMatrix4.js';
import WbVector4 from './utils/WbVector4.js';

export default class WbCadShape extends WbBaseNode {
  constructor(id, urls, ccw, castShadows, isPickable) {
    super(id);

    this.urls = urls;
    this.ccw = ccw;
    this.castShadows = castShadows;
    this.isPickable = isPickable;

    this.wrenRenderables = [];
    this.wrenMeshes = [];
    this.wrenMaterials = [];
    this.wrenTransforms = [];
    this.pbrAppearances = [];
  }

  clone(customID) {
    const cadShape = new WbCadShape(customID, this.urls, this.ccw, this.castShadows, this.isPickable);
    this.useList.push(customID);
    return cadShape;
  }

  delete() {
    if (this.wrenObjectsCreatedCalled)
      this.deleteWrenObjects();

    super.delete();
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.deleteWrenObjects();

    if (this.urls.length === 0 || this.scene === 'undefined')
      return;

    const extension = this.urls[0].substr(this.urls[0].lastIndexOf('.') + 1, this.urls[0].length).toLowerCase();
    if (extension !== 'dae' && extension !== 'obj') {
      console.error('Invalid url "' + this.urls[0] + '". CadShape node expects file in Collada (".dae") or Wavefront (".obj") format.');
      return;
    }

    // Assimp fix for up_axis, adapted from https://github.com/assimp/assimp/issues/849
    if (extension === 'dae') { // rotate around X by 90° to swap Y and Z axis
      if (!(this.scene.rootnode.transformation instanceof WbMatrix4)) { // if it is already a WbMatrix4 it means that it is a USE node where the fix has already been applied
        let matrix = new WbMatrix4();
        matrix.set(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1);
        let rootMatrix = new WbMatrix4();
        rootMatrix.setFromArray(this.scene.rootnode.transformation);
        this.scene.rootnode.transformation = matrix.mul(rootMatrix);
      }
    }

    let queue = [];
    queue.push(this.scene.rootnode);

    let node;
    while (queue.length !== 0) {
      node = queue.shift();

      for (let i = 0; i < node.meshes.length; ++i) {
        const mesh = this.scene.meshes[i];

        // compute absolute transform of this node from all the parents
        const vertices = mesh.vertices.length;
        if (vertices < 3) // silently ignore meshes with less than 3 vertices as they are invalid
          continue;

        if (vertices > 100000)
          console.warn('Mesh ' + mesh.name + ' has more than 100\'000 vertices, it is recommended to reduce the number of vertices.');

        let transform = new WbMatrix4();
        let current = node;

        while (current) {
          let transformationMatrix;
          if (current.transformation instanceof WbMatrix4)
            transformationMatrix = current.transformation;
          else {
            transformationMatrix = new WbMatrix4();
            transformationMatrix.setFromArray(current.transformation);
          }
          transform = transform.mul(transformationMatrix);
          current = current.parent;
        }

        // create the arrays
        const coordData = [];
        const normalData = [];
        const texCoordData = [];
        const indexData = [];

        for (let j = 0; j < vertices; ++j) {
          // extract the coordinate
          const vertice = transform.mulByVec4(new WbVector4(mesh.vertices[j * 3], mesh.vertices[(j * 3) + 1], mesh.vertices[(j * 3) + 2], 1));
          coordData.push(vertice.x);
          coordData.push(vertice.y);
          coordData.push(vertice.z);
          // extract the normal
          const normal = transform.mulByVec4(new WbVector4(mesh.normals[j * 3], mesh.normals[(j * 3) + 1], mesh.normals[(j * 3) + 2], 0));
          normalData.push(normal.x);
          normalData.push(normal.y);
          normalData.push(normal.z);
          // extract the texture coordinate
          if (mesh.texturecoords && mesh.texturecoords[0]) {
            texCoordData.push(mesh.texturecoords[0][j * 2]);
            texCoordData.push(mesh.texturecoords[0][(j * 2) + 1]);
          } else {
            texCoordData.push(0.5);
            texCoordData.push(0.5);
          }
        }

        // create the index array
        for (let j = 0; j < mesh.faces.length; ++j) {
          // Skip if we do not have triangles (e.g lines)
          const face = mesh.faces[j];

          if (face.length !== 3)
            continue;

          indexData.push(face[0]);
          indexData.push(face[1]);
          indexData.push(face[2]);
        }

        if (indexData.length === 0) // if all faces turned out to be invalid, ignore the mesh
          continue;

        const coordDataPointer = arrayXPointerFloat(coordData);
        const normalDataPointer = arrayXPointerFloat(normalData);
        const texCoordDataPointer = arrayXPointerFloat(texCoordData);
        const indexDataPointer = arrayXPointerInt(indexData);
        const staticMesh = _wr_static_mesh_new(vertices, indexData.length, coordDataPointer, normalDataPointer, texCoordDataPointer, texCoordDataPointer, indexDataPointer, false);

        this.wrenMeshes.push(staticMesh);
        //
        // delete[] coordData;
        // delete[] normalData;
        // delete[] texCoordData;
        // delete[] indexData;
        //
        // // retrieve material properties
        // const aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
        //
        // // determine how image textures referenced in the collada file will be searched for
        // QString fileRoot = mUrl->item(0);
        // fileRoot = fileRoot.replace("\\", "/");               // use cross-platform forward slashes
        // fileRoot = fileRoot.left(fileRoot.lastIndexOf("/"));  // do not include the final forward slash
        //
        // // init from assimp material
        // WbPbrAppearance *pbrAppearance = new WbPbrAppearance(material, fileRoot);
        // pbrAppearance->preFinalize();
        // pbrAppearance->postFinalize();
        // connect(pbrAppearance, &WbPbrAppearance::changed, this, &WbCadShape::updateAppearance);
        //
        // WrMaterial *wrenMaterial = wr_pbr_material_new();
        // pbrAppearance->modifyWrenMaterial(wrenMaterial);
        //
        // mPbrAppearances.push_back(pbrAppearance);
        // mWrenMaterials.push_back(wrenMaterial);
      }
    }
    //
    // for (int i = 0; i < mWrenMeshes.size(); ++i) {
    //   WrRenderable *renderable = wr_renderable_new();
    //   wr_renderable_set_material(renderable, mWrenMaterials[i], NULL);
    //   wr_renderable_set_mesh(renderable, WR_MESH(mWrenMeshes[i]));
    //   wr_renderable_set_receive_shadows(renderable, true);
    //   wr_renderable_set_visibility_flags(renderable, WbWrenRenderingContext::VM_REGULAR);
    //   wr_renderable_set_cast_shadows(renderable, mCastShadows->value());
    //   wr_renderable_invert_front_face(renderable, !mCcw->value());
    //   WbWrenPicker::setPickable(renderable, uniqueId(), mIsPickable->value());
    //
    //   // set material for range finder camera rendering
    //   WrMaterial *depthMaterial = wr_phong_material_new();
    //   wr_material_set_default_program(depthMaterial, WbWrenShaders::encodeDepthShader());
    //   wr_renderable_set_material(renderable, depthMaterial, "encodeDepth");
    //
    //   // set material for segmentation camera rendering
    //   WrMaterial *segmentationMaterial = wr_phong_material_new();
    //   wr_material_set_default_program(segmentationMaterial, WbWrenShaders::segmentationShader());
    //   wr_renderable_set_material(renderable, segmentationMaterial, "segmentation");
    //
    //   WrTransform *transform = wr_transform_new();
    //   wr_transform_attach_child(wrenNode(), WR_NODE(transform));
    //   setWrenNode(transform);
    //   wr_transform_attach_child(transform, WR_NODE(renderable));
    //   wr_node_set_visible(WR_NODE(transform), true);
    //
    //   mWrenRenderables.push_back(renderable);
    //   mWrenTransforms.push_back(transform);
    //   mWrenEncodeDepthMaterials.push_back(depthMaterial);
    //   mWrenSegmentationMaterials.push_back(segmentationMaterial);
    // }
    //
    // if (mBoundingSphere)
    //   recomputeBoundingSphere();
  }

  deleteWrenObjects() {
    this.wrenRenderables.forEach(renderable => {
      _wr_material_delete(Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'], [renderable, 'picking']));
      _wr_node_delete(renderable);
    })
    this.wrenMeshes.forEach(mesh => { _wr_static_mesh_delete(mesh); });
    this.wrenMaterials.forEach(material => { _wr_material_delete(material); });
    this.pbrAppearances.forEach(appearance => { appearance.delete(); });
    this.wrenTransforms.forEach(transform => { _wr_node_delete(transform); });

    this.wrenRenderables = [];
    this.wrenMeshes = [];
    this.wrenMaterials = [];
    this.wrenTransforms = [];
    this.pbrAppearances = [];
  }
}
