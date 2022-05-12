import WbBaseNode from './WbBaseNode.js';
import WbPbrAppearance from './WbPbrAppearance.js';
import {arrayXPointerFloat, arrayXPointerInt, getAnId} from './utils/utils.js';
import WbMatrix4 from './utils/WbMatrix4.js';
import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';
import WbWrenPicker from '../wren/WbWrenPicker.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';

export default class WbCadShape extends WbBaseNode {
  constructor(id, urls, ccw, castShadows, isPickable) {
    super(id);

    this.urls = urls;
    this.prefix = this.urls[0].substr(0, this.urls[0].lastIndexOf('/'));
    console.log(this.prefix)
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
    console.log(this.scene);
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

        for (let j = 0; j < vertices / 3; ++j) {
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

        // retrieve material properties
        const material = this.scene.materials[mesh.materialindex];

        // init from assimp material
        let pbrAppearance = this._createPbrAppearance(material);
        // pbrAppearance->preFinalize();
        // pbrAppearance->postFinalize();
        //
        let wrenMaterial = _wr_pbr_material_new();
        // pbrAppearance->modifyWrenMaterial(wrenMaterial);
        //
        // mPbrAppearances.push_back(pbrAppearance);
        this.wrenMaterials.push(wrenMaterial);
      }
    }

    for (let i = 0; i < this.wrenMeshes.length; ++i) {
      let renderable = _wr_renderable_new();
      _wr_renderable_set_material(renderable, this.wrenMaterials[i], null);
      _wr_renderable_set_mesh(renderable, this.wrenMeshes[i]);
      _wr_renderable_set_receive_shadows(renderable, true);
      _wr_renderable_set_visibility_flags(renderable, WbWrenRenderingContext.VM_REGULAR);
      _wr_renderable_set_cast_shadows(renderable, this.castShadows);
      _wr_renderable_invert_front_face(renderable, !this.ccw);
      WbWrenPicker.setPickable(renderable, this.id, this.isPickable);

      let transform = _wr_transform_new();
      _wr_transform_attach_child(this.wrenNode, transform);
      this.wrenNode = transform;
      _wr_transform_attach_child(transform, renderable);
      _wr_node_set_visible(transform, true);

      this.wrenRenderables.push(renderable);
      this.wrenTransforms.push(transform);
    }
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

  _createPbrAppearance(material) {
    const properties = new Map(
      material.properties.map(object => {
        if (object.key === '$tex.file')
          return [object.semantic, object.value];
        return [object.key, object.value];
      })
    );

    let baseColor;
    if (properties.get('$clr.diffuse'))
      baseColor = new WbVector3(properties.get('$clr.diffuse')[0], properties.get('$clr.diffuse')[1],properties.get('$clr.diffuse')[3])
    else
      baseColor = new WbVector3(1.0, 1.0, 1.0);

    let emissiveColor;
    if (properties.get('$clr.emissive'))
      emissiveColor = new WbVector3(properties.get('$clr.emissive')[0], properties.get('$clr.emissive')[1],properties.get('$clr.emissive')[3])
    else
      emissiveColor = new WbVector3(0.0, 0.0, 0.0);

    let opacity;
    if (properties.get('$mat.opacity'))
      opacity = properties.get('$mat.opacity');
    else
      opacity = 1.0;
    let transparency = 1.0 - opacity;

    let roughness;
    if (properties.get('$mat.shininess'))
      roughness = 1.0 - properties.get('$mat.shininess') / 255.0;
    else if (properties.get('$mat.shininess.strength'))
      roughness = 1.0 - properties.get('$mat.shininess.strength');
    else if (properties.get('$mat.reflectivity'))
      roughness = 1.0 - properties.get('$mat.reflectivity');
    else
      roughness = 1.0;

    let metalness = 0.0;
    let iblStrength = 1.0;
    let normalMapFactor = 1.0;
    let occlusionMapStrength = 1.0;
    let emissiveIntensity = 1.0;


    /* Semantic (source https://github.com/assimp/assimp/blob/master/include/assimp/material.h):
      1: diffuse (map_Kd)
      4: emissive (map_emissive or map_Ke)
      6: normal (norm or map_Kn)
      10: lightmap
      12: base color
      13: normal_camera
      14: emission_color
      15: metalness (map_Pm)
      16: (diffuse_) roughness (map_Pr)
      17: ambient occlusion
    */

    // initialize maps
    let baseColorMap;
    if (properties.get(12))
      baseColorMap = this._createImageTexture(properties.get(12));
    else if (properties.get(1))
      baseColorMap = this._createImageTexture(properties.get(1));

    let roughnessMap;
    if (properties.get(16))
      roughnessMap = this._createImageTexture(properties.get(16));

    let metalnessMap;
    if (properties.get(15))
      metalnessMap = this._createImageTexture(properties.get(15));

    let normalMap;
    if (properties.get(6))
      normalMap = this._createImageTexture(properties.get(6));
    else if (properties.get(13))
      normalMap = this._createImageTexture(properties.get(13));

    let occlusionMap;
    if (properties.get(17))
      occlusionMap = this._createImageTexture(properties.get(17));
    else if (properties.get(10))
      occlusionMap = this._createImageTexture(properties.get(10));

    let emissiveColorMap;
    if (properties.get(14))
      emissiveColorMap = this._createImageTexture(properties.get(14));
    else if (properties.get(4))
      emissiveColorMap = this._createImageTexture(properties.get(4));

    return new WbPbrAppearance(getAnId(), baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness, metalnessMap, iblStrength, normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor, emissiveColorMap, emissiveIntensity, undefined);
  }

  _createImageTexture() {
    // aiString path("");
    // material->GetTexture(textureType, 0, &path);
    // // generate url of texture from url of collada/wavefront file
    // QString relativePath = QString(path.C_Str());
    //
    // relativePath.replace("\\", "/");  // use cross-platform forward slashes
    // while (relativePath.startsWith("../")) {
    //   parentPath = parentPath.left(parentPath.lastIndexOf("/"));
    //   relativePath.remove(0, 3);
    // }
    //
    // if (!relativePath.startsWith("/"))
    //   relativePath.insert(0, '/');
    //
    // mUrl = new WbMFString(QStringList(WbUrl::computePath(this, "url", parentPath + relativePath, false)));
    // // init remaining variables with default wrl values
    // mRepeatS = new WbSFBool(true);
    // mRepeatT = new WbSFBool(true);
    // mFiltering = new WbSFInt(4);
  }
}
