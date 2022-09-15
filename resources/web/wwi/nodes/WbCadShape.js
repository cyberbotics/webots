import WbBaseNode from './WbBaseNode.js';
import WbImageTexture from './WbImageTexture.js';
import WbPbrAppearance from './WbPbrAppearance.js';
import {arrayXPointerFloat, arrayXPointerInt, getAnId} from './utils/utils.js';
import WbMatrix4 from './utils/WbMatrix4.js';
import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';
import WbWrenPicker from '../wren/WbWrenPicker.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import {webots} from '../webots.js';

import {loadImageTextureInWren} from '../image_loader.js';

export default class WbCadShape extends WbBaseNode {
  constructor(id, urls, ccw, castShadows, isPickable, prefix) {
    super(id);

    this.materials = [];
    for (let i = 0; i < urls.length; ++i) {
      if (urls[i].endsWith('.obj') || urls[i].endsWith('.dae'))
        this.url = urls[i];
      else if (urls[i].endsWith('.mtl'))
        this.materials.push(urls[i]);
      else
        console.error('Unknown file provided to CadShape node: ' + urls[i]);
    }
    this.isCollada = this.url.endsWith('.dae');

    if (typeof this.url === 'undefined') { // no '.dae' or '.obj' was provided
      console.error('Invalid url "' + this.url +
        '". CadShape node expects file in Collada (".dae") or Wavefront (".obj") format.');
      return;
    }

    this.prefix = prefix;

    this.ccw = ccw;
    this.castShadows = castShadows;
    this.isPickable = isPickable;

    this.wrenRenderables = [];
    this.wrenMeshes = [];
    this.wrenMaterials = [];
    this.wrenTransforms = [];
    this.pbrAppearances = [];

    this._promises = [];
  }

  clone(customID) {
    urls = [this.url];
    urls = urls.concat(this.materials);
    const cadShape = new WbCadShape(customID, urls, this.ccw, this.castShadows, this.isPickable, this.prefix);
    this.useList.push(customID);
    return cadShape;
  }

  delete() {
    if (this.wrenObjectsCreatedCalled)
      this.deleteWrenObjects();

    super.delete();
  }

  createWrenObjects() {
    this.deleteWrenObjects();

    super.createWrenObjects();

    if (typeof this.url === 'undefined' || this.scene === 'undefined')
      return;

    // Assimp fix for up_axis, adapted from https://github.com/assimp/assimp/issues/849
    if (this.isCollada) { // rotate around X by 90° to swap Y and Z axis
      // if it is already a WbMatrix4 it means that it is a USE node where the fix has already been applied
      if (!(this.scene.rootnode.transformation instanceof WbMatrix4)) {
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
        const vertices = mesh.vertices.length / 3;
        if (vertices < 3) // silently ignore meshes with less than 3 vertices as they are invalid
          continue;

        if (vertices > 100000)
          console.warn('Mesh ' + mesh.name +
            ' has more than 100\'000 vertices, it is recommended to reduce the number of vertices.');

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
          const vertice = transform.mulByVec4(new WbVector4(mesh.vertices[j * 3], mesh.vertices[(j * 3) + 1],
            mesh.vertices[(j * 3) + 2], 1));
          coordData.push(vertice.x);
          coordData.push(vertice.y);
          coordData.push(vertice.z);
          // extract the normal
          const normal = transform.mulByVec4(new WbVector4(mesh.normals[j * 3], mesh.normals[(j * 3) + 1],
            mesh.normals[(j * 3) + 2], 0));
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

        const staticMesh = _wr_static_mesh_new(vertices, indexData.length, coordDataPointer, normalDataPointer,
          texCoordDataPointer, texCoordDataPointer, indexDataPointer, false);

        this.wrenMeshes.push(staticMesh);

        // retrieve material properties
        const material = this.scene.materials[mesh.materialindex];

        // init from assimp material
        let pbrAppearance = this._createPbrAppearance(material, mesh.materialindex);
        pbrAppearance.preFinalize();
        pbrAppearance.postFinalize();

        let wrenMaterial = _wr_pbr_material_new();
        pbrAppearance.modifyWrenMaterial(wrenMaterial);

        this.pbrAppearances.push(pbrAppearance);
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

    Promise.all(this._promises).then(() => {
      this.actualizeAppearance();
    });
  }

  actualizeAppearance() {
    for (let i = 0; i < this.pbrAppearances.length; i++) {
      this.pbrAppearances[i].modifyWrenMaterial(this.wrenMaterials[i]);
      _wr_renderable_set_material(this.wrenRenderables[i], this.wrenMaterials[i], null);
    }
  }

  deleteWrenObjects() {
    this.wrenRenderables.forEach(renderable => {
      _wr_material_delete(Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'], [renderable, 'picking']));
      _wr_node_delete(renderable);
    });
    this.wrenMeshes.forEach(mesh => { _wr_static_mesh_delete(mesh); });
    this.wrenMaterials.forEach(material => { _wr_material_delete(material); });
    this.pbrAppearances.forEach(appearance => { appearance.delete(); });
    for (let i = this.wrenTransforms.length - 1; i >= 0; --i) {
      this.wrenNode = _wr_node_get_parent(this.wrenTransforms[i]);
      _wr_node_delete(this.wrenTransforms[i]);
    }

    this.wrenRenderables = [];
    this.wrenMeshes = [];
    this.wrenMaterials = [];
    this.wrenTransforms = [];
    this.pbrAppearances = [];
  }

  _createPbrAppearance(material, materialIndex) {
    const properties = new Map(
      material.properties.map(object => {
        if (object.key === '$tex.file')
          return [object.semantic, object.value];
        return [object.key, object.value];
      })
    );

    let baseColor;
    if (typeof properties.get('$clr.diffuse') !== 'undefined')
      baseColor = new WbVector3(properties.get('$clr.diffuse')[0], properties.get('$clr.diffuse')[1],
        properties.get('$clr.diffuse')[2]);
    else
      baseColor = new WbVector3(1.0, 1.0, 1.0);

    let emissiveColor;
    if (typeof properties.get('$clr.emissive') !== 'undefined')
      emissiveColor = new WbVector3(properties.get('$clr.emissive')[0], properties.get('$clr.emissive')[1],
        properties.get('$clr.emissive')[2]);
    else
      emissiveColor = new WbVector3(0.0, 0.0, 0.0);

    let opacity;
    if (typeof properties.get('$mat.opacity') !== 'undefined')
      opacity = properties.get('$mat.opacity');
    else
      opacity = 1.0;
    let transparency = 1.0 - opacity;

    let roughness;
    if (typeof properties.get('$mat.shininess') !== 'undefined')
      roughness = 1.0 - properties.get('$mat.shininess') / 255.0;
    else if (typeof properties.get('$mat.shininess.strength') !== 'undefined')
      roughness = 1.0 - properties.get('$mat.shininess.strength');
    else if (typeof properties.get('$mat.reflectivity') !== 'undefined')
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

    let assetPrefix;
    if (typeof webots.currentView.stream !== 'undefined' || this.url.startsWith("http")) {
      if (this.isCollada) // for collada files, the prefix is extracted from the URL of the '.dae' file
        assetPrefix = this.url.substr(0, this.url.lastIndexOf('/') + 1);
      else // for wavefront files, the prefix is extracted from the URL of the MTL file
        assetPrefix = this.materials[0].substr(0, this.materials[0].lastIndexOf('/') + 1);
    }

    // initialize maps
    let baseColorMap;
    if (properties.get(12))
      baseColorMap = this._createImageTexture(assetPrefix, properties.get(12));
    else if (properties.get(1))
      baseColorMap = this._createImageTexture(assetPrefix, properties.get(1));

    let roughnessMap;
    if (properties.get(16))
      roughnessMap = this._createImageTexture(assetPrefix, properties.get(16));

    let metalnessMap;
    if (properties.get(15))
      metalnessMap = this._createImageTexture(assetPrefix, properties.get(15));

    let normalMap;
    if (properties.get(6))
      normalMap = this._createImageTexture(assetPrefix, properties.get(6));
    else if (properties.get(13))
      normalMap = this._createImageTexture(assetPrefix, properties.get(13));

    let occlusionMap;
    if (properties.get(17))
      occlusionMap = this._createImageTexture(assetPrefix, properties.get(17));
    else if (properties.get(10))
      occlusionMap = this._createImageTexture(assetPrefix, properties.get(10));

    let emissiveColorMap;
    if (properties.get(14))
      emissiveColorMap = this._createImageTexture(assetPrefix, properties.get(14));
    else if (properties.get(4))
      emissiveColorMap = this._createImageTexture(assetPrefix, properties.get(4));

    return new WbPbrAppearance(getAnId(), baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness,
      metalnessMap, iblStrength, normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor,
      emissiveColorMap, emissiveIntensity, undefined);
  }

  _createImageTexture(assetPrefix, imageUrl) {
    // for animations the texture isn't relative to the material but included in the 'textures' folder
    let url;
    if (typeof assetPrefix === 'undefined')
      url = 'textures/' + imageUrl.substring(imageUrl.lastIndexOf('/') + 1);
    else
      url = assetPrefix + imageUrl;

    const imageTexture = new WbImageTexture(getAnId(), url, false, true, true, 4);
    const promise = loadImageTextureInWren(this.prefix, url, false, true);
    promise.then(() => imageTexture.updateUrl());
    this._promises.push(promise);
    return imageTexture;
  }
}
