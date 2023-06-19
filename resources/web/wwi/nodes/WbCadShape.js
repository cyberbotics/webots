import WbBaseNode from './WbBaseNode.js';
import WbImageTexture from './WbImageTexture.js';
import WbPbrAppearance from './WbPbrAppearance.js';
import WbWorld from './WbWorld.js';
import { getAnId } from './utils/id_provider.js';
import { arrayXPointerFloat, arrayXPointerInt } from './utils/utils.js';
import WbBoundingSphere from './utils/WbBoundingSphere.js';
import WbMatrix4 from './utils/WbMatrix4.js';
import WbVector3 from './utils/WbVector3.js';
import WbVector4 from './utils/WbVector4.js';
import WbWrenPicker from '../wren/WbWrenPicker.js';
import WbWrenRenderingContext from '../wren/WbWrenRenderingContext.js';
import { WbNodeType } from './wb_node_type.js';

import MeshLoader from '../MeshLoader.js';
import ImageLoader from '../ImageLoader.js';

export default class WbCadShape extends WbBaseNode {
  #boundingSphere;
  #ccw;
  #castShadows;
  #isCollada;
  #isPickable;
  #pbrAppearances;
  #promises;
  #url;
  #wrenMaterials;
  #wrenMeshes;
  #wrenRenderables;
  #wrenTransforms;
  constructor(id, url, ccw, castShadows, isPickable, prefix) {
    super(id);
    if (url.toLowerCase().endsWith('.obj') || url.toLowerCase().endsWith('.dae'))
      this.#url = url;

    if (typeof this.#url === 'undefined') { // no '.dae' or '.obj' was provided
      console.error('Invalid URL. CadShape node expects file in Collada (".dae") or Wavefront (".obj") format.');
      return;
    }

    this.#isCollada = this.#url.toLowerCase().endsWith('.dae');

    this.prefix = prefix;

    this.#ccw = ccw;
    this.#castShadows = castShadows;
    this.#isPickable = isPickable;

    this.#wrenRenderables = [];
    this.#wrenMeshes = [];
    this.#wrenMaterials = [];
    this.#wrenTransforms = [];
    this.#pbrAppearances = [];

    this.#promises = [];
  }

  get nodeType() {
    return WbNodeType.WB_NODE_CAD_SHAPE;
  }

  get ccw() {
    return this.#ccw;
  }

  set ccw(newCcw) {
    this.#ccw = newCcw;

    this.#updateCcw();
  }

  get castShadows() {
    return this.#castShadows;
  }

  set castShadows(newCastShadows) {
    this.#castShadows = newCastShadows;

    this.#updateCastShadows();
  }

  get isPickable() {
    return this.#isPickable;
  }

  set isPickable(newIsPickable) {
    this.#isPickable = newIsPickable;

    this.#updateIsPickable();
  }

  get url() {
    return this.#url;
  }

  set url(newUrl) {
    if (newUrl.toLowerCase().endsWith('.obj') || newUrl.toLowerCase().endsWith('.dae'))
      this.#url = newUrl;
    else
      console.error('Unknown file provided to CadShape node: ' + newUrl);

    if (typeof this.#url === 'undefined') { // no '.dae' or '.obj' was provided
      console.error('Invalid URL. CadShape node expects file in Collada (".dae") or Wavefront (".obj") format.');
      return;
    }

    this.#updateUrl();
  }

  absoluteScale() {
    const ut = this.upperTransform;
    return ut ? ut.absoluteScale() : new WbVector3(1.0, 1.0, 1.0);
  }

  boundingSphere() {
    return this.#boundingSphere;
  }

  clone(customID) {
    const cadShape = new WbCadShape(customID, this.#url, this.#ccw, this.#castShadows, this.#isPickable, this.prefix);
    cadShape.materialPath = this.materialPath;
    this.useList.push(customID);
    return cadShape;
  }

  delete() {
    super.delete();

    if (this.wrenObjectsCreatedCalled)
      this.deleteWrenObjects();

    if (typeof this.parent === 'undefined') {
      const index = WbWorld.instance.root.children.indexOf(this);
      WbWorld.instance.root.children.splice(index, 1);
    } else {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        if (typeof parent.endPoint !== 'undefined')
          parent.endPoint = undefined;
        else {
          const index = parent.children.indexOf(this);
          parent.children.splice(index, 1);
        }
      }
    }
  }

  createWrenObjects() {
    this.deleteWrenObjects();

    super.createWrenObjects();

    if (typeof this.#url === 'undefined' || this.scene === 'undefined')
      return;

    // Assimp fix for up_axis, adapted from https://github.com/assimp/assimp/issues/849
    if (this.#isCollada) { // rotate around X by 90° to swap Y and Z axis
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

        this.#wrenMeshes.push(staticMesh);

        // retrieve material properties
        const material = this.scene.materials[mesh.materialindex];

        // init from assimp material
        let pbrAppearance = this.#createPbrAppearance(material, mesh.materialindex);
        pbrAppearance.preFinalize();
        pbrAppearance.postFinalize();

        let wrenMaterial = _wr_pbr_material_new();
        pbrAppearance.modifyWrenMaterial(wrenMaterial);

        this.#pbrAppearances.push(pbrAppearance);
        this.#wrenMaterials.push(wrenMaterial);
      }
    }

    for (let i = 0; i < this.#wrenMeshes.length; ++i) {
      let renderable = _wr_renderable_new();
      _wr_renderable_set_material(renderable, this.#wrenMaterials[i], null);
      _wr_renderable_set_mesh(renderable, this.#wrenMeshes[i]);
      _wr_renderable_set_receive_shadows(renderable, true);
      _wr_renderable_set_visibility_flags(renderable, WbWrenRenderingContext.VM_REGULAR);
      _wr_renderable_set_cast_shadows(renderable, this.#castShadows);
      _wr_renderable_invert_front_face(renderable, !this.#ccw);
      WbWrenPicker.setPickable(renderable, this.id, this.#isPickable);

      let transform = _wr_transform_new();
      _wr_transform_attach_child(this.wrenNode, transform);
      this.wrenNode = transform;
      _wr_transform_attach_child(transform, renderable);
      _wr_node_set_visible(transform, true);

      this.#wrenRenderables.push(renderable);
      this.#wrenTransforms.push(transform);
    }

    Promise.all(this.#promises).then(() => {
      this.actualizeAppearance();
    });

    this.#boundingSphere = new WbBoundingSphere(this);
    this.#recomputeBoundingSphere();
  }

  actualizeAppearance() {
    for (let i = 0; i < this.#pbrAppearances.length; i++) {
      this.#pbrAppearances[i].modifyWrenMaterial(this.#wrenMaterials[i]);
      _wr_renderable_set_material(this.#wrenRenderables[i], this.#wrenMaterials[i], null);
    }
  }

  deleteWrenObjects() {
    this.#wrenRenderables?.forEach(renderable => {
      _wr_material_delete(Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'], [renderable, 'picking']));
      _wr_node_delete(renderable);
    });
    this.#wrenMeshes?.forEach(mesh => { _wr_static_mesh_delete(mesh); });
    this.#wrenMaterials?.forEach(material => { _wr_material_delete(material); });
    this.#pbrAppearances?.forEach(appearance => { appearance.delete(); });
    for (let i = (this.#wrenTransforms || []).length - 1; i >= 0; --i) {
      this.wrenNode = _wr_node_get_parent(this.#wrenTransforms[i]);
      _wr_node_delete(this.#wrenTransforms[i]);
    }

    this.#wrenRenderables = [];
    this.#wrenMeshes = [];
    this.#wrenMaterials = [];
    this.#wrenTransforms = [];
    this.#pbrAppearances = [];
  }

  #recomputeBoundingSphere() {
    this.#boundingSphere.empty();

    for (let i = 0; i < this.#wrenMeshes.length; i++) {
      let sphere = [0, 0, 0, 0];
      const spherePointer = arrayXPointerFloat(sphere);
      _wr_static_mesh_get_bounding_sphere(this.#wrenMeshes[i], spherePointer);
      sphere[0] = Module.getValue(spherePointer, 'float');
      sphere[1] = Module.getValue(spherePointer + 4, 'float');
      sphere[2] = Module.getValue(spherePointer + 8, 'float');
      sphere[3] = Module.getValue(spherePointer + 12, 'float');

      const center = new WbVector3(sphere[0], sphere[1], sphere[2]);
      this.#boundingSphere.set(center, sphere[3]);
    }
  }

  #createPbrAppearance(material, materialIndex) {
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
    if (WbCadShape.stream || this.#url.startsWith('http')) {
      if (this.#isCollada) // for collada files, the prefix is extracted from the URL of the '.dae' file
        assetPrefix = this.#url.substr(0, this.#url.lastIndexOf('/') + 1);
      else // for wavefront files, the prefix is extracted from the URL of the MTL file
        assetPrefix = this.materialPath;
    }

    // initialize maps
    let baseColorMap;
    if (properties.get(12))
      baseColorMap = this.#createImageTexture(assetPrefix, properties.get(12));
    else if (properties.get(1))
      baseColorMap = this.#createImageTexture(assetPrefix, properties.get(1));

    let roughnessMap;
    if (properties.get(16))
      roughnessMap = this.#createImageTexture(assetPrefix, properties.get(16));

    let metalnessMap;
    if (properties.get(15))
      metalnessMap = this.#createImageTexture(assetPrefix, properties.get(15));

    let normalMap;
    if (properties.get(6))
      normalMap = this.#createImageTexture(assetPrefix, properties.get(6));
    else if (properties.get(13))
      normalMap = this.#createImageTexture(assetPrefix, properties.get(13));

    let occlusionMap;
    if (properties.get(17))
      occlusionMap = this.#createImageTexture(assetPrefix, properties.get(17));
    else if (properties.get(10))
      occlusionMap = this.#createImageTexture(assetPrefix, properties.get(10));

    let emissiveColorMap;
    if (properties.get(14))
      emissiveColorMap = this.#createImageTexture(assetPrefix, properties.get(14));
    else if (properties.get(4))
      emissiveColorMap = this.#createImageTexture(assetPrefix, properties.get(4));

    return new WbPbrAppearance(getAnId(), baseColor, baseColorMap, transparency, roughness, roughnessMap, metalness,
      metalnessMap, iblStrength, normalMap, normalMapFactor, occlusionMap, occlusionMapStrength, emissiveColor,
      emissiveColorMap, emissiveIntensity, undefined);
  }

  #createImageTexture(assetPrefix, imageUrl) {
    // for animations the texture isn't relative to the material but included in the 'textures' folder
    let url;
    if (typeof assetPrefix === 'undefined')
      url = 'textures/' + imageUrl.substring(imageUrl.lastIndexOf('/') + 1);
    else
      url = assetPrefix + imageUrl;

    const imageTexture = new WbImageTexture(getAnId(), url, false, true, true, 4);
    const promise = ImageLoader.loadImageTextureInWren(imageTexture, this.prefix, url);
    promise.then(() => imageTexture.updateUrl());
    this.#promises.push(promise);
    return imageTexture;
  }

  #updateCcw() {
    this.#wrenRenderables.forEach(renderable => _wr_renderable_invert_front_face(renderable, !this.#ccw));
  }

  #updateCastShadows() {
    this.#wrenRenderables.forEach(renderable => _wr_renderable_set_cast_shadows(renderable, this.#castShadows));
  }

  #updateIsPickable() {
    this.#wrenRenderables.forEach(renderable => WbWrenPicker.setPickable(renderable, this.id, this.#isPickable));
  }

  #updateUrl() {
    if (this.#url)
      this.#isCollada = this.#url.toLowerCase().endsWith('.dae');

    MeshLoader.loadMeshData(WbWorld.instance.prefix, this.#url).then(meshContent => {
      this.scene = meshContent[0];
      this.materialPath = meshContent[1];
      if (this.wrenObjectsCreatedCalled) {
        if (typeof this.scene === 'undefined')
          this.deleteWrenObjects();
        else
          this.createWrenObjects();
      }
    });
  }
}
