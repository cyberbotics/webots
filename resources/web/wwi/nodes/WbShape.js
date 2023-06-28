import WbAppearance from './WbAppearance.js';
import WbBaseNode from './WbBaseNode.js';
import WbPbrAppearance from './WbPbrAppearance.js';
import WbPointSet from './WbPointSet.js';
import WbWorld from './WbWorld.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import {getAnId} from './utils/id_provider.js';
import {nodeIsInBoundingObject} from './utils/node_utilities.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbShape extends WbBaseNode {
  #boundingObjectFirstTimeSearch;
  #isInBoundingObject;
  #appearance;
  constructor(id, castShadow, isPickable, geometry, appearance) {
    super(id);
    this.castShadow = castShadow;
    this.isPickable = isPickable;

    this.#boundingObjectFirstTimeSearch = true;
    this.#isInBoundingObject = false;

    this.appearance = appearance;
    this.geometry = geometry;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_SHAPE;
  }

  get appearance() {
    return this.#appearance;
  }

  set appearance(value) {
    this.#appearance = value;

    for (const useId of this.useList) {
      const useNode = WbWorld.instance.nodes.get(useId);
      if (typeof value === 'undefined') {
        if (typeof useNode !== 'undefined')
          useNode.appearance.delete();
      } else {
        const newAppearance = value.clone();
        WbWorld.instance.nodes.set(newAppearance.id, newAppearance);
        useNode.appearance = newAppearance;
        useNode.appearance.parent = useNode.id;
      }
    }

    if (typeof this.notifyLed !== 'undefined')
      this.notifyLed();
  }

  applyMaterialToGeometry() {
    if (!this.wrenMaterial)
      this.#createWrenMaterial(Enum.WR_MATERIAL_PHONG);

    if (this.geometry) {
      if (this.appearance instanceof WbAppearance) {
        if (this.appearance.wrenObjectsCreatedCalled)
          this.wrenMaterial = this.appearance.modifyWrenMaterial(this.wrenMaterial);
        else
          this.wrenMaterial = WbAppearance.fillWrenDefaultMaterial(this.wrenMaterial);
      } else if ((this.appearance instanceof WbPbrAppearance) && !(this.geometry instanceof WbPointSet)) {
        this.#createWrenMaterial();
        if (this.appearance.wrenObjectsCreatedCalled)
          this.wrenMaterial = this.appearance.modifyWrenMaterial(this.wrenMaterial);
      } else
        this.wrenMaterial = WbAppearance.fillWrenDefaultMaterial(this.wrenMaterial);

      if (!this.geometry.isInBoundingObject())
        this.geometry.setWrenMaterial(this.wrenMaterial, this.castShadow);
    }
  }

  boundingSphere() {
    return this.geometry?.boundingSphere();
  }

  clone(customID) {
    let geometry, appearance;
    if (typeof this.geometry !== 'undefined') {
      geometry = this.geometry.clone(getAnId());
      geometry.parent = customID;
      WbWorld.instance.nodes.set(geometry.id, geometry);
    }

    if (typeof this.appearance !== 'undefined') {
      appearance = this.appearance.clone(getAnId());
      appearance.parent = customID;
      WbWorld.instance.nodes.set(appearance.id, appearance);
    }

    this.useList.push(customID);
    return new WbShape(customID, this.castShadow, this.isPickable, geometry, appearance);
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.appearance?.createWrenObjects();

    if (typeof this.geometry !== 'undefined') {
      this.geometry.createWrenObjects();

      this.applyMaterialToGeometry();
    }
  }

  delete(isBoundingObject) {
    if (typeof this.parent === 'undefined') {
      const index = WbWorld.instance.root.children.indexOf(this);
      WbWorld.instance.root.children.splice(index, 1);
    } else {
      const parent = WbWorld.instance.nodes.get(this.parent);
      if (typeof parent !== 'undefined') {
        if (isBoundingObject)
          parent.isBoundingObject = undefined;
        else if (typeof parent.endPoint !== 'undefined')
          parent.endPoint = undefined;
        else {
          const index = parent.children.indexOf(this);
          parent.children.splice(index, 1);
        }
      }
    }

    if (typeof this.wrenMaterial !== 'undefined') {
      _wr_material_delete(this.wrenMaterial);
      this.wrenMaterial = undefined;
    }

    this.appearance?.delete();
    this.geometry?.delete();

    super.delete();
  }

  isInBoundingObject() {
    if (this.#boundingObjectFirstTimeSearch) {
      this.#isInBoundingObject = nodeIsInBoundingObject(this);
      if (this.wrenObjectsCreatedCalled)
        this.#boundingObjectFirstTimeSearch = false;
    }

    return this.#isInBoundingObject;
  }

  updateAppearance() {
    if (this.wrenObjectsCreatedCalled)
      this.applyMaterialToGeometry();

    for (const useId of this.useList) {
      const useNode = WbWorld.instance.nodes.get(useId);
      useNode?.updateAppearance();
    }
  }

  updateBoundingObjectVisibility() {
    this.geometry?.updateBoundingObjectVisibility();
  }

  updateCastShadows() {
    if (this.isInBoundingObject())
      return;

    this.geometry?.computeCastShadows(this.castShadow);
  }

  updateIsPickable() {
    if (this.isInBoundingObject())
      return;

    this.geometry?.setPickable(this.isPickable);
  }

  updateGeometryMaterial() {
    if (this.wrenObjectsCreatedCalled)
      this.applyMaterialToGeometry();
  }

  preFinalize() {
    super.preFinalize();

    this.appearance?.preFinalize();
    this.geometry?.preFinalize();

    this.updateAppearance();
  }

  postFinalize() {
    super.postFinalize();

    this.appearance?.postFinalize();
    this.geometry?.postFinalize();

    if (!this.isInBoundingObject()) {
      this.updateCastShadows();
      this.updateIsPickable();
    }

    if (typeof this.geometry !== 'undefined') {
      this.geometry.onChange = () => this.updateGeometryMaterial();
      this.geometry.onRecreated = () => {
        this.updateGeometryMaterial();
        this.updateIsPickable();
      };
    }

    if (typeof this.appearance !== 'undefined')
      this.appearance.onChange = () => this.updateAppearance();
  }

  // Private functions

  #createWrenMaterial(type) {
    const defaultColor = _wrjs_array3(1.0, 1.0, 1.0);
    if (typeof this.wrenMaterial !== 'undefined')
      _wr_material_delete(this.wrenMaterial);

    if (type === Enum.WR_MATERIAL_PHONG) {
      this.wrenMaterial = _wr_phong_material_new();
      _wr_phong_material_set_color(this.wrenMaterial, defaultColor);
      _wr_material_set_default_program(this.wrenMaterial, WbWrenShaders.phongShader());
    } else {
      this.wrenMaterial = _wr_pbr_material_new();
      _wr_pbr_material_set_base_color(this.wrenMaterial, defaultColor);
      _wr_material_set_default_program(this.wrenMaterial, WbWrenShaders.pbrShader());
    }
  }
}
