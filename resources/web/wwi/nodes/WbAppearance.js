import WbAbstractAppearance from './WbAbstractAppearance.js';
import WbWorld from './WbWorld.js';
import WbWrenShaders from '../wren/WbWrenShaders.js';
import {getAnId} from './utils/id_provider.js';
import {WbNodeType} from './wb_node_type.js';

export default class WbAppearance extends WbAbstractAppearance {
  #material;
  #texture;

  constructor(id, material, texture, transform) {
    super(id, transform);
    this.material = material;
    this.texture = texture;
  }

  get nodeType() {
    return WbNodeType.WB_NODE_APPEARANCE;
  }

  get material() {
    return this.#material;
  }

  set material(newMaterial) {
    this.#material = newMaterial;

    if (typeof this.#material !== 'undefined')
      this.#material.onChange = () => this.#update();

    this.#update();

    if (typeof this.notifyLed !== 'undefined')
      this.notifyLed();
  }

  get texture() {
    return this.#texture;
  }

  set texture(newTexture) {
    this.#texture = newTexture;

    if (typeof this.#texture !== 'undefined')
      this.#texture.onChange = () => this.#update();

    this.#update();
  }

  clone(customID) {
    let material, texture, transform;
    if (typeof this.#material !== 'undefined') {
      material = this.#material.clone(getAnId());
      material.parent = customID;
      WbWorld.instance.nodes.set(material.id, material);
    }

    if (typeof this.#texture !== 'undefined') {
      texture = this.#texture.clone(getAnId());
      texture.parent = customID;
      WbWorld.instance.nodes.set(texture.id, texture);
    }

    if (typeof this.textureTransform !== 'undefined') {
      transform = this.textureTransform.clone(getAnId());
      transform.parent = customID;
      WbWorld.instance.nodes.set(transform.id, transform);
    }

    this.useList.push(customID);
    return new WbAppearance(customID, material, texture, transform);
  }

  createWrenObjects() {
    super.createWrenObjects();

    this.#material?.createWrenObjects();
    this.#texture?.createWrenObjects();
  }

  delete() {
    this.#material?.delete();
    this.#texture?.delete();

    super.delete();
  }

  modifyWrenMaterial(wrenMaterial) {
    if (typeof this.#material !== 'undefined') {
      _wr_material_set_default_program(wrenMaterial, WbWrenShaders.phongShader());
      _wr_material_set_stencil_ambient_emissive_program(wrenMaterial, WbWrenShaders.phongStencilAmbientEmissiveShader());
      _wr_material_set_stencil_diffuse_specular_program(wrenMaterial, WbWrenShaders.phongStencilDiffuseSpecularShader());

      this.#material.modifyWrenMaterial(wrenMaterial, this.#texture && this.#texture.wrenTexture);
    } else
      wrenMaterial = WbAppearance.fillWrenDefaultMaterial(wrenMaterial);

    if (this.#texture)
      this.#texture.modifyWrenMaterial(wrenMaterial, 0, 2);
    else
      _wr_material_set_texture(wrenMaterial, null, 0);

    if (typeof this.textureTransform !== 'undefined')
      this.textureTransform.modifyWrenMaterial(wrenMaterial);
    else
      _wr_material_set_texture_transform(wrenMaterial, null);

    return wrenMaterial;
  }

  preFinalize() {
    super.preFinalize();

    this.#material?.preFinalize();
    this.#texture?.preFinalize();
  }

  postFinalize() {
    super.postFinalize();

    this.#material?.postFinalize();
    this.#texture?.postFinalize();
  }

  #update() {
    if (this.isPostFinalizedCalled && typeof this.onChange === 'function')
      this.onChange();
  }

  // Static functions

  static fillWrenDefaultMaterial(wrenMaterial) {
    if (typeof wrenMaterial === 'undefined') {
      _wr_material_delete(wrenMaterial);
      wrenMaterial = _wr_phong_material_new();
    }

    _wr_material_set_default_program(wrenMaterial, WbWrenShaders.defaultShader());

    return wrenMaterial;
  }
}
