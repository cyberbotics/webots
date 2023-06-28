import {arrayXPointerInt, arrayXPointerFloat} from '../nodes/utils/utils.js';
import WbVector3 from '../nodes/utils/WbVector3.js';
import WbWrenShaders from './WbWrenShaders.js';

export default class WbWrenPicker {
  #width;
  #height;
  #viewport;
  #viewportDepth;
  #frameBuffer;
  #frameBufferDepth;
  #outputTexture;
  #outputTextureDepth;
  constructor() {
    this.selectedId = -1;
    this.coordinates = new WbVector3();
    this.#width = 0;
    this.#height = 0;

    this.#viewport = _wr_viewport_new();
    this.#viewportDepth = _wr_viewport_new();

    const colorPointer = _wrjs_array4(0.0, 0.0, 0.0, 0.0);
    _wr_viewport_set_clear_color_rgba(this.#viewport, colorPointer);

    this.#setup();
  }

  pick(x, y) {
    const viewport = _wr_scene_get_viewport(_wr_scene_get_instance());
    // This fix a bug that occurs when switching between different worlds
    _wr_viewport_set_camera(this.#viewport, _wr_viewport_get_camera(viewport));
    _wr_viewport_set_camera(this.#viewportDepth, _wr_viewport_get_camera(viewport));

    this.coordinates.setXyz(0.0, 0.0, 0.0);
    this.selectedId = -1;

    // Recreate framebuffer and textures if viewport's size has changed
    if (this.#hasSizeChanged()) {
      this.#cleanup();
      this.#setup();
    }

    // Check if object was picked & decode ID
    const scene = _wr_scene_get_instance();
    _wr_viewport_enable_skybox(this.#viewport, false);
    _wr_scene_enable_translucence(scene, false);
    _wr_scene_enable_depth_reset(scene, false);
    Module.ccall('wr_scene_render_to_viewports', null, ['number', 'number', 'number', 'string', 'boolean'],
      [scene, 1, _wrjs_pointerOnInt(this.#viewport), 'picking', true]);
    _wr_scene_enable_depth_reset(scene, true);
    _wr_viewport_enable_skybox(this.#viewport, true);
    _wr_scene_enable_translucence(scene, true);

    let data = [];
    let dataPointer = arrayXPointerInt(data);
    _wr_frame_buffer_copy_pixel(this.#frameBuffer, 0, x, y, dataPointer, true);
    data[0] = Module.getValue(dataPointer, 'i8');
    data[1] = Module.getValue(dataPointer + 1, 'i8');
    data[2] = Module.getValue(dataPointer + 2, 'i8');
    data[3] = Module.getValue(dataPointer + 3, 'i8');

    _free(dataPointer);

    data[0] = data[0] >= 0 ? data[0] : 256 + data[0];
    data[1] = data[1] >= 0 ? data[1] : 256 + data[1];
    data[2] = data[2] >= 0 ? data[2] : 256 + data[2];
    data[3] = data[3] >= 0 ? data[3] : 256 + data[3];

    const id = (data[2] << 24) | (data[1] << 16) | (data[0] << 8) | data[3];
    if (id === 0)
      return false;
    else
      this.selectedId = id - 1;

    _wr_viewport_enable_skybox(this.#viewportDepth, false);
    _wr_scene_enable_translucence(scene, false);
    _wr_scene_enable_depth_reset(scene, false);
    Module.ccall('wr_scene_render_to_viewports', null, ['number', 'number', 'number', 'string', 'boolean'],
      [scene, 1, _wrjs_pointerOnIntBis(this.#viewportDepth), 'depth', true]);
    _wr_scene_enable_depth_reset(scene, true);
    _wr_viewport_enable_skybox(this.#viewportDepth, true);
    _wr_scene_enable_translucence(scene, true);

    data = [0];
    dataPointer = arrayXPointerFloat(data);
    _wr_frame_buffer_copy_depth_pixel(this.#frameBufferDepth, x, y, dataPointer, true);

    data[0] = Module.getValue(dataPointer, 'float');
    _free(dataPointer);

    this.coordinates = new WbVector3(x, this.#height - y - 1, data[0]);
    return true;
  }

  // Setup & attach picking material, based on the unique ID
  // ID is encoded in the following way:
  // Most significant word: red and green channels of ambient color
  // Least signigicant word: red and green channels of diffuse color
  // These are combined in RGBA channels in the picking fragment shader
  static setPickable(renderable, uniqueId, pickable) {
    uniqueId = parseFloat(uniqueId.substring(1));

    let material = Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'], [renderable, 'picking']);

    if (!material) {
      material = _wr_phong_material_new();
      _wr_material_set_default_program(material, WbWrenShaders.pickingShader());

      Module.ccall('wr_renderable_set_material', null, ['number', 'number', 'string'], [renderable, material, 'picking']);
    }

    let depthMaterial = Module.ccall('wr_renderable_get_material', 'number', ['number', 'string'], [renderable, 'depth']);

    if (!depthMaterial) {
      depthMaterial = _wr_phong_material_new();
      _wr_material_set_default_program(depthMaterial, WbWrenShaders.depthPixelShader());

      Module.ccall('wr_renderable_set_material', null, ['number', 'number', 'string'], [renderable, depthMaterial, 'depth']);
    }

    const encodedId = [0, 0, 0, 0, 0, 0];

    if (pickable) {
      // ID is incremented since a 0 value would mean that no object was picked
      // (ID = 0 is valid)
      let id = uniqueId + 1;

      for (let i = 4; i >= 0; --i) {
        if (i === 2)
          continue;

        encodedId[i] = (id & 0x000000FF) / 255.0;
        id >>= 8;
      }
    }

    const encodedIdPointer = arrayXPointerFloat(encodedId);
    const encodedIdPointerSecondHalf = arrayXPointerFloat([encodedId[3], encodedId[4], encodedId[5]]);
    _wr_phong_material_set_linear_ambient(material, encodedIdPointer);
    _wr_phong_material_set_linear_diffuse(material, encodedIdPointerSecondHalf);
    _free(encodedIdPointer);
    _free(encodedIdPointerSecondHalf);
  }

  // Private functions

  #cleanup() {
    if (typeof this.#outputTexture !== 'undefined')
      _wr_texture_delete(this.#outputTexture);
    if (typeof this.#frameBuffer !== 'undefined')
      _wr_frame_buffer_delete(this.#frameBuffer);
    if (typeof this.#outputTextureDepth !== 'undefined')
      _wr_texture_delete(this.#outputTextureDepth);
    if (typeof this.#frameBufferDepth !== 'undefined')
      _wr_frame_buffer_delete(this.#frameBufferDepth);
  }

  #hasSizeChanged() {
    const viewport = _wr_scene_get_viewport(_wr_scene_get_instance());
    const width = _wr_viewport_get_width(viewport);
    const height = _wr_viewport_get_height(viewport);

    if (this.#width !== width || this.#height !== height) {
      this.#width = width;
      this.#height = height;
      return true;
    }
    return false;
  }

  #setup() {
    const viewport = _wr_scene_get_viewport(_wr_scene_get_instance());
    this.#width = _wr_viewport_get_width(viewport);
    this.#height = _wr_viewport_get_height(viewport);
    _wr_viewport_set_size(this.#viewport, this.#width, this.#height);

    this.#frameBuffer = _wr_frame_buffer_new();
    this.#outputTexture = _wr_texture_rtt_new();

    _wr_frame_buffer_set_size(this.#frameBuffer, this.#width, this.#height);
    _wr_frame_buffer_enable_depth_buffer(this.#frameBuffer, true);

    _wr_frame_buffer_append_output_texture(this.#frameBuffer, this.#outputTexture);
    _wr_frame_buffer_enable_copying(this.#frameBuffer, 0, true);
    _wr_frame_buffer_setup(this.#frameBuffer);

    _wr_viewport_set_frame_buffer(this.#viewport, this.#frameBuffer);
    _wr_viewport_set_camera(this.#viewport, _wr_viewport_get_camera(viewport));

    // DEPTH
    _wr_viewport_set_size(this.#viewportDepth, this.#width, this.#height);

    this.#frameBufferDepth = _wr_frame_buffer_new();
    this.#outputTextureDepth = _wr_texture_rtt_new();
    _wr_texture_set_internal_format(this.#outputTextureDepth, Enum.WR_TEXTURE_INTERNAL_FORMAT_R32F);

    _wr_frame_buffer_set_size(this.#frameBufferDepth, this.#width, this.#height);
    _wr_frame_buffer_enable_depth_buffer(this.#frameBufferDepth, true);

    _wr_frame_buffer_append_output_texture(this.#frameBufferDepth, this.#outputTextureDepth);
    _wr_frame_buffer_setup(this.#frameBufferDepth);

    _wr_viewport_set_frame_buffer(this.#viewportDepth, this.#frameBufferDepth);
    _wr_viewport_set_camera(this.#viewportDepth, _wr_viewport_get_camera(viewport));
  }
}
