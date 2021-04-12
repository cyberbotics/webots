import {WbWorld} from './nodes/wbWorld.js';
import {disableShadows} from './nodes/wbPreferences.js';

class WrenRenderer {
  constructor() {
    this.canvas = document.createElement('canvas');
    this.canvas.id = 'canvas';
    let div = document.getElementById('view3d');

    if (typeof div === 'undefined' || div === null)
      div = document.getElementById('playerDiv');
    div.insertBefore(this.canvas, div.firstChild);

    _wr_config_enable_shadows(!disableShadows);

    // glGetError causes freeze with Firefox on Windows 10
    _wr_gl_state_disable_check_error();
  }

  setSize(width, height) {
    canvas.width = width;
    canvas.height = height;
  }

  render() {
    if (!_wr_gl_state_is_initialized())
      return;

    try {
      WbWorld.instance.viewpoint.updatePostProcessingParameters();

      _wr_scene_render(_wr_scene_get_instance(), null, true);
      // console.log("render");
    } catch (error) {
      console.log('No Context');
    }
  }

  renderMinimal() {
    if (!_wr_gl_state_is_initialized())
      return;

    try {
      _wr_scene_render(_wr_scene_get_instance(), null, true);
    } catch (error) {
      console.log('No Context');
    }
  }
}

export {WrenRenderer};
