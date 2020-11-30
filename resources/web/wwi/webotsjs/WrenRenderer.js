import {World} from "./World.js"

class WrenRenderer {
  constructor () {
  }

  setSize ( width, height ) {
    canvas.width = width;
    canvas.height = height;
  }

  render() {
    if(!_wr_gl_state_is_initialized())
      return;

    try {
      //World.instance.viewpoint.updatePostProcessingParameters();

      _wr_scene_render(_wr_scene_get_instance(), null, true);
      console.log("render");
    }
    catch(error) {
      console.log("No Context");
    }
  }
}

export {WrenRenderer}
