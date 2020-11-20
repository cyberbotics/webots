class WrenRenderer {
  constructor () {
  }

  setSize ( width, height ) {
    canvas.width = width;
    canvas.height = height;
  }

  render() {
    try {
      //TODO fix
      //VIEWPOINT.updatePostProcessingParameters();
      _wr_scene_render(_wr_scene_get_instance(), null, true);
      console.log("render");
    }
    catch(error) {
      console.log("No Context");
    }
  }
}

export {WrenRenderer}
