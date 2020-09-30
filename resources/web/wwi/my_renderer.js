

class WebGL2Renderer extends THREE.WebGLRenderer {
  constructor(){
    super();
    const canvas = document.createElementNS( 'http://www.w3.org/1999/xhtml', 'canvas' );

    const gl = canvas.getContext("webgl2");

    if (gl === null) {
      alert("Unable to initialize WebGL. Your browser or machine may not support it.");
      return;
    }

    //check render from webglbufferrenderer

    this.render = function (scene, camera){
      console.log("coucou");
      // Set clear color to black, fully opaque
      gl.clearColor(0.0, 1.0, 1.0, 0.0);
      // Clear the color buffer with specified clear color
      gl.clear(gl.COLOR_BUFFER_BIT);
    }
  }
}
