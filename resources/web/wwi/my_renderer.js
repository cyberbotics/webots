class WebGL2Renderer extends THREE.WebGLRenderer {
  constructor(){
    super();
    const canvas = document.createElementNS( 'http://www.w3.org/1999/xhtml', 'canvas' );

    const gl = canvas.getContext("webgl");

    if (gl === null) {
      alert("Unable to initialize WebGL. Your browser or machine may not support it.");
      return;
    }

    let fragCode =
        'precision highp float;'+
        'void main() {'+
        '}';
    let vertCode =
        'void main() {'+
        '}';

    //the scene is non null;
    this.renderBufferDirect = function ( camera, scene, geometry, material, object, group ) {
      let program = gl.createProgram();
      let fragShader = gl.createShader(gl.FRAGMENT_SHADER);
      let vertShader = gl.createShader(gl.VERTEX_SHADER);
      gl.shaderSource(fragShader, fragCode);
      gl.shaderSource(vertShader, vertCode);
      gl.compileShader(fragShader);
      gl.compileShader(vertShader);
      gl.attachShader(program, fragShader);
      gl.attachShader(program, vertShader);
      gl.linkProgram(program);
      gl.useProgram(program);

      let index = geometry.index;
      const dataCount = ( index !== null ) ? index.count : position.count;

      const rangeStart = geometry.drawRange.start;
      const rangeCount = geometry.drawRange.count;

      const groupStart = group !== null ? group.start : 0;
      const groupCount = group !== null ? group.count : Infinity;

      const drawStart = Math.max( rangeStart, groupStart );
      const drawEnd = Math.min( dataCount, rangeStart + rangeCount, groupStart + groupCount ) - 1;

      const drawCount = Math.max( 0, drawEnd - drawStart + 1 )

      gl.drawArrays( gl.TRIANGLES, drawStart, drawStart );
    }

    //check render from webglbufferrenderer

    /*
    this.render = function (scene, camera){
      console.log("coucou");
      if ( camera.parent === null ) camera.updateMatrixWorld();
      if ( scene.autoUpdate === true ) scene.updateMatrixWorld();
          console.log(x);
      //gl.drawArrays( mode, start, count );
    }
    */

    //render: définir et sort les objects à render grâce à renderlist.get();
    //renderobjects: for sur les objets et ajoutes certaines caractéristiques par défaut si absentes puis call chaque fois renderobject
    //renderobject:change les matrices d'un object en fonction de la caméra puis call renderbufferdirect
    //renderbufferdirect:
    //créer Program?
    //mettre les objets dans le bon ordre (dans le vao?)
    //bind?
    //définir les start et end de chaque objets
    //puis les draw avec gl.drawArrays (mode = triangle sûrement)
  }
}
