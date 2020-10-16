function WebGLAttributes( gl ) {

  let buffers = new WeakMap();

  function createBuffer( array, bufferType ) {
    let usage = gl.STATIC_DRAW;
    let buffer = gl.createBuffer();

    gl.bindBuffer( bufferType, buffer );
    gl.bufferData( bufferType, array, usage );
    let type = gl.FLOAT;

    if ( array instanceof Uint16Array ) {
      type = gl.UNSIGNED_SHORT;
    }

    return {
      buffer: buffer,
      type: type,
      bytesPerElement: 2,
    };

  }
  function get( attribute ) {
    return buffers.get( attribute );
  }


  function update( attribute, bufferType ) {
    buffers.set( attribute, createBuffer( attribute, bufferType ) );
  }

  return {
    get: get,
    update: update
  };

}
function WebGLProgram( renderer, material) {
  let gl = renderer.gl;
  let vertexGlsl = meshbasic_vert;
  let fragmentGlsl = meshbasic_frag;

  let program = gl.createProgram();

  let glVertexShader = gl.createShader( 35633 );
  gl.shaderSource( glVertexShader, vertexGlsl );
  gl.compileShader( glVertexShader );

  let glFragmentShader = gl.createShader( 35632 );
  gl.shaderSource( glFragmentShader, fragmentGlsl );
  gl.compileShader( glFragmentShader );

  gl.attachShader( program, glVertexShader );
  gl.attachShader( program, glFragmentShader );

  gl.linkProgram( program );

  gl.deleteShader( glVertexShader );
  gl.deleteShader( glFragmentShader );

  return program;
}

function setValue4fm( gl, name, v, program) {
	var elements = v.elements;
  let addr = gl.getUniformLocation( program, name );

	if ( elements === undefined ) {
		gl.uniformMatrix4fv( addr, false, v );

	} else {
		gl.uniformMatrix4fv( addr, false, elements );
	}
}

const meshbasic_frag =
  "precision highp float;\n"+
  "void main() {\n"+
  "\tgl_FragColor = vec4(1,1,1,1 );\n"+
  "}";

const meshbasic_vert =
  "uniform mat4 modelViewMatrix;\n"+
  "uniform mat4 projectionMatrix;\n"+
  "attribute vec3 position;\n"+
  "void main() {\n"+
  "\tvec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );\n"+
  "\tgl_Position = projectionMatrix * mvPosition;\n"+
  "}";

class WebGL2Renderer {
  constructor(){
    this.canvas = document.createElementNS( 'http://www.w3.org/1999/xhtml', 'canvas' );
    let contextAttributes = {
      alpha: false,
    };
    this.gl = this.canvas.getContext("webgl2", contextAttributes);
    this.domElement = this.canvas;

    if (this.gl === null) {
      alert("Unable to initialize WebGL. Your browser or machine may not support it.");
      return;
    }

    this.attributes = new WebGLAttributes(this.gl);
    this.currentProgram = null;
    this.viewport= new glm.vec4( 0, 0, this.domElement.width, this.domElement.height )
  }

  setSize ( width, height, updateStyle ) {
    this.canvas.width = width;
    this.canvas.height = height;

    this.viewport = new glm.vec4( 0, 0, width, height );
    this.gl.viewport( this.viewport.x, this.viewport.y, this.viewport.z, this.viewport.w );
  }

  render ( scene, camera ) {
    camera.updateMatrixWorld();

    this.renderObjects( scene, camera );
  }

  drawBuffer ( camera, geometry, material, object) {
    this.setProgram( camera, material, object );

    let index = geometry.index;
    let attribute = this.attributes.get( index );
    this.setupVertexAttributes( material, this.currentProgram, geometry );

    let	dataCount = index.length;

    let drawStart = geometry.drawRange.start;
    let drawEnd = Math.min( dataCount, drawStart + geometry.drawRange.count) - 1;
    let drawCount = Math.max( 0, drawEnd - drawStart + 1 );

    if ( drawCount === 0 ) return;
    this.gl.drawElements( this.gl.TRIANGLES, drawCount, attribute.type, drawStart * attribute.bytesPerElement );
  }

  //helper function
  renderObjects ( object, camera ) {
      if ( object.isMesh) {
          let geometry = object.geometry;
          let index = geometry.index;
          let geometryAttributes = geometry.attributes;
          if ( index !== null ) {
            this.attributes.update( index, 34963 );
          }
          for ( let name in geometryAttributes ) {
            this.attributes.update( geometryAttributes[ name ], 34962 );
          }
          let material = object.material;
          if ( material.visible ) {
            object.modelViewMatrix = camera.matrixWorldInverse['*'](object.matrixWorld);
            this.drawBuffer( camera, geometry, material, object );
          }
      }
    let children = object.children;
    for ( let i = 0, l = children.length; i < l; i ++ ) {
      this.renderObjects( children[ i ], camera );
    }
  }

  //helper function
  setProgram( camera, material, object ) {

    if ( this.currentProgram === null ) {
      let program = new WebGLProgram( this, material);
      this.gl.useProgram( program );
      this.currentProgram = program;
      setValue4fm( this.gl, 'projectionMatrix', camera.projectionMatrix, this.currentProgram );
    }
    setValue4fm( this.gl, 'modelViewMatrix', object.modelViewMatrix, this.currentProgram );
  }

  //helper function
  setupVertexAttributes( material, program, geometry ) {
    let geometryAttribute = geometry.attributes["position"];

    if ( geometryAttribute !== undefined ) {
      let attribute = this.attributes.get( geometryAttribute );
      let buffer = attribute.buffer;
      let type = attribute.type;
      this.gl.enableVertexAttribArray( "position"  );
      this.gl.vertexAttribPointer( "position" , 3, type, false, 0, 0 );
    }
  }
}
