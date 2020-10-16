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

  //

  function get( attribute ) {
    if ( attribute.isInterleavedBufferAttribute ) attribute = attribute.data;

    return buffers.get( attribute );
  }


  function update( attribute, bufferType ) {


    if ( attribute.isInterleavedBufferAttribute ) attribute = attribute.data;
    let data = buffers.get( attribute );

    if ( data === undefined ) {

      buffers.set( attribute, createBuffer( attribute, bufferType ) );

    } else if ( data.version < attribute.version ) {

      updateBuffer( data.buffer, attribute, bufferType );

      data.version = attribute.version;

    }

  }

  return {

    get: get,
    update: update

  };

}
function WebGLRenderList() {

  let renderItems = [];
  let renderItemsIndex = 0;

  let opaque = [];
  let transparent = [];

  let defaultProgram = { id: - 1 };

  function init() {

    renderItemsIndex = 0;

    opaque.length = 0;
    transparent.length = 0;

  }

  function getNextRenderItem( object, geometry, material, groupOrder, z, group ) {

    let renderItem = renderItems[ renderItemsIndex ];

    if ( renderItem === undefined ) {
      renderItem = {
        id: object.id,
        object: object,
        geometry: geometry,
        material: material,
        program: material.program || defaultProgram,
        groupOrder: groupOrder,
        renderOrder: object.renderOrder,
        z: z,
        group: group
      };

      renderItems[ renderItemsIndex ] = renderItem;

    } else {
      renderItem.id = object.id;
      renderItem.object = object;
      renderItem.geometry = geometry;
      renderItem.material = material;
      renderItem.program = material.program || defaultProgram;
      renderItem.groupOrder = groupOrder;
      renderItem.renderOrder = object.renderOrder;
      renderItem.z = z;
      renderItem.group = group;

    }

    renderItemsIndex ++;

    return renderItem;

  }

  function push( object, geometry, material, groupOrder, z, group ) {

    let renderItem = getNextRenderItem( object, geometry, material, groupOrder, z, group );

    ( material.transparent === true ? transparent : opaque ).push( renderItem );

  }

  return {
    opaque: opaque,
    transparent: transparent,

    init: init,
    push: push,
  };

}
function WebGLPrograms( renderer ) {

  let program;

  this.acquireProgram = function ( material ) {
    if ( program === undefined ) {
      program = new WebGLProgram( renderer, material);
    }
    return program;
  };
}
function WebGLProgram( renderer, material) {
  let gl = renderer.gl;

  let defines = material.defines;

  let vertexShader = meshbasic_vert;
  let fragmentShader = meshbasic_frag;

  let program = gl.createProgram();

  let prefixVertex, prefixFragment;

    prefixVertex = [
      'uniform mat4 modelViewMatrix;',
      'uniform mat4 projectionMatrix;',
      'attribute vec3 position;',
      '\n'

    ].join( '\n' );

    prefixFragment = [
      'precision highp float;',
      '\n'
    ].join( '\n' );


  let vertexGlsl = prefixVertex + vertexShader;
  let fragmentGlsl = prefixFragment + fragmentShader;

  let glVertexShader = gl.createShader( 35633 );
  gl.shaderSource( glVertexShader, vertexGlsl );
  gl.compileShader( glVertexShader );

  let glFragmentShader = gl.createShader( 35632 );
  gl.shaderSource( glFragmentShader, fragmentGlsl );
  gl.compileShader( glFragmentShader );

  gl.attachShader( program, glVertexShader );
  gl.attachShader( program, glFragmentShader );

  // Force a particular attribute to index 0.

  gl.linkProgram( program );

  // clean up

  gl.deleteShader( glVertexShader );
  gl.deleteShader( glFragmentShader );

  this.program = program;

  return this;

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

const meshbasic_frag = "void main() {\tgl_FragColor = vec4(1,1,1,1 );\n}";

const meshbasic_vert = "void main() {\n\tvec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );\n\tgl_Position = projectionMatrix * mvPosition;\n}";

class WebGL2Renderer {
  constructor(){
    this.canvas = document.createElementNS( 'http://www.w3.org/1999/xhtml', 'canvas' );
    let contextAttributes = {
      alpha: false,
    };
    this.gl = this.canvas.getContext("webgl", contextAttributes);
    this.domElement = this.canvas;

    if (this.gl === null) {
      alert("Unable to initialize WebGL. Your browser or machine may not support it.");
      return;
    }

    this.attributes = new WebGLAttributes(this.gl);
    this.renderLists = new WebGLRenderList();
    this.programCache = new WebGLPrograms(this);
    this.currentRenderList = null;
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

    this.currentRenderList = this.renderLists;
    this.currentRenderList.init();

    this.projectObject( scene, camera );

    // render scene
    let opaqueObjects = this.currentRenderList.opaque;
    if ( opaqueObjects.length ) this.renderObjects( opaqueObjects, camera );

  }

  renderBufferDirect ( camera, fog, geometry, material, object, group ) {
    let program = this.setProgram( camera, fog, material, object );

    let index = geometry.index;
    let attribute = this.attributes.get( index );
    this.setupVertexAttributes( material, program, geometry );

    let	dataCount = index.length;

    let rangeStart = geometry.drawRange.start;
    let rangeCount = geometry.drawRange.count;

    let groupStart = group !== null ? group.start : 0;
    let groupCount = group !== null ? group.count : Infinity;

    let drawStart = Math.max( rangeStart, groupStart );
    let drawEnd = Math.min( dataCount, rangeStart + rangeCount, groupStart + groupCount ) - 1;

    let drawCount = Math.max( 0, drawEnd - drawStart + 1 );

    if ( drawCount === 0 ) return;
    this.gl.drawElements( this.gl.TRIANGLES, drawCount, attribute.type, drawStart * attribute.bytesPerElement );
  }

  projectObject ( object, camera ) {
      if ( object.isMesh) {
          let geometry = object.geometry;//objects.update( object );
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
            this.currentRenderList.push( object, geometry, material, 0, undefined, null );
          }
      }
    let children = object.children;
    for ( let i = 0, l = children.length; i < l; i ++ ) {
      this.projectObject( children[ i ], camera );
    }
  }

  renderObjects( renderList, camera) {
    for ( let i = 0, l = renderList.length; i < l; i ++ ) {
      let renderItem = renderList[ i ];
      let object = renderItem.object;
      let geometry = renderItem.geometry;
      let material = renderItem.material;
      let group = renderItem.group;
      object.modelViewMatrix = camera.matrixWorldInverse['*'](object.matrixWorld);
      this.renderBufferDirect( camera, undefined, geometry, material, object, group );
    }

  }

  setProgram( camera, fog, material, object ) {
    let program = this.programCache.acquireProgram( material);
    material.program = program;

    if ( this.useProgram( program.program ) ) {
      setValue4fm( this.gl, 'projectionMatrix', camera.projectionMatrix, program.program );
    }

    setValue4fm( this.gl, 'modelViewMatrix', object.modelViewMatrix, program.program );

    return program;

  }

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


  useProgram( program ) {
    if ( this.currentProgram !== program ) {
      this.gl.useProgram( program );
      this.currentProgram = program;
      return true;
    }
    return false;
  }
}
