function WebGLAttributes( gl ) {

  let buffers = new WeakMap();

  function createBuffer( attribute, bufferType ) {
    let array = attribute.array;
    let usage = attribute.dynamic ? 35048 : 35044;

    let buffer = gl.createBuffer();

    gl.bindBuffer( bufferType, buffer );
    gl.bufferData( bufferType, array, usage );
    let type = 5126;

    if ( array instanceof Float32Array ) {

      type = 5126;

    } else if ( array instanceof Float64Array ) {

      console.warn( 'WebGLAttributes: Unsupported data buffer format: Float64Array.' );

    } else if ( array instanceof Uint16Array ) {

      type = 5123;

    } else if ( array instanceof Int16Array ) {

      type = 5122;

    } else if ( array instanceof Uint32Array ) {

      type = 5125;

    } else if ( array instanceof Int32Array ) {

      type = 5124;

    } else if ( array instanceof Int8Array ) {

      type = 5120;

    } else if ( array instanceof Uint8Array ) {

      type = 5121;

    }

    return {
      buffer: buffer,
      type: type,
      bytesPerElement: 2,
      version: attribute.version
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
function WebGLProperties() {

  let properties = new WeakMap();

  function get( object ) {

    let map = properties.get( object );

    if ( map === undefined ) {

      map = {};
      properties.set( object, map );

    }

    return map;

  }

  return {
    get: get,
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

  // set up caching for uniform locations

  let cachedUniforms;

  this.getUniforms = function () {

    if ( cachedUniforms === undefined ) {

      cachedUniforms = new WebGLUniforms( gl, program );

    }
    return cachedUniforms;

  };

  this.program = program;

  return this;

}
function SingleUniform( id, activeInfo, addr ) {

	this.id = id;
	this.addr = addr;
	this.cache = [];
	this.setValue = getSingularSetter( activeInfo.type );

	// this.path = activeInfo.name; // DEBUG

}
const RePathPart = /([\w\d_]+)(\])?(\[|\.)?/g;
var mat4array = new Float32Array( 16 );
function addUniform( container, uniformObject ) {

	container.seq.push( uniformObject );
	container.map[ uniformObject.id ] = uniformObject;

}
function parseUniform( activeInfo, addr, container ) {

  var path = activeInfo.name,
    pathLength = path.length;

  // reset RegExp object, because of the early exit of a previous run
  RePathPart.lastIndex = 0;

  while ( true ) {

    var match = RePathPart.exec( path ),
      matchEnd = RePathPart.lastIndex,

      id = match[ 1 ],
      idIsIndex = match[ 2 ] === ']',
      subscript = match[ 3 ];

    if ( idIsIndex ) id = id | 0; // convert to integer

    if ( subscript === undefined || subscript === '[' && matchEnd + 2 === pathLength ) {

      // bare name or "pure" bottom-level array "[0]" suffix

      addUniform( container, subscript === undefined ?
        new SingleUniform( id, activeInfo, addr ) :
        new PureArrayUniform( id, activeInfo, addr ) );

      break;

    } else {

      // step into inner node / create it in case it doesn't exist

      var map = container.map, next = map[ id ];

      if ( next === undefined ) {

        next = new StructuredUniform( id );
        addUniform( container, next );

      }

      container = next;

    }

  }

}
function getSingularSetter( type ) {

	switch ( type ) {
		case 0x1406: return setValue1f; // FLOAT
		case 0x8b51: return setValue3fv; // _VEC3
		case 0x8b5c: return setValue4fm; // _MAT4
	}

}
function setValue4fm( gl, v ) {

	var cache = this.cache;
	var elements = v.elements;

	if ( elements === undefined ) {
		if ( arraysEqual( cache, v ) ) return;

		gl.uniformMatrix4fv( this.addr, false, v );

		copyArray( cache, v );

	} else {
		if ( arraysEqual( cache, elements ) ) return;

		mat4array.set( elements );

		gl.uniformMatrix4fv( this.addr, false, mat4array );

		copyArray( cache, elements );

	}

}
function setValue3fv( gl, v ) {

	var cache = this.cache;

	if ( v.r !== undefined ) {
		if ( cache[ 0 ] !== v.r || cache[ 1 ] !== v.g || cache[ 2 ] !== v.b ) {
			gl.uniform3f( this.addr, v.r, v.g, v.b );

			cache[ 0 ] = v.r;
			cache[ 1 ] = v.g;
			cache[ 2 ] = v.b;

		}

	}
}
function setValue1f( gl, v ) {

	var cache = this.cache;

	if ( cache[ 0 ] === v ) return;

	gl.uniform1f( this.addr, v );

	cache[ 0 ] = v;

}
function arraysEqual( a, b ) {

	if ( a.length !== b.length ) return false;

	for ( var i = 0, l = a.length; i < l; i ++ ) {
		if ( a[ i ] !== b[ i ] ) return false;
	}
	return true;
}
function copyArray( a, b ) {

	for ( var i = 0, l = b.length; i < l; i ++ ) {
		a[ i ] = b[ i ];
	}
}
function WebGLUniforms( gl, program ) {

  this.seq = [];
  this.map = {};

  let n = gl.getProgramParameter( program, 35718 );

  for ( let i = 0; i < n; ++ i ) {

    let info = gl.getActiveUniform( program, i ),
      addr = gl.getUniformLocation( program, info.name );

    parseUniform( info, addr, this );
  }
}

WebGLUniforms.prototype.setValue = function ( gl, name, value, textures ) {
  let u = this.map[ name ];

  if ( u !== undefined ) u.setValue( gl, value, textures );

};

// Static interface
WebGLUniforms.upload = function ( gl, seq, values, textures ) {
  for ( let i = 0, n = seq.length; i !== n; ++ i ) {

    let u = seq[ i ],
      v = values[ u.id ];
    if ( v.needsUpdate !== false ) {
      u.setValue( gl, v.value, textures );
    }

  }

};

WebGLUniforms.seqWithValue = function ( seq, values ) {
  let r = [];

  for ( let i = 0, n = seq.length; i !== n; ++ i ) {
    let u = seq[ i ];
    if ( u.id in values ) r.push( u );
  }
  return r;

};


const common = {
    diffuse: { value: new Module.Color( 238,238,238 ) }
};

const meshbasic_frag = "void main() {\tgl_FragColor = vec4(1,1,1,1 );\n}";


const meshbasic_vert = "void main() {\n\tvec3 transformed = vec3( position );\n\tvec4 mvPosition = modelViewMatrix * vec4( transformed, 1.0 );\n\tgl_Position = projectionMatrix * mvPosition;\n}";

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
    this.properties = new WebGLProperties();
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
    if ( index !== null ) {
      this.gl.bindBuffer( 34963, attribute.buffer );
    }

    let	dataCount = index.count;

    let rangeStart = geometry.drawRange.start;
    let rangeCount = geometry.drawRange.count;

    let groupStart = group !== null ? group.start : 0;
    let groupCount = group !== null ? group.count : Infinity;

    let drawStart = Math.max( rangeStart, groupStart );
    let drawEnd = Math.min( dataCount, rangeStart + rangeCount, groupStart + groupCount ) - 1;

    let drawCount = Math.max( 0, drawEnd - drawStart + 1 );

    if ( drawCount === 0 ) return;
    this.gl.drawElements( this.gl.TRIANGLES, drawCount, this.gl.UNSIGNED_SHORT, drawStart * 2 );
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
    let materialProperties = this.properties.get( material );
    this.initMaterial( material, fog, object );

    let program = materialProperties.program,
      p_uniforms = program.getUniforms(),
      m_uniforms = common;

    if ( this.useProgram( program.program ) ) {
      p_uniforms.setValue( this.gl, 'projectionMatrix', camera.projectionMatrix );
      WebGLUniforms.upload(this.gl, materialProperties.uniformsList, m_uniforms);
    }

    p_uniforms.setValue( this.gl, 'modelViewMatrix', object.modelViewMatrix );

    return program;

  }

  setupVertexAttributes( material, program, geometry ) {
    let geometryAttribute = geometry.attributes["position"];

    if ( geometryAttribute !== undefined ) {
      let normalized = geometryAttribute.normalized;
      let size = geometryAttribute.itemSize;
      let attribute = this.attributes.get( geometryAttribute );
      let buffer = attribute.buffer;
      let type = attribute.type;
      this.gl.enableVertexAttribArray( "position"  );
      this.gl.bindBuffer( 34962, buffer );
      this.gl.vertexAttribPointer( "position" , size, type, normalized, 0, 0 );
    }
  }

  initMaterial( material, fog, object ) {
    let materialProperties = this.properties.get( material );

    let program = this.programCache.acquireProgram( material);

    materialProperties.program = program;
    material.program = program;

    let uniforms = common;

    let progUniforms = materialProperties.program.getUniforms(),
      uniformsList =
        WebGLUniforms.seqWithValue( progUniforms.seq, uniforms );

    materialProperties.uniformsList = uniformsList;
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
