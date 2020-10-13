function WebGLAttributes( gl ) {

  var buffers = new WeakMap();

  function createBuffer( attribute, bufferType ) {

    var array = attribute.array;
    var usage = attribute.dynamic ? 35048 : 35044;

    var buffer = gl.createBuffer();

    gl.bindBuffer( bufferType, buffer );
    gl.bufferData( bufferType, array, usage );

    attribute.onUploadCallback();

    var type = 5126;

    if ( array instanceof Float32Array ) {

      type = 5126;

    } else if ( array instanceof Float64Array ) {

      console.warn( 'THREE.WebGLAttributes: Unsupported data buffer format: Float64Array.' );

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
      bytesPerElement: array.BYTES_PER_ELEMENT,
      version: attribute.version
    };

  }

  function updateBuffer( buffer, attribute, bufferType ) {

    var array = attribute.array;
    var updateRange = attribute.updateRange;

    gl.bindBuffer( bufferType, buffer );

    if ( attribute.dynamic === false ) {

      gl.bufferData( bufferType, array, 35044 );

    } else if ( updateRange.count === - 1 ) {

      // Not using update ranges

      gl.bufferSubData( bufferType, 0, array );

    } else if ( updateRange.count === 0 ) {

      console.error( 'THREE.WebGLObjects.updateBuffer: dynamic THREE.BufferAttribute marked as needsUpdate but updateRange.count is 0, ensure you are using set methods or updating manually.' );

    } else {

      gl.bufferSubData( bufferType, updateRange.offset * array.BYTES_PER_ELEMENT,
        array.subarray( updateRange.offset, updateRange.offset + updateRange.count ) );

      updateRange.count = - 1; // reset range

    }

  }

  //

  function get( attribute ) {

    if ( attribute.isInterleavedBufferAttribute ) attribute = attribute.data;

    return buffers.get( attribute );

  }


  function update( attribute, bufferType ) {

    if ( attribute.isInterleavedBufferAttribute ) attribute = attribute.data;

    var data = buffers.get( attribute );

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

class WebGL2Renderer extends THREE.WebGLRenderer {
  constructor(){
    super();
    const canvas = document.createElementNS( 'http://www.w3.org/1999/xhtml', 'canvas' );

    const gl = canvas.getContext("webgl");

    if (gl === null) {
      alert("Unable to initialize WebGL. Your browser or machine may not support it.");
      return;
    }
    console.log(this.render);
    console.log(this.renderBufferDirect);
    let attributes = new WebGLAttributes(gl);
    let currentRenderList = null;
    //the scene is non null but undefined;
    //index is non null
    //material morphTargets et morphNormals et wureframe === undefined
    //material = meshbasicmaterial =>basic shaders

		/*
    this.renderBufferDirect = function ( camera, scene, geometry, material, object, group ) {
      //let vao = gl.createVertexArray();
      //gl.bindVertexArray(vao);
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
			//console.log(gl.getShaderInfoLog(fragShader));

      let index = geometry.index;
      index.usage = gl.DYNAMIC_DRAW; //not sur if right value (try STATIC_DRAW)
      bindingStates.setup( object, material, program, geometry, index );

      const dataCount = ( index !== null ) ? index.count : position.count;

      const rangeStart = geometry.drawRange.start;
      const rangeCount = geometry.drawRange.count;

      const groupStart = group !== null ? group.start : 0;
      const groupCount = group !== null ? group.count : Infinity;

      const drawStart = Math.max( rangeStart, groupStart );
      const drawEnd = Math.min( dataCount, rangeStart + rangeCount, groupStart + groupCount ) - 1;

      const drawCount = Math.max( 0, drawEnd - drawStart + 1 )

      //gl.drawArrays( gl.TRIANGLES, drawStart, drawCount );
      attribute = attributes.get( index );
      gl.drawElements( gl.TRIANGLES, drawCount, attribute.type, 0 * attribute.bytesPerElement );
    }
		*/

    this.render = function ( scene, camera ) {

      camera.updateMatrixWorld();

      currentRenderList = this.renderLists;
      currentRenderList.init();

      projectObject( scene, camera );

      // render scene
      var opaqueObjects = currentRenderList.opaque;
      if ( opaqueObjects.length ) renderObjects( opaqueObjects, camera );
    };

    function projectObject( object, camera ) {
        if ( object.isMesh) {
            var geometry = object.geometry;//objects.update( object );
            var index = geometry.index;
            var geometryAttributes = geometry.attributes;
            if ( index !== null ) {
              attributes.update( index, 34963 );
            }
            for ( var name in geometryAttributes ) {
              attributes.update( geometryAttributes[ name ], 34962 );
            }
            var material = object.material;
            if ( material.visible ) {
              currentRenderList.push( object, geometry, material, 0, undefined, null );
            }
        }
      var children = object.children;
      for ( var i = 0, l = children.length; i < l; i ++ ) {
        projectObject( children[ i ], camera );
      }
    }

    function renderObjects( renderList, camera) {
      for ( var i = 0, l = renderList.length; i < l; i ++ ) {
        console.log(i);
        var renderItem = renderList[ i ];
        var object = renderItem.object;
        var geometry = renderItem.geometry;
        var material = renderItem.material;
        var group = renderItem.group;
        object.modelViewMatrix.multiplyMatrices( camera.matrixWorldInverse, object.matrixWorld );
        this.renderBufferDirect( camera, undefined, geometry, material, object, group );
      }
    }
  }

}
