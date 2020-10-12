function WebGLExtensions( gl ) {

	const extensions = {};

	return {

		has: function ( name ) {

			if ( extensions[ name ] !== undefined ) {

				return extensions[ name ] !== null;

			}

			let extension;

			switch ( name ) {

				case 'WEBGL_depth_texture':
					extension = gl.getExtension( 'WEBGL_depth_texture' ) || gl.getExtension( 'MOZ_WEBGL_depth_texture' ) || gl.getExtension( 'WEBKIT_WEBGL_depth_texture' );
					break;

				case 'EXT_texture_filter_anisotropic':
					extension = gl.getExtension( 'EXT_texture_filter_anisotropic' ) || gl.getExtension( 'MOZ_EXT_texture_filter_anisotropic' ) || gl.getExtension( 'WEBKIT_EXT_texture_filter_anisotropic' );
					break;

				case 'WEBGL_compressed_texture_s3tc':
					extension = gl.getExtension( 'WEBGL_compressed_texture_s3tc' ) || gl.getExtension( 'MOZ_WEBGL_compressed_texture_s3tc' ) || gl.getExtension( 'WEBKIT_WEBGL_compressed_texture_s3tc' );
					break;

				case 'WEBGL_compressed_texture_pvrtc':
					extension = gl.getExtension( 'WEBGL_compressed_texture_pvrtc' ) || gl.getExtension( 'WEBKIT_WEBGL_compressed_texture_pvrtc' );
					break;

				default:
					extension = gl.getExtension( name );

			}

			extensions[ name ] = extension;

			return extension !== null;

		},

		get: function ( name ) {

			if ( ! this.has( name ) ) {

				console.warn( 'WebGLRenderer: ' + name + ' extension not supported.' );

			}

			return extensions[ name ];

		}

	};

}

function WebGLCapabilities( gl, extensions, parameters ) {

	let maxAnisotropy;

	function getMaxAnisotropy() {

		if ( maxAnisotropy !== undefined ) return maxAnisotropy;

		const extension = extensions.get( 'EXT_texture_filter_anisotropic' );

		if ( extension !== null ) {

			maxAnisotropy = gl.getParameter( extension.MAX_TEXTURE_MAX_ANISOTROPY_EXT );

		} else {

			maxAnisotropy = 0;

		}

		return maxAnisotropy;

	}

	function getMaxPrecision( precision ) {

		if ( precision === 'highp' ) {

			if ( gl.getShaderPrecisionFormat( gl.VERTEX_SHADER, gl.HIGH_FLOAT ).precision > 0 &&
				gl.getShaderPrecisionFormat( gl.FRAGMENT_SHADER, gl.HIGH_FLOAT ).precision > 0 ) {

				return 'highp';

			}

			precision = 'mediump';

		}

		if ( precision === 'mediump' ) {

			if ( gl.getShaderPrecisionFormat( gl.VERTEX_SHADER, gl.MEDIUM_FLOAT ).precision > 0 &&
				gl.getShaderPrecisionFormat( gl.FRAGMENT_SHADER, gl.MEDIUM_FLOAT ).precision > 0 ) {

				return 'mediump';

			}

		}

		return 'lowp';

	}

	/* eslint-disable no-undef */
	const isWebGL2 = ( typeof WebGL2RenderingContext !== 'undefined' && gl instanceof WebGL2RenderingContext ) ||
		( typeof WebGL2ComputeRenderingContext !== 'undefined' && gl instanceof WebGL2ComputeRenderingContext );
	/* eslint-enable no-undef */

	let precision = parameters.precision !== undefined ? parameters.precision : 'highp';
	const maxPrecision = getMaxPrecision( precision );

	if ( maxPrecision !== precision ) {

		console.warn( 'WebGLRenderer:', precision, 'not supported, using', maxPrecision, 'instead.' );
		precision = maxPrecision;

	}

	const logarithmicDepthBuffer = parameters.logarithmicDepthBuffer === true;

	const maxTextures = gl.getParameter( gl.MAX_TEXTURE_IMAGE_UNITS );
	const maxVertexTextures = gl.getParameter( gl.MAX_VERTEX_TEXTURE_IMAGE_UNITS );
	const maxTextureSize = gl.getParameter( gl.MAX_TEXTURE_SIZE );
	const maxCubemapSize = gl.getParameter( gl.MAX_CUBE_MAP_TEXTURE_SIZE );

	const maxAttributes = gl.getParameter( gl.MAX_VERTEX_ATTRIBS );
	const maxVertexUniforms = gl.getParameter( gl.MAX_VERTEX_UNIFORM_VECTORS );
	const maxVaryings = gl.getParameter( gl.MAX_VARYING_VECTORS );
	const maxFragmentUniforms = gl.getParameter( gl.MAX_FRAGMENT_UNIFORM_VECTORS );

	const vertexTextures = maxVertexTextures > 0;
	const floatFragmentTextures = isWebGL2 || !! extensions.get( 'OES_texture_float' );
	const floatVertexTextures = vertexTextures && floatFragmentTextures;

	const maxSamples = isWebGL2 ? gl.getParameter( gl.MAX_SAMPLES ) : 0;

	return {

		isWebGL2: isWebGL2,

		getMaxAnisotropy: getMaxAnisotropy,
		getMaxPrecision: getMaxPrecision,

		precision: precision,
		logarithmicDepthBuffer: logarithmicDepthBuffer,

		maxTextures: maxTextures,
		maxVertexTextures: maxVertexTextures,
		maxTextureSize: maxTextureSize,
		maxCubemapSize: maxCubemapSize,

		maxAttributes: maxAttributes,
		maxVertexUniforms: maxVertexUniforms,
		maxVaryings: maxVaryings,
		maxFragmentUniforms: maxFragmentUniforms,

		vertexTextures: vertexTextures,
		floatFragmentTextures: floatFragmentTextures,
		floatVertexTextures: floatVertexTextures,

		maxSamples: maxSamples

	};

}

function WebGLAttributes( gl, capabilities ) {

	const isWebGL2 = capabilities.isWebGL2;

	const buffers = new WeakMap();

	function createBuffer( attribute, bufferType ) {

		const array = attribute.array;
		const usage = attribute.usage;
		const buffer = gl.createBuffer();

		gl.bindBuffer( bufferType, buffer );
		gl.bufferData( bufferType, array, usage );

		attribute.onUploadCallback();

		let type = gl.FLOAT;

		if ( array instanceof Float32Array ) {

			type = gl.FLOAT;

		} else if ( array instanceof Float64Array ) {

			console.warn( 'WebGLAttributes: Unsupported data buffer format: Float64Array.' );

		} else if ( array instanceof Uint16Array ) {

			type = gl.UNSIGNED_SHORT;

		} else if ( array instanceof Int16Array ) {

			type = gl.SHORT;

		} else if ( array instanceof Uint32Array ) {

			type = gl.UNSIGNED_INT;

		} else if ( array instanceof Int32Array ) {

			type = gl.INT;

		} else if ( array instanceof Int8Array ) {

			type = gl.BYTE;

		} else if ( array instanceof Uint8Array ) {

			type = gl.UNSIGNED_BYTE;

		}

		return {
			buffer: buffer,
			type: type,
			bytesPerElement: array.BYTES_PER_ELEMENT,
			version: attribute.version
		};

	}

	function updateBuffer( buffer, attribute, bufferType ) {

		const array = attribute.array;
		const updateRange = attribute.updateRange;

		gl.bindBuffer( bufferType, buffer );

		if ( updateRange.count === - 1 ) {

			// Not using update ranges

			gl.bufferSubData( bufferType, 0, array );

		} else {

			if ( isWebGL2 ) {

				gl.bufferSubData( bufferType, updateRange.offset * array.BYTES_PER_ELEMENT,
					array, updateRange.offset, updateRange.count );

			} else {

				gl.bufferSubData( bufferType, updateRange.offset * array.BYTES_PER_ELEMENT,
					array.subarray( updateRange.offset, updateRange.offset + updateRange.count ) );

			}

			updateRange.count = - 1; // reset range

		}

	}

	//

	function get( attribute ) {

		if ( attribute.isInterleavedBufferAttribute ) attribute = attribute.data;

		return buffers.get( attribute );

	}

	function remove( attribute ) {

		if ( attribute.isInterleavedBufferAttribute ) attribute = attribute.data;

		const data = buffers.get( attribute );

		if ( data ) {

			gl.deleteBuffer( data.buffer );

			buffers.delete( attribute );

		}

	}

	function update( attribute, bufferType ) {

		if ( attribute.isGLBufferAttribute ) {

			var cached = buffers.get( attribute );

			if ( ! cached || cached.version < attribute.version ) {

				buffers.set( attribute, {
					buffer: attribute.buffer,
					type: attribute.type,
					bytesPerElement: attribute.elementSize,
					version: attribute.version
				} );

			}

			return;

		}

		if ( attribute.isInterleavedBufferAttribute ) attribute = attribute.data;

		const data = buffers.get( attribute );

		if ( data === undefined ) {

			buffers.set( attribute, createBuffer( attribute, bufferType ) );

		} else if ( data.version < attribute.version ) {

			updateBuffer( data.buffer, attribute, bufferType );

			data.version = attribute.version;

		}

	}

	return {

		get: get,
		remove: remove,
		update: update,
    buffers: buffers
	};

}

function WebGLBindingStates( gl, extensions, attributes, capabilities ) {

	const maxVertexAttributes = gl.getParameter( gl.MAX_VERTEX_ATTRIBS );

	const extension = capabilities.isWebGL2 ? null : extensions.get( 'OES_vertex_array_object' );
	const vaoAvailable = capabilities.isWebGL2 || extension !== null;

	const bindingStates = {};

	const defaultState = createBindingState( null );
	let currentState = defaultState;

	function setup( object, material, program, geometry, index ) {

		let updateBuffers = false;

		if ( vaoAvailable ) {

			const state = getBindingState( geometry, program, material );

			if ( currentState !== state ) {

				currentState = state;
				bindVertexArrayObject( currentState.object );

			}

			updateBuffers = needsUpdate( geometry, index );

			if ( updateBuffers ) saveCache( geometry, index );

		} else {

			const wireframe = ( material.wireframe === true );

			if ( currentState.geometry !== geometry.id ||
				currentState.program !== program.id ||
				currentState.wireframe !== wireframe ) {

				currentState.geometry = geometry.id;
				currentState.program = program.id;
				currentState.wireframe = wireframe;

				updateBuffers = true;

			}

		}

		if ( object.isInstancedMesh === true ) {

			updateBuffers = true;

		}

		if ( index !== null ) {
			attributes.update( index, gl.ELEMENT_ARRAY_BUFFER );

		}

		if ( updateBuffers ) {

			setupVertexAttributes( object, material, program, geometry );

			if ( index !== null ) {
				gl.bindBuffer( gl.ELEMENT_ARRAY_BUFFER, attributes.get( index ).buffer );

			}

		}

	}

	function createVertexArrayObject() {

		if ( capabilities.isWebGL2 ) return gl.createVertexArray();

		return extension.createVertexArrayOES();

	}

	function bindVertexArrayObject( vao ) {

		if ( capabilities.isWebGL2 ) return gl.bindVertexArray( vao );

		return extension.bindVertexArrayOES( vao );

	}

	function deleteVertexArrayObject( vao ) {

		if ( capabilities.isWebGL2 ) return gl.deleteVertexArray( vao );

		return extension.deleteVertexArrayOES( vao );

	}

	function getBindingState( geometry, program, material ) {

		const wireframe = ( material.wireframe === true );

		let programMap = bindingStates[ geometry.id ];

		if ( programMap === undefined ) {

			programMap = {};
			bindingStates[ geometry.id ] = programMap;

		}

		let stateMap = programMap[ program.id ];

		if ( stateMap === undefined ) {

			stateMap = {};
			programMap[ program.id ] = stateMap;

		}

		let state = stateMap[ wireframe ];

		if ( state === undefined ) {

			state = createBindingState( createVertexArrayObject() );
			stateMap[ wireframe ] = state;

		}

		return state;

	}

	function createBindingState( vao ) {

		const newAttributes = [];
		const enabledAttributes = [];
		const attributeDivisors = [];

		for ( let i = 0; i < maxVertexAttributes; i ++ ) {

			newAttributes[ i ] = 0;
			enabledAttributes[ i ] = 0;
			attributeDivisors[ i ] = 0;

		}

		return {

			// for backward compatibility on non-VAO support browser
			geometry: null,
			program: null,
			wireframe: false,

			newAttributes: newAttributes,
			enabledAttributes: enabledAttributes,
			attributeDivisors: attributeDivisors,
			object: vao,
			attributes: {},
			index: null

		};

	}

	function needsUpdate( geometry, index ) {

		const cachedAttributes = currentState.attributes;
		const geometryAttributes = geometry.attributes;

		if ( Object.keys( cachedAttributes ).length !== Object.keys( geometryAttributes ).length ) return true;

		for ( const key in geometryAttributes ) {

			const cachedAttribute = cachedAttributes[ key ];
			const geometryAttribute = geometryAttributes[ key ];

			if ( cachedAttribute === undefined ) return true;

			if ( cachedAttribute.attribute !== geometryAttribute ) return true;

			if ( cachedAttribute.data !== geometryAttribute.data ) return true;

		}

		if ( currentState.index !== index ) return true;

		return false;

	}

	function saveCache( geometry, index ) {

		const cache = {};
		const attributes = geometry.attributes;

		for ( const key in attributes ) {

			const attribute = attributes[ key ];

			const data = {};
			data.attribute = attribute;

			if ( attribute.data ) {

				data.data = attribute.data;

			}

			cache[ key ] = data;

		}

		currentState.attributes = cache;

		currentState.index = index;

	}

	function initAttributes() {

		const newAttributes = currentState.newAttributes;

		for ( let i = 0, il = newAttributes.length; i < il; i ++ ) {

			newAttributes[ i ] = 0;

		}

	}

	function enableAttribute( attribute ) {

		enableAttributeAndDivisor( attribute, 0 );

	}

	function enableAttributeAndDivisor( attribute, meshPerAttribute ) {

		const newAttributes = currentState.newAttributes;
		const enabledAttributes = currentState.enabledAttributes;
		const attributeDivisors = currentState.attributeDivisors;

		newAttributes[ attribute ] = 1;

		if ( enabledAttributes[ attribute ] === 0 ) {
			console.log(attribute);
			gl.enableVertexAttribArray( attribute );
			enabledAttributes[ attribute ] = 1;

		}

		if ( attributeDivisors[ attribute ] !== meshPerAttribute ) {

			const extension = capabilities.isWebGL2 ? gl : extensions.get( 'ANGLE_instanced_arrays' );

			extension[ capabilities.isWebGL2 ? 'vertexAttribDivisor' : 'vertexAttribDivisorANGLE' ]( attribute, meshPerAttribute );
			attributeDivisors[ attribute ] = meshPerAttribute;

		}

	}

	function disableUnusedAttributes() {

		const newAttributes = currentState.newAttributes;
		const enabledAttributes = currentState.enabledAttributes;

		for ( let i = 0, il = enabledAttributes.length; i < il; i ++ ) {

			if ( enabledAttributes[ i ] !== newAttributes[ i ] ) {

				gl.disableVertexAttribArray( i );
				enabledAttributes[ i ] = 0;

			}

		}

	}

	function vertexAttribPointer( index, size, type, normalized, stride, offset ) {

		if ( capabilities.isWebGL2 === true && ( type === gl.INT || type === gl.UNSIGNED_INT ) ) {
			gl.vertexAttribIPointer( index, size, type, stride, offset );

		} else {
			gl.vertexAttribPointer( index, size, type, normalized, stride, offset );

		}

	}

	function fetchAttributeLocations( gl, program ) {

		const attributes = {};

		const n = gl.getProgramParameter( program, gl.ACTIVE_ATTRIBUTES );

		for ( let i = 0; i < n; i ++ ) {

			const info = gl.getActiveAttrib( program, i );
			const name = info.name;

			// console.log( 'THREE.WebGLProgram: ACTIVE VERTEX ATTRIBUTE:', name, i );

			attributes[ name ] = gl.getAttribLocation( program, name );

		}

		return attributes;

	}
	function setupVertexAttributes( object, material, program, geometry ) {
		if ( capabilities.isWebGL2 === false && ( object.isInstancedMesh || geometry.isInstancedBufferGeometry ) ) {

			if ( extensions.get( 'ANGLE_instanced_arrays' ) === null ) return;

		}

		initAttributes();

		const geometryAttributes = geometry.attributes;

		//const programAttributes = program.getAttributes();
		const programAttributes = fetchAttributeLocations( gl, program );
		const materialDefaultAttributeValues = material.defaultAttributeValues;

		for ( const name in programAttributes ) {
			const programAttribute = programAttributes[ name ];

			if ( programAttribute >= 0 ) {
				const geometryAttribute = geometryAttributes[ name ];

				if ( geometryAttribute !== undefined ) {
					const normalized = geometryAttribute.normalized;
					const size = geometryAttribute.itemSize;

					const attribute = attributes.get( geometryAttribute );

					// TODO Attribute may not be available on context restore

					if ( attribute === undefined ) continue;

					const buffer = attribute.buffer;
					const type = attribute.type;
					const bytesPerElement = attribute.bytesPerElement;

					if ( geometryAttribute.isInterleavedBufferAttribute ) {

						const data = geometryAttribute.data;
						const stride = data.stride;
						const offset = geometryAttribute.offset;

						if ( data && data.isInstancedInterleavedBuffer ) {

							enableAttributeAndDivisor( programAttribute, data.meshPerAttribute );

							if ( geometry._maxInstanceCount === undefined ) {

								geometry._maxInstanceCount = data.meshPerAttribute * data.count;

							}

						} else {

							enableAttribute( programAttribute );

						}

						gl.bindBuffer( gl.ARRAY_BUFFER, buffer );
						vertexAttribPointer( programAttribute, size, type, normalized, stride * bytesPerElement, offset * bytesPerElement );

					} else {

						if ( geometryAttribute.isInstancedBufferAttribute ) {

							enableAttributeAndDivisor( programAttribute, geometryAttribute.meshPerAttribute );

							if ( geometry._maxInstanceCount === undefined ) {

								geometry._maxInstanceCount = geometryAttribute.meshPerAttribute * geometryAttribute.count;

							}

						} else {

							enableAttribute( programAttribute );

						}

						gl.bindBuffer( gl.ARRAY_BUFFER, buffer );
						vertexAttribPointer( programAttribute, size, type, normalized, 0, 0 );

					}

				} else if ( name === 'instanceMatrix' ) {

					const attribute = attributes.get( object.instanceMatrix );

					// TODO Attribute may not be available on context restore

					if ( attribute === undefined ) continue;

					const buffer = attribute.buffer;
					const type = attribute.type;

					enableAttributeAndDivisor( programAttribute + 0, 1 );
					enableAttributeAndDivisor( programAttribute + 1, 1 );
					enableAttributeAndDivisor( programAttribute + 2, 1 );
					enableAttributeAndDivisor( programAttribute + 3, 1 );

					gl.bindBuffer( gl.ARRAY_BUFFER, buffer );
					gl.vertexAttribPointer( programAttribute + 0, 4, type, false, 64, 0 );
					gl.vertexAttribPointer( programAttribute + 1, 4, type, false, 64, 16 );
					gl.vertexAttribPointer( programAttribute + 2, 4, type, false, 64, 32 );
					gl.vertexAttribPointer( programAttribute + 3, 4, type, false, 64, 48 );

				} else if ( name === 'instanceColor' ) {

					const attribute = attributes.get( object.instanceColor );

					// TODO Attribute may not be available on context restore

					if ( attribute === undefined ) continue;

					const buffer = attribute.buffer;
					const type = attribute.type;

					enableAttributeAndDivisor( programAttribute, 1 );

					gl.bindBuffer( gl.ARRAY_BUFFER, buffer );
					gl.vertexAttribPointer( programAttribute, 3, type, false, 12, 0 );

				} else if ( materialDefaultAttributeValues !== undefined ) {

					const value = materialDefaultAttributeValues[ name ];

					if ( value !== undefined ) {

						switch ( value.length ) {

							case 2:
								gl.vertexAttrib2fv( programAttribute, value );
								break;

							case 3:
								gl.vertexAttrib3fv( programAttribute, value );
								break;

							case 4:
								gl.vertexAttrib4fv( programAttribute, value );
								break;

							default:
								gl.vertexAttrib1fv( programAttribute, value );

						}

					}

				}

			}

		}

		disableUnusedAttributes();

	}

	function dispose() {

		reset();

		for ( const geometryId in bindingStates ) {

			const programMap = bindingStates[ geometryId ];

			for ( const programId in programMap ) {

				const stateMap = programMap[ programId ];

				for ( const wireframe in stateMap ) {

					deleteVertexArrayObject( stateMap[ wireframe ].object );

					delete stateMap[ wireframe ];

				}

				delete programMap[ programId ];

			}

			delete bindingStates[ geometryId ];

		}

	}

	function releaseStatesOfGeometry( geometry ) {

		if ( bindingStates[ geometry.id ] === undefined ) return;

		const programMap = bindingStates[ geometry.id ];

		for ( const programId in programMap ) {

			const stateMap = programMap[ programId ];

			for ( const wireframe in stateMap ) {

				deleteVertexArrayObject( stateMap[ wireframe ].object );

				delete stateMap[ wireframe ];

			}

			delete programMap[ programId ];

		}

		delete bindingStates[ geometry.id ];

	}

	function releaseStatesOfProgram( program ) {

		for ( const geometryId in bindingStates ) {

			const programMap = bindingStates[ geometryId ];

			if ( programMap[ program.id ] === undefined ) continue;

			const stateMap = programMap[ program.id ];

			for ( const wireframe in stateMap ) {

				deleteVertexArrayObject( stateMap[ wireframe ].object );

				delete stateMap[ wireframe ];

			}

			delete programMap[ program.id ];

		}

	}

	function reset() {

		resetDefaultState();

		if ( currentState === defaultState ) return;

		currentState = defaultState;
		bindVertexArrayObject( currentState.object );

	}

	// for backward-compatilibity

	function resetDefaultState() {

		defaultState.geometry = null;
		defaultState.program = null;
		defaultState.wireframe = false;

	}

	return {

		setup: setup,
		reset: reset,
		resetDefaultState: resetDefaultState,
		dispose: dispose,
		releaseStatesOfGeometry: releaseStatesOfGeometry,
		releaseStatesOfProgram: releaseStatesOfProgram,

		initAttributes: initAttributes,
		enableAttribute: enableAttribute,
		disableUnusedAttributes: disableUnusedAttributes

	};

}

class WebGL2Renderer extends THREE.WebGLRenderer {
  constructor(){
    super();
    const canvas = document.createElementNS( 'http://www.w3.org/1999/xhtml', 'canvas' );

    const gl = canvas.getContext("webgl2");

    if (gl === null) {
      alert("Unable to initialize WebGL. Your browser or machine may not support it.");
      return;
    }
    /*
    let fragCode =
		"#version 300 es\n"+
		"precision highp float;"+
		"out vec4 color;\n"+
		"void main()\n"+
		"{\n"+
			"color = vec4(1,0,0,1);\n"+
		"}\n";

    let vertCode =
		"#version 300 es\n"+
		"layout(location = 0) in vec3 position;\n"+
		"void main()\n"+
		"{\n"+
			"gl_Position = vec4(position, 1);\n"+
		"}\n";
		*/
		let fragCode =
		'void main(void) {' +
		' gl_FragColor = vec4(1.0, 0.0, 0.0, 0.1);' +
		'}';

		let vertCode =
		'attribute vec3 position;' +
		'void main(void) {' +
		' gl_Position = vec4(position, 1.0);' +
		'}';

    let parameters = {};
    let extensions = new WebGLExtensions(gl);
    let capabilities = new WebGLCapabilities(gl, extensions, parameters);
    let attributes = new WebGLAttributes(gl, capabilities);
    let bindingStates = new WebGLBindingStates(gl, extensions, attributes, capabilities);

    //the scene is non null but undefined;
    //index is non null
    //material morphTargets et morphNormals et wureframe === undefined
    //material = meshbasicmaterial =>basic shaders
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
