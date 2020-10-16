
class Layer {
  constructor() {
    this.mask = 1 | 0;
  }
  test( layer ) {
		return ( this.mask & layer.mask ) !== 0;
	}
}

class Obj3d {
  constructor(){
    this.type="";
    this.userData = {"": ""};
    this.name="";
    this.position = new glm.vec3(1,1,1);
    this.isObject3D = true;
    this.matrixAutoUpdate = true;
    this.matrixWorldNeedsUpdate = false;
    this.parent = null;
    this.children = [];
    this.matrix = new glm.mat4();
    this.matrixWorld = new glm.mat4();
    this.modelViewMatrix = new glm.mat4();
    this.normalMatrix = new glm.mat3();
    this.layers = new Layer();
    this.drawMode = 0;
    this.quaternion = new glm.quat();
    this.scale = new glm.vec3( 1, 1, 1 );
  }


	compose( matrix, position, quaternion, scale ) {

		const te = matrix.elements;

		const x = quaternion.x, y = quaternion.y, z = quaternion.z, w = quaternion.w;
		const x2 = x + x,	y2 = y + y, z2 = z + z;
		const xx = x * x2, xy = x * y2, xz = x * z2;
		const yy = y * y2, yz = y * z2, zz = z * z2;
		const wx = w * x2, wy = w * y2, wz = w * z2;

		const sx = scale.x, sy = scale.y, sz = scale.z;

		te[ 0 ] = ( 1 - ( yy + zz ) ) * sx;
		te[ 1 ] = ( xy + wz ) * sx;
		te[ 2 ] = ( xz - wy ) * sx;
		te[ 3 ] = 0;

		te[ 4 ] = ( xy - wz ) * sy;
		te[ 5 ] = ( 1 - ( xx + zz ) ) * sy;
		te[ 6 ] = ( yz + wx ) * sy;
		te[ 7 ] = 0;

		te[ 8 ] = ( xz + wy ) * sz;
		te[ 9 ] = ( yz - wx ) * sz;
		te[ 10 ] = ( 1 - ( xx + yy ) ) * sz;
		te[ 11 ] = 0;

		te[ 12 ] = position.x;
		te[ 13 ] = position.y;
		te[ 14 ] = position.z;
		te[ 15 ] = 1;

		return matrix;

	}

  add(object) {
    if (object == this) {
      console.error("Object cannot be added as a child of itself");
    }
    if((object && object.isObject3D)) {
      if (object.parent !== null)
        object.parent.remove(object);
      object.parent = this;
      this.children.push(object);
    } else {
      console.error( "obj3d add: object not an instance of obj3d.", object );
    }
  }

  remove(object) {
  		const index = this.children.indexOf( object );
  		if ( index !== - 1 ) {
  			object.parent = null;
  			this.children.splice( index, 1 );
  		}
  		return this;
  	}

  updateMatrix() {
    this.matrix = this.compose(this.matrix, this.position, this.quaternion, this.scale );
    this.matrixWorldNeedsUpdate = true;
  }

  updateMatrixWorld(force) {
    if ( this.matrixAutoUpdate ) this.updateMatrix();

    if ( this.matrixWorldNeedsUpdate || force ) {
      if ( this.parent === null ) {
        this.matrixWorld.copy( this.matrix );
      } else {
            this.matrixWorld = this.parent.matrixWorld['*'](this.matrix);
      }
      this.matrixWorldNeedsUpdate = false;
      force = true;
    }

    const children = this.children;

    for ( let i = 0, l = children.length; i < l; i ++ ) {
      children[ i ].updateMatrixWorld( force );
    }
  }

  onBeforeRender() {

  }

  onAfterRender() {

  }

  dispatchEvent(e) {

  }
  addEventListener() {

  }
}

class Groupe extends Obj3d {
  constructor (){
    super();
    this.type = 'Group';
    this.isGroup = true;
  }
}

class Meche extends Obj3d {
  constructor (geometry, material) {
    super();
    this.type = 'Mesh';
    this.geometry = geometry;
    this.material = material;
    this.isMesh = true;
  }
}

class Saine extends Obj3d {
  constructor () {
    super();
    this.isScene = true;
    this.type= 'Scene';
    this.background;
  }
}

class Cam extends Obj3d {
  constructor(fov, aspect, near, far){
    super();
    this.fov = fov;
    this.aspect = aspect
    this.near = near;
    this.far = far;
    this.zoom = 1;
    this.view = null;
    this.projectionMatrix = new glm.mat4();
    this.matrixWorldInverse = new glm.mat4();
    this.isCamera = true;
  }

  makePerspective(out, left, right, top, bottom, near, far ) {

  const te = out.elements;
  const x = 2 * near / ( right - left );
  const y = 2 * near / ( top - bottom );

  const a = ( right + left ) / ( right - left );
  const b = ( top + bottom ) / ( top - bottom );
  const c = - ( far + near ) / ( far - near );
  const d = - 2 * far * near / ( far - near );

  te[ 0 ] = x;	te[ 4 ] = 0;	te[ 8 ] = a;	te[ 12 ] = 0;
  te[ 1 ] = 0;	te[ 5 ] = y;	te[ 9 ] = b;	te[ 13 ] = 0;
  te[ 2 ] = 0;	te[ 6 ] = 0;	te[ 10 ] = c;	te[ 14 ] = d;
  te[ 3 ] = 0;	te[ 7 ] = 0;	te[ 11 ] = - 1;	te[ 15 ] = 0;

  return te;

}

  updateProjectionMatrix() {
    const near = this.near;
    let top = near * Math.tan( glm.radians(0.5 * this.fov )) / this.zoom;
    let height = 2 * top;
    let width = this.aspect * height;
    let left = - 0.5 * width;
    const view = this.view;

		if ( this.view !== null && this.view.enabled ) {
		    const fullWidth = view.fullWidth,
    		fullHeight = view.fullHeight;

    		left += view.offsetX * width / fullWidth;
    		top -= view.offsetY * height / fullHeight;
    		width *= view.width / fullWidth;
    		height *= view.height / fullHeight;
		}

		this.projectionMatrix = this.makePerspective(this.projectionMatrix , left, left + width, top, top - height, near, this.far );
  }

  updateMatrixWorld(force) {
    super.updateMatrixWorld(force);
    this.matrixWorldInverse = glm.inverse( this.matrixWorld );
  }

}

function BufferAttribute( array, itemSize, normalized ) {

	this.name = '';

	this.array = array;
	this.itemSize = itemSize;
	this.count = array !== undefined ? array.length / itemSize : 0;
	this.normalized = normalized === true;

	this.usage = 35044;
	this.updateRange = { offset: 0, count: - 1 };

	this.version = 0;
}


function Uint16BufferAttribute( array, itemSize, normalized ) {

	return new BufferAttribute ( new Uint16Array( array ), itemSize, normalized );

}


function Float32BufferAttribute( array, itemSize, normalized ) {

	return new BufferAttribute( new Float32Array( array ), itemSize, normalized );

}

//

class BoxBufferGeo {
  constructor( width = 1, height = 1, depth = 1, widthSegments = 1, heightSegments = 1, depthSegments = 1 ) {
    this.type = 'BoxBufferGeometry';
		this.width = width;
		this.height = height;
		this.depth = depth;
		this.widthSegments = widthSegments;
		this.heightSegments = heightSegments;
		this.depthSegments = depthSegments;
    this.attributes = {};
    this.isBufferGeometry = true
    this.drawRange = { start: 0, count: Infinity };

		// segments
		widthSegments = Math.floor( widthSegments );
		heightSegments = Math.floor( heightSegments );
		depthSegments = Math.floor( depthSegments );

		// buffers
		const indices = [];
		const vertices = [];

		// helper variables
		let numberOfVertices = 0;

		// build each side of the box geometry
		buildPlane( 'z', 'y', 'x', - 1, - 1, depth, height, width, depthSegments, heightSegments, 0 ); // px
		buildPlane( 'z', 'y', 'x', 1, - 1, depth, height, - width, depthSegments, heightSegments, 1 ); // nx
		buildPlane( 'x', 'z', 'y', 1, 1, width, depth, height, widthSegments, depthSegments, 2 ); // py
		buildPlane( 'x', 'z', 'y', 1, - 1, width, depth, - height, widthSegments, depthSegments, 3 ); // ny
		buildPlane( 'x', 'y', 'z', 1, - 1, width, height, depth, widthSegments, heightSegments, 4 ); // pz
		buildPlane( 'x', 'y', 'z', - 1, - 1, width, height, - depth, widthSegments, heightSegments, 5 ); // nz

		// build geometry
		this.setIndex( indices );
		this.attributes['position'] = new Float32BufferAttribute( vertices, 3 );

		function buildPlane( u, v, w, udir, vdir, width, height, depth, gridX, gridY, materialIndex ) {
			const segmentWidth = width / gridX;
			const segmentHeight = height / gridY;

			const widthHalf = width / 2;
			const heightHalf = height / 2;
			const depthHalf = depth / 2;

			const gridX1 = gridX + 1;
			const gridY1 = gridY + 1;

			let vertexCounter = 0;

      const vector = new glm.vec3();
			// generate vertices, normals and uvs
			for ( let iy = 0; iy < gridY1; iy ++ ) {
				const y = iy * segmentHeight - heightHalf;
				for ( let ix = 0; ix < gridX1; ix ++ ) {
					const x = ix * segmentWidth - widthHalf;

					// set values to correct vector component
					vector[ u ] = x * udir;
					vector[ v ] = y * vdir;
					vector[ w ] = depthHalf;

					// now apply vector to vertex buffer
					vertices.push( vector.x, vector.y, vector.z );

					// set values to correct vector component
					vector[ u ] = 0;
					vector[ v ] = 0;
					vector[ w ] = depth > 0 ? 1 : - 1;

					vertexCounter += 1;
				}
			}

			// indices
			// 1. you need three indices to draw a single face
			// 2. a single segment consists of two faces
			// 3. so we need to generate six (2*3) indices per segment
			for ( let iy = 0; iy < gridY; iy ++ ) {
				for ( let ix = 0; ix < gridX; ix ++ ) {
					const a = numberOfVertices + ix + gridX1 * iy;
					const b = numberOfVertices + ix + gridX1 * ( iy + 1 );
					const c = numberOfVertices + ( ix + 1 ) + gridX1 * ( iy + 1 );
					const d = numberOfVertices + ( ix + 1 ) + gridX1 * iy;

					// faces
					indices.push( a, b, d );
					indices.push( b, c, d );

				}
			}

			// update total number of vertices
			numberOfVertices += vertexCounter;
		}
	}
  arrayMax( array ) {
    if ( array.length === 0 ) return - Infinity;
    let max = array[ 0 ];
    for ( let i = 1, l = array.length; i < l; ++ i ) {
      if ( array[ i ] > max ) max = array[ i ];
    }
    return max;
  }
  setIndex(index) {
    if ( Array.isArray( index ) ) {
      this.index = new Uint16BufferAttribute ( index, 1 );
    } else {
      this.index = index;
    }
    return this;
  }
  addEventListener(){

  }
}

class MeshBasicMat {
  constructor (){
    this.type = 'MeshBasicMaterial';
    this.color = new Module.Color(255,255,255);
    this.isMeshBasicMaterial = true;
    this.lights = false;
	  this.needsUpdate = false
  	this.colorWrite = true;
	  this.precision = null; // override the renderer's default precision for this material
	  this.visible = true;
	  this.userData = {};
  }

  onBeforeCompile() {

  }
  addEventListener(){

  }
}

Object.assign( glm.vec2.prototype, {
	set: function(x, y) {
		return new glm.vec2(x,y);
	}
})

function EffectCompo(renderer, renderTarget) {
	this.renderer = renderer;

	this.renderToScreen = true;

	this.passes = [];

	this._previousFrameTime = Date.now();
}

Object.assign( EffectCompo.prototype, {
	swapBuffers: function () {
		let tmp = this.readBuffer;
		this.readBuffer = this.writeBuffer;
		this.writeBuffer = tmp;
	},

	addPass: function ( pass ) {
		this.passes.push( pass );
	},

	isLastEnabledPass: function ( passIndex ) {
		for ( let i = passIndex + 1; i < this.passes.length; i ++ ) {
			if ( this.passes[ i ].enabled ) {
				return false;
			}
		}
		return true;
	},

	render: function () {
		this._previousFrameTime = Date.now();
		let pass, i, il = this.passes.length;

		for ( i = 0; i < il; i ++ ) {
			pass = this.passes[ i ];

			if ( pass.enabled === false ) continue;

			pass.renderToScreen = ( this.renderToScreen && this.isLastEnabledPass( i ));
			pass.render( this.renderer)
		}
	}
})

function RenPass (scene, camera) {
	this.enable=true;
	this.needsSwap = false;
	this.clear = true;
	this.renderToScreen = false;

	this.scene = scene;
	this.camera = camera;


}

RenPass.prototype = Object.assign( Object.create( RenPass.prototype ), {

	constructor: RenPass,

	setSize: function ( width, height ) {},

	render: function ( renderer) {
		renderer.render( this.scene, this.camera );
	}
} );
