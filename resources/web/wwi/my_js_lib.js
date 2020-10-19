
class Object3D {
  constructor(){
    this.userData = {"": ""};
    this.isObject3D = true;
    this.matrixAutoUpdate = true;
    this.matrixWorldNeedsUpdate = false;
    this.parent = null;
    this.children = [];
    this.position = new glm.vec3(0,0,0);
    this.scale = new glm.vec3( 1, 1, 1 );
    this.matrix = new glm.mat4();
    this.matrixWorld = new glm.mat4();
    this.quaternion = new glm.quat();
  }


	compose() {

		const te = this.matrix.elements;

		const x = this.quaternion.x, y = this.quaternion.y, z = this.quaternion.z, w = this.quaternion.w;

		const x2 = x + x,	y2 = y + y, z2 = z + z;
		const xx = x * x2, xy = x * y2, xz = x * z2;
		const yy = y * y2, yz = y * z2, zz = z * z2;
		const wx = w * x2, wy = w * y2, wz = w * z2;

		const sx = this.scale.x, sy = this.scale.y, sz = this.scale.z;

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

		te[ 12 ] = this.position.x;
		te[ 13 ] = this.position.y;
		te[ 14 ] = this.position.z;
		te[ 15 ] = 1;
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
  	}

  updateMatrixWorld(force) {
    if ( this.matrixAutoUpdate ) {
      this.compose(this.matrix );
      this.matrixWorldNeedsUpdate = true;
    }

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
}

class Group extends Object3D {
  constructor (){
    super();
  }
}

class Mesh extends Object3D {
  constructor (geometry, material) {
    super();
    this.geometry = geometry;
    this.material = material;
    this.isMesh = true;
  }
}

class Scene extends Object3D {
  constructor () {
    super();
  }
}

class Camera extends Object3D {
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

class BoxBufferGeometry {
  constructor( width = 1, height = 1, depth = 1) {
    this.attributes = {};
    this.drawRange = { start: 0, count: Infinity };

		// buffers
		const indices = [];
		const vertices = [];

		// helper variables
		let numberOfVertices = 0;

		// build each side of the box geometry
		buildPlane( 'z', 'y', 'x', - 1, - 1, depth, height, width); // px
		buildPlane( 'z', 'y', 'x', 1, - 1, depth, height, - width); // nx
		buildPlane( 'x', 'z', 'y', 1, 1, width, depth, height); // py
		buildPlane( 'x', 'z', 'y', 1, - 1, width, depth, - height ); // ny
		buildPlane( 'x', 'y', 'z', 1, - 1, width, height, depth); // pz
		buildPlane( 'x', 'y', 'z', - 1, - 1, width, height, - depth ); // nz

		// build geometry
		this.setIndex( indices );
		this.attributes['position'] = new Float32Array( vertices);

		function buildPlane( u, v, w, udir, vdir, width, height, depth ) {
			const segmentWidth = width;
			const segmentHeight = height;

			const widthHalf = width / 2;
			const heightHalf = height / 2;
			const depthHalf = depth / 2;

			const gridX1 = 2 ;
			const gridY1 = 2;

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

					const a = numberOfVertices;
					const b = numberOfVertices + gridX1;
					const c = numberOfVertices + 1 + gridX1;
					const d = numberOfVertices + 1;

					// faces
					indices.push( a, b, d );
					indices.push( b, c, d );

			// update total number of vertices
			numberOfVertices += vertexCounter;
		}
	}

  setIndex(index) {
    this.index = new Uint16Array ( index );
    return this;
  }
}

/*
class MeshBasicMaterial {
  constructor (){
	  this.visible = true;
  }
}
*/
function EffectComposer(renderer, renderTarget) {
	this.renderer = renderer;

	this.passes = [];

}

Object.assign( EffectComposer.prototype, {
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
		let previousFrameTime = Date.now();
		let pass, i, il = this.passes.length;

		for ( i = 0; i < il; i ++ ) {
			pass = this.passes[ i ];

			if ( pass.enabled === false ) continue;

			pass.render( this.renderer)
		}
	}
})

function RenderPass (scene, camera) {
	this.scene = scene;
	this.camera = camera;
}

RenderPass.prototype = Object.assign( Object.create( RenderPass.prototype ), {

	constructor: RenderPass,

	render: function ( renderer) {
		renderer.render( this.scene, this.camera );
	}
} );
