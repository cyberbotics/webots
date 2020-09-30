
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
    this.position = new THREE.Vector3(1,1,1);
    this.isObject3D = true;
    this.matrixAutoUpdate = true;
    this.matrixWorldNeedsUpdate = false;
    this.parent = null;
    this.children = [];
    this.matrix = new THREE.Matrix4();
    this.matrixWorld = new THREE.Matrix4();
    this.modelViewMatrix = new THREE.Matrix4();
    this.normalMatrix = new THREE.Matrix3();
    this.layers = new Layer();//new THREE.Layers();
    this.drawMode = 0;
    this.quaternion = new THREE.Quaternion();
    this.scale = new THREE.Vector3( 1, 1, 1 );
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
    this.matrix.compose( this.position, this.quaternion, this.scale );
    this.matrixWorldNeedsUpdate = true;
  }

  updateMatrixWorld(force) {
    if ( this.matrixAutoUpdate ) this.updateMatrix();

    if ( this.matrixWorldNeedsUpdate || force ) {
      if ( this.parent === null ) {
        this.matrixWorld.copy( this.matrix );
      } else {
            this.matrixWorld.multiplyMatrices( this.parent.matrixWorld, this.matrix );
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
    this.projectionMatrix = new THREE.Matrix4();
    this.matrixWorldInverse = new THREE.Matrix4();
    this.isCamera = true;
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

		this.projectionMatrix.makePerspective( left, left + width, top, top - height, near, this.far );
  	//this.projectionMatrixInverse.getInverse( this.projectionMatrix );
  }

  updateMatrixWorld(force) {
    super.updateMatrixWorld(force);
    this.matrixWorldInverse.getInverse( this.matrixWorld );
  }

}

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
		this.attributes['position'] = new THREE.Float32BufferAttribute( vertices, 3 );

		function buildPlane( u, v, w, udir, vdir, width, height, depth, gridX, gridY, materialIndex ) {
			const segmentWidth = width / gridX;
			const segmentHeight = height / gridY;

			const widthHalf = width / 2;
			const heightHalf = height / 2;
			const depthHalf = depth / 2;

			const gridX1 = gridX + 1;
			const gridY1 = gridY + 1;

			let vertexCounter = 0;

			//const vector = new THREE.Vector3();
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
      this.index = new ( this.arrayMax( index ) > 65535 ? THREE.Uint32BufferAttribute : THREE.Uint16BufferAttribute )( index, 1 );
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
    this.color = new THREE.Color(0xffffff);
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

/*
class WebGL2Renderer extends THREE.WebGLRenderer{
  constructor() {
    super();
    this.width;
    this.height;
    //canvas is normally private
    //this.domElement = this.canvas;
    this.pixelRatio = 1;
    this.viewport = glm.vec4(0, 0, this.width, this.height);
    this.currentRenderTarget = null;
    this.currentActiveMipmapLevel = 0;
    this.setSize = function (width, height, updateStyle) {
      this.width = width;
      this.height = height;
      this.domElement.width = Math.floor( width * this.pixelRatio );
      this.domElement.height = Math.floor( height * this.pixelRatio );

      if ( updateStyle !== false ) {
        this.domElement.style.width = width + 'px';
        this.domElement.style.height = height + 'px';
      }
      this.setViewport( 0, 0, width, height ); //setViewport est sensé modifier state.viewport. je vais d'abord essayer en me passant de hasTransparentTexture
      //this.viewport = new THREE.Vector4(0,0, width, height)
    }
}



  getDrawingBufferSize (target) {
    return target.set(Math.floor(this.widtht * this.pixelRatio, this.height * this.pixelRatio));
  }

  getRenderTarget() {
    return this.currentRenderTarget;
  }
  setRenderTarget(){
  }
}
*/
Object.assign( glm.vec2.prototype, {
	set: function(x, y) {
		return new glm.vec2(x,y);
	}
})

EffectCompo = function (renderer, renderTarget) {
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
		let size = this.renderer.getDrawingBufferSize( new glm.vec2() );
		pass.setSize( size.width, size.height );
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
		let currentRenderTarget = this.renderer.getRenderTarget();

		let pass, i, il = this.passes.length;

		for ( i = 0; i < il; i ++ ) {
			pass = this.passes[ i ];

			if ( pass.enabled === false ) continue;

			pass.renderToScreen = ( this.renderToScreen && this.isLastEnabledPass( i ));
			pass.render( this.renderer)
		}

		this.renderer.setRenderTarget( currentRenderTarget );
	}
})

RenPass = function (scene, camera) {
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
		// TODO: Avoid using autoClear properties, see https://github.com/mrdoob/three.js/pull/15571#issuecomment-465669600
		renderer.render( this.scene, this.camera );
	}
} );
