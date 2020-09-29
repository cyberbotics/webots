
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
    this.layers = new THREE.Layers();
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


class BoxBufferGeo extends THREE.BufferGeometry {
  constructor( width = 1, height = 1, depth = 1, widthSegments = 1, heightSegments = 1, depthSegments = 1 ) {
    super(width,height,depth,widthSegments,heightSegments,depthSegments );
    this.type = 'BoxBufferGeometry';
		this.width = width;
		this.height = height;
		this.depth = depth;
		this.widthSegments = widthSegments;
		this.heightSegments = heightSegments;
		this.depthSegments = depthSegments;
  
		const scope = this;

		// segments

		widthSegments = Math.floor( widthSegments );
		heightSegments = Math.floor( heightSegments );
		depthSegments = Math.floor( depthSegments );

		// buffers

		const indices = [];
		const vertices = [];
		const normals = [];
		const uvs = [];

		// helper variables

		let numberOfVertices = 0;
		let groupStart = 0;

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
		this.attributes['normal'] = new THREE.Float32BufferAttribute( normals, 3 );
		this.attributes ['uv'] = new THREE.Float32BufferAttribute( uvs, 2 );

		function buildPlane( u, v, w, udir, vdir, width, height, depth, gridX, gridY, materialIndex ) {

			const segmentWidth = width / gridX;
			const segmentHeight = height / gridY;

			const widthHalf = width / 2;
			const heightHalf = height / 2;
			const depthHalf = depth / 2;

			const gridX1 = gridX + 1;
			const gridY1 = gridY + 1;

			let vertexCounter = 0;
			let groupCount = 0;

			const vector = new THREE.Vector3();

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

					// now apply vector to normal buffer

					normals.push( vector.x, vector.y, vector.z );

					// uvs

					uvs.push( ix / gridX );
					uvs.push( 1 - ( iy / gridY ) );

					// counters

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

					// increase counter

					groupCount += 6;

				}

			}

			// add a group to the geometry. this will ensure multi material support

			scope.addGroup( groupStart, groupCount, materialIndex );

			// calculate new start value for groups

			groupStart += groupCount;

			// update total number of vertices

			numberOfVertices += vertexCounter;

		}

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

class WebGL2Renderer {
  constructor() {
    this.width;
    this.height;
    //canvas is normally private
    this.canvas = document.createElementNS( 'http://www.w3.org/1999/xhtml', 'canvas' );
    this.domElement = this.canvas;
    this.pixelRatio = 1;
    this.viewport = glm.vec4(0, 0, this.width, this.height);
    this.currentRenderTarget = null;
  }

  setSize(width, height, updateStyle) {
    this.width = width;
    this.height = height;
    this.canvas.width = Math.floor( width * this.pixelRatio );
    this.canvas.height = Math.floor( height * this.pixelRatio );

    if ( updateStyle !== false ) {
      this.canvas.style.width = width + 'px';
      this.canvas.style.height = height + 'px';
		}
    //this.setViewport( 0, 0, width, height ); setViewport est sensé modifier state.viewport. je vais d'abord essayer en me passant de hasTransparentTexture
    this.viewport = new glm.vec4(0,0, width, height)
  }

  getDrawingBufferSize (target) {
    return target.set(Math.floor(this.widtht * this.pixelRatio, this.height * this.pixelRatio));
  }

  getRenderTarget() {
    return this.currentRenderTarget;
  }

  render(scene, camera) {

  }
}

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
