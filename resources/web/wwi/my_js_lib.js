
class obj3d {
  constructor(){
    this.type="";
    this.userData = {"": ""};
    this.name="";
    this.isObject3D = true;
    this.parent = null;
    this.children = [];
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
  remove( object ) {
  		const index = this.children.indexOf( object );
  		if ( index !== - 1 ) {
  			object.parent = null;
  			this.children.splice( index, 1 );
  		}
  		return this;
  	}
}

class groupe extends obj3d {
  constructor (){
    super();
    this.type = 'Group';
    this.isGroup = true;
  }
}

class meche extends obj3d {
  constructor (geometry, material) {
    super();
    this.type = 'Mesh';
    this.geometry = geometry;
    this.material = material;
    this.isMesh = true;
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
