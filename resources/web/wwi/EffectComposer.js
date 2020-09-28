
EffectCompo = function (renderer, renderTarget) {
	this.renderer = renderer;

	this.renderToScreen = true;

	this.passes = [];

	this._previousFrameTime = Date.now();
}

Object.assign( glm.vec2.prototype, {
	set: function(x, y) {
		return new glm.vec2(x,y);
	}
})

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
