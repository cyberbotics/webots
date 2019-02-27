/* global THREE */
'use strict';

class TextureManager { // eslint-disable-line no-unused-vars
  constructor() {
    if (!TextureManager.instance) {
      TextureManager.instance = this;
      this.textures = [];
    }
    return TextureManager.instance;
  }

  getTexture(name) {
    return this.textures[name];
  }

  loadTexture(uri, name) {
    var image = new Image();
    image.src = uri;
    var texture = this.textures[name];
    if (!texture)
      texture = new THREE.Texture();
    texture.image = image;
    image.onload = function() {
      texture.needsUpdate = true;
    };
    this.textures[name] = texture;
  }
}
