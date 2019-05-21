/* global THREE */
/* exported TextureLoader */
'use strict';

var TextureLoader = {
  loadOrRetrieve: function(name, texture, cubeTextureIndex) {
    console.assert(typeof name === 'string', 'TextureLoader.loadOrRetrieve: name is not a string.');
    if (typeof name === 'undefined' || name === '')
      return undefined;
    return this._getInstance().loadOrRetrieve(name, texture, cubeTextureIndex);
  },

  loadFromUri: function(uri, name) {
    this._getInstance().loadFromUri(uri, name);
  },

  setOnTextureLoad: function(onLoad) {
    this._getInstance().onTextureLoad = onLoad;
  },

  setStreamingMode: function(enabled) {
    this._getInstance().streamingMode = enabled;
  },

  setTexturePathPrefix: function(texturePathPrefix) {
    this._getInstance().texturePathPrefix = texturePathPrefix;
  },

  _getInstance: function() {
    if (typeof this.instance === 'undefined')
      this.instance = new _TextureLoaderObject();
    return this.instance;
  }
};

class _TextureLoaderObject {
  constructor() {
    this.textures = [];
    this.loadingTextures = [];
    this.loadingCubeTextureObjects = [];
    this.streamingMode = false;
    this.onTextureLoad = undefined;
  }

  loadOrRetrieve(name, texture, cubeTextureIndex) {
    name = this.texturePathPrefix + name;
    if (this.textures[name])
      return this.textures[name];

    if (texture instanceof THREE.CubeTexture) {
      var missingImages;
      if (this.loadingCubeTextureObjects[texture]) {
        missingImages = this.loadingCubeTextureObjects[texture];
        if (missingImages[name])
          missingImages[name].push(cubeTextureIndex);
        else
          missingImages[name] = [cubeTextureIndex];
      } else {
        missingImages = {};
        missingImages[name] = [cubeTextureIndex];
        this.loadingCubeTextureObjects[texture] = missingImages;
      }
    }

    if (this.loadingTextures[name]) {
      this.loadingTextures[name].objects.push(texture);
      return undefined; // texture is already loading
    }

    this.loadingTextures[name] = {objects: [texture]};

    if (this.streamingMode)
      return; // textures will be sent throug socket

    // Load from url.
    var loader = new THREE.ImageLoader();
    loader.load(
      name,
      (image) => {
        if (this.loadingTextures[name]) {
          this.loadingTextures[name].data = image;
          this._onImageLoaded(name);
        } // else image already loaded
      },
      undefined, // onProgress callback
      function(err) { // onError callback
        console.error('An error happened when loading the texure "' + name + '": ' + err);
        // else image could be received later
      }
    );
    return undefined;
  }

  loadFromUri(uri, name) {
    name = this.texturePathPrefix + name;
    var image = new Image();
    if (this.loadingTextures[name])
      this.loadingTextures[name].data = image;
    else
      this.loadingTextures[name] = {data: image, objects: []};
    image.onload = () => { this._onImageLoaded(name); };
    image.src = uri;
  }

  _onImageLoaded(name) {
    if (!this.loadingTextures[name])
      return;

    var image = this.loadingTextures[name].data;
    this.textures[name] = image;
    var textureObjects = this.loadingTextures[name].objects;
    // JPEGs can't have an alpha channel, so memory can be saved by storing them as RGB.
    var isJPEG = name.search(/\.jpe?g($|\?)/i) > 0 || name.search(/^data:image\/jpeg/) === 0;
    textureObjects.forEach((textureObject) => {
      if (textureObject instanceof THREE.CubeTexture) {
        var missingImages = this.loadingCubeTextureObjects[textureObject];
        var indices = missingImages[name];
        indices.forEach((indice) => {
          textureObject.images[indice] = image;
        });
        delete missingImages[name];
        if (Object.keys(missingImages).length === 0) {
          textureObject.needsUpdate = true;
          delete this.loadingCubeTextureObjects[textureObject];
        }
      } else {
        textureObject.image = image;
        textureObject.format = isJPEG ? THREE.RGBFormat : THREE.RGBAFormat;
        textureObject.needsUpdate = true;
      }
    });
    delete this.loadingTextures[name];

    if (typeof this.onTextureLoad === 'function')
      this.onTextureLoad();
  }
};
