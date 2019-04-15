/* global THREE */
'use strict';

function TextureManager() {
  if (!TextureManager.instance) {
    TextureManager.instance = this;
    this.textures = [];
    this.loadingTextures = [];
    this.loadingCubeTextureObjects = [];
    this.streamingMode = false;
    this.onTextureLoad = null;
  }
  return TextureManager.instance;
};

TextureManager.prototype = {
  constructor: TextureManager,

  setStreamingMode: function(enabled) {
    this.streamingMode = enabled;
  },

  getTexture: function(name) {
    return this.textures[name];
  },

  loadOrRetrieveTexture: function(name, texture, cubeTextureIndex) {
    console.assert(typeof name === 'string', 'TextureManager.loadOrRetrieveTexture: name is not a string.');
    if (typeof name === 'undefined' || name === '')
      return null;

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
      return null; // texture is already loading
    }

    console.log('loadOrRetrieveTexture ' + name);
    this.loadingTextures[name] = {data: null, objects: [texture]};

    if (this.streamingMode)
      return; // textures will be sent throug socket

    // Load from url.
    var that = this;
    var loader = new THREE.ImageLoader();
    loader.load(
      name,
      function(image) {
        if (that.loadingTextures[name]) {
          that.loadingTextures[name].data = image;
          that._onImageLoaded(name);
        } // else image already loaded
      },
      undefined, // onProgress callback
      function(err) { // onError callback
        console.error('An error happened when loading the texure "' + name + '": ' + err);
        // else image could be received later
      }
    );
    return null;
  },

  loadTexture: function(uri, name) {
    var that = this;
    var image = new Image();
    if (this.loadingTextures[name])
      this.loadingTextures[name].data = image;
    else
      this.loadingTextures[name] = {data: image, objects: []};
    image.onload = function() { that._onImageLoaded(name); };
    image.src = uri;
  },

  _onImageLoaded: function(name) {
    if (!this.loadingTextures[name])
      return;

    var image = this.loadingTextures[name].data;
    this.textures[name] = image;
    var textureObjects = this.loadingTextures[name].objects;
    // JPEGs can't have an alpha channel, so memory can be saved by storing them as RGB.
    var isJPEG = name.search(/\.jpe?g($|\?)/i) > 0 || name.search(/^data:image\/jpeg/) === 0;
    for (var i = 0; i < textureObjects.length; i++) {
      if (textureObjects[i] instanceof THREE.CubeTexture) {
        var missingImages = this.loadingCubeTextureObjects[textureObjects[i]];
        console.log(missingImages);
        var indices = missingImages[name];
        for (var j = 0; j < indices.length; j++)
          textureObjects[i].images[j] = image;
        delete missingImages[name];
        if (Object.keys(missingImages).length === 0) {
          textureObjects[i].needsUpdate = true;
          delete this.loadingCubeTextureObjects[textureObjects[i]];
        }
      } else {
        textureObjects[i].image = image;
        textureObjects[i].format = isJPEG ? THREE.RGBFormat : THREE.RGBAFormat;
        textureObjects[i].needsUpdate = true;
      }
    }
    delete this.loadingTextures[name];

    if (this.onTextureLoad)
      this.onTextureLoad();
  }
};
