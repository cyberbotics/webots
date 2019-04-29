/* global THREE */
/* exported TextureLoader */
'use strict';

var TextureLoader = {
  loadOrRetrieve: function(name, texture, cubeTextureIndex) {
    console.assert(typeof name === 'string', 'TextureLoader.loadOrRetrieve: name is not a string.');
    if (typeof name === 'undefined' || name === '')
      return undefined;
    this._getInstance().loadOrRetrieve(name, texture, cubeTextureIndex);
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
    for (let i = 0; i < textureObjects.length; i++) {
      if (textureObjects[i] instanceof THREE.CubeTexture) {
        var missingImages = this.loadingCubeTextureObjects[textureObjects[i]];
        var indices = missingImages[name];
        for (let j = 0; j < indices.length; j++) {
          if (indices[j] === 2 || indices[j] === 3)
            // Flip the top and bottom images of the cubemap to ensure a similar projection as the Webots one.
            image.src = flipImage(image.src);
          textureObjects[i].images[indices[j]] = image;
        }
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

    if (typeof this.onTextureLoad === 'function')
      this.onTextureLoad();
  }
};

// Inspired from: https://stackoverflow.com/questions/17040360/javascript-function-to-rotate-a-base-64-image-by-x-degrees-and-return-new-base64
// Flip a base64 image by 180 degrees.
function flipImage(base64Image) {
  // Create an off-screen canvas.
  var offScreenCanvas = document.createElement('canvas');
  var context = offScreenCanvas.getContext('2d');

  // Create a new Image.
  var img = new Image();
  img.src = base64Image;

  // Set its dimension to rotated size.
  offScreenCanvas.width = img.width;
  offScreenCanvas.height = img.height;

  // Rotate and draw source image into the off-screen canvas.
  context.scale(-1, -1);
  context.translate(-offScreenCanvas.height, -offScreenCanvas.width);
  context.drawImage(img, 0, 0);

  // Encode the image to data-uri with base64:
  return offScreenCanvas.toDataURL('image/jpeg', 95);
}
