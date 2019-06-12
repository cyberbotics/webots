/* global THREE */
/* exported TextureLoader */
'use strict';

var TextureLoader = {
  createEmptyTexture: function(name) {
    if (hasHDRExtension(name)) {
      var texture = new THREE.DataTexture();
      texture.encoding = THREE.RGBEEncoding;
      texture.minFilter = THREE.NearestFilter;
      texture.magFilter = THREE.NearestFilter;
      texture.flipY = true;
      return texture;
    }
    return new THREE.Texture();
  },

  createColoredCubeTexture: function(color, width = 1, height = 1) {
    // Create an off-screen canvas.
    var canvas = document.createElement('canvas');
    var context = canvas.getContext('2d');
    canvas.width = width;
    canvas.height = height;

    // Create RGB values.
    var r = Math.floor(color.r * 255);
    var g = Math.floor(color.g * 255);
    var b = Math.floor(color.b * 255);

    // Push pixels.
    var data = context.createImageData(width, height);
    var size = width * height;
    for (let i = 0; i < size; i++) {
      let stride = i * 4;
      data.data[stride + 0] = r;
      data.data[stride + 1] = g;
      data.data[stride + 2] = b;
      data.data[stride + 3] = 255;
    }
    context.putImageData(data, 0, 0);

    // Create the CubeTexture.
    var src = canvas.toDataURL();
    var loader = new THREE.CubeTextureLoader();
    return loader.load([src, src, src, src, src, src]);
  },

  loadOrRetrieve: function(name, texture, cubeTextureIndex = undefined, onLoad = undefined) {
    console.assert(typeof name === 'string', 'TextureLoader.loadOrRetrieve: name is not a string.');
    if (typeof name === 'undefined' || name === '')
      return undefined;
    return this._getInstance().loadOrRetrieve(name, texture, cubeTextureIndex, onLoad);
  },

  setOnTextureLoad: function(onLoad) {
    this._getInstance().onTextureLoad = onLoad;
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
    this.onTextureLoad = undefined;
    this.texturePathPrefix = '';
  }

  loadOrRetrieve(name, texture, cubeTextureIndex, onLoad) {
    if (this.texturePathPrefix)
      name = this.texturePathPrefix + name;
    if (this.textures[name]) {
      if (typeof onLoad !== 'undefined')
        onLoad(this.textures[name]);
      return this.textures[name];
    }

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
      if (typeof texture !== 'undefined')
        this.loadingTextures[name].objects.push(texture);
      if (typeof onLoad !== 'undefined')
        this.loadingTextures[name].onLoad.push(onLoad);
      return undefined; // texture is already loading
    }

    this.loadingTextures[name] = {objects: [], onLoad: []};
    if (typeof texture !== 'undefined')
      this.loadingTextures[name].objects.push(texture);
    if (typeof onLoad !== 'undefined')
      this.loadingTextures[name].onLoad.push(onLoad);

    // Load from url.
    var loader;
    var isHDR = hasHDRExtension(name);
    if (isHDR)
      loader = new THREE.RGBELoader();
    else
      loader = new THREE.ImageLoader();
    loader.load(
      name,
      (data) => {
        if (this.loadingTextures[name]) {
          if (isHDR)
            // HDR loader returns a THREE.DataTexture object
            this.loadingTextures[name].data = data.image;
          else // data has Image type
            this.loadingTextures[name].data = data;
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

  _onImageLoaded(name) {
    if (!this.loadingTextures[name])
      return;

    var image = this.loadingTextures[name].data;
    this.textures[name] = image;
    var textureObjects = this.loadingTextures[name].objects;
    // JPEGs can't have an alpha channel, so memory can be saved by storing them as RGB.
    var isJPEG = hasJPEGExtension(name);
    var isHDR = isJPEG ? false : hasHDRExtension(name);
    textureObjects.forEach((textureObject) => {
      if (textureObject instanceof THREE.CubeTexture) {
        var missingImages = this.loadingCubeTextureObjects[textureObject];
        var indices = missingImages[name];
        indices.forEach((indice) => {
          if (indice === 2 || indice === 3)
            // Flip the top and bottom images of the cubemap to ensure a similar projection as the Webots one.
            image.src = flipImage(image);
          textureObject.images[indice] = image;
        });
        delete missingImages[name];
        if (Object.keys(missingImages).length === 0) {
          textureObject.needsUpdate = true;
          delete this.loadingCubeTextureObjects[textureObject];
        }
      } else {
        if (!isHDR)
          textureObject.format = isJPEG ? THREE.RGBFormat : THREE.RGBAFormat;
        textureObject.image = image;
        textureObject.needsUpdate = true;
      }
    });

    var callbackFunctions = this.loadingTextures[name].onLoad;
    callbackFunctions.forEach((callback) => {
      if (typeof callback === 'function')
        callback(image);
    });
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

  // Set its dimension to rotated size.
  offScreenCanvas.width = base64Image.width;
  offScreenCanvas.height = base64Image.height;

  // Rotate and draw source image into the off-screen canvas.
  context.scale(-1, -1);
  context.translate(-offScreenCanvas.height, -offScreenCanvas.width);
  context.drawImage(base64Image, 0, 0);

  // Encode the image to data-uri with base64:
  return offScreenCanvas.toDataURL('image/jpeg', 95);
}

function hasJPEGExtension(name) {
  return name.search(/\.jpe?g($|\?)/i) > 0 || name.search(/^data:image\/jpeg/) === 0;
}

function hasHDRExtension(name) {
  return name.search(/\.hdr($|\?)/i) > 0 || name.search(/^data:image\/hdr/) === 0;
}
