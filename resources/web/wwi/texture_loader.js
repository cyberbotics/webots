/* global THREE */
/* exported TextureLoader, TextureData */
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

  createOrRetrieveTexture: function(filename, textureData) {
    console.assert(typeof filename === 'string', 'TextureLoader.createOrRetrieveTexture: name is not a string.');
    return this._getInstance().createOrRetrieveTexture(filename, textureData);
  },

  loadOrRetrieveImage: function(filename, texture, cubeTextureIndex = undefined, onLoad = undefined) {
    console.assert(typeof filename === 'string', 'TextureLoader.loadOrRetrieveImage: name is not a string.');
    if (typeof filename === 'undefined' || filename === '')
      return undefined;
    return this._getInstance().loadOrRetrieveImage(filename, texture, cubeTextureIndex, onLoad);
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

class TextureData {
  constructor(transparent, wrap, transform) {
    this.transparent = transparent;
    this.wrap = wrap;
    this.transform = transform;
  }

  equals(other) {
    return this.transparent === other.transparent &&
           JSON.stringify(this.wrap) === JSON.stringify(other.wrap) &&
           JSON.stringify(this.transform) === JSON.stringify(other.transform);
  }
};

class _TextureLoaderObject {
  constructor() {
    this.images = []; // list of image names
    this.textures = {}; // dictionary <texture file name, array <[texture data, texture object]>
    this.loadingTextures = {}; // dictionary <texture file name, dictionary <'objects': [texture objects], 'onLoad': [callback functions] > > 
    this.loadingCubeTextureObjects = {}; // dictionary <cube texture object, dictionary < image name: [cube image index] > >
    this.onTextureLoad = undefined;
    this.texturePathPrefix = '';
  }

  createOrRetrieveTexture(filename, textureData) {
    let textures = this.textures[filename];
    if (textures) {
      for (let i in textures) {
        if (textures[i][0].equals(textureData))
          return textures[i][1];
      }
    } else
      this.textures[filename] = [];

    // Create THREE.Texture or THREE.DataTexture based on image extension.
    let newTexture = TextureLoader.createEmptyTexture(filename);
    this.textures[filename].push([textureData, newTexture]);

    // Look for already loaded texture or load the texture in an asynchronous way.
    var image = this.loadOrRetrieveImage(filename, newTexture);
    if (typeof image !== 'undefined') { // else it could be updated later
      newTexture.image = image;
      newTexture.needsUpdate = true;
    }

    newTexture.userData = {
      'isTransparent': textureData.transparent,
      'url': filename
    };
    newTexture.wrapS = textureData.wrap.s === 'true' ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;
    newTexture.wrapT = textureData.wrap.t === 'true' ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;

    if (typeof textureData.transform !== 'undefined') {
      newTexture.matrixAutoUpdate = false;
      newTexture.onUpdate = () => {
        // X3D UV transform matrix differs from THREE.js default one
        // http://www.web3d.org/documents/specifications/19775-1/V3.2/Part01/components/texturing.html#TextureTransform
        var c = Math.cos(-textureData.transform.rotation);
        var s = Math.sin(-textureData.transform.rotation);
        var sx = textureData.transform.scale.x;
        var sy = textureData.transform.scale.y;
        var cx = textureData.transform.center.x;
        var cy = textureData.transform.center.y;
        var tx = textureData.transform.translation.x;
        var ty = textureData.transform.translation.y;
        newTexture.matrix.set(
          sx * c, sx * s, sx * (tx * c + ty * s + cx * c + cy * s) - cx,
          -sy * s, sy * c, sy * (-tx * s + ty * c - cx * s + cy * c) - cy,
          0, 0, 1
        );
      };
      newTexture.needsUpdate = true;
    }
    // This is the encoding used in Webots.
    newTexture.encoding = THREE.sRGBEncoding;
    return newTexture;
  }

  loadOrRetrieveImage(name, texture, cubeTextureIndex = undefined, onLoad = undefined) {
    if (this.texturePathPrefix)
      name = this.texturePathPrefix + name;
    if (this.images[name]) {
      if (typeof onLoad !== 'undefined')
        onLoad(this.images[name]);
      return this.images[name];
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
    this.images[name] = image;
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
