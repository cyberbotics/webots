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

  applyTextureTransform: function(texture, transformData) {
    this._getInstance().applyTextureTransform(texture, transformData);
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

  hasPendingData: function() {
    return this._getInstance().hasPendingData;
  },

  _getInstance: function() {
    if (typeof this.instance === 'undefined')
      this.instance = new _TextureLoaderObject();
    return this.instance;
  }
};

class TextureData {
  constructor(transparent, wrap, anisotropy, transform) {
    this.transparent = transparent;
    this.wrap = wrap;
    this.anisotropy = anisotropy;
    this.transform = transform;
  }

  equals(other) {
    return this.transparent === other.transparent &&
           this.anisotropy === other.anisotropy &&
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
    this.hasPendingData = false;
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
    newTexture.anisotropy = textureData.anisotropy;

    if (typeof textureData.transform !== 'undefined')
      this.applyTextureTransform(newTexture, textureData.transform);
    // This is the encoding used in Webots.
    newTexture.encoding = THREE.sRGBEncoding;
    return newTexture;
  }

  applyTextureTransform(texture, transformData) {
    if (transformData !== 'undefined') {
      texture.matrixAutoUpdate = false;
      texture.onUpdate = () => {
        // X3D UV transform matrix differs from THREE.js default one
        // http://www.web3d.org/documents/specifications/19775-1/V3.2/Part01/components/texturing.html#TextureTransform
        var c = Math.cos(-transformData.rotation);
        var s = Math.sin(-transformData.rotation);
        var sx = transformData.scale.x;
        var sy = transformData.scale.y;
        var cx = transformData.center.x;
        var cy = transformData.center.y;
        var tx = transformData.translation.x;
        var ty = transformData.translation.y;
        texture.matrix.set(
          sx * c, sx * s, sx * (tx * c + ty * s + cx * c + cy * s) - cx,
          -sy * s, sy * c, sy * (-tx * s + ty * c - cx * s + cy * c) - cy,
          0, 0, 1
        );
      };
    } else {
      texture.matrixAutoUpdate = true;
      texture.onUpdate = null;
    }
    texture.needsUpdate = true;
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
    this.hasPendingData = true;
    this._setTimeout();

    // Load from url.
    var loader;
    var isHDR = hasHDRExtension(name);
    if (isHDR) {
      loader = new THREE.RGBELoader();
      loader.type = THREE.FloatType;
    } else
      loader = new THREE.ImageLoader();
    loader.load(
      name,
      (data) => {
        if (this.loadingTextures[name]) {
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
          if (indice === 2 || indice === 3) {
            // Flip the top and bottom images of the cubemap to ensure a similar projection as the Webots one.
            if (isHDR)
              flipHDRImage(image.image);
            else
              flipRegularImage(image);
          }
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

    this._evaluatePendingData();
  }

  _setTimeout() {
    // Set texture loading timeout.
    // If after some time no new textures are loaded, the hasPendingData variable is automatically
    // reset to false in order to handle not found textures.
    // The `this.loadingTextures` dictionary is not reset so that it is still possible to load late textures.
    if (this.timeoutHandle)
      window.clearTimeout(this.timeoutHandle);

    this.timeoutHandle = window.setTimeout(() => {
      var message = 'ERROR: Texture loader timeout elapsed. The following textures could not be loaded: \n';
      for (let key in this.loadingTextures)
        message += key + '\n';
      console.error(message);
      this.hasPendingData = false;

      if (typeof this.onTextureLoad === 'function')
        this.onTextureLoad();
    }, 10000); // wait 10 seconds
  }

  _evaluatePendingData() {
    this.hasPendingData = false;
    for (let key in this.loadingTextures) {
      if (this.loadingTextures.hasOwnProperty(key)) {
        this.hasPendingData = true;
        break;
      }
    }

    if (this.hasPendingData)
      this._setTimeout();
    else if (this.timeoutHandle)
      window.clearTimeout(this.timeoutHandle);
  }
};

// Inspired from: https://stackoverflow.com/questions/17040360/javascript-function-to-rotate-a-base-64-image-by-x-degrees-and-return-new-base64
// Flip a base64 image by 180 degrees.
function flipRegularImage(base64Image) {
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
  base64Image.src = offScreenCanvas.toDataURL('image/jpeg', 95);
}

function flipHDRImage(image) {
  let size = image.width * image.height;
  let d = new Float32Array(3 * size);
  let max = 3 * (size - 1);
  let i = 0;
  let c = 0;
  for (i = 0; i < 3 * size; i += 3) {
    let m = max - i;
    for (c = 0; c < 3; c++)
      d[i + c] = image.data[m + c];
  }
  image.data = d;
}

function hasJPEGExtension(name) {
  return name.search(/\.jpe?g($|\?)/i) > 0 || name.search(/^data:image\/jpeg/) === 0;
}

function hasHDRExtension(name) {
  return name.search(/\.hdr($|\?)/i) > 0 || name.search(/^data:image\/hdr/) === 0;
}
