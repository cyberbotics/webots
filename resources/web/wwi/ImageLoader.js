import loadHdr from './hdr_loader.js';
import WbImage from './nodes/WbImage.js';
import {arrayXPointer} from './nodes/utils/utils.js';

export default class ImageLoader {
  static loadImageTextureInWren(imageTexture, prefix, url, checkTransparency = true) {
    const needToDownloadTexture = Module.ccall('wr_texture_2d_copy_from_cache', 'number', ['string'], [url]);
    if (needToDownloadTexture !== 0)
      return Promise.resolve();

    return this.loadTextureData(prefix, url).then((image) => {
      let isTransparent = false;
      if (checkTransparency) {
        for (let i = 3, n = image.bits.length; i < n; i += 4) {
          if (image.bits[i] < 255) {
            isTransparent = true;
            break;
          }
        }
      }

      imageTexture.isTransparent = isTransparent;

      const texture = _wr_texture_2d_new();
      _wr_texture_set_size(texture, image.width, image.height);
      _wr_texture_set_translucent(texture, isTransparent);
      const bitsPointer = arrayXPointer(image.bits);
      _wr_texture_2d_set_data(texture, bitsPointer);
      Module.ccall('wr_texture_2d_set_file_path', null, ['number', 'string'], [texture, url]);
      _wr_texture_setup(texture);
      _free(bitsPointer);
    });
  }

  static loadTextureData(prefix, url, isHdr, rotation) {
    const canvas2 = document.createElement('canvas');
    const context = canvas2.getContext('2d');

    if (url.startsWith('webots://')) {
      if (typeof ImageLoader.repository === 'undefined')
        ImageLoader.repository = 'cyberbotics';
      if (typeof ImageLoader.branch === 'undefined' || ImageLoader.branch === '')
        ImageLoader.branch = 'released';
      url = url.replace('webots://', 'https://raw.githubusercontent.com/' + ImageLoader.repository + '/webots/' + ImageLoader.branch + '/');
    }
    if (typeof prefix !== 'undefined' && !url.startsWith('http')) {
      if (['smaa_area_texture.png', 'smaa_search_texture.png', 'gtao_noise_texture.png'].includes(url) || ImageLoader.stream)
        url = prefix + url;
      else {
        // in simulations the asset is provided relative to the world, therefore the URL has to be resolved before requesting it
        let worldsPath = ImageLoader.currentWorld;
        if (typeof worldsPath !== 'undefined')
          worldsPath = worldsPath.substring(0, worldsPath.lastIndexOf('/')) + '/';
        else
          worldsPath = '';
        url = prefix + worldsPath + url;
      }
    }

    if (isHdr) {
      return this.#loadHDRImage(url).then(img => {
        const image = new WbImage();
        image.bits = img.data;
        image.width = img.width;
        image.height = img.height;
        image.url = url;
        if (typeof rotation !== 'undefined' && rotation !== 0)
          image.bits = this.#rotateHDR(image, rotation);
        return image;
      });
    } else {
      return this.#loadImage(url).then(img => {
        const image = new WbImage();

        canvas2.width = img.width;
        canvas2.height = img.height;
        if (typeof rotation !== 'undefined' && rotation !== 0) {
          context.save();
          context.translate(canvas2.width / 2, canvas2.height / 2);
          context.rotate(rotation * Math.PI / 180);
          context.drawImage(img, -canvas2.width / 2, -canvas2.height / 2);
          context.restore();
        } else
          context.drawImage(img, 0, 0);
        if (img.width > 0 && img.height > 0) {
          const dataBGRA = context.getImageData(0, 0, img.width, img.height).data;
          let data = new Uint8ClampedArray(dataBGRA.length);
          data = dataBGRA;

          image.bits = data;
        }
        image.width = img.width;
        image.height = img.height;
        image.url = url;
        return image;
      });
    }
  }

  static #loadImage(src) {
    return new Promise((resolve, reject) => {
      const img = new Image();
      img.onload = () => {
        resolve(img);
      };
      img.onerror = () => {
        if (typeof img.failed === 'undefined') {
          console.error('Error in loading: ' + src);
          img.src = 'https://cyberbotics.com/wwi/images/missing_texture.png';
          img.failed = true;
        } else
          // Fail to get the missing texture image, exit to avoid an infinite loop
          resolve(img);
      };
      img.setAttribute('crossOrigin', '');
      img.src = src;
    });
  }

  static #loadHDRImage(src) {
    return new Promise((resolve, reject) => {
      loadHdr(src, function(img) { resolve(img); });
    });
  }

  static #rotateHDR(image, rotate) {
    const rotatedbits = [];
    if (rotate === 90) {
      for (let x = 0; x < image.width; x++) {
        for (let y = 0; y < image.height; y++) {
          const u = y * image.width * 3 + x * 3;
          const v = (image.width - 1 - x) * image.height * 3 + y * 3;
          for (let c = 0; c < 3; c++)
            rotatedbits[u + c] = image.bits[v + c];
        }
      }
      const swap = image.width;
      image.width = image.height;
      image.height = swap;
    } else if (rotate === -90) {
      for (let x = 0; x < image.width; x++) {
        for (let y = 0; y < image.height; y++) {
          const u = y * image.width * 3 + x * 3;
          const v = x * image.width * 3 + (image.height - 1 - y) * 3;
          for (let c = 0; c < 3; c++)
            rotatedbits[u + c] = image.bits[v + c];
        }
      }
      const swap = image.width;
      image.width = image.height;
      image.height = swap;
    } else if (rotate === 180) {
      for (let x = 0; x < image.width; x++) {
        for (let y = 0; y < image.height; y++) {
          const u = y * image.width * 3 + x * 3;
          const v = (image.height - 1 - y) * image.width * 3 + (image.width - 1 - x) * 3;
          for (let c = 0; c < 3; c++)
            rotatedbits[u + c] = image.bits[v + c];
        }
      }
    }
    return rotatedbits;
  }
}
