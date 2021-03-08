import { FileLoader } from './file_loader.js';
import { DefaultLoadingManager } from './loading_manager.js';

/**
* Enum for WebGL pixel format.
* @name zen3d.WEBGL_PIXEL_FORMAT
* @readonly
* @enum {number}
*/
export const WEBGL_PIXEL_FORMAT = {
  DEPTH_COMPONENT: 0x1902,
  DEPTH_STENCIL: 0x84F9,
  ALPHA: 0x1906,
  RED: 0x1903, // webgl2
  RGB: 0x1907,
  RGBA: 0x1908,
  LUMINANCE: 0x1909,
  LUMINANCE_ALPHA: 0x190A,
  // only for internal formats
  R8: 0x8229, // webgl2
  RGBA8: 0x8058,
  RGBA16F: 0x881A,
  RGBA32F: 0x8814,
  DEPTH_COMPONENT16: 0x81A5,
  DEPTH_COMPONENT24: 0x81A6,
  DEPTH_COMPONENT32F: 0x8CAC,
  DEPTH24_STENCIL8: 0x88F0,
  DEPTH32F_STENCIL8: 0x8CAD
};

/**
* Enum for WebGL pixel Type.
* @name zen3d.WEBGL_PIXEL_TYPE
* @readonly
* @enum {number}
*/
export const WEBGL_PIXEL_TYPE = {
  BYTE: 0x1400,
  UNSIGNED_BYTE: 0x1401,
  SHORT: 0x1402,
  UNSIGNED_SHORT: 0x1403,
  INT: 0x1404,
  UNSIGNED_INT: 0x1405,
  FLOAT: 0x1406,
  HALF_FLOAT: 36193,
  UNSIGNED_INT_24_8: 0x84FA,
  UNSIGNED_SHORT_4_4_4_4: 0x8033,
  UNSIGNED_SHORT_5_5_5_1: 0x8034,
  UNSIGNED_SHORT_5_6_5: 0x8363,
  FLOAT_32_UNSIGNED_INT_24_8_REV: 0x8DAD
};
/**
* A loader for loading a .hdr Image.
* @constructor
* @memberof zen3d
* @param {zen3d.LoadingManager} manager — The loadingManager for the loader to use. Default is zen3d.DefaultLoadingManager.
*/
function RGBELoader(manager) {
  console.log("YOYOYO")
  this.manager = (manager !== undefined) ? manager : DefaultLoadingManager;
  this.type = WEBGL_PIXEL_TYPE.FLOAT;
}
Object.assign(RGBELoader.prototype, /** @lends zen3d.RGBELoader.prototype */{
  /**
  * Load the URL and pass the response to the onLoad function.
  * @param {string} url — the path or URL to the file. This can also be a Data URI.
  * @param {Function} [onLoad=] — Will be called when loading completes. The argument will be the loaded image ( draw to an canvas element ).
  * @param {Function} [onProgress=] — Will be called while load progresses. The argument will be the XMLHttpRequest instance, which contains .total and .loaded bytes.
  * @param {Function} [onError=] — Will be called if an error occurs.
  */
  load: function(url, onLoad, onProgress, onError) {
    const that = this;
    const loader = new FileLoader(this.manager);
    loader.setResponseType('arraybuffer');
    loader.load(url, function(buffer) {
      if (onLoad !== undefined)
        onLoad(that.parse(buffer));
    }, onProgress, onError);
  },
  // adapted from http://www.graphics.cornell.edu/~bjw/rgbe.html
  parse: function(buffer) {
    /* return codes for rgbe routines */
    // RGBE_RETURN_SUCCESS = 0,
    const RGBE_RETURN_FAILURE = -1;
    /* default error routine.  change this to change error handling */
    const rgbeReadError = 1;
    const rgbeWriteError = 2;
    const rgbeFormatError = 3;
    const rgbeMemoryError = 4;
    let rgbeError = function(rgbeErrorCode, msg) {
      switch (rgbeErrorCode) {
        case rgbeReadError: console.error('zen3d.RGBELoader Read Error: ' + (msg || ''));
          break;
        case rgbeWriteError: console.error('zen3d.RGBELoader Write Error: ' + (msg || ''));
          break;
        case rgbeFormatError: console.error('zen3d.RGBELoader Bad File Format: ' + (msg || ''));
          break;
        default:
        case rgbeMemoryError: console.error('zen3d.RGBELoader: Error: ' + (msg || ''));
      }
      return RGBE_RETURN_FAILURE;
    };
    /* offsets to red, green, and blue components in a data (float) pixel */
    // RGBE_DATA_RED = 0,
    // RGBE_DATA_GREEN = 1,
    // RGBE_DATA_BLUE = 2,
    /* number of floats per pixel, use 4 since stored in rgba image format */
    // RGBE_DATA_SIZE = 4,
    /* flags indicating which fields in an rgbeHeaderInfo are valid */
    const RGBE_VALID_PROGRAMTYPE = 1;
    const RGBE_VALID_FORMAT = 2;
    const RGBE_VALID_DIMENSIONS = 4;
    const NEWLINE = '\n';
    let fgets = function(buffer, lineLimit, consume) {
      lineLimit = !lineLimit ? 1024 : lineLimit;
      let p = buffer.pos;
      let i = -1;
      let len = 0;
      let s = '';
      let chunkSize = 128;
      let chunk = String.fromCharCode.apply(null, new Uint16Array(buffer.subarray(p, p + chunkSize)));
      while (((i = chunk.indexOf(NEWLINE)) < 0) && (len < lineLimit) && (p < buffer.byteLength)) {
        s += chunk; len += chunk.length;
        p += chunkSize;
        chunk += String.fromCharCode.apply(null, new Uint16Array(buffer.subarray(p, p + chunkSize)));
      }
      if (i > -1) {
        /* for (i=l-1; i>=0; i--) {
        byteCode = m.charCodeAt(i);
        if (byteCode > 0x7f && byteCode <= 0x7ff) byteLen++;
        else if (byteCode > 0x7ff && byteCode <= 0xffff) byteLen += 2;
        if (byteCode >= 0xDC00 && byteCode <= 0xDFFF) i--; //trail surrogate
      } */
        if (consume !== false) buffer.pos += len + i + 1;
        return s + chunk.slice(0, i);
      }
      return false;
    };
    /* minimal header reading.  modify if you want to parse more information */
    let RGBEReadHeader = function(buffer) {
      let line, match;
      // regexes to parse header info fields
      const magicTokenRe = /^#\?(\S+)$/;
      const gammaRe = /^\s*GAMMA\s*=\s*(\d+(\.\d+)?)\s*$/;
      const exposureRe = /^\s*EXPOSURE\s*=\s*(\d+(\.\d+)?)\s*$/;
      const formatRe = /^\s*FORMAT=(\S+)\s*$/;
      const dimensionsRe = /^\s*\-Y\s+(\d+)\s+\+X\s+(\d+)\s*$/;
      // RGBE format header struct
      const header = {
        valid: 0, /* indicate which fields are valid */
        string: '', /* the actual header string */
        comments: '', /* comments found in header */
        programtype: 'RGBE', /* listed at beginning of file to identify it after "#?". defaults to "RGBE" */
        format: '', /* RGBE format, default 32-bit_rle_rgbe */
        gamma: 1.0, /* image has already been gamma corrected with given gamma. defaults to 1.0 (no correction) */
        exposure: 1.0, /* a value of 1.0 in an image corresponds to <exposure> watts/steradian/m^2. defaults to 1.0 */
        width: 0,
        height: 0 /* image dimensions, width/height */
      };

      if (buffer.pos >= buffer.byteLength || !(line = fgets(buffer)))
        return rgbeError(rgbeReadError, 'no header found');

      /* if you want to require the magic token then uncomment the next line */
      if (!(match = line.match(magicTokenRe)))
        return rgbeError(rgbeFormatError, 'bad initial token');

      header.valid |= RGBE_VALID_PROGRAMTYPE;
      header.programtype = match[1];
      header.string += line + '\n';
      while (true) {
        line = fgets(buffer);
        if (line === false) break;
        header.string += line + '\n';
        if (line.charAt(0) === '#') {
          header.comments += line + '\n';
          continue; // comment line
        }
        if ((match = line.match(gammaRe)))
          header.gamma = parseFloat(match[1]);
        if ((match = line.match(exposureRe)))
          header.exposure = parseFloat(match[1]);

        if ((match = line.match(formatRe))) {
          header.valid |= RGBE_VALID_FORMAT;
          header.format = match[1];// '32-bit_rle_rgbe';
        }
        if ((match = line.match(dimensionsRe))) {
          header.valid |= RGBE_VALID_DIMENSIONS;
          header.height = parseInt(match[1], 10);
          header.width = parseInt(match[2], 10);
        }
        if ((header.valid & RGBE_VALID_FORMAT) && (header.valid & RGBE_VALID_DIMENSIONS)) break;
      }
      if (!(header.valid & RGBE_VALID_FORMAT))
        return rgbeError(rgbeFormatError, 'missing format specifier');

      if (!(header.valid & RGBE_VALID_DIMENSIONS))
        return rgbeError(rgbeFormatError, 'missing image size specifier');

      return header;
    };

    let RGBEReadPixelsRLE = function(buffer, w, h) {
      let dataRgba, offset, pos, count, byteValue,
        scanlineBuffer, ptr, ptrEnd, i, l, off, isEncodedRun, rgbeStart;
      let scanlineWidth = w;
      let numScanlines = h;
      if (
        // run length encoding is not allowed so read flat
        ((scanlineWidth < 8) || (scanlineWidth > 0x7fff)) ||
        // this file is not run length encoded
        ((buffer[0] !== 2) || (buffer[1] !== 2) || (buffer[2] & 0x80))
      ) {
        // return the flat buffer
        return new Uint8Array(buffer);
      }
      if (scanlineWidth !== ((buffer[2] << 8) | buffer[3]))
        return rgbeError(rgbeFormatError, 'wrong scanline width');

      dataRgba = new Uint8Array(4 * w * h);
      if (!dataRgba.length)
        return rgbeError(rgbeMemoryError, 'unable to allocate buffer space');

      offset = 0; pos = 0; ptrEnd = 4 * scanlineWidth;
      rgbeStart = new Uint8Array(4);
      scanlineBuffer = new Uint8Array(ptrEnd);
      // read in each successive scanline
      while ((numScanlines > 0) && (pos < buffer.byteLength)) {
        if (pos + 4 > buffer.byteLength)
          return rgbeError(rgbeReadError);

        rgbeStart[0] = buffer[pos++];
        rgbeStart[1] = buffer[pos++];
        rgbeStart[2] = buffer[pos++];
        rgbeStart[3] = buffer[pos++];
        if ((rgbeStart[0] !== 2) || (rgbeStart[1] !== 2) || (((rgbeStart[2] << 8) | rgbeStart[3]) !== scanlineWidth))
          return rgbeError(rgbeFormatError, 'bad rgbe scanline format');

        // read each of the four channels for the scanline into the buffer
        // first red, then green, then blue, then exponent
        ptr = 0;
        while ((ptr < ptrEnd) && (pos < buffer.byteLength)) {
          count = buffer[pos++];
          isEncodedRun = count > 128;
          if (isEncodedRun) count -= 128;
          if ((count === 0) || (ptr + count > ptrEnd))
            return rgbeError(rgbeFormatError, 'bad scanline data');

          if (isEncodedRun) {
            // a (encoded) run of the same value
            byteValue = buffer[pos++];
            for (i = 0; i < count; i++)
              scanlineBuffer[ptr++] = byteValue;

            // ptr += count;
          } else {
            // a literal-run
            scanlineBuffer.set(buffer.subarray(pos, pos + count), ptr);
            ptr += count; pos += count;
          }
        }
        // now convert data from buffer into rgba
        // first red, then green, then blue, then exponent (alpha)
        l = scanlineWidth; // scanlineBuffer.byteLength;
        for (i = 0; i < l; i++) {
          off = 0;
          dataRgba[offset] = scanlineBuffer[i + off];
          off += scanlineWidth; // 1;
          dataRgba[offset + 1] = scanlineBuffer[i + off];
          off += scanlineWidth; // 1;
          dataRgba[offset + 2] = scanlineBuffer[i + off];
          off += scanlineWidth; // 1;
          dataRgba[offset + 3] = scanlineBuffer[i + off];
          offset += 4;
        }
        numScanlines--;
      }
      return dataRgba;
    };
    const byteArray = new Uint8Array(buffer);
    byteArray.pos = 0;
    const rgbeHeaderInfo = RGBEReadHeader(byteArray);
    if (RGBE_RETURN_FAILURE !== rgbeHeaderInfo) {
      const w = rgbeHeaderInfo.width;
      const h = rgbeHeaderInfo.height;
      const imageRgbaData = RGBEReadPixelsRLE(byteArray.subarray(byteArray.pos), w, h);
      let data, format, type;
      if (RGBE_RETURN_FAILURE !== imageRgbaData) {
        if (this.type === WEBGL_PIXEL_TYPE.UNSIGNED_BYTE) {
          data = imageRgbaData;
          format = WEBGL_PIXEL_FORMAT.RGBA; // RGBE handled as RGBA in shaders
          type = WEBGL_PIXEL_TYPE.UNSIGNED_BYTE;
        } else if (this.type === WEBGL_PIXEL_TYPE.FLOAT) {
          const RGBEByteToRGBFloat = function(sourceArray, sourceOffset, destArray, destOffset) {
            const e = sourceArray[sourceOffset + 3];
            const scale = Math.pow(2.0, e - 128.0) / 255.0;
            destArray[destOffset + 0] = sourceArray[sourceOffset + 0] * scale;
            destArray[destOffset + 1] = sourceArray[sourceOffset + 1] * scale;
            destArray[destOffset + 2] = sourceArray[sourceOffset + 2] * scale;
          };
          const numElements = (imageRgbaData.length / 4) * 3;
          const floatArray = new Float32Array(numElements);
          for (let j = 0; j < numElements; j++)
            RGBEByteToRGBFloat(imageRgbaData, j * 4, floatArray, j * 3);

          data = floatArray;
          format = WEBGL_PIXEL_FORMAT.RGB;
          type = WEBGL_PIXEL_TYPE.FLOAT;
        } else
          console.error('zen3d.RGBELoader: unsupported type: ', this.type);

        return {
          width: w,
          height: h,
          data: data,
          header: rgbeHeaderInfo.string,
          gamma: rgbeHeaderInfo.gamma,
          exposure: rgbeHeaderInfo.exposure,
          format: format,
          type: type
        };
      }
    }
    return null;
  }
});
export { RGBELoader };
