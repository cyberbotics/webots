'use strict';

class DefaultUrl {
  static wwiUrl() {
    if (typeof this._wwiUrl === 'undefined') {
      this._wwiUrl = '';
      const scripts = document.getElementsByTagName('script');
      for (let i = scripts.length - 1; i >= 0; i--) {
        let src = scripts[i].src;
        if (src.indexOf('?') > 0)
          src = src.substring(0, src.indexOf('?'));
        if (src.endsWith('setup_viewer.js') || src.endsWith('enum.js') || src.endsWith('wrenjs.js')) {
          src = src.substring(0, src.lastIndexOf('/')); // remove "wrenjs.js"
          this._wwiUrl = src.substring(0, src.lastIndexOf('/') + 1); // remove "streaming_viewer"
          break;
        }
      }
    }
    return this._wwiUrl;
  }

  static wwiImagesUrl() {
    return this.wwiUrl() + 'wwi/images/';
  }

  static wrenImagesUrl() {
    let url = this._wwiUrl.substring(0, this._wwiUrl.length - 1);
    url = url.substring(0, url.lastIndexOf('/') + 1); // remove "web"

    return url + 'wren/textures/';
  }
};

export {DefaultUrl};
