/* exported DefaultUrl */
'use strict';

var DefaultUrl = {
  wwiUrl: function() {
    if (typeof this._wwiUrl === 'undefined') {
      this._wwiUrl = '';
      var scripts = document.getElementsByTagName('script');
      for (let i = scripts.length - 1; i >= 0; i--) {
        var src = scripts[i].src;
        if (src.indexOf('?') > 0)
          src = src.substring(0, src.indexOf('?'));
        if (src.endsWith('setup_viewer.js') || src.endsWith('init_animation.js')) {
          src = src.substring(0, src.lastIndexOf('/')); // remove "webots.js"
          this._wwiUrl = src.substring(0, src.lastIndexOf('/') + 1); // remove "streaming_viewer"
          break;
        }

      }
    }
    return this._wwiUrl;
  },

  wwiImagesUrl: function(name) {
    return this.wwiUrl() + 'wwi/images/';
  },


  wrenImagesUrl: function() {
    let url = this._wwiUrl.substring(0, this._wwiUrl.length - 1);
    url= url.substring(0, url.lastIndexOf('/') + 1); // remove "web"

    return url + "wren/textures/";
  },

  currentScriptUrl: function() {
    // Get the directory path to the currently executing script file
    // for example: https://cyberbotics.com/wwi/8.6/
    var scripts = document.querySelectorAll('script[src]');
    for (let i in scripts) {
      var src = scripts[i].src;
      var index = src.indexOf('?');
      if (index > 0)
        src = src.substring(0, index); // remove query string
      if (!src.endsWith('webots.js') && !src.endsWith('webots.min.js') && !src.endsWith('webots.mjpeg.js'))
        continue;
      index = src.lastIndexOf('/');
      return src.substring(0, index + 1);
    }
    return '';
  }
};

export {DefaultUrl}
