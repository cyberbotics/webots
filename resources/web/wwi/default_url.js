/* exported DefaultUrl */
'use strict';

var DefaultUrl = {
  wwiUrl: function() {
    if (typeof _wwiUrl === 'undefined') {
      this._wwiUrl = '';
      var scripts = document.getElementsByTagName('script');
      for (let i = scripts.length - 1; i >= 0; i--) {
        var src = scripts[i].src;
        if (src.endsWith('webots.js') || src.endsWith('webots.min.js')) {
          this._wwiUrl = src.substr(0, src.lastIndexOf('/') + 1); // remove "webots.js"
          break;
        }
      }
    }
    return this._wwiUrl;
  },

  getImageUrl: function(name) {
    return 'url(' + this.wwiUrl() + 'images/' + name + '.png)';
  },

  currentScriptUrl: function() {
    // Get the directory path to the currently executing script file
    // for example: https://cyberbotics.com/wwi/8.6/
    var scripts = document.querySelectorAll('script[src]');
    for (let i = 0; i < scripts.length; i++) {
      var src = scripts[i].src;
      var index = src.indexOf('?');
      if (index > 0)
        src = src.substring(0, index); // remove query string
      if (!src.endsWith('webots.js') && !src.endsWith('webots.min.js'))
        continue;
      index = src.lastIndexOf('/');
      return src.substring(0, index + 1);
    }
    return '';
  }
};
