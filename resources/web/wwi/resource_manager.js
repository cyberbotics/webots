'use strict';

function ResourceManager() {
  if (!ResourceManager.instance) {
    ResourceManager.instance = this;
    this.wwiUrl = '';
    var scripts = document.getElementsByTagName('script');
    for (var i = scripts.length - 1; i >= 0; i--) {
      var src = scripts[i].src;
      if (src.endsWith('webots.js')) {
        this.wwiUrl = src.substr(0, src.lastIndexOf('/') + 1); // remove "webots.js"
        break;
      }
    }
  }
  return ResourceManager.instance;
};

ResourceManager.prototype = {
  constructor: ResourceManager,

  getImageUrl: function(name) {
    return 'url(' + this.wwiUrl + name + '.png)';
  },

  // Get the directory path to the currently executing script file
  // for example: https://cyberbotics.com/wwi/8.6/
  currentScriptPath: function() {
    var scripts = document.querySelectorAll('script[src]');
    for (var i = 0; i < scripts.length; i++) {
      var src = scripts[i].src;
      var index = src.indexOf('?');
      if (index > 0)
        src = src.substring(0, index); // remove query string
      if (!src.endsWith('webots.js'))
        continue;
      index = src.lastIndexOf('/');
      return src.substring(0, index + 1);
    }
    return '';
  }
};
