'use strict';

class ResourceManager { // eslint-disable-line no-unused-vars
  constructor() {
    if (!ResourceManager.instance) {
      var scripts = document.getElementsByTagName('script');
      this.wwiUrl = scripts[scripts.length - 1].src;
      this.wwiUrl = this.wwiUrl.substr(0, this.wwiUrl.lastIndexOf('/') + 1); // remove "webots.js"
      ResourceManager.instance = this;
    }
    return ResourceManager.instance;
  }

  getImageUrl(name) {
    return 'url(' + this.wwiUrl + name + '.png)';
  }

  // get the directory path to the currently executing script file
  // for example: https://cyberbotics.com/wwi/8.6/
  currentScriptPath() {
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
}
