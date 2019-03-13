/* global THREE */
'use strict';

function ShaderManager() { // eslint-disable-line no-unused-vars
  if (!ShaderManager.instance) {
    ShaderManager.instance = this;
    this.vertexShaders = {};
    this.fragmentShaders = {};
  }
  return ShaderManager.instance;
};

ShaderManager.prototype = {
  constructor: ShaderManager,

  load: function(vertexUrl, fragmentUrl, onLoad, onProgress, onError) {
    var that = this;
    var loadFragmentShader = function(vertexText) {
      var fragmentText = that.fragmentShaders[fragmentUrl];
      if (!fragmentText) {
        var fragmentLoader = new THREE.FileLoader(THREE.DefaultLoadingManager);
        fragmentLoader.setResponseType('text');
        fragmentLoader.load(fragmentUrl, function(fragmentText) {
          onLoad(vertexText, fragmentText);
          that.fragmentShaders[fragmentUrl] = fragmentText;
        });
      } else
        onLoad(vertexText, fragmentText);
      if (!that.vertexShaders[vertexUrl])
        that.vertexShaders[vertexUrl] = vertexText;
    };

    var vertexText = this.vertexShaders[vertexUrl];
    if (!vertexText) {
      var vertexLoader = new THREE.FileLoader(THREE.DefaultLoadingManager);
      vertexLoader.setResponseType('text');
      vertexLoader.load(vertexUrl, loadFragmentShader, onProgress, onError);
    } else
      loadFragmentShader(vertexText);
  }
};
