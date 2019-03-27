/* global TextureManager, webots */
'use strict';

function Stream(url, view, onready) {
  this.url = url;
  this.view = view;
  this.onready = onready;
  this.socket = null;
  this.videoStream = null;
  this.textureManager = new TextureManager();
  this.textureManager.setStreamingMode(true);
};

Stream.prototype = {
  constructor: Stream,

  connect: function() {
    var that = this;
    this.socket = new WebSocket(this.url);
    $('#webotsProgressMessage').html('Connecting to Webots instance...');
    this.socket.onopen = function(event) { that.onSocketOpen(event); };
    this.socket.onmessage = function(event) { that.onSocketMessage(event); };
    this.socket.onclose = function(event) { that.onSocketClose(event); };
    this.socket.onerror = function(event) {
      that.view.destroyWorld();
      that.view.onerror('WebSocket error: ' + event.data);
    };
  },

  close: function() {
    if (this.socket)
      this.socket.close();
    if (this.videoStream)
      this.videoStream.close();
  },

  onSocketOpen: function(event) {
    var mode = this.view.mode;
    if (mode === 'video')
      mode += ': ' + this.view.video.width + 'x' + this.view.video.height;
    else if (this.view.broadcast)
      mode += ';broadcast';
    this.socket.send(mode);
  },

  onSocketClose: function(event) {
    this.view.onerror('Disconnected from ' + this.url + ' (' + event.code + ')');
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001 && this.view.quitting === false)) { // https://tools.ietf.org/html/rfc6455#section-7.4.1
      webots.alert('Streaming server error',
        'Connection closed abnormally.<br>(Error code: ' + event.code + ')<br><br>' +
        'Please reset the simulation by clicking ' +
        '<a href="' + window.location.href + '">here</a>.');
    }
    this.view.destroyWorld();
    if (this.view.onclose)
      this.view.onclose();
  },

  onSocketMessage: function(event) {
    var lines, i;
    var data = event.data;
    if (data.startsWith('robot:') ||
        data.startsWith('stdout:') ||
        data.startsWith('stderr:')) {
      lines = data.split('\n'); // in that case, we support one message per line
      for (i = 0; i < lines.length; i++) {
        var line = lines[i];
        if (line === '') // FIXME: should not happen
          continue;
        if (line.startsWith('stdout:'))
          this.view.console.stdout(line.substring(7));
        else if (line.startsWith('stderr:'))
          this.view.console.stderr(line.substring(7));
        else if (line.startsWith('robot:')) {
          var secondColonIndex = line.indexOf(':', 6);
          var robot = line.substring(6, secondColonIndex);
          var message = line.substring(secondColonIndex + 1);
          this.view.onrobotmessage(robot, message);
        }
      }
    } else if (data.startsWith('application/json:')) {
      if (this.view.time !== undefined) { // otherwise ignore late updates until the scene loading is completed
        data = data.substring(data.indexOf(':') + 1);
        var frame = JSON.parse(data);
        this.view.time = frame.time;
        $('#webotsClock').html(webots.parseMillisecondsIntoReadableTime(frame.time));
        if (frame.hasOwnProperty('poses')) {
          for (i = 0; i < frame.poses.length; i++)
            this.view.x3dSceneManager.applyPose(frame.poses[i]);
        }
        this.view.x3dSceneManager.viewpoint.updateViewpointPosition(null, this.view.time);
        this.view.x3dSceneManager.onSceneUpdateCompleted();
      }
    } else if (data.startsWith('node:')) {
      data = data.substring(data.indexOf(':') + 1);
      var parentId = data.split(':')[0];
      data = data.substring(data.indexOf(':') + 1);
      this.view.x3dSceneManager.loadObject(data, parentId);
    } else if (data.startsWith('delete:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      this.view.x3dSceneManager.deleteObject(data);
    } else if (data.startsWith('model:')) {
      $('#webotsProgressMessage').html('Loading 3D scene...');
      $('#webotsProgressPercent').html('');
      this.view.destroyWorld();
      data = data.substring(data.indexOf(':') + 1).trim();
      if (!data) // received an empty model case: just destroy the view
        return;
      this.view.x3dSceneManager.loadObject(data);
      // TODO
      // this.view.onresize();
    } else if (data.startsWith('world:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      var currentWorld = data.substring(0, data.indexOf(':')).trim();
      data = data.substring(data.indexOf(':') + 1).trim();
      this.view.updateWorldList(currentWorld, data.split(';'));
    } else if (data.startsWith('image')) {
      // extract texture url: the url should only contains escaped ']' characters
      var urlPattern = /[^\\]\]/g; // first occurrence of non-escaped ']'
      var match = urlPattern.exec(data);
      var textureUrlEndIndex = match.index + 1;
      var textureUrl = data.substring(data.indexOf('[') + 1, textureUrlEndIndex).replace(/\\]/g, ']');
      data = data.substring(data.indexOf(':', textureUrlEndIndex) + 1);
      this.textureManager.loadTexture(data, textureUrl);
      // TODO need to replace in ImageTexture and Background as before?
    } else if (data.startsWith('video: ')) {
      console.log('Received data = ' + data);
      var list = data.split(' ');
      var url = list[1];
      var streamId = list[2];
      console.log('Received video message on ' + url + ' stream = ' + streamId);
      this.videoStream = new webots.VideoStream(url, this.view.video, document.getElementById('BitrateViewer'), streamId);
      if (this.onready)
        this.onready();
    } else if (data.startsWith('set controller:')) {
      var slash = data.indexOf('/', 15);
      var dirname = data.substring(15, slash);
      var filename = data.substring(slash + 1, data.indexOf(':', slash + 1));
      if (this.view.editor.dirname === dirname)
        this.view.editor.addFile(filename, data.substring(data.indexOf('\n') + 1)); // remove the first line
      else
        console.log('Warning: ' + filename + ' not in controller directory: ' + dirname + ' != ' + this.view.editor.dirname);
    } else if (data === 'pause') {
      this.view.toolBar.setMode(data);
      // update timeout
      if (this.view.timeout > 0 && !this.view.isAutomaticallyPaused) {
        this.view.deadline = this.view.timeout;
        if (this.view.time !== undefined)
          this.view.deadline += this.view.time;
        $('#webotsTimeout').html(webots.parseMillisecondsIntoReadableTime(this.view.deadline));
      }
    } else if (data === 'real-time' || data === 'run' || data === 'fast') {
      this.view.toolBar.setMode(data);
      if (this.view.timeout >= 0)
        this.view.stream.socket.send('timeout:' + this.view.timeout);
    } else if (data.startsWith('loading:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      var loadingStatus = data.substring(0, data.indexOf(':')).trim();
      data = data.substring(data.indexOf(':') + 1).trim();
      $('#webotsProgressMessage').html('Loading: ' + loadingStatus);
      $('#webotsProgressPercent').html('<progress value="' + data + '" max="100"></progress>');
    } else if (data === 'scene load completed') {
      this.view.time = 0;
      $('#webotsClock').html(webots.parseMillisecondsIntoReadableTime(0));
      if (this.onready)
        this.onready();
    } else if (data === 'reset finished') {
      this.view.resetSimulation();
      if (this.onready)
        this.onready();
    } else if (data.startsWith('label')) {
      var semiColon = data.indexOf(';');
      var id = data.substring(data.indexOf(':'), semiColon);
      var previousSemiColon;
      var labelProperties = []; // ['font', 'color', 'size', 'x', 'y', 'text']
      for (i = 0; i < 5; i++) {
        previousSemiColon = semiColon + 1;
        semiColon = data.indexOf(';', previousSemiColon);
        labelProperties.push(data.substring(previousSemiColon, semiColon));
      }
      this.view.setLabel({
        id: id,
        text: data.substring(semiColon + 1, data.length),
        font: labelProperties[0],
        color: labelProperties[1],
        size: labelProperties[2],
        x: labelProperties[3],
        y: labelProperties[4]
      });
    } else
      console.log('WebSocket error: Unknown message received: "' + data + '"');
  }

  /* TODO check if needed: remove or move in x3d related class
  compareTextureUrl: function(attributeUrl, textureUrl) {
    // url attribute is an array (x3dom.field.MFString)
    if (!attributeUrl || !textureUrl || (typeof attributeUrl !== 'object') || !(attributeUrl instanceof Array))
      return false;

    var length = attributeUrl.length;
    for (var i = 0; i < length; i++) {
      if (attributeUrl[i] === textureUrl)
        return true;
    }
    return false;
  }; */
};
