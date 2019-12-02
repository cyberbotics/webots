/* global webots */
'use strict';

class Stream { // eslint-disable-line no-unused-vars
  constructor(wsServer, view, onready) {
    this.wsServer = wsServer;
    this.view = view;
    this.onready = onready;
    this.socket = null;
    this.videoStream = null;
  }

  connect() {
    this.socket = new WebSocket(this.wsServer);
    $('#webotsProgressMessage').html('Connecting to Webots instance...');
    this.socket.onopen = (event) => { this.onSocketOpen(event); };
    this.socket.onmessage = (event) => { this.onSocketMessage(event); };
    this.socket.onclose = (event) => { this.onSocketClose(event); };
    this.socket.onerror = (event) => {
      this.view.destroyWorld();
      this.view.onerror('WebSocket error: ' + event.data);
    };
  }

  close() {
    if (this.socket)
      this.socket.close();
    if (this.videoStream)
      this.videoStream.close();
  }

  onSocketOpen(event) {
    var mode = this.view.mode;
    if (mode === 'video')
      mode += ': ' + this.view.video.width + 'x' + this.view.video.height;
    else if (this.view.broadcast)
      mode += ';broadcast';
    this.socket.send(mode);
  }

  onSocketClose(event) {
    this.view.onerror('Disconnected from ' + this.wsServer + ' (' + event.code + ')');
    if ((event.code > 1001 && event.code < 1016) || (event.code === 1001 && this.view.quitting === false)) { // https://tools.ietf.org/html/rfc6455#section-7.4.1
      webots.alert('Streaming server error',
        'Connection closed abnormally.<br>(Error code: ' + event.code + ')<br><br>' +
        'Please reset the simulation by clicking ' +
        '<a href="' + window.location.href + '">here</a>.');
    }
    this.view.destroyWorld();
    if (typeof this.view.onclose === 'function')
      this.view.onclose();
  }

  onSocketMessage(event) {
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
      if (typeof this.view.time !== 'undefined') { // otherwise ignore late updates until the scene loading is completed
        data = data.substring(data.indexOf(':') + 1);
        var frame = JSON.parse(data);
        this.view.time = frame.time;
        $('#webotsClock').html(webots.parseMillisecondsIntoReadableTime(frame.time));
        if (frame.hasOwnProperty('poses')) {
          for (i = 0; i < frame.poses.length; i++)
            this.view.x3dScene.applyPose(frame.poses[i]);
        }
        if (this.view.x3dScene.viewpoint.updateViewpointPosition(null, this.view.time))
          this.view.x3dScene.viewpoint.notifyCameraParametersChanged(false);
        this.view.x3dScene.onSceneUpdate();
      }
    } else if (data.startsWith('node:')) {
      data = data.substring(data.indexOf(':') + 1);
      var parentId = data.split(':')[0];
      data = data.substring(data.indexOf(':') + 1);
      this.view.x3dScene.loadObject(data, parentId);
    } else if (data.startsWith('delete:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      this.view.x3dScene.deleteObject(data);
    } else if (data.startsWith('model:')) {
      $('#webotsProgressMessage').html('Loading 3D scene...');
      $('#webotsProgressPercent').html('');
      this.view.destroyWorld();
      data = data.substring(data.indexOf(':') + 1).trim();
      if (!data) // received an empty model case: just destroy the view
        return;
      this.view.x3dScene.loadObject(data);
    } else if (data.startsWith('world:')) {
      data = data.substring(data.indexOf(':') + 1).trim();
      var currentWorld = data.substring(0, data.indexOf(':')).trim();
      data = data.substring(data.indexOf(':') + 1).trim();
      this.view.updateWorldList(currentWorld, data.split(';'));
    } else if (data.startsWith('video: ')) {
      console.log('Received data = ' + data);
      var list = data.split(' ');
      var url = list[1];
      var streamId = list[2];
      console.log('Received video message on ' + url + ' stream = ' + streamId);
      this.videoStream = new webots.VideoStream(url, this.view.video, document.getElementById('BitrateViewer'), streamId);
      if (typeof this.onready === 'function')
        this.onready();
    } else if (data.startsWith('set controller:')) {
      var slash = data.indexOf('/', 15);
      var dirname = data.substring(15, slash);
      var filename = data.substring(slash + 1, data.indexOf(':', slash + 1));
      if (this.view.editor.dirname === dirname)
        this.view.editor.addFile(filename, data.substring(data.indexOf('\n') + 1)); // remove the first line
      else
        console.log('Warning: ' + filename + ' not in controller directory: ' + dirname + ' != ' + this.view.editor.dirname);
    } else if (data === 'pause' || data === 'paused by client') {
      this.view.toolBar.setMode('pause');
      // Update timeout.
      if (data === 'pause')
        this.view.isAutomaticallyPaused = undefined;
      if (this.view.timeout > 0 && !this.view.isAutomaticallyPaused) {
        this.view.deadline = this.view.timeout;
        if (typeof this.view.time !== 'undefined')
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
      if (typeof this.onready === 'function')
        this.onready();
    } else if (data === 'reset finished') {
      this.view.resetSimulation();
      if (typeof this.onready === 'function')
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
}
