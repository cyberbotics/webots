// Requires: webots.js adapter.js, janus.nojquery.js

webots.VideoStream = function(serverUrl, videoElement, bitrateElement, streamId) {
  this.videoElement = videoElement;
  this.bitrateElement = bitrateElement;
  this.janus = null;
  this.started = false;
  this.selectedStream = null;
  this.streaming = null;
  this.videoServerUrl = serverUrl + '/janus';
  this.streamId = streamId;
  this._init(this.videoServerUrl);
};

webots.VideoStream.prototype.close = function() {
  this._stopStream();
};

webots.VideoStream.prototype._updateStreamsList = function() {
  const that = this;
  that.streaming.send({'message': {'request': 'list'},
    success: function(result) {
      if (result === null || typeof result === 'undefined') {
        console.error('Got no response to our query for available streams');
        return;
      }
      if (typeof result['list'] !== 'undefined' && result['list'] !== null) {
        const list = result['list'];
        Janus.log('Got a list of available streams');
        Janus.debug(list);
        for (let mp in list) {
          Janus.debug('  >> [' + list[mp]['id'] + '] ' + list[mp]['description'] + ' (' + list[mp]['type'] + ')');
          if (list[mp]['id'] === that.streamId)
            that.selectedStream = list[mp]['id'];
        }
        that._startStream();
      }
    }});
};

webots.VideoStream.prototype._startStream = function() {
  const that = this;
  Janus.log('Selected video id #' + that.selectedStream);
  if (typeof that.selectedStream === 'undefined' || that.selectedStream === null) {
    console.error('Stream not found');
    return;
  }
  const body = {'request': 'watch', id: parseInt(that.selectedStream)};
  that.streaming.send({'message': body});
};

webots.VideoStream.prototype._init = function(serverUrl) {
  const that = this;
  Janus.init({debug: 'all',
    callback: function() {
      if (that.started)
        return;
      that.started = true;
      if (!Janus.isWebrtcSupported()) {
        console.error('No WebRTC support... ');
        return;
      }
      that.janus = new Janus({
        server: serverUrl,
        success: function() {
          that.janus.attach({
            plugin: 'janus.plugin.streaming',
            success: function(pluginHandle) {
              that.streaming = pluginHandle;
              Janus.log('Plugin attached! (' + that.streaming.getPlugin() + ', id=' + that.streaming.getId() + ')');
              that._updateStreamsList();
            },
            error: function(error) {
              Janus.error('Error attaching plugin...', error);
              console.error('Error attaching plugin...' + error);
            },
            onmessage: function(msg, jsep) {
              Janus.debug('Got a message.');
              Janus.debug(JSON.stringify(msg));
              const result = msg['result'];
              if (result !== null && typeof result !== 'undefined') {
                if (typeof result['status'] !== 'undefined' && result['status'] !== null) {
                  const status = result['status'];
                  if (status === 'starting')
                    Janus.debug('Starting, please wait...');
                  else if (status === 'started')
                    Janus.debug('Started');
                  else if (status === 'stopped')
                    that._stopStream();
                }
              } else if (typeof msg['error'] !== 'undefined' && msg['error'] !== null) {
                console.error(msg['error']);
                that._stopStream();
                return;
              }
              if (typeof jsep !== 'undefined' && jsep !== null) {
                Janus.debug('Handling SDP as well...');
                Janus.debug(jsep);
                that.streaming.createAnswer({
                  jsep: jsep,
                  media: { audioSend: false, videoSend: false }, // We want only to receive, not send audio and video
                  success: function(jsep) {
                    Janus.debug('Got SDP!');
                    Janus.debug(jsep);
                    that.streaming.send({'message': {'request': 'start'}, 'jsep': jsep});
                  },
                  error: function(error) {
                    Janus.error('WebRTC error:', error);
                    console.error('WebRTC error... ' + JSON.stringify(error));
                  }
                });
              }
            },
            onremotestream: function(stream) {
              Janus.debug(' ::: Got a remote stream :::');
              Janus.debug(JSON.stringify(stream));
              attachMediaStream(that.videoElement, stream);
              if (that.bitrateElement) { // Show bitrate
                if (webrtcDetectedBrowser === 'chrome' || webrtcDetectedBrowser === 'firefox') {
                  that.bitrateSum = 0.0;
                  that.bitrateCount = 0.0;
                  bitrateTimer = setInterval(function() {
                    const bitrateTxt = that.streaming.getBitrate();
                    if (bitrateTxt.indexOf('Invalid') === -1) {
                      const bitrate = parseFloat(bitrateTxt);
                      if (bitrate > 0.0) {
                        that.bitrateSum += bitrate;
                        that.bitrateCount++;
                      }
                      that.bitrateElement.innerHTML = bitrateTxt + '   average = ' + Math.round(that.bitrateSum / that.bitrateCount);
                    }
                  }, 1000);
                }
              }
            },
            oncleanup: function() {
              Janus.log(' ::: Got a cleanup notification :::');
            }
          });
        },
        error: function(error) {
          Janus.error(error);
          console.error(error, function() {
            window.location.reload();
          });
        },
        destroyed: function() {
          window.location.reload();
        }
      });
    }});
};

webots.VideoStream.prototype._stopStream = function() {
  this.streaming.send({'message': {'request': 'stop'}});
  this.streaming.hangup();
};
