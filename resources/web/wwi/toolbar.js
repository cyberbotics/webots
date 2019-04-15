/* global webots, DialogWindow, HelpWindow, ResourceManager */

function Toolbar(parent, view) {
  var that = this;
  this.view = view;

  this.domElement = document.createElement('div');
  this.domElement.id = 'toolBar';
  this.domElement.left = document.createElement('div');
  this.domElement.left.className = 'toolBarLeft';

  if (typeof webots.showQuit === 'undefined' || webots.showQuit) { // enabled by default
    this.domElement.left.appendChild(this.createToolBarButton('quit', 'Quit the simulation'));
    this.quitButton.onclick = function() { that.requestQuit(); };
  }

  this.domElement.left.appendChild(this.createToolBarButton('info', 'Open the information window'));
  this.infoButton.onclick = function() { that.toggleInfo(); };

  this.worldSelectionDiv = document.createElement('div');
  this.domElement.left.appendChild(this.worldSelectionDiv);

  if (webots.showRevert) { // disabled by default
    this.domElement.left.appendChild(this.createToolBarButton('revert', 'Save controllers and revert the simulation'));
    this.revertButton.addEventListener('click', function() { that.reset(true); });
  }

  this.domElement.left.appendChild(this.createToolBarButton('reset', 'Save controllers and reset the simulation'));
  this.resetButton.addEventListener('click', function() { that.reset(false); });

  this.domElement.left.appendChild(this.createToolBarButton('step', 'Perform one simulation step'));
  this.stepButton.onclick = function() { that.step(); };

  this.domElement.left.appendChild(this.createToolBarButton('real_time', 'Run the simulation in real time'));
  this.real_timeButton.onclick = function() { that.realTime(); };

  this.domElement.left.appendChild(this.createToolBarButton('pause', 'Pause the simulation'));
  this.pauseButton.onclick = function() { that.pause(); };
  this.pauseButton.style.display = 'none';

  this.domElement.left.appendChild(this.createToolBarButton('fast', 'Run the simulation as fast as possible'));
  this.fastButton.onclick = function() { that.fast(); };

  var div = document.createElement('div');
  div.className = 'webotsTime';
  var clock = document.createElement('span');
  clock.id = 'webotsClock';
  clock.title = 'Current simulation time';
  clock.innerHTML = webots.parseMillisecondsIntoReadableTime(0);
  div.appendChild(clock);
  var timeout = document.createElement('span');
  timeout.id = 'webotsTimeout';
  timeout.title = 'Simulation time out';
  timeout.innerHTML = webots.parseMillisecondsIntoReadableTime(this.view.timeout >= 0 ? this.view.timeout : 0);
  div.appendChild(document.createElement('br'));
  div.appendChild(timeout);
  this.domElement.left.appendChild(div);

  this.domElement.left.appendChild(this.createToolBarButton('console', 'Open the console window'));
  this.consoleButton.onclick = function() { that.toggleConsole(); };

  this.domElement.right = document.createElement('div');
  this.domElement.right.className = 'toolBarRight';
  this.domElement.right.appendChild(this.createToolBarButton('help', 'Get help on the simulator'));
  this.helpButton.onclick = function() { that.toggleHelp(); };

  if (this.view.fullscreenEnabled) {
    this.domElement.right.appendChild(this.createToolBarButton('exit_fullscreen', 'Exit fullscreen'));
    this.exit_fullscreenButton.onclick = function() { that.exitFullscreen(); };
    this.exit_fullscreenButton.style.display = 'none';
    this.domElement.right.appendChild(this.createToolBarButton('fullscreen', 'Enter fullscreen'));
    this.fullscreenButton.onclick = function() { that.requestFullscreen(); };
  }

  this.domElement.appendChild(this.domElement.left);
  this.domElement.appendChild(this.domElement.right);
  parent.appendChild(this.domElement);

  this.enableToolBarButtons(false);
  if (this.view.broadcast && this.quitButton) {
    this.quitButton.disabled = true;
    this.quitButton.classList.add('toolBarButtonDisabled');
    this.view.contextMenu.disableEdit();
  }

  document.addEventListener('fullscreenchange', function() { that.onFullscreenChange(); });
  document.addEventListener('webkitfullscreenchange', function() { that.onFullscreenChange(); });
  document.addEventListener('mozfullscreenchange', function() { that.onFullscreenChange(); });
  document.addEventListener('MSFullscreenChange', function() { that.onFullscreenChange(); });
};

Toolbar.prototype = {
  constructor: Toolbar,

  toggleInfo: function() {
    this.view.contextMenu.hide();
    if (!this.view.infoWindow)
      return;
    if (this.view.infoWindow.isOpen()) {
      this.view.infoWindow.close();
      this.infoButton.classList.remove('toolBarButtonActive');
    } else {
      this.view.infoWindow.open();
      this.infoButton.classList.add('toolBarButtonActive');
    }
  },

  toggleConsole: function() {
    this.view.contextMenu.hide();
    if ($('#webotsConsole').is(':visible')) {
      $('#webotsConsole').dialog('close');
      this.consoleButton.classList.remove('toolBarButtonActive');
    } else {
      $('#webotsConsole').dialog('open');
      this.consoleButton.classList.add('toolBarButtonActive');
    }
  },

  toggleHelp: function() {
    this.view.contextMenu.hide();
    if (!this.view.helpWindow) {
      if (!webots.broadcast && webots.webotsDocUrl)
        var webotsDocUrl = webots.webotsDocUrl;
      this.view.helpWindow = new HelpWindow(this.view.view3D, this.view.mobileDevice, webotsDocUrl);
      this.helpButton.classList.add('toolBarButtonActive');
    } else if ($('#webotsHelp').is(':visible')) {
      $('#webotsHelp').dialog('close');
      this.helpButton.classList.remove('toolBarButtonActive');
    } else {
      $('#webotsHelp').dialog('open');
      this.helpButton.classList.add('toolBarButtonActive');
    }
  },

  exitFullscreen: function() {
    this.view.contextMenu.hide();
    if (document.exitFullscreen)
      document.exitFullscreen();
    else if (document.msExitFullscreen)
      document.msExitFullscreen();
    else if (document.mozCancelFullScreen)
      document.mozCancelFullScreen();
    else if (document.webkitExitFullscreen)
      document.webkitExitFullscreen();
  },

  requestFullscreen: function() {
    this.view.contextMenu.hide();
    var elem = this.view.view3D;
    if (elem.requestFullscreen)
      elem.requestFullscreen();
    else if (elem.msRequestFullscreen)
      elem.msRequestFullscreen();
    else if (elem.mozRequestFullScreen)
      elem.mozRequestFullScreen();
    else if (elem.webkitRequestFullscreen)
      elem.webkitRequestFullscreen();
  },

  onFullscreenChange: function(event) {
    var element = document.fullScreenElement || document.mozFullScreenElement || document.webkitFullScreenElement || document.msFullScreenElement || document.webkitCurrentFullScreenElement;
    if (element != null) {
      this.fullscreenButton.style.display = 'none';
      this.exit_fullscreenButton.style.display = 'inline';
    } else {
      this.fullscreenButton.style.display = 'inline';
      this.exit_fullscreenButton.style.display = 'none';
    }
  },

  requestQuit: function() {
    if (this.view.editor.hasUnsavedChanges()) {
      var text;
      if (this.view.editor.unloggedFileModified || !webots.User1Id)
        text = 'Your changes to the robot controller will be lost because you are not logged in.';
      else
        text = 'Your unsaved changes to the robot controller will be lost.';
      var quitDialog = document.getElementById('quitDialog');
      if (!quitDialog) {
        var that = this;
        quitDialog = document.createElement('div');
        quitDialog.id = 'quitDialog';
        $(quitDialog).html(text);
        this.view.view3D.appendChild(quitDialog);
        $(quitDialog).dialog({
          title: 'Quit the simulation?',
          modal: true,
          resizable: false,
          appendTo: this.view.view3D,
          open: DialogWindow.openDialog,
          buttons: {
            'Cancel': function() {
              $(this).dialog('close');
            },
            'Quit': function() {
              $(this).dialog('close');
              that.view.quitSimulation();
            }
          }
        });
      } else
        $(quitDialog).dialog('open');
      return;
    }
    this.view.quitSimulation();
  },

  reset: function(revert = false) {
    if (webots.broadcast)
      return;
    this.time = 0; // reset time to correctly compute the initial deadline
    if (revert)
      $('#webotsProgressMessage').html('Reverting simulation...');
    else
      $('#webotsProgressMessage').html('Restarting simulation...');
    $('#webotsProgress').show();
    this.runOnLoad = this.pauseButton.style.display === 'inline';
    this.pause();
    for (var i = 0; i < this.view.editor.filenames.length; i++) {
      this.view.editor.save(i);
      if (this.view.editor.needToUploadFiles[i])
        this.view.editor.upload(i);
    }
    this.view.onrobotwindowsdestroy();
    if (this.view.timeout >= 0) {
      this.view.deadline = this.view.timeout;
      $('#webotsTimeout').html(webots.parseMillisecondsIntoReadableTime(this.view.timeout));
    } else
      $('#webotsTimeout').html(webots.parseMillisecondsIntoReadableTime(0));
    this.enableToolBarButtons(false);
    if (revert)
      this.view.stream.socket.send('revert');
    else
      this.view.stream.socket.send('reset');
  },

  isPaused: function() {
    return this.real_timeButton.style.display === 'inline';
  },

  pause: function() {
    if (webots.broadcast)
      return;
    this.view.contextMenu.hide();
    this.view.stream.socket.send('pause');
  },

  realTime: function() {
    if (webots.broadcast)
      return;
    this.view.contextMenu.hide();
    this.view.stream.socket.send('real-time:' + this.view.timeout);
    this.pauseButton.style.display = 'inline';
    this.real_timeButton.style.display = 'none';
    if (typeof this.fastButton !== 'undefined')
      this.fastButton.style.display = 'inline';
  },

  fast: function() {
    if (webots.broadcast)
      return;
    this.view.contextMenu.hide();
    this.view.stream.socket.send('fast:' + this.view.timeout);
    this.pauseButton.style.display = 'inline';
    this.real_timeButton.style.display = 'inline';
    this.fastButton.style.display = 'none';
  },

  step: function() {
    if (webots.broadcast)
      return;
    this.view.contextMenu.hide();
    this.pauseButton.style.display = 'none';
    this.real_timeButton.style.display = 'inline';
    if (typeof this.fastButton !== 'undefined')
      this.fastButton.style.display = 'inline';
    this.view.stream.socket.send('step');
  },

  enableToolBarButtons: function(enabled) {
    var buttons = [this.infoButton, this.revertButton, this.resetButton, this.stepButton, this.real_timeButton, this.fastButton, this.pauseButton, this.consoleButton, this.worldSelect];
    for (var i in buttons) {
      if (buttons[i]) {
        if ((!webots.broadcast || buttons[i] === this.consoleButton) && enabled) {
          buttons[i].disabled = false;
          buttons[i].classList.remove('toolBarButtonDisabled');
        } else {
          buttons[i].disabled = true;
          buttons[i].classList.add('toolBarButtonDisabled');
        }
      }
    }
  },

  createToolBarButton: function(name, tooltip) {
    var buttonName = name + 'Button';
    this[buttonName] = document.createElement('button');
    this[buttonName].id = buttonName;
    this[buttonName].className = 'toolBarButton';
    this[buttonName].title = tooltip;
    var resourceManager = new ResourceManager();
    this[buttonName].style.backgroundImage = 'url(' + resourceManager.wwiUrl + 'images/' + name + '.png)';
    return this[buttonName];
  },

  setMode: function(mode) {
    if (mode === 'pause')
      this.pauseButton.style.display = 'none';
    else
      this.pauseButton.style.display = 'inline';
    this.real_timeButton.style.display = 'inline';
    if (typeof this.fastButton !== 'undefined')
      this.fastButton.style.display = 'inline';
  }
};
