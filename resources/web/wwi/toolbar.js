/* global webots, DialogWindow, HelpWindow, DefaultUrl */

class Toolbar { // eslint-disable-line no-unused-vars
  constructor(parent, view) {
    this.view = view;

    this.domElement = document.createElement('div');
    this.domElement.id = 'toolBar';
    this.domElement.left = document.createElement('div');
    this.domElement.left.className = 'toolBarLeft';

    if (typeof webots.showQuit === 'undefined' || webots.showQuit) { // enabled by default
      this.domElement.left.appendChild(this.createToolBarButton('quit', 'Quit the simulation'));
      this.quitButton.onclick = () => { this.requestQuit(); };
    }

    this.domElement.left.appendChild(this.createToolBarButton('info', 'Open the information window'));
    this.infoButton.onclick = () => { this.toggleInfo(); };

    this.worldSelectionDiv = document.createElement('div');
    this.worldSelectionDiv.id = 'worldSelectionDiv';
    this.domElement.left.appendChild(this.worldSelectionDiv);

    if (webots.showRevert) { // disabled by default
      this.domElement.left.appendChild(this.createToolBarButton('revert', 'Save controllers and revert the simulation'));
      this.revertButton.addEventListener('click', () => { this.reset(true); });
    }

    this.domElement.left.appendChild(this.createToolBarButton('reset', 'Save controllers and reset the simulation'));
    this.resetButton.addEventListener('click', () => { this.reset(false); });

    this.domElement.left.appendChild(this.createToolBarButton('step', 'Perform one simulation step'));
    this.stepButton.onclick = () => { this.step(); };

    this.domElement.left.appendChild(this.createToolBarButton('real_time', 'Run the simulation in real time'));
    this.real_timeButton.onclick = () => { this.realTime(); };

    this.domElement.left.appendChild(this.createToolBarButton('pause', 'Pause the simulation'));
    this.pauseButton.onclick = () => { this.pause(); };
    this.pauseButton.style.display = 'none';

    if (webots.showRun) { // disabled by default
      this.domElement.left.appendChild(this.createToolBarButton('run', 'Run the simulation as fast as possible'));
      this.runButton.onclick = () => { this.run(); };
    }

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
    this.consoleButton.onclick = () => { this.toggleConsole(); };

    this.domElement.right = document.createElement('div');
    this.domElement.right.className = 'toolBarRight';
    this.domElement.right.appendChild(this.createToolBarButton('help', 'Get help on the simulator'));
    this.helpButton.onclick = () => { this.toggleHelp(); };

    if (this.view.fullscreenEnabled) {
      this.domElement.right.appendChild(this.createToolBarButton('exit_fullscreen', 'Exit fullscreen'));
      this.exit_fullscreenButton.onclick = () => { this.exitFullscreen(); };
      this.exit_fullscreenButton.style.display = 'none';
      this.domElement.right.appendChild(this.createToolBarButton('fullscreen', 'Enter fullscreen'));
      this.fullscreenButton.onclick = () => { this.requestFullscreen(); };
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

    document.addEventListener('fullscreenchange', () => { this.onFullscreenChange(); });
    document.addEventListener('webkitfullscreenchange', () => { this.onFullscreenChange(); });
    document.addEventListener('mozfullscreenchange', () => { this.onFullscreenChange(); });
    document.addEventListener('MSFullscreenChange', () => { this.onFullscreenChange(); });
  }

  toggleInfo() {
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
  }

  toggleConsole() {
    this.view.contextMenu.hide();
    if ($('#webotsConsole').is(':visible')) {
      $('#webotsConsole').dialog('close');
      this.consoleButton.classList.remove('toolBarButtonActive');
    } else {
      $('#webotsConsole').dialog('open');
      this.consoleButton.classList.add('toolBarButtonActive');
    }
  }

  toggleHelp() {
    this.view.contextMenu.hide();
    if (!this.view.helpWindow) {
      if (!this.view.broadcast && webots.webotsDocUrl)
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
  }

  exitFullscreen() {
    this.view.contextMenu.hide();
    if (document.exitFullscreen)
      document.exitFullscreen();
    else if (document.msExitFullscreen)
      document.msExitFullscreen();
    else if (document.mozCancelFullScreen)
      document.mozCancelFullScreen();
    else if (document.webkitExitFullscreen)
      document.webkitExitFullscreen();
  }

  requestFullscreen() {
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
  }

  onFullscreenChange(event) {
    var element = document.fullScreenElement || document.mozFullScreenElement || document.webkitFullScreenElement || document.msFullScreenElement || document.webkitCurrentFullScreenElement;
    if (element != null) {
      this.fullscreenButton.style.display = 'none';
      this.exit_fullscreenButton.style.display = 'inline';
    } else {
      this.fullscreenButton.style.display = 'inline';
      this.exit_fullscreenButton.style.display = 'none';
    }
  }

  requestQuit() {
    if (this.view.editor.hasUnsavedChanges()) {
      var text;
      if (this.view.editor.unloggedFileModified || typeof webots.User1Id === 'undefined' || webots.User1Id === '')
        text = 'Your changes to the robot controller will be lost because you are not logged in.';
      else
        text = 'Your unsaved changes to the robot controller will be lost.';
      var quitDialog = document.getElementById('quitDialog');
      if (!quitDialog) {
        quitDialog = document.createElement('div');
        quitDialog.id = 'quitDialog';
        $(quitDialog).html(text);
        this.view.view3D.appendChild(quitDialog);
        $(quitDialog).dialog({
          title: 'Quit the simulation?',
          modal: true,
          resizable: false,
          appendTo: this.view.view3D,
          open: () => { DialogWindow.openDialog(quitDialog); },
          buttons: {
            'Cancel': () => {
              $(quitDialog).dialog('close');
            },
            'Quit': () => {
              $(quitDialog).dialog('close');
              this.view.quitSimulation();
            }
          }
        });
      } else
        $(quitDialog).dialog('open');
      return;
    }
    this.view.quitSimulation();
  }

  reset(revert = false) {
    if (this.view.broadcast)
      return;
    this.time = 0; // reset time to correctly compute the initial deadline
    if (revert)
      $('#webotsProgressMessage').html('Reverting simulation...');
    else
      $('#webotsProgressMessage').html('Restarting simulation...');
    $('#webotsProgress').show();
    this.view.runOnLoad = this.pauseButton.style.display === 'inline';
    this.pause();
    for (let i in this.view.editor.filenames) {
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
  }

  isPaused() {
    return this.real_timeButton.style.display === 'inline';
  }

  pause() {
    if (this.view.broadcast)
      return;
    this.view.contextMenu.hide();
    this.view.stream.socket.send('pause');
  }

  realTime() {
    if (this.view.broadcast)
      return;
    this.view.contextMenu.hide();
    this.view.stream.socket.send('real-time:' + this.view.timeout);
    this.pauseButton.style.display = 'inline';
    this.real_timeButton.style.display = 'none';
    if (typeof this.runButton !== 'undefined')
      this.runButton.style.display = 'inline';
  }

  run() {
    if (this.view.broadcast)
      return;
    this.view.contextMenu.hide();
    this.view.stream.socket.send('fast:' + this.view.timeout);
    this.pauseButton.style.display = 'inline';
    this.real_timeButton.style.display = 'inline';
    this.runButton.style.display = 'none';
  }

  step() {
    if (this.view.broadcast)
      return;
    this.view.contextMenu.hide();
    this.pauseButton.style.display = 'none';
    this.real_timeButton.style.display = 'inline';
    if (typeof this.runButton !== 'undefined')
      this.runButton.style.display = 'inline';
    this.view.stream.socket.send('step');
  }

  enableToolBarButtons(enabled) {
    var buttons = [this.infoButton, this.revertButton, this.resetButton, this.stepButton, this.real_timeButton, this.runButton, this.pauseButton, this.consoleButton, this.worldSelect];
    for (let i in buttons) {
      if (buttons[i]) {
        if (enabled && (!this.view.broadcast || buttons[i] === this.consoleButton)) {
          buttons[i].disabled = false;
          buttons[i].classList.remove('toolBarButtonDisabled');
        } else {
          buttons[i].disabled = true;
          buttons[i].classList.add('toolBarButtonDisabled');
        }
      }
    }
  }

  createToolBarButton(name, tooltip) {
    var buttonName = name + 'Button';
    this[buttonName] = document.createElement('button');
    this[buttonName].id = buttonName;
    this[buttonName].className = 'toolBarButton';
    this[buttonName].title = tooltip;
    this[buttonName].style.backgroundImage = 'url(' + DefaultUrl.wwiImagesUrl() + name + '.png)';
    return this[buttonName];
  }

  setMode(mode) {
    var runEnabled = typeof this.runButton !== 'undefined';
    if (mode === 'pause') {
      this.pauseButton.style.display = 'none';
      this.real_timeButton.style.display = 'inline';
      if (runEnabled)
        this.runButton.style.display = 'inline';
      return;
    }

    this.pauseButton.style.display = 'inline';
    if (runEnabled && (mode === 'run' || mode === 'fast')) {
      this.runButton.style.display = 'none';
      this.real_timeButton.style.display = 'inline';
    } else {
      if (runEnabled)
        this.runButton.style.display = 'inline';
      this.real_timeButton.style.display = 'none';
    }
  }
}
