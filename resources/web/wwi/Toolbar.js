import InformationPanel from './InformationPanel.js';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './fullscreen_handler.js';
import {webots} from './webots.js';

export default class Toolbar {
  constructor(parent, view) {
    this._view = view;

    this.domElement = document.createElement('div');
    this.domElement.id = 'toolBar';

    this.domElement.left = document.createElement('div');
    this.domElement.left.className = 'toolBarLeft';

    if (typeof webots.showQuit === 'undefined' || webots.showQuit) { // enabled by default
      this.domElement.left.appendChild(this._createToolBarButton('quit', 'Quit the simulation'));
      this.quitButton.onclick = () => { this._view.quitSimulation(); };
    }

    if (!webots.showReload) { // disabled by default
      this.domElement.left.appendChild(this._createToolBarButton('reload', 'Reload the simulation'));
      this.reloadButton.addEventListener('click', () => { this.reset(true); });
    }

    const div = document.createElement('div');
    div.className = 'webotsTime';
    const clock = document.createElement('span');
    clock.id = 'webotsClock';
    clock.title = 'Current simulation time';
    clock.innerHTML = webots.parseMillisecondsIntoReadableTime(0);
    div.appendChild(clock);
    const timeout = document.createElement('span');
    timeout.id = 'webotsTimeout';
    timeout.title = 'Simulation time out';
    timeout.innerHTML = webots.parseMillisecondsIntoReadableTime(this._view.timeout >= 0 ? this._view.timeout : 0);
    div.appendChild(document.createElement('br'));
    div.appendChild(timeout);
    this.domElement.left.appendChild(div);

    this.domElement.left.appendChild(this._createToolBarButton('reset', 'Reset the simulation'));
    this.resetButton.addEventListener('click', () => { this.reset(false); });

    this.domElement.left.appendChild(this._createToolBarButton('real_time', 'Run the simulation in real time'));
    this.real_timeButton.onclick = () => { this.realTime(); };

    this.domElement.left.appendChild(this._createToolBarButton('pause', 'Pause the simulation'));
    this.pauseButton.onclick = () => { this.pause(); };
    this.pauseButton.style.display = 'none';

    this.domElement.left.appendChild(this._createToolBarButton('step', 'Perform one simulation step'));
    this.stepButton.onclick = () => { this.step(); };

    if (!webots.showRun) { // disabled by default
      this.domElement.left.appendChild(this._createToolBarButton('run', 'Run the simulation as fast as possible'));
      this.runButton.onclick = () => { this.run(); };
    }

    this._worldSelectionDiv = document.createElement('div');
    this._worldSelectionDiv.id = 'worldSelectionDiv';
    this.domElement.left.appendChild(this._worldSelectionDiv);

    this.domElement.right = document.createElement('div');
    this.domElement.right.className = 'toolBarRight';

    if (typeof webots.infoButton === 'undefined' || webots.infoButton) {
      let webotsView = document.getElementsByTagName('webots-view')[0];
      if (webotsView) {
        let infoButton = this._createToolBarButton('information', 'Information');
        this.domElement.right.appendChild(infoButton);
        this.informationPlaceHolder = document.createElement('div');
        this.informationPlaceHolder.id = 'informationPlaceHolder';
        this.informationPanel = new InformationPanel(this.informationPlaceHolder);
        this.informationPlaceHolder.style.width = '100%';
        this.informationPlaceHolder.style.height = '100%';
        this.informationPlaceHolder.style.position = 'absolute';
        this.informationPlaceHolder.style.top = '0px';
        this.informationPlaceHolder.style.left = '0px';
        this.informationPlaceHolder.style.pointerEvents = 'none';
        this.informationPlaceHolder.style.zIndex = 1;
        webotsView.appendChild(this.informationPlaceHolder);
        infoButton.onclick = () => this._displayInformationWindow();
        window.addEventListener('click', this._closeInfoOnClick);
      }
    }

    if (typeof webots.settingsButton === 'undefined' || webots.settingsButton) {
      this.domElement.right.appendChild(this._createToolBarButton('settings', 'Settings'));
    }

    if (this._view.fullscreenEnabled) {
      this.domElement.right.appendChild(this._createToolBarButton('exit_fullscreen', 'Exit fullscreen'));
      this.exit_fullscreenButton.onclick = () => { exitFullscreen(); };
      this.exit_fullscreenButton.style.display = 'none';
      this.domElement.right.appendChild(this._createToolBarButton('fullscreen', 'Enter fullscreen'));
      this.fullscreenButton.onclick = () => { requestFullscreen(this._view); };
    }

    this.domElement.appendChild(this.domElement.left);
    this.domElement.appendChild(this.domElement.right);
    parent.appendChild(this.domElement);

    this.enableToolBarButtons(false);
    if (this._view.broadcast && this.quitButton) {
      this.quitButton.disabled = true;
      this.quitButton.classList.add('toolBarButtonDisabled');
    }

    document.addEventListener('fullscreenchange', () => { onFullscreenChange(this.fullscreenButton, this.exit_fullscreenButton); });
  }

  _displayInformationWindow() {
    let infoPanel = document.getElementsByClassName('information-panel')[0];
    if (infoPanel) {
      if (infoPanel.style.display === 'block')
        infoPanel.style.display = 'none';
      else
        infoPanel.style.display = 'block';
    }
  }

  _closeInfoOnClick(event) {
    let infoPanel = document.getElementsByClassName('information-panel')[0];
    if (infoPanel && !infoPanel.contains(event.target) && !document.getElementsByClassName('toolbar-information')[0].contains(event.target))
      infoPanel.style.display = 'none';
  }

  reset(reload = false) {
    if (this._view.broadcast)
      return;
    this.time = 0; // reset time to correctly compute the initial deadline
    if (document.getElementById('webotsProgressMessage')) {
      if (reload)
        document.getElementById('webotsProgressMessage').innerHTML = 'Reloading simulation...';
      else
        document.getElementById('webotsProgressMessage').innerHTML = 'Restarting simulation...';
    }
    if (document.getElementById('webotsProgress'))
      document.getElementById('webotsProgress').style.display = 'block';
    this._view.runOnLoad = this.pauseButton.style.display === 'inline';
    this.pause();

    if (this._view.timeout >= 0) {
      this._view.deadline = this._view.timeout;
      document.getElementById('webotsTimeout').innerHTML = webots.parseMillisecondsIntoReadableTime(this._view.timeout);
    } else
      document.getElementById('webotsTimeout').innerHTML = webots.parseMillisecondsIntoReadableTime(0);
    this.enableToolBarButtons(false);
    if (reload)
      this._view.stream.socket.send('reload');
    else
      this._view.stream.socket.send('reset');
  }

  isPaused() {
    return this.real_timeButton.style.display === 'inline';
  }

  pause() {
    if (this._view.broadcast)
      return;
    this._view.stream.socket.send('pause');
  }

  realTime(force) {
    if (this._view.broadcast && !force)
      return;
    this._view.stream.socket.send('real-time:' + this._view.timeout);
    this.pauseButton.style.display = 'inline';
    this.real_timeButton.style.display = 'none';
    if (typeof this.runButton !== 'undefined')
      this.runButton.style.display = 'inline';
  }

  run(force) {
    if (this._view.broadcast && !force)
      return;
    this._view.stream.socket.send('fast:' + this._view.timeout);
    this.pauseButton.style.display = 'inline';
    this.runButton.style.display = 'inline';
    if (typeof this.real_timeButton !== 'undefined')
      this.real_timeButton.style.display = 'none';
  }

  step() {
    if (this._view.broadcast)
      return;
    this.pauseButton.style.display = 'none';
    this.real_timeButton.style.display = 'inline';
    if (typeof this.runButton !== 'undefined')
      this.runButton.style.display = 'inline';
    this._view.stream.socket.send('step');
  }

  enableToolBarButtons(enabled) {
    const buttons = [this.quitButton, this.reloadButton, this.resetButton, this.stepButton, this.real_timeButton, this.runButton, this.pauseButton, this.worldSelect];
    for (let i in buttons) {
      if (buttons[i]) {
        if (enabled && (!this._view.broadcast)) {
          buttons[i].disabled = false;
          buttons[i].classList.remove('toolBarButtonDisabled');
        } else {
          buttons[i].disabled = true;
          buttons[i].classList.add('toolBarButtonDisabled');
        }
      }
    }

    if (typeof this.worldSelect !== 'undefined')
      this.worldSelect.disabled = !enabled;
  }

  _createToolBarButton(name, tooltip) {
    const buttonName = name + 'Button';
    this[buttonName] = document.createElement('button');
    this[buttonName].id = buttonName;
    this[buttonName].className = 'toolBarButton toolbar-' + name;
    this[buttonName].title = tooltip;
    return this[buttonName];
  }

  deleteWorldSelect() {
    this._worldSelectionDiv.removeChild(this.worldSelect);
    this.worldSelect = undefined;
  }

  createWorldSelect() {
    this.worldSelect = document.createElement('select');
    this.worldSelect.id = 'worldSelection';
    this.worldSelect.classList.add('select-css');
    this._worldSelectionDiv.appendChild(this.worldSelect);

    // check if toolbar buttons are disabled
    if (this.real_timeButton && this.real_timeButton.disabled)
      this.worldSelect.disabled = true;
  }

  setMode(mode) {
    const runEnabled = typeof this.runButton !== 'undefined';
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
