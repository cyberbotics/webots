import InformationPanel from './InformationPanel.js';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './fullscreen_handler.js';
import {webots} from './webots.js';
import WbWorld from './nodes/WbWorld.js';
import {changeShadows, changeGtaoLevel, GtaoLevel} from './nodes/wb_preferences.js';

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
    this.domElement.left.appendChild(div);

    this.domElement.left.appendChild(this._createToolBarButton('reset', 'Reset the simulation'));
    this.resetButton.addEventListener('click', () => { this.reset(false); });

    this.domElement.left.appendChild(this._createToolBarButton('step', 'Perform one simulation step'));
    this.stepButton.onclick = () => { this.step(); };

    this.domElement.left.appendChild(this._createToolBarButton('real_time', 'Run the simulation in real time'));
    this.real_timeButton.onclick = () => { this.realTime(); };

    this.domElement.left.appendChild(this._createToolBarButton('pause', 'Pause the simulation'));
    this.pauseButton.onclick = () => { this.pause(); };
    this.pauseButton.style.display = 'none';

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
        this._setTitleAndDescription();
      }
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

  createSettingsButton() {
    if ((typeof webots.settingsButton === 'undefined' || webots.settingsButton)) {
      if (typeof this.settingsButton === 'undefined') {
        this.settingsButton = this._createToolBarButton('settings', 'Settings');
        this.settingsButton.id = 'settings-button';
        if (typeof this.exit_fullscreenButton !== 'undefined')
          this.domElement.right.insertBefore(this.settingsButton, this.exit_fullscreenButton);
        else
          this.domElement.right.appendChild(this.settingsButton);
        this._createSettingsMenu();

        if (this._view.mode === 'mjpeg')
          this.settingsButton.style.display = 'none';
      } else
        this.settingsButton.style.display = 'inline';
    }
  }

  hideSettingsButton() {
    if (typeof this.settingsButton !== 'undefined')
      this.settingsButton.style.display = 'none';
  }
  _setTitleAndDescription() {
    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.readyForUpdates) {
      this.informationPanel.setTitle(WbWorld.instance.title);
      this.informationPanel.setDescription(WbWorld.instance.description);
    } else
      setTimeout(() => this._setTitleAndDescription(), 100);
  }

  _createSettingsMenu() {
    this._settingsPane = document.createElement('div');
    this._settingsPane.className = 'settings-pane';
    this._settingsPane.id = 'settings-pane';
    this._settingsPane.style.visibility = 'hidden';
    document.addEventListener('mouseup', this.settingsRef = _ => this._changeSettingsPaneVisibility(_));
    this._view.x3dScene.domElement.appendChild(this._settingsPane);

    const settingsList = document.createElement('ul');
    settingsList.id = 'settings-list';
    this._settingsPane.appendChild(settingsList);

    this._createResetViewpoint();
    this._createChangeShadows();
    this._createChangeGtao();
  }

  _createResetViewpoint() {
    const resetViewpoint = document.createElement('li');
    resetViewpoint.onclick = () => this._resetViewpoint();
    document.getElementById('settings-list').appendChild(resetViewpoint);

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Reset viewpoint';
    resetViewpoint.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    resetViewpoint.appendChild(label);
  }

  _resetViewpoint() {
    WbWorld.instance.viewpoint.resetViewpoint();
    this._view.x3dScene.render(); // render once to reset immediatly the viewpoint even if the animation is on pause
  }

  _changeSettingsPaneVisibility(event) {
    if (event.srcElement.id === 'enable-shadows' || event.srcElement.id === 'playback-li' || event.srcElement.id === 'gtao-settings') // avoid to close the settings when modifying the shadows or the other options
      return;
    if (typeof this._settingsPane === 'undefined' || typeof this._gtaoPane === 'undefined')
      return;
    if (event.target.id === 'settings-button' && this._settingsPane.style.visibility === 'hidden' && this._gtaoPane.style.visibility === 'hidden') {
      this._settingsPane.style.visibility = 'visible';
      document.getElementById('settings-button').style.transform = 'rotate(10deg)';
      const tooltips = document.getElementsByClassName('tooltip');
      for (let i of tooltips)
        i.style.visibility = 'hidden';
    } else if (this._settingsPane.style.visibility === 'visible' || this._gtaoPane.style.visibility === 'visible') {
      this._settingsPane.style.visibility = 'hidden';
      if (this._gtaoPane.style.visibility === 'hidden') {
        document.getElementById('settings-button').style.transform = '';
        const tooltips = document.getElementsByClassName('tooltip');
        for (let i of tooltips)
          i.style.visibility = '';
      }
    }

    this._gtaoPane.style.visibility = 'hidden';
  }

  _createChangeShadows() {
    const shadowLi = document.createElement('li');
    shadowLi.id = 'enable-shadows';
    document.getElementById('settings-list').appendChild(shadowLi);

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Shadows';
    shadowLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    shadowLi.appendChild(label);

    const button = document.createElement('label');
    button.className = 'switch';
    shadowLi.appendChild(button);

    label = document.createElement('input');
    label.type = 'checkbox';
    label.checked = true;
    button.appendChild(label);

    label = document.createElement('span');
    label.className = 'slider round';
    button.appendChild(label);

    shadowLi.onclick = _ => {
      button.click();
      changeShadows();
      this._view.x3dScene.render();
    };
  }

  _createChangeGtao() {
    const gtaoLi = document.createElement('li');
    gtaoLi.id = 'gtao-settings';
    document.getElementById('settings-list').appendChild(gtaoLi);
    gtaoLi.onclick = () => this._openGtaoPane();

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Ambient Occlusion';
    gtaoLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    gtaoLi.appendChild(label);

    label = document.createElement('span');
    label.className = 'setting-text';
    label.innerHTML = this._gtaoLevelToText(GtaoLevel);
    label.id = 'gtao-display';
    gtaoLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'arrow-right';
    gtaoLi.appendChild(label);

    this._createGtaoPane();
  }

  _createGtaoPane() {
    this._gtaoPane = document.createElement('div');
    this._gtaoPane.className = 'settings-pane';
    this._gtaoPane.id = 'gtao-pane';
    this._gtaoPane.style.visibility = 'hidden';
    this._view.x3dScene.domElement.appendChild(this._gtaoPane);

    const gtaoList = document.createElement('ul');
    this._gtaoPane.appendChild(gtaoList);

    let gtaoLevelLi = document.createElement('li');
    gtaoLevelLi.className = 'first-li';

    let label = document.createElement('div');
    label.className = 'arrow-left';
    gtaoLevelLi.appendChild(label);

    label = document.createElement('span');
    label.innerHTML = 'Ambient Occlusion Level';
    label.className = 'setting-span';
    gtaoLevelLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    gtaoLevelLi.appendChild(label);
    gtaoLevelLi.onclick = () => this._closeGtaoPane();
    gtaoList.appendChild(gtaoLevelLi);

    for (let i of ['Low', 'Normal', 'High', 'Ultra']) {
      gtaoLevelLi = document.createElement('li');
      gtaoLevelLi.id = i;
      label = document.createElement('span');
      if (this._gtaoLevelToText(GtaoLevel) === i)
        label.innerHTML = '&check;';
      label.id = 'c' + i;
      label.className = 'check-gtao';
      gtaoLevelLi.appendChild(label);
      label = document.createElement('span');
      label.innerHTML = i;
      label.className = 'setting-span';
      gtaoLevelLi.appendChild(label);
      label = document.createElement('div');
      label.className = 'spacer';
      gtaoLevelLi.appendChild(label);
      gtaoLevelLi.onclick = _ => this._changeGtao(_);
      gtaoList.appendChild(gtaoLevelLi);
    }
  }

  _changeGtao(event) {
    changeGtaoLevel(this._textToGtaoLevel(event.srcElement.id));
    this._gtaoPane.style.visibility = 'hidden';
    document.getElementById('gtao-display').innerHTML = event.srcElement.id;
    this._settingsPane.style.visibility = 'visible';
    for (let i of document.getElementsByClassName('check-gtao')) {
      if (i.id === 'c' + event.srcElement.id)
        i.innerHTML = '&check;';
      else
        i.innerHTML = '';
    }
    this._start = new Date().getTime() - this._data.basicTimeStep * this._step / this._speed;
    this._view.x3dScene.render();
  }

  _openGtaoPane() {
    this._settingsPane.style.visibility = 'hidden';
    this._gtaoPane.style.visibility = 'visible';
  }

  _closeGtaoPane() {
    this._settingsPane.style.visibility = 'visible';
    this._gtaoPane.style.visibility = 'hidden';
  }

  _gtaoLevelToText(number) {
    const pairs = {
      1: 'Low',
      2: 'Medium',
      3: 'High',
      4: 'Ultra'
    };
    return (number in pairs) ? pairs[number] : '';
  }

  _textToGtaoLevel(text) {
    const pairs = {
      'Low': 1,
      'Medium': 2,
      'High': 3,
      'Ultra': 4
    };
    return (text in pairs) ? pairs[text] : 4;
  }

  _displayInformationWindow() {
    let infoPanel = document.getElementsByClassName('information-panel')[0];
    if (infoPanel) {
      if (infoPanel.style.display === 'block')
        infoPanel.style.display = 'none';
      else
        infoPanel.style.display = 'block';
    } else {
      let webotsView = document.getElementsByTagName('webots-view')[0];
      if (webotsView) {
        webotsView.appendChild(this.informationPlaceHolder);
        document.getElementsByClassName('information-panel')[0].style.display = 'block';
      }
    }

    // if mjpeg streaming do not display the description of the world (not present)
    let tabsBar = document.getElementsByClassName('info-tabs-bar')[0];
    if (this._view.mode === 'mjpeg') {
      if (tabsBar)
        tabsBar.style.display = 'none';
      this.informationPanel.switchTab(1);
    } else {
      if (tabsBar)
        tabsBar.style.display = 'block';
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
