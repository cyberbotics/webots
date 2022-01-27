import Animation from './Animation.js';
import AnimationSlider from './AnimationSlider.js';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './fullscreen_handler.js';
import InformationPanel from './InformationPanel.js';
import {changeShadows, changeGtaoLevel, GtaoLevel} from './nodes/wb_preferences.js';
import WbWorld from './nodes/WbWorld.js';

export default class ToolbarUnifed {
  constructor(view, type, parentNode) {
    this._view = view;
    this.type = type;
    this.parentNode = parentNode;
    this._createToolbar(parentNode);
    if (type === 'animation')
      this.createAnimationToolbar();
    else if (type === 'scene')
      this.createSceneToolbar();
    else if (type === 'streaming')
      this.createStreamingToolbar();
  }

  setType(type) {
    if (this.type !== type) {
      this.type = type;
      // TODO update toolbar accordingly
      return 0; // filler, remove when TODO is done;
    }
  }

  createAnimationToolbar() {
    if (this.type !== 'animation' || typeof this._view === 'undefined' || typeof this._view.animation === 'undefined')
      return;

    this._createSlider();

    // Left part
    this._createPlayButton();
    this._createAnimationTimeIndicator();

    // Right part
    this._createInfoButton();
    this._createSettings();
    this._createFullscreenButtons();
  }

  createSceneToolbar() {
    if (this.type !== 'scene' || typeof this._view === 'undefined')
      return;

    this._createInfoButton();
    this._createRestoreViewpointButton();
    this._createFullscreenButtons();
  }

  createStreamingToolbar() {
    this.toolbar.style.backgroundColor = 'rgba(0, 0, 0, 0.4)';

    // Left part
    this._createQuitButton();
    this._createReloadButton();
    this._createStreamingTimeIndicator();
    this._createResetButton();
    this._createStepButton();
    this._createPlayButton();
    this._createRunButton();
    this._createWorldSelection();

    // Right part
    this._createInfoButton();
    this._createSettings();
    this._createFullscreenButtons();
  }

  _createToolBarButton(name, tooltipText, click) {
    const button = document.createElement('button');
    button.id = name + 'Button';
    button.className = 'toolbar-btn icon-' + name;
    button.addEventListener('click', click);

    const tooltip = document.createElement('span');
    tooltip.className = 'tooltip ' + name + '-tooltip';
    tooltip.id = name + 'Tooltip';
    tooltip.innerHTML = tooltipText;
    button.appendChild(tooltip);

    return button;
  }

  _createToolbar(parentNode) {
    this.toolbar = document.createElement('div');
    this.toolbar.id = 'toolbar';
    this.toolbar.className = 'toolbar';
    parentNode.appendChild(this.toolbar);

    // this.toolbar.addEventListener('mouseover', () => this._showPlayBar());
    // this.toolbar.addEventListener('mouseleave', _ => this._onMouseLeave(_));

    this.toolbarLeft = document.createElement('div');
    this.toolbarLeft.className = 'toolbar-left';
    this.toolbarLeft.id = 'toolbar-left';

    this.toolbarRight = document.createElement('div');
    this.toolbarRight.className = 'toolbar-right';
    this.toolbarRight.id = 'toolbar-right';

    this.toolbar.appendChild(this.toolbarLeft);
    this.toolbar.appendChild(this.toolbarRight);
    // this._view.mouseEvents.hidePlayBar = () => this._hidePlayBar();
    // this._view.mouseEvents.showPlayBar = () => this._showPlayBar();
  }

  hideToolbar() {
    if (typeof this.toolbar !== 'undefined')
      this.toolbar.style.display = 'none';
  }

  showToolbar() {
    if (typeof this.toolbar !== 'undefined')
      this.toolbar.style.display = 'block';
  }

  _createPlayButton() {
    let action;
    if (this.type === 'animation')
      action = (this._view.animation._gui === 'real-time') ? 'pause' : 'play';
    else if (this.type === 'streaming')
      action = 'play';
    this.playButton = this._createToolBarButton('play', 'Play (k)', () => this._triggerPlayPauseButton());
    this.playTooltip = this.playButton.childNodes[0];

    if (action === 'pause') {
      this.playButton.className = 'toolbar-btn icon-pause';
      this.playTooltip.innerHTML = 'Pause (k)';
    }

    this.toolbarLeft.appendChild(this.playButton);
    document.addEventListener('keydown', this.keydownRef = _ => this._playKeyboardHandler(_));
  }

  _triggerPlayPauseButton() {
    let animation = this._view.animation;
    let action;
    if (this.type === 'animation' && typeof animation !== 'undefined') {
      if (animation._gui === 'real-time')
        animation.pause();
      else {
        animation._gui = 'real-time';
        animation._start = new Date().getTime() - animation._data.basicTimeStep * animation._step / animation._speed;
        window.requestAnimationFrame(() => animation._updateAnimation());
      }
      action = (animation._gui === 'real-time') ? 'pause' : 'play';
    } else if (this.type === 'streaming') {
      if (this._view.runOnLoad === 'real-time') {
        this.pause();
        action = 'play';
      } else {
        this.realTime();
        action = 'pause';
      }
    }

    if (typeof this.runButton !== 'undefined') {
      this.runTooltip.innerHTML = 'Run';
      this.runButton.className = 'toolbar-btn icon-run';
    }

    this.playTooltip.innerHTML = 'P' + action.substring(1) + ' (k)';
    this.playButton.className = 'toolbar-btn icon-' + action;
  }

  _playKeyboardHandler(e) {
    if (e.code === 'KeyK')
      this._triggerPlayPauseButton();
  }

  _createInfoButton() {
    this.infoButton = this._createToolBarButton('info', 'Simulation information', () => this._displayInformationWindow());
    this.toolbarRight.appendChild(this.infoButton);
    this._createInformation();
    window.addEventListener('click', _ => this._closeInfoOnClick(_));
  }

  _createInformation() {
    this.informationPanel = new InformationPanel(this.parentNode);

    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.readyForUpdates) {
      this.informationPanel.setTitle(WbWorld.instance.title);
      this.informationPanel.setDescription(WbWorld.instance.description);
    }
  }

  _displayInformationWindow() {
    if (typeof this.informationPanel !== 'undefined') {
      if (this.informationPanel.informationPanel.style.display === 'block')
        this.informationPanel.informationPanel.style.display = 'none';
      else
        this.informationPanel.informationPanel.style.display = 'block';
    }
  }

  _closeInfoOnClick(event) {
    if (typeof this.informationPanel !== 'undefined' && !this.informationPanel.informationPanel.contains(event.target) && !this.infoButton.contains(event.target))
      this.informationPanel.informationPanel.style.display = 'none';
  }

  _createSettings() {
    this.toolbarRight.appendChild(this._createToolBarButton('settings', 'Settings'));

    this._createSettingsPane();
  }

  _createSettingsPane() {
    this._settingsPane = document.createElement('div');
    this._settingsPane.className = 'settings-pane';
    this._settingsPane.id = 'settings-pane';
    this._settingsPane.style.visibility = 'hidden';
    document.addEventListener('mouseup', this.settingsRef = _ => this._changeSettingsPaneVisibility(_));
    this.parentNode.appendChild(this._settingsPane);

    this.settingsList = document.createElement('ul');
    this.settingsList.id = 'settings-list';
    this._settingsPane.appendChild(this.settingsList);

    this._createResetViewpoint();
    this._createChangeShadows();
    this._createChangeGtao();
    if (this.type === 'animation')
      this._createChangeSpeed();
  }

  _changeSettingsPaneVisibility(event) {
    if (event.srcElement.id === 'enable-shadows' || event.srcElement.id === 'playback-li' || event.srcElement.id === 'gtao-settings') // avoid to close the settings when modifying the shadows or the other options
      return;

    if (typeof this._settingsPane === 'undefined' || typeof this._gtaoPane === 'undefined' || typeof this._speedPane === 'undefined')
      return;

    if (event.target.id === 'settingsButton' && this._settingsPane.style.visibility === 'hidden' && this._gtaoPane.style.visibility === 'hidden' && this._speedPane.style.visibility === 'hidden') {
      this._settingsPane.style.visibility = 'visible';
      let settingsButton = document.getElementById('settingsButton');
      if (settingsButton)
        settingsButton.style.transform = 'rotate(10deg)';
      const tooltips = document.getElementsByClassName('tooltip');
      for (let i of tooltips)
        i.style.visibility = 'hidden';
    } else if (this._settingsPane.style.visibility === 'visible' || this._gtaoPane.style.visibility === 'visible' || this._speedPane.style.visibility === 'visible') {
      this._settingsPane.style.visibility = 'hidden';
      if (this._gtaoPane.style.visibility === 'hidden' && this._speedPane.style.visibility === 'hidden') {
        let settingsButton = document.getElementById('settingsButton');
        if (settingsButton)
          settingsButton.style.transform = '';
        const tooltips = document.getElementsByClassName('tooltip');
        for (let i of tooltips)
          i.style.visibility = '';
      }
    }

    this._gtaoPane.style.visibility = 'hidden';
    this._speedPane.style.visibility = 'hidden';
  }

  _createResetViewpoint() {
    const resetViewpoint = document.createElement('li');
    resetViewpoint.onclick = () => this._resetViewpoint();
    this.settingsList.appendChild(resetViewpoint);

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Reset viewpoint';
    resetViewpoint.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    resetViewpoint.appendChild(label);
  }

  _createChangeShadows() {
    const shadowLi = document.createElement('li');
    shadowLi.id = 'enable-shadows';
    this.settingsList.appendChild(shadowLi);

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
    this.settingsList.appendChild(gtaoLi);
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
    this.parentNode.appendChild(this._gtaoPane);

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
    let gtaoLabel = document.getElementById('gtao-display');
    if (gtaoLabel)
      gtaoLabel.innerHTML = event.srcElement.id;
    this._settingsPane.style.visibility = 'visible';
    for (let i of document.getElementsByClassName('check-gtao')) {
      if (i.id === 'c' + event.srcElement.id)
        i.innerHTML = '&check;';
      else
        i.innerHTML = '';
    }
    let animation = this._view.animation;
    if (animation)
      animation._start = new Date().getTime() - animation._data.basicTimeStep * animation._step / animation._speed;
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

  _resetViewpoint() {
    WbWorld.instance.viewpoint.resetViewpoint();
    this._view.x3dScene.render(); // render once to visually reset immediatly the viewpoint.
  }

  _createFullscreenButtons() {
    this._fullscreenButton = this._createToolBarButton('fullscreen', 'Full screen (f)', () => requestFullscreen(this._view));
    this.toolbarRight.appendChild(this._fullscreenButton);

    this._exitFullscreenButton = this._createToolBarButton('partscreen', 'Exit full screen (f)', () => exitFullscreen());
    this.toolbarRight.appendChild(this._exitFullscreenButton);
    this._exitFullscreenButton.style.display = 'none';

    document.addEventListener('fullscreenchange', this.fullscreenRef = () => onFullscreenChange(this._fullscreenButton, this._exitFullscreenButton));
    document.addEventListener('keydown', this.keydownRef = _ => this._fullscrenKeyboardHandler(_));
  }

  _parseMillisecondsIntoReadableTime(milliseconds) {
    const hours = (milliseconds + 0.9) / (1000 * 60 * 60);
    const absoluteHours = Math.floor(hours);
    const h = absoluteHours > 9 ? absoluteHours : '0' + absoluteHours;
    const minutes = (hours - absoluteHours) * 60;
    const absoluteMinutes = Math.floor(minutes);
    const m = absoluteMinutes > 9 ? absoluteMinutes : '0' + absoluteMinutes;
    const seconds = (minutes - absoluteMinutes) * 60;
    const absoluteSeconds = Math.floor(seconds);
    const s = absoluteSeconds > 9 ? absoluteSeconds : '0' + absoluteSeconds;
    let ms = Math.floor((seconds - absoluteSeconds) * 1000);
    if (ms < 10)
      ms = '00' + ms;
    else if (ms < 100)
      ms = '0' + ms;
    return h + ':' + m + ':' + s + ':<small>' + ms + '<small>';
  };

  _fullscrenKeyboardHandler(e) {
    if (e.code === 'KeyF')
      this._fullscreenButton.style.display === 'none' ? exitFullscreen() : requestFullscreen(this._view);
  }

  // Animations functions

  _createSlider() {
    if (!Animation.sliderDefined) {
      window.customElements.define('animation-slider', AnimationSlider);
      Animation.sliderDefined = true;
    }
    this._timeSlider = document.createElement('animation-slider');
    this._timeSlider.id = 'timeSlider';
    document.addEventListener('sliderchange', this.sliderchangeRef = _ => this._updateSlider(_));
    this.toolbar.appendChild(this._timeSlider);
    // this._timeSlider.shadowRoot.getElementById('range').addEventListener('mousemove', this.updateFloatingTimeRef = _ => this._updateFloatingTimePosition(_));
    // this._timeSlider.shadowRoot.getElementById('range').addEventListener('mouseleave', this.hideFloatingTimeRef = _ => this._hideFloatingTimePosition(_));
  }

  _updateSlider(event) {
    let animation = this._view.animation;
    if (event.mouseup && animation) {
      if (animation._previousState === 'real-time' && animation._gui === 'pause') {
        animation._previousState = undefined;
        this._triggerPlayPauseButton();
      }
      return;
    }

    const value = event.detail;

    if (animation._gui === 'real-time') {
      animation._previousState = 'real-time';
      this._triggerPlayPauseButton();
    }

    const clampedValued = Math.min(value, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(animation._data.frames.length * clampedValued / 100);
    animation._start = (new Date().getTime()) - Math.floor(animation._data.basicTimeStep * animation._step);
    animation._updateAnimationState(requestedStep);

    this._timeSlider.setTime(this._formatTime(animation._data.frames[requestedStep].time));
  }

  _createAnimationTimeIndicator() {
    this._currentTime = document.createElement('span');
    this._currentTime.className = 'current-time';
    this._currentTime.id = 'currentTime';
    this._currentTime.innerHTML = this._formatTime(this._view.animation._data.frames[0].time);
    this.toolbarLeft.appendChild(this._currentTime);

    const timeDivider = document.createElement('span');
    timeDivider.innerHTML = '/';
    timeDivider.className = 'time-divider';
    this.toolbarLeft.appendChild(timeDivider);

    const totalTime = document.createElement('span');
    totalTime.className = 'total-time';
    const time = this._formatTime(this._view.animation._data.frames[this._view.animation._data.frames.length - 1].time);
    totalTime.innerHTML = time;
    this.toolbarLeft.appendChild(totalTime);

    let offset;
    switch (time.length) {
      case 20:
        offset = 19;
        break;
      case 22:
        offset = 25;
        break;
      case 23:
        offset = 30;
        break;
      case 25:
        offset = 36;
        break;
      default:
        offset = 0;
    }

    if (typeof this._timeSlider !== 'undefined')
      this._timeSlider.setOffset(offset);
  }

  _formatTime(time) {
    if (typeof this._unusedPrefix === 'undefined') {
      const maxTime = this._view.animation._data.frames[this._view.animation._data.frames.length - 1].time;
      if (maxTime < 60000)
        this._unusedPrefix = 6;
      else if (maxTime < 600000)
        this._unusedPrefix = 4;
      else if (maxTime < 3600000)
        this._unusedPrefix = 3;
      else if (maxTime < 36000000)
        this._unusedPrefix = 1;
    }

    return this._parseMillisecondsIntoReadableTime(time).substring(this._unusedPrefix);
  }

  _createChangeSpeed() {
    const playbackLi = document.createElement('li');
    playbackLi.id = 'playback-li';
    this.settingsList.appendChild(playbackLi);
    playbackLi.onclick = () => this._openSpeedPane();

    let label = document.createElement('span');
    label.innerHTML = 'Playback speed';
    label.className = 'setting-span';
    playbackLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    playbackLi.appendChild(label);

    label = document.createElement('span');
    label.className = 'setting-text';
    label.innerHTML = 'Normal';
    label.id = 'speed-display';
    playbackLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'arrow-right';
    playbackLi.appendChild(label);

    this._createSpeedPane();
  }

  _createSpeedPane() {
    this._speedPane = document.createElement('div');
    this._speedPane.className = 'settings-pane';
    this._speedPane.id = 'speed-pane';
    this._speedPane.style.visibility = 'hidden';

    const speedList = document.createElement('ul');
    this._speedPane.appendChild(speedList);
    this.parentNode.appendChild(this._speedPane);

    let playbackLi = document.createElement('li');
    playbackLi.className = 'first-li';

    let label = document.createElement('div');
    label.className = 'arrow-left';
    playbackLi.appendChild(label);

    label = document.createElement('span');
    label.innerHTML = 'Playback speed';
    label.className = 'setting-span';
    playbackLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    playbackLi.appendChild(label);
    playbackLi.onclick = () => this._closeSpeedPane();
    speedList.appendChild(playbackLi);

    for (let i of ['0.25', '0.5', '0.75', '1', '1.25', '1.5', '1.75', '2']) {
      playbackLi = document.createElement('li');
      playbackLi.id = i;
      label = document.createElement('span');
      if (i === '1')
        label.innerHTML = '&check;';
      label.id = 'c' + i;
      label.className = 'check-speed';
      playbackLi.appendChild(label);
      label = document.createElement('span');
      if (i === '1')
        label.innerHTML = 'Normal';
      else
        label.innerHTML = i;
      label.className = 'setting-span';
      playbackLi.appendChild(label);
      label = document.createElement('div');
      label.className = 'spacer';
      playbackLi.appendChild(label);
      playbackLi.onclick = _ => this._changeSpeed(_);
      speedList.appendChild(playbackLi);
    }
  }

  _changeSpeed(event) {
    let animation = this._view.animation;
    if (animation) {
      animation._speed = event.srcElement.id;
      this._speedPane.style.visibility = 'hidden';
      let speedDisplay = document.getElementById('speed-display');
      if (speedDisplay)
        speedDisplay.innerHTML = animation._speed === '1' ? 'Normal' : animation._speed;
      this._settingsPane.style.visibility = 'visible';
      for (let i of document.getElementsByClassName('check-speed')) {
        if (i.id === 'c' + animation._speed)
          i.innerHTML = '&check;';
        else
          i.innerHTML = '';
      }
      animation._start = new Date().getTime() - animation._data.basicTimeStep * animation._step / animation._speed;
    }
  }

  _openSpeedPane() {
    this._settingsPane.style.visibility = 'hidden';
    this._speedPane.style.visibility = 'visible';
  }

  _closeSpeedPane() {
    this._settingsPane.style.visibility = 'visible';
    this._speedPane.style.visibility = 'hidden';
  }

  // Scene functions

  _createRestoreViewpointButton() {
    this.toolbarRight.appendChild(this._createToolBarButton('reset-scene', 'Reset the Scene', () => this._resetViewpoint()));
  }

  // Streaming functions

  _createQuitButton() {
    this.toolbarLeft.appendChild(this._createToolBarButton('quit', 'Quit the simulation', () => this._view.quitSimulation()));
  }

  _createReloadButton() {
    this.toolbarLeft.appendChild(this._createToolBarButton('reload', 'Reload the simulation', () => { this.reset(true); }));
  }

  reset(reload = false) {
    if (this._view.broadcast)
      return;
    if (document.getElementById('webotsProgressMessage')) {
      if (reload)
        document.getElementById('webotsProgressMessage').innerHTML = 'Reloading simulation...';
      else
        document.getElementById('webotsProgressMessage').innerHTML = 'Restarting simulation...';
    }
    if (document.getElementById('webotsProgress'))
      document.getElementById('webotsProgress').style.display = 'block';

    if (typeof this.pauseButton !== 'undefined' && this.playButton.className === 'toolbar-btn icon-pause')
      this._view.runOnLoad = 'real-time';
    else if (typeof this.runButton !== 'undefined' && this.runButton.className === 'toolbar-btn icon-pause')
      this._view.runOnLoad = 'run';

    let state = this._view.runOnLoad;
    this.pause();
    this._view.runOnLoad = state;

    this.hideToolbar();
    this._view.onready = () => {
      switch (this._view.runOnLoad) {
        case 'real-time':
          this.realTime();
          break;
        case 'fast':
        case 'run':
          this.run();
          break;
      }
      this.showToolbar();
    }
    if (reload)
      this._view.stream.socket.send('reload');
    else
      this._view.stream.socket.send('reset');
  }

  pause() {
    if (this._view.broadcast)
      return;
    this._view.stream.socket.send('pause');
    this._view.runOnLoad = 'pause';
  }

  realTime(force) {
    if (this._view.broadcast)
      return;
    this._view.stream.socket.send('real-time:' + this._view.timeout);
    this._view.runOnLoad = 'real-time';
  }

  _createStreamingTimeIndicator() {
    const clock = document.createElement('span');
    clock.className = 'webots-streaming-time';

    clock.id = 'webotsClock';
    clock.title = 'Current simulation time';
    clock.innerHTML = this._parseMillisecondsIntoReadableTime(0);
    this.toolbarLeft.appendChild(clock);
  }

  _createResetButton() {
    this.toolbarLeft.appendChild(this._createToolBarButton('reset', 'Reset the simulation', () => this.reset(false)));
  }

  _createStepButton() {
    this.toolbarLeft.appendChild(this._createToolBarButton('step', 'Perform one simulation step', () => this.step()));
  }

  step() {
    if (this._view.broadcast)
      return;

    if (typeof this.playButton !== 'undefined') {
      this.playTooltip.innerHTML = 'Play (k)';
      this.playButton.className = 'toolbar-btn icon-play';
    }

    if (typeof this.runButton !== 'undefined') {
      this.runTooltip.innerHTML = 'Run';
      this.runButton.className = 'toolbar-btn icon-run';
    }

    this.pause();

    this._view.stream.socket.send('step');
  }

  _createRunButton() {
    this.runButton = this._createToolBarButton('run', 'Run the simulation as fast as possible', () => this._triggerRunPauseButton());
    this.runTooltip = this.runButton.childNodes[0];
    this.toolbarLeft.appendChild(this.runButton);
  }

  _triggerRunPauseButton() {
    let action;
    if (this.type === 'streaming') {
      if (this._view.runOnLoad === 'run' || this._view.runOnLoad === 'fast') {
        this.pause();
        action = 'run';
      } else {
        this.run();
        action = 'pause';
      }
    }
    if (typeof this.playButton !== 'undefined') {
      this.playTooltip.innerHTML = 'Play (k)';
      this.playButton.className = 'toolbar-btn icon-play';
    }

    this.runTooltip.innerHTML = action.charAt(0).toUpperCase() + action.slice(1);
    this.runButton.className = 'toolbar-btn icon-' + action;
  }

  run() {
    if (this._view.broadcast)
      return;
    this._view.stream.socket.send('fast:' + this._view.timeout);
    this._view.runOnLoad = 'fast';
  }

  _createWorldSelection() {
    this._worldSelectionDiv = document.createElement('div');
    this._worldSelectionDiv.id = 'worldSelectionDiv';
    this.toolbarLeft.appendChild(this._worldSelectionDiv);
    this.createWorldSelect();
  }

  createWorldSelect() {
    this.worldSelect = document.createElement('select');
    this.worldSelect.id = 'worldSelection';
    this.worldSelect.classList.add('select-css');
    this._worldSelectionDiv.appendChild(this.worldSelect);

    // check if toolbar buttons are disabled
    // if (this.real-timeButton && this.real-timeButton.disabled)
    //   this.worldSelect.disabled = true;

    this.worldSelect.innerHTML = this._view.toolBar.innerHTML;
  }
}
