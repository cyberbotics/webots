import Animation from './Animation.js';
import AnimationSlider from './AnimationSlider.js';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './fullscreen_handler.js';
import InformationPanel from './InformationPanel.js';
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
      this.createSimulationToolbar();
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

  createSimulationToolbar() {
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
    button.id = 'name';
    button.className = 'toolbar-btn icon-' + name;
    button.addEventListener('click', click);

    const tooltip = document.createElement('span');
    tooltip.className = 'tooltip ' + name + '-tooltip';
    tooltip.id = name + '-tooltip';
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

  _createPlayButton() {
    let action;
    if (this.type === 'animation')
      action = (this._view.animation._gui === 'real_time') ? 'pause' : 'play';
    else if (this.type === 'streaming')
      action = 'play';
    this.toolbarLeft.appendChild(this._createToolBarButton(action, 'P' + action.substring(1) + ' (k)', this._triggerPlayPauseButton));
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

    // this._createSettingsPane();
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
    this._timeSlider.id = 'time-slider';
    document.addEventListener('sliderchange', this.sliderchangeRef = _ => this._updateSlider(_));
    this.toolbar.appendChild(this._timeSlider);
    // this._timeSlider.shadowRoot.getElementById('range').addEventListener('mousemove', this.updateFloatingTimeRef = _ => this._updateFloatingTimePosition(_));
    // this._timeSlider.shadowRoot.getElementById('range').addEventListener('mouseleave', this.hideFloatingTimeRef = _ => this._hideFloatingTimePosition(_));
  }

  _createAnimationTimeIndicator() {
    this._currentTime = document.createElement('span');
    this._currentTime.className = 'current-time';
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

  // Scene functions

  _createRestoreViewpointButton() {
    this.toolbarRight.appendChild(this._createToolBarButton('reset-scene', 'Reset the Scene', () => {
      WbWorld.instance.viewpoint.resetViewpoint();
      this._view.x3dScene.render(); // render once to visually reset immediatly the viewpoint.
    }));
  }

  // Streaming functions

  _createQuitButton() {
    this.toolbarLeft.appendChild(this._createToolBarButton('quit', 'Quit the simulation', () => this._view.quitSimulation()));
  }

  _createReloadButton() {
    this.toolbarLeft.appendChild(this._createToolBarButton('reload', 'Reload the simulation', () => { this.reset(true); }));
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
    this.toolbarLeft.appendChild(this._createToolBarButton('reset', 'Reset the simulation', () => { this.reset(false); }));
  }

  _createStepButton() {
    this.toolbarLeft.appendChild(this._createToolBarButton('step', 'Perform one simulation step', () => { this.step(); }));
  }

  _createRunButton() {
    this.toolbarLeft.appendChild(this._createToolBarButton('run', 'Run the simulation as fast as possible', () => { this.run(); }));
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
    // if (this.real_timeButton && this.real_timeButton.disabled)
    //   this.worldSelect.disabled = true;

  this.worldSelect.innerHTML = this._view.toolBar.innerHTML;
    console.log(this._view.toolBar);
  }
}
