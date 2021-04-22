'use strict';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './fullscreen_handler.js';
import AnimationSlider from './AnimationSlider.js';
import WbWorld from './nodes/WbWorld.js';
import {changeShadows, changeGtaoLevel, GtaoLevel} from './nodes/wb_preferences.js';

export default class Animation {
  constructor(url, scene, view, gui, loop) {
    this.url = url;
    this.scene = scene;
    this.view = view;
    this.gui = typeof gui === 'undefined' || gui === 'play' ? 'real_time' : 'pause';
    this.loop = typeof loop === 'undefined' ? true : loop;
    this.speed = 1;
  };

  init(onReady) {
    this.onReady = onReady;
    fetch(this.url)
      .then(response => response.json())
      .then(data => this._setup(data))
      .catch((error) => console.log(error));
  }

  // Return the animation status: play or pause.
  // This should be used to store the current animation status and restore it later when calling webots.View.setAnimation().
  // This is for example used in robotbenchmark.net benchmark page.
  getStatus() {
    return this.gui === 'real_time' ? 'play' : 'pause';
  }

  // private methods
  _setup(data) {
    this.data = data;

    this._createPlayBar();
    this._createSlider();
    this._createPlayButton();
    this._createTimeIndicator();
    this._createSettings();
    this._createFullscreenButton();

    document.addEventListener('keydown', _ => this._keyboardHandler(_));
    // Initialize animation data.
    this.start = new Date().getTime();
    this.step = 0;
    this.previousStep = 0;
    this._updateAnimation();

    // Notify creation completed.
    if (typeof this.onReady === 'function')
      this.onReady();
  }

  _elapsedTime() {
    const end = new Date().getTime();
    return end - this.start;
  }

  _triggerPlayPauseButton() {
    if (this.gui === 'real_time') {
      this.gui = 'pause';
      if (this.step < 0 || this.step >= this.data.frames.length) {
        this.start = new Date().getTime();
        this._updateAnimationState();
      } else
        this.start = new Date().getTime() - this.data.basicTimeStep * this.step;
    } else {
      this.gui = 'real_time';
      this.start = new Date().getTime() - this.data.basicTimeStep * this.step / this.speed;
      window.requestAnimationFrame(() => { this._updateAnimation(); });
    }
    const action = (this.gui === 'real_time') ? 'pause' : 'play';
    document.getElementById('playTooltip').innerHTML = 'P' + action.substring(1) + ' (k)';
    document.getElementById('playButton').className = 'player-btn icon-' + action;
  }

  _updateSlider(event) {
    if (event.mouseup) {
      if (this.previousState === 'real_time' && this.gui === 'pause') {
        this.previousState = undefined;
        this._triggerPlayPauseButton();
      }
      return;
    }

    let value = event.detail;

    if (this.gui === 'real_time') {
      this.previousState = 'real_time';
      this._triggerPlayPauseButton();
    }

    const clampedValued = Math.min(value, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(this.data.frames.length * clampedValued / 100);
    this.start = (new Date().getTime()) - Math.floor(this.data.basicTimeStep * this.step);
    this._updateAnimationState(requestedStep);

    document.getElementById('timeSlider').setTime(this._formatTime(this.data.frames[requestedStep].time));
  }

  _updateAnimationState(requestedStep = undefined) {
    const automaticMove = typeof requestedStep === 'undefined';
    if (automaticMove) {
      requestedStep = Math.floor(this._elapsedTime() * this.speed / this.data.basicTimeStep);
      if (requestedStep < 0 || requestedStep >= this.data.frames.length) {
        if (this.loop) {
          if (requestedStep > this.data.frames.length) {
            requestedStep = 0;
            this.previousStep = 0;
            this.start = new Date().getTime();
          } else
            return;
        } else if (this.gui === 'real_time') {
          this._triggerPlayPauseButton();
          return;
        } else
          return;
      }
    }
    if (requestedStep !== this.step) {
      this.step = requestedStep;

      const appliedIds = [];
      if (this.data.frames[this.step].hasOwnProperty('poses')) {
        const poses = this.data.frames[this.step].poses;
        for (let p = 0; p < poses.length; p++)
          appliedIds[poses[p].id] = this.scene.applyPose(poses[p], this.data.frames[this.step].time);
      }

      if (automaticMove)
        document.getElementById('timeSlider').setValue(100 * this.step / this.data.frames.length);

      this.previousStep = this.step;
      this.view.time = this.data.frames[this.step].time;
      this.currentTime.innerHTML = this._formatTime(this.view.time);
      this.scene.render();
    }
  }

  _updateAnimation() {
    if (this.gui === 'real_time')
      this._updateAnimationState();

    window.requestAnimationFrame(() => { this._updateAnimation(); });
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

  _formatTime(time) {
    if (typeof this.unusedPrefix === 'undefined') {
      let maxTime = this.data.frames[this.data.frames.length - 1].time;
      if (maxTime < 60000)
        this.unusedPrefix = 6;
      else if (maxTime < 600000)
        this.unusedPrefix = 4;
      else if (maxTime < 3600000)
        this.unusedPrefix = 3;
      else if (maxTime < 36000000)
        this.unusedPrefix = 1;
    }

    return this._parseMillisecondsIntoReadableTime(time).substring(this.unusedPrefix);
  }

  _showPlayBar() {
    document.getElementById('playBar').style.opacity = '1';
    document.getElementById('canvas').style.cursor = 'auto';
  }

  _hidePlayBar() {
    let isPlaying = document.getElementById('playButton').className === 'player-btn icon-pause';
    let isSelected = document.getElementById('timeSlider').selected();

    if (!isSelected && isPlaying &&
    document.getElementById('settingsPane').style.visibility === 'hidden' &&
    document.getElementById('gtaoPane').style.visibility === 'hidden' &&
    document.getElementById('speedPane').style.visibility === 'hidden') {
      document.getElementById('playBar').style.opacity = '0';
      document.getElementById('canvas').style.cursor = 'none'; // Warning: it does not always work if chrome dev tools is open
    }
  }

  _onMouseLeave(e) {
    if (e.relatedTarget != null &&
    e.relatedTarget.id !== 'canvas')
      this.view.mouseEvents.onMouseLeave();
  }

  _changeSettingsPaneVisibility(event) {
    if (event.srcElement.id === 'enableShadows' || event.srcElement.id === 'playBackLi' || event.srcElement.id === 'gtaoSettings') // avoid to close the settings when modifying the shadows or the other options
      return;

    if (event.target.id === 'settingsButton' && document.getElementById('settingsPane').style.visibility === 'hidden' && document.getElementById('gtaoPane').style.visibility === 'hidden' && document.getElementById('speedPane').style.visibility === 'hidden') {
      document.getElementById('settingsPane').style.visibility = 'visible';
      document.getElementById('settingsButton').style.transform = 'rotate(10deg)';
      let tooltips = document.getElementsByClassName('tooltip');
      for (let i of tooltips)
        i.style.visibility = 'hidden';
    } else if (document.getElementById('settingsPane').style.visibility === 'visible' || document.getElementById('gtaoPane').style.visibility === 'visible' || document.getElementById('speedPane').style.visibility === 'visible') {
      document.getElementById('settingsPane').style.visibility = 'hidden';
      if (document.getElementById('gtaoPane').style.visibility === 'hidden' && document.getElementById('speedPane').style.visibility === 'hidden') {
        document.getElementById('settingsButton').style.transform = '';
        let tooltips = document.getElementsByClassName('tooltip');
        for (let i of tooltips)
          i.style.visibility = '';
      }
    }

    document.getElementById('gtaoPane').style.visibility = 'hidden';
    document.getElementById('speedPane').style.visibility = 'hidden';
  }

  _resetViewpoint() {
    WbWorld.instance.viewpoint.resetViewpoint();
    this.scene.render(); // render once to reset immediatly the viewpoint even if the animation is on pause
  }

  _changeSpeed(event) {
    this.speed = event.srcElement.id;
    document.getElementById('speedPane').style.visibility = 'hidden';
    document.getElementById('speedDisplay').innerHTML = this.speed === '1' ? 'Normal' : this.speed;
    document.getElementById('settingsPane').style.visibility = 'visible';
    for (let i of document.getElementsByClassName('check-speed')) {
      if (i.id === 'c' + this.speed)
        i.innerHTML = '&check;';
      else
        i.innerHTML = '';
    }
    this.start = new Date().getTime() - this.data.basicTimeStep * this.step / this.speed;
  }

  _openSpeedPane() {
    document.getElementById('settingsPane').style.visibility = 'hidden';
    document.getElementById('speedPane').style.visibility = 'visible';
  }

  _closeSpeedPane() {
    document.getElementById('settingsPane').style.visibility = 'visible';
    document.getElementById('speedPane').style.visibility = 'hidden';
  }

  _changeGtao(event) {
    changeGtaoLevel(this.textToGtaoLevel(event.srcElement.id));
    document.getElementById('gtaoPane').style.visibility = 'hidden';
    document.getElementById('gtaoDisplay').innerHTML = event.srcElement.id;
    document.getElementById('settingsPane').style.visibility = 'visible';
    for (let i of document.getElementsByClassName('check-gtao')) {
      if (i.id === 'c' + event.srcElement.id)
        i.innerHTML = '&check;';
      else
        i.innerHTML = '';
    }
    this.start = new Date().getTime() - this.data.basicTimeStep * this.step / this.speed;
    this.scene.render();
  }

  _openGtaoPane() {
    document.getElementById('settingsPane').style.visibility = 'hidden';
    document.getElementById('gtaoPane').style.visibility = 'visible';
  }

  _closeGtaoPane() {
    document.getElementById('settingsPane').style.visibility = 'visible';
    document.getElementById('gtaoPane').style.visibility = 'hidden';
  }

  _gtaoLevelToText(number) {
    let string = '';
    switch (number) {
      case 1:
        string = 'Low';
        break;
      case 2:
        string = 'Medium';
        break;
      case 3:
        string = 'High';
        break;
      case 4:
        string = 'Ultra';
        break;
    }

    return string;
  }

  textToGtaoLevel(text) {
    let level = 4;
    switch (text) {
      case 'Low':
        level = 1;
        break;
      case 'Medium':
        level = 2;
        break;
      case 'High':
        level = 3;
        break;
      case 'Ultra':
        level = 4;
        break;
    }

    return level;
  }

  _createPlayBar() {
    const div = document.createElement('div');
    div.id = 'playBar';
    this.view.view3D.appendChild(div);

    div.addEventListener('mouseover', () => this._showPlayBar());
    div.addEventListener('mouseleave', _ => this._onMouseLeave(_));

    let leftPane = document.createElement('div');
    leftPane.className = 'left';
    leftPane.id = 'leftPane';

    let rightPane = document.createElement('div');
    rightPane.className = 'right';
    rightPane.id = 'rightPane';

    document.getElementById('playBar').appendChild(leftPane);
    document.getElementById('playBar').appendChild(rightPane);
    this.view.mouseEvents.hidePlayBar = this._hidePlayBar;
    this.view.mouseEvents.showPlayBar = this._showPlayBar;
  }

  _createSlider() {
    window.customElements.define('my-slider', AnimationSlider);
    let timeSlider = document.createElement('my-slider');
    timeSlider.id = 'timeSlider';
    let those = this;
    document.addEventListener('slider_input', (e) => { those._updateSlider(e); });
    document.getElementById('playBar').appendChild(timeSlider);
    document.querySelector('my-slider').shadowRoot.getElementById('range').addEventListener('mousemove', _ => this._updateFloatingTimePosition(_));
    document.querySelector('my-slider').shadowRoot.getElementById('range').addEventListener('mouseleave', _ => this._hideFloatingTimePosition(_));
  }

  _createPlayButton() {
    let playButton = document.createElement('button');
    const action = (this.gui === 'real_time') ? 'pause' : 'play';
    playButton.className = 'player-btn icon-' + action;
    playButton.id = 'playButton';
    playButton.addEventListener('click', () => this._triggerPlayPauseButton());
    document.getElementById('leftPane').appendChild(playButton);

    let playTooltip = document.createElement('span');
    playTooltip.className = 'tooltip play-tooltip';
    playTooltip.id = 'playTooltip';
    playTooltip.innerHTML = 'P' + action.substring(1) + ' (k)';
    playButton.appendChild(playTooltip);
  }

  _createTimeIndicator() {
    this.currentTime = document.createElement('span');
    this.currentTime.className = 'current-time';
    this.currentTime.disabled = false;
    this.currentTime.innerHTML = this._formatTime(this.data.frames[0].time);
    document.getElementById('leftPane').appendChild(this.currentTime);

    let timeDivider = document.createElement('span');
    timeDivider.innerHTML = '\\';
    timeDivider.className = 'time-divider';
    document.getElementById('leftPane').appendChild(timeDivider);

    let totalTime = document.createElement('span');
    totalTime.className = 'total-time';
    let time = this._formatTime(this.data.frames[this.data.frames.length - 1].time);
    totalTime.innerHTML = time;
    document.getElementById('leftPane').appendChild(totalTime);

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

    document.getElementById('timeSlider').setOffset(offset);
  }

  _createSettings() {
    this._createSettingsButton();
    this._createSettingsPane();
  }

  _createSettingsButton() {
    let settingsButton = document.createElement('button');
    settingsButton.className = 'player-btn icon-settings';
    settingsButton.id = 'settingsButton';
    document.getElementById('rightPane').appendChild(settingsButton);

    let settingsTooltip = document.createElement('span');
    settingsTooltip.className = 'tooltip settings-tooltip';
    settingsTooltip.innerHTML = 'Settings';
    settingsButton.appendChild(settingsTooltip);
  }

  _createSettingsPane() {
    let settingsPane = document.createElement('div');
    settingsPane.className = 'settings-pane';
    settingsPane.id = 'settingsPane';
    settingsPane.style.visibility = 'hidden';
    document.addEventListener('mouseup', _ => this._changeSettingsPaneVisibility(_));
    document.getElementById('view3d').appendChild(settingsPane);

    let settingsList = document.createElement('ul');
    settingsList.id = 'settingsList';
    document.getElementById('settingsPane').appendChild(settingsList);

    this._createResetViewpoint();
    this._createChangeShadows();
    this._createChangeGtao();
    this._createChangeSpeed();
  }

  _createResetViewpoint() {
    let resetViewpoint = document.createElement('li');
    resetViewpoint.id = 'resetViewpoint';
    resetViewpoint.onclick = () => this._resetViewpoint();
    document.getElementById('settingsList').appendChild(resetViewpoint);

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Reset viewpoint';
    resetViewpoint.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    resetViewpoint.appendChild(label);
  }

  _createChangeShadows() {
    let shadowLi = document.createElement('li');
    shadowLi.id = 'enableShadows';
    document.getElementById('settingsList').appendChild(shadowLi);

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Shadows';
    shadowLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    shadowLi.appendChild(label);

    let button = document.createElement('label');
    button.className = 'switch';
    shadowLi.appendChild(button);

    label = document.createElement('input');
    label.type = 'checkbox';
    label.checked = true;
    button.appendChild(label);

    label = document.createElement('span');
    label.className = 'slider round';
    label.id = 'shadowSlider';
    button.appendChild(label);

    shadowLi.onclick = _ => {
      button.click();
      changeShadows();
      this.scene.render();
    };
  }

  _createChangeGtao() {
    let gtaoLi = document.createElement('li');
    gtaoLi.id = 'gtaoSettings';
    document.getElementById('settingsList').appendChild(gtaoLi);
    gtaoLi.onclick = () => this._openGtaoPane();

    let label = document.createElement('span');
    label.className = 'setting-span';
    label.innerHTML = 'Ambiant Occlusion';
    gtaoLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    gtaoLi.appendChild(label);

    label = document.createElement('span');
    label.className = 'setting-text';
    label.innerHTML = this._gtaoLevelToText(GtaoLevel);
    label.id = 'gtaoDisplay';
    gtaoLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'arrow-right';
    gtaoLi.appendChild(label);

    this._createGtaoPane();
  }

  _createGtaoPane() {
    let gtaoPane = document.createElement('div');
    gtaoPane.className = 'settings-pane';
    gtaoPane.id = 'gtaoPane';
    gtaoPane.style.visibility = 'hidden';
    document.getElementById('view3d').appendChild(gtaoPane);

    let gtaoList = document.createElement('ul');
    gtaoList.id = 'gtaoList';
    gtaoPane.appendChild(gtaoList);

    let gtaoLevelLi = document.createElement('li');
    gtaoLevelLi.id = 'gtaoTitle';
    gtaoLevelLi.className = 'first-li';

    let label = document.createElement('div');
    label.className = 'arrow-left';
    gtaoLevelLi.appendChild(label);

    label = document.createElement('span');
    label.innerHTML = 'Ambiant Occlusion Level';
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

  _createChangeSpeed() {
    let playBackLi = document.createElement('li');
    playBackLi.id = 'playBackLi';
    document.getElementById('settingsList').appendChild(playBackLi);
    playBackLi.onclick = () => this._openSpeedPane();

    let label = document.createElement('span');
    label.innerHTML = 'Playback speed';
    label.className = 'setting-span';
    playBackLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    playBackLi.appendChild(label);

    label = document.createElement('span');
    label.className = 'setting-text';
    label.innerHTML = 'Normal';
    label.id = 'speedDisplay';
    playBackLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'arrow-right';
    playBackLi.appendChild(label);

    this._createSpeedPane();
  }

  _createSpeedPane() {
    let speedPane = document.createElement('div');
    speedPane.className = 'settings-pane';
    speedPane.id = 'speedPane';
    speedPane.style.visibility = 'hidden';

    let speedList = document.createElement('ul');
    speedList.id = 'speedList';
    speedPane.appendChild(speedList);
    document.getElementById('view3d').appendChild(speedPane);

    let playBackLi = document.createElement('li');
    playBackLi.id = 'speedTitle';
    playBackLi.className = 'first-li';

    let label = document.createElement('div');
    label.className = 'arrow-left';
    playBackLi.appendChild(label);

    label = document.createElement('span');
    label.innerHTML = 'Playback speed';
    label.className = 'setting-span';
    playBackLi.appendChild(label);

    label = document.createElement('div');
    label.className = 'spacer';
    playBackLi.appendChild(label);
    playBackLi.onclick = () => this._closeSpeedPane();
    speedList.appendChild(playBackLi);

    for (let i of ['0.25', '0.5', '0.75', '1', '1.25', '1.5', '1.75', '2']) {
      playBackLi = document.createElement('li');
      playBackLi.id = i;
      label = document.createElement('span');
      if (i === '1')
        label.innerHTML = '&check;';
      label.id = 'c' + i;
      label.className = 'check-speed';
      playBackLi.appendChild(label);
      label = document.createElement('span');
      if (i === '1')
        label.innerHTML = 'Normal';
      else
        label.innerHTML = i;
      label.className = 'setting-span';
      playBackLi.appendChild(label);
      label = document.createElement('div');
      label.className = 'spacer';
      playBackLi.appendChild(label);
      playBackLi.onclick = _ => this._changeSpeed(_);
      speedList.appendChild(playBackLi);
    }
  }

  _createFullscreenButton() {
    this.fullscreenButton = document.createElement('button');
    this.fullscreenButton.className = 'player-btn icon-fullscreen';
    this.fullscreenButton.title = 'Full screen (f)';
    this.fullscreenButton.onclick = () => { requestFullscreen(this.view); };
    document.getElementById('rightPane').appendChild(this.fullscreenButton);

    let fullscreenTooltip = document.createElement('span');
    fullscreenTooltip.className = 'tooltip fullscreen-tooltip';
    fullscreenTooltip.innerHTML = 'Full screen (f)';
    this.fullscreenButton.appendChild(fullscreenTooltip);

    let exitFullscreenButton = document.createElement('button');
    exitFullscreenButton.title = 'Exit full screen (f)';
    exitFullscreenButton.className = 'player-btn icon-partscreen';
    exitFullscreenButton.style.display = 'none';
    exitFullscreenButton.onclick = () => { exitFullscreen(); };
    document.getElementById('rightPane').appendChild(exitFullscreenButton);

    fullscreenTooltip = document.createElement('span');
    fullscreenTooltip.className = 'tooltip fullscreen-tooltip';
    fullscreenTooltip.innerHTML = 'Exit full screen (f)';
    exitFullscreenButton.appendChild(fullscreenTooltip);

    document.addEventListener('fullscreenchange', () => { onFullscreenChange(this.fullscreenButton, exitFullscreenButton); });
    document.addEventListener('webkitfullscreenchange', () => { onFullscreenChange(this.fullscreenButton, exitFullscreenButton); });
    document.addEventListener('mozfullscreenchange', () => { onFullscreenChange(this.fullscreenButton, exitFullscreenButton); });
    document.addEventListener('MSFullscreenChange', () => { onFullscreenChange(this.fullscreenButton, exitFullscreenButton); });
  }

  _keyboardHandler(e) {
    if (e.code === 'KeyK')
      this._triggerPlayPauseButton();
    else if (e.code === 'KeyF')
      this.fullscreenButton.style.display === 'none' ? exitFullscreen() : requestFullscreen(this.view);
  }

  _updateFloatingTimePosition(e) {
    document.querySelector('my-slider').shadowRoot.getElementById('floating-time').style.visibility = 'visible';

    let bounds = document.querySelector('my-slider').shadowRoot.getElementById('range').getBoundingClientRect();
    let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
    if (x > 100)
      x = 100;
    else if (x < 0)
      x = 0;

    const clampedValued = Math.min(x, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(this.data.frames.length * clampedValued / 100);
    document.getElementById('timeSlider').setTime(this._formatTime(this.data.frames[requestedStep].time));

    document.getElementById('timeSlider').setFloatingTimePosition(e.clientX);
  }

  _hideFloatingTimePosition() {
    document.querySelector('my-slider').shadowRoot.getElementById('floating-time').style.visibility = '';
  }
}
