'use strict';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './fullscreen_handler.js';
import AnimationSlider from './AnimationSlider.js';
import WbWorld from './nodes/WbWorld.js';
import {changeShadows, changeGtaoLevel, GtaoLevel} from './nodes/wb_preferences.js';

export default class Animation {
  constructor(url, scene, view, gui, loop) {
    this._url = url;
    this._scene = scene;
    this._view = view;
    this._gui = typeof gui === 'undefined' || gui === 'play' ? 'real_time' : 'pause';
    this._loop = typeof loop === 'undefined' ? true : loop;
    this._speed = 1;
    this._view3d = scene.domElement;
    for (let i = 0; i < this._view3d.childNodes.length; i++) {
      const child = this._view3d.childNodes[i];
      if (child.id === 'canvas')
        this._canvas = child;
    }
  };

  init(onReady) {
    this._onReady = onReady;
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', this._url, true);
    xmlhttp.overrideMimeType('application/json');
    xmlhttp.onreadystatechange = () => {
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0))
        this._setup(JSON.parse(xmlhttp.responseText));
    };
    xmlhttp.send();
  }

  pause() {
    this._gui = 'pause';
    if (typeof this._data === 'undefined')
      return;
    if (this._step < 0 || this._step >= this._data.frames.length) {
      this._start = new Date().getTime();
      this._updateAnimationState();
    } else
      this._start = new Date().getTime() - this._data.basicTimeStep * this._step;
  }

  removePlayBar() {
    if (typeof this._view3d === 'undefined')
      return;

    if (typeof this._view.mouseEvents !== 'undefined' && typeof this._view.mouseEvents.hidePlayBar) {
      this._view.mouseEvents.hidePlayBar = undefined;
      if (typeof this._view.mouseEvents.showPlayBar !== 'undefined') {
        this._view.mouseEvents.showPlayBar();
        this._view.mouseEvents.showPlayBar = undefined;
      }
    }

    document.removeEventListener('keydown', this.keydownRef);
    this.keydownRef = undefined;
    document.removeEventListener('sliderchange', this.sliderchangeRef);
    this.sliderchangeRef = undefined;
    document.removeEventListener('fullscreenchange', this.fullscreenRef);
    this.fullscreenRef = undefined;
    document.removeEventListener('mouseup', this.settingsRef);
    this.settingsRef = undefined;

    if (typeof this._timeSlider !== 'undefined') {
      this._timeSlider.shadowRoot.getElementById('range').removeEventListener('mousemove', this.updateFloatingTimeRef);
      this.updateFloatingTimeRef = undefined;
      this._timeSlider.shadowRoot.getElementById('range').removeEventListener('mouseleave', this.hideFloatingTimeRef);
      this.hideFloatingTimeRef = undefined;
      this._timeSlider.removeEventListeners();
      this._timeSlider = undefined;
    }

    if (typeof this._gtaoPane !== 'undefined') {
      this._view3d.removeChild(this._gtaoPane);
      this._gtaoPane = undefined;
    }

    if (typeof this._speedPane !== 'undefined') {
      this._view3d.removeChild(this._speedPane);
      this._speedPane = undefined;
    }

    if (typeof this._settingsPane !== 'undefined') {
      this._view3d.removeChild(this._settingsPane);
      this._settingsPane = undefined;
    }

    if (typeof this.play_bar !== 'undefined') {
      this._view3d.removeChild(this.play_bar);
      this.play_bar = undefined;
    }

    this._fullscreenButton = undefined;
    this._exitFullscreenButton = undefined;
  }
  // private methods
  _setup(data) {
    this._data = data;
    // extract animated node ids: remove empty items and convert to integer
    this._allIds = this._data.ids.split(';').filter(Boolean).map(s => parseInt(s));
    this._labelsIds = typeof this._data.labelsIds === 'undefined' ? [] : this._data.labelsIds.split(';').filter(Boolean).map(s => parseInt(s));

    this._createPlayBar();
    this._createSlider();
    this._createPlayButton();
    this._createTimeIndicator();
    this._createSettings();
    this._createFullscreenButton();

    document.addEventListener('keydown', this.keydownRef = _ => this._keyboardHandler(_));
    // Initialize animation data.
    this._start = new Date().getTime();
    this._step = 0;
    this._previousStep = 0;
    this._updateAnimation();

    // Notify creation completed.
    if (typeof this._onReady === 'function')
      this._onReady();
  }

  _elapsedTime() {
    const end = new Date().getTime();
    return end - this._start;
  }

  _triggerPlayPauseButton() {
    if (this._gui === 'real_time')
      this.pause();
    else {
      this._gui = 'real_time';
      this._start = new Date().getTime() - this._data.basicTimeStep * this._step / this._speed;
      window.requestAnimationFrame(() => this._updateAnimation());
    }
    const action = (this._gui === 'real_time') ? 'pause' : 'play';
    document.getElementById('play-tooltip').innerHTML = 'P' + action.substring(1) + ' (k)';
    document.getElementById('play-button').className = 'player-btn icon-' + action;
  }

  _updateSlider(event) {
    if (event.mouseup) {
      if (this._previousState === 'real_time' && this._gui === 'pause') {
        this._previousState = undefined;
        this._triggerPlayPauseButton();
      } else {
        // Fix gtao "ghost" when modifying manually the position on the slidebar
        for (let x = 0; x < 5; x++)
          this._scene.renderer.render();
      }
      return;
    }

    const value = event.detail;

    if (this._gui === 'real_time') {
      this._previousState = 'real_time';
      this._triggerPlayPauseButton();
    }

    const clampedValued = Math.min(value, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(this._data.frames.length * clampedValued / 100);
    this._start = (new Date().getTime()) - Math.floor(this._data.basicTimeStep * this._step);
    this._updateAnimationState(requestedStep);

    this._timeSlider.setTime(this._formatTime(this._data.frames[requestedStep].time));
  }

  _updateAnimationState(requestedStep = undefined) {
    const automaticMove = typeof requestedStep === 'undefined';
    if (automaticMove) {
      requestedStep = Math.floor(this._elapsedTime() * this._speed / this._data.basicTimeStep);
      if (requestedStep < 0 || requestedStep >= this._data.frames.length) {
        if (this._loop) {
          if (requestedStep > this._data.frames.length) {
            requestedStep = 0;
            this._previousStep = 0;
            this._start = new Date().getTime();
          } else
            return;
        } else if (this._gui === 'real_time') {
          this._triggerPlayPauseButton();
          return;
        } else
          return;
      }
    }
    if (requestedStep !== this._step) {
      this._step = requestedStep;

      const appliedIds = [];
      const appliedLabelsIds = [];

      if (this._data.frames[this._step].hasOwnProperty('poses')) {
        const poses = this._data.frames[this._step].poses;
        for (let p = 0; p < poses.length; p++)
          appliedIds[poses[p].id] = this._scene.applyPose(poses[p], undefined, automaticMove);
      }

      if (this._data.frames[this._step].hasOwnProperty('labels')) {
        const labels = this._data.frames[this._step].labels;
        for (let i = 0; i < labels.length; i++) {
          this._scene.applyLabel(labels[i], this._view);
          appliedLabelsIds.push(labels[i].id);
        }
      }

      // lookback mechanism: search in history
      if (this._step !== this._previousStep + 1) {
        let previousPoseStep;
        if (this._step > this._previousStep)
          // in forward animation check only the changes since last pose
          previousPoseStep = this._previousStep;
        else
          previousPoseStep = 0;
        for (let i in this._allIds) {
          const id = this._allIds[i];
          let appliedFields = appliedIds[id];
          for (let f = this._step - 1; f >= previousPoseStep; f--) {
            if (this._data.frames[f].poses) {
              for (let p = 0; p < this._data.frames[f].poses.length; p++) {
                if (this._data.frames[f].poses[p].id === id)
                  appliedFields = this._scene.applyPose(this._data.frames[f].poses[p], appliedFields, automaticMove);
              }
            }
          }
        }

        for (let id of this._labelsIds) {
          for (let f = this._step - 1; f >= previousPoseStep; f--) {
            if (this._data.frames[f].labels) {
              for (let p = 0; p < this._data.frames[f].labels.length; p++) {
                if (this._data.frames[f].labels[p].id === id) {
                  if (!appliedLabelsIds.includes(id)) {
                    this._scene.applyLabel(this._data.frames[f].labels[p], this._view);
                    appliedLabelsIds.push(id);
                  }
                }
              }
            }
          }
        }
      }

      if (automaticMove)
        this._timeSlider.setValue(100 * this._step / this._data.frames.length);

      this._previousStep = this._step;
      this._view.time = this._data.frames[this._step].time;
      this._currentTime.innerHTML = this._formatTime(this._view.time);
      WbWorld.instance.viewpoint.updateFollowUp(this._view.time, !automaticMove || this.step === 0);
      this._scene.render();
    }
  }

  _updateAnimation() {
    if (this._gui === 'real_time')
      this._updateAnimationState();

    window.requestAnimationFrame(() => this._updateAnimation());
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
    if (typeof this._unusedPrefix === 'undefined') {
      const maxTime = this._data.frames[this._data.frames.length - 1].time;
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

  _showPlayBar() {
    this.play_bar.style.opacity = '1';
    this._canvas.style.cursor = 'auto';
  }

  _hidePlayBar() {
    const isPlaying = document.getElementById('play-button').className === 'player-btn icon-pause';
    const isSelected = this._timeSlider.selected();

    if (!isSelected && isPlaying &&
    this._settingsPane.style.visibility === 'hidden' &&
    this._gtaoPane.style.visibility === 'hidden' &&
    this._speedPane.style.visibility === 'hidden') {
      this.play_bar.style.opacity = '0';
      this._canvas.style.cursor = 'none'; // Warning: it does not always work if chrome dev tools is open
    }
  }

  _onMouseLeave(e) {
    if (e.relatedTarget != null &&
    e.relatedTarget.id !== 'canvas')
      this._view.mouseEvents.onMouseLeave();
  }

  _changeSettingsPaneVisibility(event) {
    if (event.srcElement.id === 'enable-shadows' || event.srcElement.id === 'playback-li' || event.srcElement.id === 'gtao-settings') // avoid to close the settings when modifying the shadows or the other options
      return;

    if (typeof this._settingsPane === 'undefined' || typeof this._gtaoPane === 'undefined' || typeof this._speedPane === 'undefined')
      return;
    if (event.target.id === 'settings-button' && this._settingsPane.style.visibility === 'hidden' && this._gtaoPane.style.visibility === 'hidden' && this._speedPane.style.visibility === 'hidden') {
      this._settingsPane.style.visibility = 'visible';
      document.getElementById('settings-button').style.transform = 'rotate(10deg)';
      const tooltips = document.getElementsByClassName('tooltip');
      for (let i of tooltips)
        i.style.visibility = 'hidden';
    } else if (this._settingsPane.style.visibility === 'visible' || this._gtaoPane.style.visibility === 'visible' || this._speedPane.style.visibility === 'visible') {
      this._settingsPane.style.visibility = 'hidden';
      if (this._gtaoPane.style.visibility === 'hidden' && this._speedPane.style.visibility === 'hidden') {
        document.getElementById('settings-button').style.transform = '';
        const tooltips = document.getElementsByClassName('tooltip');
        for (let i of tooltips)
          i.style.visibility = '';
      }
    }

    this._gtaoPane.style.visibility = 'hidden';
    this._speedPane.style.visibility = 'hidden';
  }

  _resetViewpoint() {
    WbWorld.instance.viewpoint.resetViewpoint();
    this._scene.render(); // render once to reset immediatly the viewpoint even if the animation is on pause
  }

  _changeSpeed(event) {
    this._speed = event.srcElement.id;
    this._speedPane.style.visibility = 'hidden';
    document.getElementById('speed-display').innerHTML = this._speed === '1' ? 'Normal' : this._speed;
    this._settingsPane.style.visibility = 'visible';
    for (let i of document.getElementsByClassName('check-speed')) {
      if (i.id === 'c' + this._speed)
        i.innerHTML = '&check;';
      else
        i.innerHTML = '';
    }
    this._start = new Date().getTime() - this._data.basicTimeStep * this._step / this._speed;
  }

  _openSpeedPane() {
    this._settingsPane.style.visibility = 'hidden';
    this._speedPane.style.visibility = 'visible';
  }

  _closeSpeedPane() {
    this._settingsPane.style.visibility = 'visible';
    this._speedPane.style.visibility = 'hidden';
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
    this._scene.render();
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

  _createPlayBar() {
    this.play_bar = document.createElement('div');
    this.play_bar.id = 'play-bar';
    this._view3d.appendChild(this.play_bar);

    this.play_bar.addEventListener('mouseover', () => this._showPlayBar());
    this.play_bar.addEventListener('mouseleave', _ => this._onMouseLeave(_));

    const leftPane = document.createElement('div');
    leftPane.className = 'left-pane';
    leftPane.id = 'left-pane';

    const rightPane = document.createElement('div');
    rightPane.className = 'right-pane';
    rightPane.id = 'right-pane';

    this.play_bar.appendChild(leftPane);
    this.play_bar.appendChild(rightPane);
    this._view.mouseEvents.hidePlayBar = () => this._hidePlayBar();
    this._view.mouseEvents.showPlayBar = () => this._showPlayBar();
  }

  _createSlider() {
    if (!Animation.sliderDefined) {
      window.customElements.define('animation-slider', AnimationSlider);
      Animation.sliderDefined = true;
    }
    this._timeSlider = document.createElement('animation-slider');
    this._timeSlider.id = 'time-slider';
    document.addEventListener('sliderchange', this.sliderchangeRef = _ => this._updateSlider(_));
    this.play_bar.appendChild(this._timeSlider);
    this._timeSlider.shadowRoot.getElementById('range').addEventListener('mousemove', this.updateFloatingTimeRef = _ => this._updateFloatingTimePosition(_));
    this._timeSlider.shadowRoot.getElementById('range').addEventListener('mouseleave', this.hideFloatingTimeRef = _ => this._hideFloatingTimePosition(_));
  }

  _createPlayButton() {
    const playButton = document.createElement('button');
    const action = (this._gui === 'real_time') ? 'pause' : 'play';
    playButton.className = 'player-btn icon-' + action;
    playButton.id = 'play-button';
    playButton.addEventListener('click', () => this._triggerPlayPauseButton());
    document.getElementById('left-pane').appendChild(playButton);

    const playTooltip = document.createElement('span');
    playTooltip.className = 'tooltip play-tooltip';
    playTooltip.id = 'play-tooltip';
    playTooltip.innerHTML = 'P' + action.substring(1) + ' (k)';
    playButton.appendChild(playTooltip);
  }

  _createTimeIndicator() {
    this._currentTime = document.createElement('span');
    this._currentTime.className = 'current-time';
    this._currentTime.disabled = false;
    this._currentTime.innerHTML = this._formatTime(this._data.frames[0].time);
    document.getElementById('left-pane').appendChild(this._currentTime);

    const timeDivider = document.createElement('span');
    timeDivider.innerHTML = '/';
    timeDivider.className = 'time-divider';
    document.getElementById('left-pane').appendChild(timeDivider);

    const totalTime = document.createElement('span');
    totalTime.className = 'total-time';
    const time = this._formatTime(this._data.frames[this._data.frames.length - 1].time);
    totalTime.innerHTML = time;
    document.getElementById('left-pane').appendChild(totalTime);

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

    this._timeSlider.setOffset(offset);
  }

  _createSettings() {
    this._createSettingsButton();
    this._createSettingsPane();
  }

  _createSettingsButton() {
    const settingsButton = document.createElement('button');
    settingsButton.className = 'player-btn icon-settings';
    settingsButton.id = 'settings-button';
    document.getElementById('right-pane').appendChild(settingsButton);

    const settingsTooltip = document.createElement('span');
    settingsTooltip.className = 'tooltip settings-tooltip';
    settingsTooltip.innerHTML = 'Settings';
    settingsButton.appendChild(settingsTooltip);
  }

  _createSettingsPane() {
    this._settingsPane = document.createElement('div');
    this._settingsPane.className = 'settings-pane';
    this._settingsPane.id = 'settings-pane';
    this._settingsPane.style.visibility = 'hidden';
    document.addEventListener('mouseup', this.settingsRef = _ => this._changeSettingsPaneVisibility(_));
    this._view3d.appendChild(this._settingsPane);

    const settingsList = document.createElement('ul');
    settingsList.id = 'settings-list';
    this._settingsPane.appendChild(settingsList);

    this._createResetViewpoint();
    this._createChangeShadows();
    this._createChangeGtao();
    this._createChangeSpeed();
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
      this._scene.render();
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
    this._view3d.appendChild(this._gtaoPane);

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

  _createChangeSpeed() {
    const playbackLi = document.createElement('li');
    playbackLi.id = 'playback-li';
    document.getElementById('settings-list').appendChild(playbackLi);
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
    this._view3d.appendChild(this._speedPane);

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

  _createFullscreenButton() {
    this._fullscreenButton = document.createElement('button');
    this._fullscreenButton.className = 'player-btn icon-fullscreen';
    this._fullscreenButton.title = 'Full screen (f)';
    this._fullscreenButton.onclick = () => requestFullscreen(this._view);
    document.getElementById('right-pane').appendChild(this._fullscreenButton);

    let fullscreenTooltip = document.createElement('span');
    fullscreenTooltip.className = 'tooltip fullscreen-tooltip';
    fullscreenTooltip.innerHTML = 'Full screen (f)';
    this._fullscreenButton.appendChild(fullscreenTooltip);

    this._exitFullscreenButton = document.createElement('button');
    this._exitFullscreenButton.title = 'Exit full screen (f)';
    this._exitFullscreenButton.className = 'player-btn icon-partscreen';
    this._exitFullscreenButton.style.display = 'none';
    this._exitFullscreenButton.onclick = () => exitFullscreen();
    document.getElementById('right-pane').appendChild(this._exitFullscreenButton);

    fullscreenTooltip = document.createElement('span');
    fullscreenTooltip.className = 'tooltip fullscreen-tooltip';
    fullscreenTooltip.innerHTML = 'Exit full screen (f)';
    this._exitFullscreenButton.appendChild(fullscreenTooltip);

    document.addEventListener('fullscreenchange', this.fullscreenRef = () => onFullscreenChange(this._fullscreenButton, this._exitFullscreenButton));
  }

  _keyboardHandler(e) {
    if (e.code === 'KeyK')
      this._triggerPlayPauseButton();
    else if (e.code === 'KeyF')
      this._fullscreenButton.style.display === 'none' ? exitFullscreen() : requestFullscreen(this._view);
  }

  _updateFloatingTimePosition(e) {
    this._timeSlider.shadowRoot.getElementById('floating-time').style.visibility = 'visible';

    const bounds = this._timeSlider.shadowRoot.getElementById('range').getBoundingClientRect();
    let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
    if (x > 100)
      x = 100;
    else if (x < 0)
      x = 0;

    const clampedValued = Math.min(x, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(this._data.frames.length * clampedValued / 100);
    this._timeSlider.setTime(this._formatTime(this._data.frames[requestedStep].time));

    this._timeSlider.setFloatingTimePosition(e.clientX);
  }

  _hideFloatingTimePosition() {
    this._timeSlider.shadowRoot.getElementById('floating-time').style.visibility = '';
  }
}

Animation.sliderDefined = false;
