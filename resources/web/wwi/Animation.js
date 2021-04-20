'use strict';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './Fullscreen_handler.js';
import Animation_slider from './Animation_slider.js';
import WbWorld from './nodes/WbWorld.js'
import {changeShadows, changeGtaoLevel, GtaoLevel} from './nodes/WbPreferences.js';

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
    // extract animated node ids: remove empty items and convert to integer
    this.allIds = this.data.ids.split(';').filter(Boolean).map(s => parseInt(s));

    const div = document.createElement('div');
    div.id = 'playBar';
    this.view.view3D.appendChild(div);

    div.addEventListener('mouseover', this._showPlayBar);
    div.addEventListener('mouseout', this._hidePlayBar);

    let leftPane = document.createElement('div');
    leftPane.className = 'left';
    leftPane.id = 'leftPane';

    let rightPane = document.createElement('div');
    rightPane.className = 'right';
    rightPane.id = 'rightPane';

    window.customElements.define('my-slider', Animation_slider);
    this.timeSlider = document.createElement('my-slider');
    this.timeSlider.id = 'timeSlider';
    let those = this;
    document.addEventListener('slider_input', (e) => { those._updateSlider(e); });

    this.playButton = document.createElement('button');
    const action = (this.gui === 'real_time') ? 'pause' : 'play';
    this.playButton.className = 'player-btn icon-' + action;
    this.playButton.addEventListener('click', () => this._triggerPlayPauseButton());

    let settingsButton = document.createElement('button');
    settingsButton.className = 'player-btn icon-settings';
    settingsButton.id = 'settingsButton';

    let settingsPane = document.createElement('div');
    settingsPane.className = 'jsm-settings';
    settingsPane.id = 'settingsPane';
    settingsPane.style.visibility = 'hidden';
    document.addEventListener('mouseup', _ => this._changeSettingsPaneVisibility(_));
    document.getElementById('view3d').appendChild(settingsPane);

    let settingsList = document.createElement('ul');
    settingsList.id = 'settingsList';
    document.getElementById('settingsPane').appendChild(settingsList);

    let resetViewpoint = document.createElement('li');
    resetViewpoint.id = 'resetViewpoint';
    let label = document.createElement('span');
    label.className = 'settingTitle';
    label.innerHTML = 'Reset viewpoint';
    resetViewpoint.appendChild(label);
    label = document.createElement('div');
    label.className = 'spacer';
    resetViewpoint.appendChild(label);
    resetViewpoint.onclick = () => this._resetViewpoint();
    document.getElementById('settingsList').appendChild(resetViewpoint);

    let shadowLi = document.createElement('li');
    shadowLi.id = 'resetViewpoint';
    label = document.createElement('span');
    label.className = 'settingTitle';
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
    button.appendChild(label);
    button.onclick = _ => changeShadows(event.target.checked);
    document.getElementById('settingsList').appendChild(shadowLi);

    let gtaoLi = document.createElement('li');
    gtaoLi.id = 'gtaoSettings';
    label = document.createElement('span');
    label.className = 'settingTitle';
    label.innerHTML = 'Ambiant Occlusion';
    gtaoLi.appendChild(label);
    label = document.createElement('div');
    label.className = 'spacer';
    gtaoLi.appendChild(label);
    label = document.createElement('span');
    label.className = 'textSetting';
    label.innerHTML = this._gtaoLevelToText(GtaoLevel);
    label.id = 'gtaoDisplay';
    gtaoLi.appendChild(label);
    label = document.createElement('div');
    label.className = 'arrowRight';
    gtaoLi.appendChild(label);
    document.getElementById('settingsList').appendChild(gtaoLi);
    gtaoLi.onclick = () => this._openGtaoPane();

    let gtaoPane = document.createElement('div');
    gtaoPane.className = 'jsm-settings';
    gtaoPane.id = 'gtaoPane';
    gtaoPane.style.visibility = 'hidden';

    let gtaoList = document.createElement('ul');
    gtaoList.id = 'gtaoList';
    gtaoPane.appendChild(gtaoList);
    document.getElementById('view3d').appendChild(gtaoPane);

    let gtaoLevelLi = document.createElement('li');
    gtaoLevelLi.id = 'gtaoTitle';
    gtaoLevelLi.className = 'firstLi';
    label = document.createElement('div');
    label.className = 'arrowLeft';
    gtaoLevelLi.appendChild(label);
    label = document.createElement('span');
    label.innerHTML = 'Ambiant Occlusion Level';
    label.className = 'settingTitle';
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
      label.className = 'checkGtao';
      gtaoLevelLi.appendChild(label);
      label = document.createElement('span');
      label.innerHTML = i;
      label.className = 'settingTitle';
      gtaoLevelLi.appendChild(label);
      label = document.createElement('div');
      label.className = 'spacer';
      gtaoLevelLi.appendChild(label);
      gtaoLevelLi.onclick = _ => this._changeGtao(_);
      gtaoList.appendChild(gtaoLevelLi);
    }

    let playBackLi = document.createElement('li');
    playBackLi.id = 'playBackLi';
    label = document.createElement('span');
    label.innerHTML = 'Playback speed';
    label.className = 'settingTitle';
    playBackLi.appendChild(label);
    label = document.createElement('div');
    label.className = 'spacer';
    playBackLi.appendChild(label);
    label = document.createElement('span');
    label.className = 'textSetting';
    label.innerHTML = 'Normal';
    label.id = 'speedDisplay';
    playBackLi.appendChild(label);
    label = document.createElement('div');
    label.className = 'arrowRight';
    playBackLi.appendChild(label);
    document.getElementById('settingsList').appendChild(playBackLi);
    playBackLi.onclick = () => this._openSpeedPane();

    let speedPane = document.createElement('div');
    speedPane.className = 'jsm-settings';
    speedPane.id = 'speedPane';
    speedPane.style.visibility = 'hidden';

    let speedList = document.createElement('ul');
    speedList.id = 'speedList';
    speedPane.appendChild(speedList);
    document.getElementById('view3d').appendChild(speedPane);

    playBackLi = document.createElement('li');
    playBackLi.id = 'speedTitle';
    playBackLi.className = 'firstLi';
    label = document.createElement('div');
    label.className = 'arrowLeft';
    playBackLi.appendChild(label);
    label = document.createElement('span');
    label.innerHTML = 'Playback speed';
    label.className = 'settingTitle';
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
      label.className = 'checkSpeed';
      playBackLi.appendChild(label);
      label = document.createElement('span');
      if (i === '1')
        label.innerHTML = 'Normal';

      label.innerHTML = i;
      label.className = 'settingTitle';
      playBackLi.appendChild(label);
      label = document.createElement('div');
      label.className = 'spacer';
      playBackLi.appendChild(label);
      playBackLi.onclick = _ => this._changeSpeed(_);
      speedList.appendChild(playBackLi);
    }

    let fullscreenButton = document.createElement('button');
    fullscreenButton.className = 'player-btn icon-fullscreen';
    fullscreenButton.onclick = () => { requestFullscreen(this.view); };

    let exitFullscreenButton = document.createElement('button');
    exitFullscreenButton.className = 'player-btn icon-partscreen';
    exitFullscreenButton.style.display = 'none';
    exitFullscreenButton.onclick = () => { exitFullscreen(); };

    document.addEventListener('fullscreenchange', () => { onFullscreenChange(fullscreenButton, exitFullscreenButton); });
    document.addEventListener('webkitfullscreenchange', () => { onFullscreenChange(fullscreenButton, exitFullscreenButton); });
    document.addEventListener('mozfullscreenchange', () => { onFullscreenChange(fullscreenButton, exitFullscreenButton); });
    document.addEventListener('MSFullscreenChange', () => { onFullscreenChange(fullscreenButton, exitFullscreenButton); });

    this.currentTime = document.createElement('span');
    this.currentTime.className = 'current-time';
    this.currentTime.disabled = false;
    this.currentTime.innerHTML = this._formatTime(this.data.frames[0].time);

    let timeDivider = document.createElement('span');
    timeDivider.innerHTML = '\\';
    timeDivider.className = 'time-divider';

    let totalTime = document.createElement('span');
    totalTime.className = 'total-time';
    totalTime.innerHTML = this._formatTime(this.data.frames[this.data.frames.length - 1].time);

    document.getElementById('playBar').appendChild(this.timeSlider);
    document.getElementById('playBar').appendChild(leftPane);
    document.getElementById('playBar').appendChild(rightPane);
    document.getElementById('leftPane').appendChild(this.playButton);
    document.getElementById('leftPane').appendChild(this.currentTime);
    document.getElementById('leftPane').appendChild(timeDivider);
    document.getElementById('leftPane').appendChild(totalTime);
    document.getElementById('rightPane').appendChild(settingsButton);
    document.getElementById('rightPane').appendChild(fullscreenButton);
    document.getElementById('rightPane').appendChild(exitFullscreenButton);
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
    this.playButton.className = 'player-btn icon-' + action;
  }

  _updateSlider(event) {
    if (event.mouseup) {
      if (this.previousState === 'real_time' && this.gui === 'pause') {
        this.previousState = undefined;
        this._triggerPlayPauseButton();
        this.isMoving = false;
      }
      return;
    }

    let value = event.detail;
    if (event.move)
      this.isMoving = true;
    if (event.click && !this.isMoving)
      this._triggerPlayPauseButton();
    else if (this.gui === 'real_time') {
      this.previousState = 'real_time';
      this._triggerPlayPauseButton();
    }

    const clampedValued = Math.min(value, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(this.data.frames.length * clampedValued / 100);
    this.start = (new Date().getTime()) - Math.floor(this.data.basicTimeStep * this.step);
    this._updateAnimationState(requestedStep, event.click);
  }

  _updateAnimationState(requestedStep = undefined, click) {
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
        this.timeSlider.setValue(100 * this.step / this.data.frames.length);

      this.previousStep = this.step;
      this.view.time = this.data.frames[this.step].time;
      this.currentTime.innerHTML = this._formatTime(this.view.time);
      this.scene.render();
    }

    if (click && !this.isMoving)
      this._triggerPlayPauseButton();
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

  _showPlayBar(e) {
    clearTimeout(this.timeout);
    document.getElementById('playBar').style.opacity = '1';
  }

  _hidePlayBar(e) {
    this.timeout = setTimeout(() => { if (!Animation_slider.isSelected) document.getElementById('playBar').style.opacity = '0'; }, 500);
  }

  _changeSettingsPaneVisibility(event) {
    if (event.target.id === 'settingsButton' && document.getElementById('settingsPane').style.visibility === 'hidden') {
      document.getElementById('settingsPane').style.visibility = 'visible';
      document.getElementById('settingsButton').style.transform = 'rotate(5deg)';
    } else if (document.getElementById('settingsPane').style.visibility === 'visible') {
      document.getElementById('settingsPane').style.visibility = 'hidden';
      document.getElementById('settingsButton').style.transform = 'rotate(-5deg)';
    }
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
    for (let i of document.getElementsByClassName('checkSpeed')) {
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
    for (let i of document.getElementsByClassName('checkGtao')) {
      if (i.id === 'c' + event.srcElement.id)
        i.innerHTML = '&check;';
      else
        i.innerHTML = '';
    }
    this.start = new Date().getTime() - this.data.basicTimeStep * this.step / this.speed;
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
}
