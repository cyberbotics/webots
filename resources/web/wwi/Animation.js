'use strict';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './Fullscreen_handler.js';
import Animation_slider from './Animation_slider.js';

export default class Animation {
  constructor(url, scene, view, gui, loop) {
    this.url = url;
    this.scene = scene;
    this.view = view;
    this.gui = typeof gui === 'undefined' || gui === 'play' ? 'real_time' : 'pause';
    this.loop = typeof loop === 'undefined' ? true : loop;
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

    let settingsList = document.createElement('ul');
    settingsList.id = 'settingsList';

    let playBackLi = document.createElement('li');
    playBackLi.id = 'playBackLi';
    let label = document.createElement('span');
    label.innerHTML = 'Playback speed';
    label.className = 'settingTitle';
    playBackLi.appendChild(label);
    label = document.createElement('div');
    label.className = 'spacer';
    playBackLi.appendChild(label);
    label = document.createElement('span');
    label.className = 'speed';
    label.innerHTML = '1';
    playBackLi.appendChild(label);
    label = document.createElement('div');
    label.className = 'arrowRight';
    playBackLi.appendChild(label);

    let resetViewpoint = document.createElement('li');
    resetViewpoint.id = 'resetViewpoint';
    label = document.createElement('span');
    label.className = 'settingTitle';
    label.innerHTML = 'Reset viewpoint';
    resetViewpoint.appendChild(label);
    label = document.createElement('div');
    label.className = 'spacer';
    resetViewpoint.appendChild(label);

    let graphicalSettings = document.createElement('li');
    graphicalSettings.id = 'graphicalSettings';
    label = document.createElement('span');
    label.className = 'settingTitle';
    label.innerHTML = 'Graphical settings';
    graphicalSettings.appendChild(label);
    label = document.createElement('div');
    label.className = 'spacer';
    graphicalSettings.appendChild(label);
    label = document.createElement('div');
    label.className = 'arrowRight';
    graphicalSettings.appendChild(label);

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
    document.getElementById('view3d').appendChild(settingsPane);
    document.getElementById('leftPane').appendChild(this.playButton);
    document.getElementById('leftPane').appendChild(this.currentTime);
    document.getElementById('leftPane').appendChild(timeDivider);
    document.getElementById('leftPane').appendChild(totalTime);
    document.getElementById('rightPane').appendChild(settingsButton);
    document.getElementById('rightPane').appendChild(fullscreenButton);
    document.getElementById('rightPane').appendChild(exitFullscreenButton);
    document.getElementById('settingsPane').appendChild(resetViewpoint);
    document.getElementById('settingsPane').appendChild(playBackLi);
    document.getElementById('settingsPane').appendChild(graphicalSettings);
    // Initialize animation data.
    this.start = new Date().getTime();
    this.step = 0;
    this.previousStep = 0;
    this._updateAnimation();

    // Notify creation completed.
    if (typeof this.onReady === 'function')
      this.onReady();

    console.time('time');
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
      this.start = new Date().getTime() - this.data.basicTimeStep * this.step;
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
      requestedStep = Math.floor(this._elapsedTime() / this.data.basicTimeStep);
      if (requestedStep < 0 || requestedStep >= this.data.frames.length) {
        console.timeEnd('time');
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
}
