'use strict';
import DefaultUrl from './Default_url.js';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './Fullscreen_handler.js';

export default class Animation {
  constructor(url, scene, view, gui, loop) {
    this.url = url;
    this.scene = scene;
    this.view = view;
    this.gui = typeof gui === 'undefined' || gui === 'play' ? 'real_time' : 'pause';
    this.loop = typeof loop === 'undefined' ? true : loop;
    this.sliding = false;
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

    this.timeSlider = document.createElement('input');
    this.timeSlider.className = 'time-slider';
    this.timeSlider.type = 'range';
    this.timeSlider.min = 0;
    this.timeSlider.max = 100;
    this.timeSlider.step = 1;
    this.timeSlider.value = 0;
    this.timeSlider.id = 'timeSlider';
    let those = this;
    this.timeSlider.oninput = function() {
      those._updateSlider(this.value);
    };

    this.button = document.createElement('button');
    const action = (this.gui === 'real_time') ? 'pause' : 'play';
    this.button.className = 'player-btn icon-' + action;
    this.button.addEventListener('click', () => { this._triggerPlayPauseButton(); });

    this.fullscreenButton = document.createElement('button');
    this.fullscreenButton.className = 'player-btn icon-fullscreen';
    this.fullscreenButton.onclick = () => { requestFullscreen(this.view); };

    this.exit_fullscreenButton = document.createElement('button');
    this.exit_fullscreenButton.className = 'player-btn icon-partscreen';
    this.exit_fullscreenButton.style.display = 'none';
    this.exit_fullscreenButton.onclick = () => { exitFullscreen(); };

    document.addEventListener('fullscreenchange', () => { onFullscreenChange(this.fullscreenButton, this.exit_fullscreenButton); });
    document.addEventListener('webkitfullscreenchange', () => { onFullscreenChange(this.fullscreenButton, this.exit_fullscreenButton); });
    document.addEventListener('mozfullscreenchange', () => { onFullscreenChange(this.fullscreenButton, this.exit_fullscreenButton); });
    document.addEventListener('MSFullscreenChange', () => { onFullscreenChange(this.fullscreenButton, this.exit_fullscreenButton); });

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
    document.getElementById('leftPane').appendChild(this.button);
    document.getElementById('leftPane').appendChild(this.currentTime);
    document.getElementById('leftPane').appendChild(timeDivider);
    document.getElementById('leftPane').appendChild(totalTime);
    document.getElementById('rightPane').appendChild(this.fullscreenButton);
    document.getElementById('rightPane').appendChild(this.exit_fullscreenButton);

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
      this.start = new Date().getTime() - this.data.basicTimeStep * this.step;
      window.requestAnimationFrame(() => { this._updateAnimation(); });
    }

    const action = (this.gui === 'real_time') ? 'pause' : 'play';
    this.button.className = 'player-btn icon-' + action;
  }

  _updateSlider(value) {
    this._triggerPlayPauseButton();
    const clampedValued = Math.min(value, 99); // set maximum value to get valid step index
    const requestedStep = Math.floor(this.data.frames.length * clampedValued / 100);
    this.start = (new Date().getTime()) - Math.floor(this.data.basicTimeStep * this.step);
    this._updateAnimationState(requestedStep);
  }

  _updateAnimationState(requestedStep = undefined) {
    const automaticMove = typeof requestedStep === 'undefined';
    if (automaticMove) {
      requestedStep = Math.floor(this._elapsedTime() / this.data.basicTimeStep);
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
    if (requestedStep === this.step)
      return;
    this.step = requestedStep;

    const appliedIds = [];
    if (this.data.frames[this.step].hasOwnProperty('poses')) {
      const poses = this.data.frames[this.step].poses;
      for (let p = 0; p < poses.length; p++)
        appliedIds[poses[p].id] = this.scene.applyPose(poses[p], this.data.frames[this.step].time);
    }
    const x3dScene = this.view.x3dScene;
    // lookback mechanism: search in history
    if (this.step !== this.previousStep + 1) {
      let previousPoseStep;
      if (this.step > this.previousStep)
        // in forward animation check only the changes since last pose
        previousPoseStep = this.previousStep;
      else
        previousPoseStep = 0;
      for (let i in this.allIds) {
        const id = this.allIds[i];
        for (let f = this.step - 1; f >= previousPoseStep; f--) {
          if (this.data.frames[f].poses) {
            for (let p = 0; p < this.data.frames[f].poses.length; p++) {
              if (this.data.frames[f].poses[p].id === id)
                x3dScene.applyPose(this.data.frames[f].poses[p], this.data.frames[f].time);
            }
          }
        }
      }
    }
    if (automaticMove)
      this.timeSlider.value = 100 * this.step / this.data.frames.length;
    else
      this._triggerPlayPauseButton();

    this._updateSliderBackground(this.timeSlider.value);
    this.previousStep = this.step;
    this.view.time = this.data.frames[this.step].time;
    this.currentTime.innerHTML = this._formatTime(this.view.time);
    x3dScene.render();
  }

  _updateAnimation() {
    if (this.gui === 'real_time' && !this.sliding)
      this._updateAnimationState();

    window.requestAnimationFrame(() => { this._updateAnimation(); });
  }

  _updateSliderBackground(value) {
    // Hack for webkit-browsers which don't support input range progress indication
    let percent = (value / 100) * 100;
    document.getElementById('timeSlider').style.background = '-webkit-linear-gradient(left, #F00 0%, #F00 ' + percent + '%, rgba(240,240,240, 1) ' + percent + '%)';
  };

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
    this.timeout = setTimeout(_ => { document.getElementById('playBar').style.opacity = '0';}, 500);
  }
}
