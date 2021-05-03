'use strict';
import DefaultUrl from './DefaultUrl.js';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './fullscreen_handler.js';

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
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', this.url, true);
    xmlhttp.overrideMimeType('application/json');
    xmlhttp.onreadystatechange = () => {
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0))
        this._setup(JSON.parse(xmlhttp.responseText));
    };
    xmlhttp.send();
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
    this.labelsIds = this.data.labelsIds.split(';').filter(Boolean).map(s => parseInt(s));

    const canvas = document.getElementById('canvas');
    canvas.insertAdjacentHTML('afterend', "<div id='playBar'></div>");
    const div = document.getElementById('playBar');

    this.button = document.createElement('button');
    this.button.id = 'playPauseButton';
    const action = (this.gui === 'real_time') ? 'pause' : 'real_time';
    this.button.style.backgroundImage = 'url(' + DefaultUrl.wwiImagesUrl() + action + '.png)';
    this.button.style.padding = '0';
    this.button.addEventListener('click', () => { this._triggerPlayPauseButton(); });
    div.appendChild(this.button);

    const slider = document.createElement('div');
    slider.id = 'playSlider';
    div.appendChild(slider);
    this.playSlider = $('#playSlider').slider();
    this._connectSliderEvents();

    div.appendChild(this._createToolBarButton('exit_fullscreen', 'Exit fullscreen'));
    this.exit_fullscreenButton.onclick = () => { exitFullscreen(); };
    this.exit_fullscreenButton.style.display = 'none';
    div.appendChild(this._createToolBarButton('fullscreen', 'Enter fullscreen'));
    this.fullscreenButton.onclick = () => { requestFullscreen(this.view); };

    document.addEventListener('fullscreenchange', () => { onFullscreenChange(this.fullscreenButton, this.exit_fullscreenButton); });
    document.addEventListener('webkitfullscreenchange', () => { onFullscreenChange(this.fullscreenButton, this.exit_fullscreenButton); });
    document.addEventListener('mozfullscreenchange', () => { onFullscreenChange(this.fullscreenButton, this.exit_fullscreenButton); });
    document.addEventListener('MSFullscreenChange', () => { onFullscreenChange(this.fullscreenButton, this.exit_fullscreenButton); });

    // Initialize animation data.
    this.start = new Date().getTime();
    this.step = 0;
    this.previousStep = 0;
    this._updateAnimation();

    // Notify creation completed.
    if (typeof this.onReady === 'function')
      this.onReady();
  }

  _createToolBarButton(name, tooltip) {
    const buttonName = name + 'Button';
    this[buttonName] = document.createElement('button');
    this[buttonName].id = buttonName;
    this[buttonName].className = 'toolBarButton';
    this[buttonName].title = tooltip;
    this[buttonName].style.backgroundImage = 'url(' + DefaultUrl.wwiImagesUrl() + name + '.png)';
    return this[buttonName];
  }

  _elapsedTime() {
    const end = new Date().getTime();
    return end - this.start;
  }

  _triggerPlayPauseButton() {
    this.button.style.backgroundImage = 'url(' + DefaultUrl.wwiImagesUrl() + this._getIconBaseName(this.gui) + '.png)';
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
  }

  _connectSliderEvents() {
    this.playSlider = this.playSlider.slider({
      change: (e, ui) => {
        this._updateSlider(ui.value);
        // continue running the animation
        this._updateAnimation();
      },
      slide: (e, ui) => { this._updateSlider(ui.value); },
      start: (e, ui) => { this.sliding = true; },
      stop: (e, ui) => { this.sliding = false; }
    });
  }

  _disconnectSliderEvents() {
    this.playSlider.slider({change: null, slide: null});
  }

  _updateSlider(value) {
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
    const appliedLabelsIds = [];

    if (this.data.frames[this.step].hasOwnProperty('poses')) {
      const poses = this.data.frames[this.step].poses;
      for (let p = 0; p < poses.length; p++)
        appliedIds[poses[p].id] = this.scene.applyPose(poses[p], this.data.frames[this.step].time);
    }

    if (this.data.frames[this.step].hasOwnProperty('labels')) {
      let labels = this.data.frames[this.step].labels;
      for (let i = 0; i < labels.length; i++) {
        this.scene.applyLabel(labels[i], this.view);
        appliedLabelsIds.push(labels[i].id);
      }
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
        let appliedFields = appliedIds[id];
        for (let f = this.step - 1; f >= previousPoseStep; f--) {
          if (this.data.frames[f].poses) {
            for (let p = 0; p < this.data.frames[f].poses.length; p++) {
              if (this.data.frames[f].poses[p].id === id)
                appliedFields = this.scene.applyPose(this.data.frames[f].poses[p], this.data.frames[f].time, appliedFields);
            }
          }
        }
      }

      for (let id of this.labelsIds) {
        for (let f = this.step - 1; f >= previousPoseStep; f--) {
          if (this.data.frames[f].labels) {
            for (let p = 0; p < this.data.frames[f].labels.length; p++) {
              if (this.data.frames[f].labels[p].id === id) {
                if (!appliedLabelsIds.includes(id)) {
                  this.scene.applyLabel(this.data.frames[f].labels[p], this.view);
                  appliedLabelsIds.push(id);
                }
              }
            }
          }
        }
      }
    }
    if (automaticMove) {
      this._disconnectSliderEvents();
      this.playSlider.slider('option', 'value', 100 * this.step / this.data.frames.length);
      this._connectSliderEvents();
    }
    this.previousStep = this.step;
    this.view.time = this.data.frames[this.step].time;
    x3dScene.render();
  }

  _updateAnimation() {
    if (this.gui === 'real_time' && !this.sliding)
      this._updateAnimationState();

    window.requestAnimationFrame(() => { this._updateAnimation(); });
  }

  _getIconBaseName() {
    return this.gui === 'real_time' ? 'real_time' : 'pause';
  }
}
