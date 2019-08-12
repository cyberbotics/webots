/* global DefaultUrl, TextureLoader */
'use strict';

class Animation { // eslint-disable-line no-unused-vars
  constructor(url, scene, view, gui, loop) {
    this.url = url;
    this.scene = scene;
    this.view = view;
    this.gui = typeof gui === 'undefined' || gui === 'play' ? 'real_time' : 'pause';
    this.loop = typeof loop === 'undefined' ? true : loop;
    this.sliding = false;
    this.onReady = null;
  };

  init(onReady) {
    this.onReady = onReady;
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', this.url, true);
    xmlhttp.overrideMimeType('application/json');
    xmlhttp.onreadystatechange = () => {
      if (xmlhttp.readyState === 4 && xmlhttp.status === 200)
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

    // Automatically start the animation only when all the textures are loaded.
    if (this.gui === 'real_time' && TextureLoader.hasPendingData())
      this.gui = 'play_on_load'; // wait for textures loading

    // Create play bar.
    var div = document.createElement('div');
    div.id = 'playBar';
    this.view.view3D.appendChild(div);

    this.button = document.createElement('button');
    this.button.id = 'playPauseButton';
    var action = (this.gui === 'real_time') ? 'pause' : 'real_time';
    this.button.style.backgroundImage = 'url(' + DefaultUrl.wwiImagesUrl() + action + '.png)';
    this.button.style.padding = '0';
    this.button.addEventListener('click', () => { this._triggerPlayPauseButton(); });
    div.appendChild(this.button);

    var slider = document.createElement('div');
    slider.id = 'playSlider';
    div.appendChild(slider);
    this.playSlider = $('#playSlider').slider();
    this._connectSliderEvents();

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
    var end = new Date().getTime();
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
    var clampedValued = Math.min(value, 99); // set maximum value to get valid step index
    var requestedStep = Math.floor(this.data.frames.length * clampedValued / 100);
    this.start = (new Date().getTime()) - Math.floor(this.data.basicTimeStep * this.step);
    this._updateAnimationState(requestedStep);
  }

  _updateAnimationState(requestedStep = undefined) {
    var automaticMove = typeof requestedStep === 'undefined';
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

    var p;
    var appliedIds = [];
    if (this.data.frames[this.step].hasOwnProperty('poses')) {
      var poses = this.data.frames[this.step].poses;
      for (p = 0; p < poses.length; p++)
        appliedIds[poses[p].id] = this.scene.applyPose(poses[p]);
    }
    var x3dScene = this.view.x3dScene;
    // lookback mechanism: search in history
    if (this.step !== this.previousStep + 1) {
      var previousPoseStep;
      if (this.step > this.previousStep)
        // in forward animation check only the changes since last pose
        previousPoseStep = this.previousStep;
      else
        previousPoseStep = 0;
      for (let i in this.allIds) {
        var id = this.allIds[i];
        var appliedFields = appliedIds[id];
        for (let f = this.step - 1; f >= previousPoseStep; f--) {
          if (this.data.frames[f].poses) {
            for (p = 0; p < this.data.frames[f].poses.length; p++) {
              if (this.data.frames[f].poses[p].id === id)
                appliedFields = x3dScene.applyPose(this.data.frames[f].poses[p], appliedFields);
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
    x3dScene.viewpoint.updateViewpointPosition(!automaticMove | this.step === 0, this.view.time);
    x3dScene.viewpoint.notifyCameraParametersChanged();
  }

  _updateAnimation() {
    if (this.gui === 'real_time' && !this.sliding) {
      this._updateAnimationState();
      window.requestAnimationFrame(() => { this._updateAnimation(); });
    } else if (this.gui === 'play_on_load') {
      if (!TextureLoader.hasPendingData())
        this._triggerPlayPauseButton();
      window.requestAnimationFrame(() => { this._updateAnimation(); });
    }
  }

  _getIconBaseName() {
    return this.gui === 'real_time' ? 'real_time' : 'pause';
  }
}
