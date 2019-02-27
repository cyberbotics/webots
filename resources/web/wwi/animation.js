/* global ResourceManager */
'use strict';

class Animation { // eslint-disable-line no-unused-vars
  constructor(url, scene, view, gui, loop) {
    this.url = url;
    this.scene = scene;
    this.view = view;
    this.gui = gui === undefined || gui === 'play' ? 'play' : 'pause';
    this.loop = loop === undefined ? true : loop;
    this.sliding = false;
    this.onReady = null;
  }

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

  moveSlider(event) {
    if (!this.playSlider || !this.sliding)
      return;

    var w = event.target.clientWidth - 66; // size of the borders of the slider
    var x = event.clientX - event.target.getBoundingClientRect().left - 48; // size of the left border (including play button) of the slider
    var value = 100 * x / w;
    if (value < 0)
      value = 0;
    else if (value >= 100)
      value = 99.999;
    this.playSlider.slider('value', value);
    // setting the value should trigger the change event, unfortunately, doesn't seem to work reliably,
    // therefore, we need to trigger this event manually:
    var ui = {};
    ui.value = value;
    this.playSlider.slider('option', 'change').call(this.playSlider, event, ui);
  }

  // private methods
  _setup(data) {
    this.data = data;

    // create play bar.
    var div = document.createElement('div');
    div.id = 'playBar';
    this.view.view3D.appendChild(div);

    this.button = document.createElement('button');
    this.button.id = 'playPauseButton';
    var action = (this.gui === 'play') ? 'pause' : 'play';
    var resourceManager = new ResourceManager();
    this.button.style.backgroundImage = resourceManager.getImageUrl(action);
    this.button.style.padding = '0';
    this.button.addEventListener('click', () => this._triggerPlayPauseButton());
    div.appendChild(this.button);

    var slider = document.createElement('div');
    slider.id = 'playSlider';
    div.appendChild(slider);
    this.playSlider = $('#playSlider').slider({
      change: function(e, ui) { this._updateSlider(ui.value); },
      slide: function(e, ui) { this._updateSlider(ui.value); },
      start: function(e, ui) { this.sliding = true; },
      stop: function(e, ui) { this.sliding = false; }
    });

    // init animation data.
    this.start = new Date().getTime();
    this.step = 0;
    this.previousStep = 0;
    this._updateAnimation();

    // notify creation completed.
    if (this.onReady)
      this.onReady();
  }

  _elapsedTime() {
    var end = new Date().getTime();
    return end - this.start;
  }

  _triggerPlayPauseButton() {
    var resourceManager = new ResourceManager();
    this.button.style.backgroundImage = resourceManager.getImageUrl(this.gui);
    if (this.gui === 'play') {
      this.gui = 'pause';
      if (this.step < 0 || this.step >= this.data.frames.length) {
        this.start = new Date().getTime();
        this._updateAnimationState(true);
      } else
        this.start = new Date().getTime() - this.data.basicTimeStep * this.step;
    } else {
      this.gui = 'play';
      this.start = new Date().getTime() - this.data.basicTimeStep * this.step;
      window.requestAnimationFrame(this._updateAnimation);
    }
  }

  _connectSliderEvents() {
    this.playSlider = this.playSlider.slider({
      change: (e, ui) => this._updateSlider(ui.value),
      slide: (e, ui) => this._updateSlider(ui.value),
      start: this.sliding = (e, ui) => true,
      stop: this.sliding = (e, ui) => false
    });
  }

  _disconnectSliderEvents() {
    this.playSlider.slider({change: null, slide: null});
  }

  _updateSlider(value) {
    this.step = Math.floor(this.data.frames.length * value / 100);
    this.start = (new Date().getTime()) - Math.floor(this.data.basicTimeStep * this.step);
    this._updateAnimationState(false);
  }

  _updateAnimationState(moveSlider) {
    if (moveSlider) {
      this.step = Math.floor(this._elapsedTime() / this.data.basicTimeStep);
      if (this.step < 0 || this.step >= this.data.frames.length) {
        if (this.loop) {
          if (this.step > this.data.frames.length) {
            this.step = 0;
            this.previousStep = 0;
            this.start = new Date().getTime();
          } else
            return;
        } else if (this.gui === 'play') {
          this._triggerPlayPauseButton();
          return;
        } else
          return;
      }
    }
    var p;
    var appliedIds = [];
    if (this.data.frames[this.step].hasOwnProperty('poses')) {
      var poses = this.data.frames[this.step].poses;
      for (p = 0; p < poses.length; p++) {
        this._applyPose(poses[p]);
        appliedIds[appliedIds.length] = poses[p].id;
      }
    }
    // lookback mechanism: search in history
    if (this.step !== this.previousStep + 1) {
      var previousPoseStep;
      if (this.step > this.previousStep)
        // in forward animation check only the changes since last pose
        previousPoseStep = this.previousStep;
      else
        previousPoseStep = 0;
      var allIds = this.data.ids.split(';');
      for (var i = 0; i < allIds.length; i++) {
        var id = parseInt(allIds[i]);
        if (appliedIds.indexOf(id) === -1) {
          outer:
          for (var f = this.step - 1; f >= previousPoseStep; f--) {
            if (this.data.frames[f].poses) {
              for (p = 0; p < this.data.frames[f].poses.length; p++) {
                if (this.data.frames[f].poses[p].id === id) {
                  this._applyPose(this.data.frames[f].poses[p]);
                  break outer;
                }
              }
            }
          }
        }
      }
    }
    if (moveSlider) {
      this._disconnectSliderEvents();
      this.playSlider.slider('option', 'value', 100 * this.step / this.data.frames.length);
      this._connectSliderEvents();
    }
    this.previousStep = this.step;
    this.view.time = this.data.frames[this.step].time;
    if (this.view.followedObject != null && this.view.followedObject !== 'none')
      this.view.updateViewpointPosition(!moveSlider | this.step === 0);
  }

  _updateAnimation() {
    if (this.gui === 'play') {
      this._updateAnimationState(true);
      window.requestAnimationFrame(() => this._updateAnimation());
    }
  }

  _applyPose(pose) {
    var id = pose.id;
    // TODO if (el.getAttribute('blockWebotsUpdate')) return;
    for (var key in pose) {
      if (key === 'id')
        continue;
      var value = pose[key];
      this.scene.applyPose(id, key, value);
    }
  }
}
