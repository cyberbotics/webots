'use strict';
import {requestFullscreen, exitFullscreen, onFullscreenChange} from './fullscreen_handler.js';
import InformationPanel from './InformationPanel.js';
import AnimationSlider from './AnimationSlider.js';
import WbWorld from './nodes/WbWorld.js';
import {changeShadows, changeGtaoLevel, GtaoLevel} from './nodes/wb_preferences.js';

export default class Animation {
  constructor(jsonPromise, scene, view, gui, loop) {
    this._jsonPromise = jsonPromise;
    this._scene = scene;
    this._view = view;
    this.gui = typeof gui === 'undefined' || gui === 'play' ? 'real-time' : 'pause';
    this._loop = typeof loop === 'undefined' ? true : loop;
    this.speed = 1;
    this._view3d = scene.domElement;
    for (let i = 0; i < this._view3d.childNodes.length; i++) {
      const child = this._view3d.childNodes[i];
      if (child.id === 'canvas')
        this._canvas = child;
    }
  };

  init(onReady) {
    this._onReady = onReady;
    this._jsonPromise.then(json => this._setup(json))
      .catch(error => console.error(error));
  }

  pause() {
    this.gui = 'pause';
    if (typeof this.data === 'undefined')
      return;
    if (this.step < 0 || this.step >= this.data.frames.length) {
      this.start = new Date().getTime();
      this.updateAnimationState();
    } else
      this.start = new Date().getTime() - this.data.basicTimeStep * this.step;
  }

  // private methods
  _setup(data) {
    this.data = data;
    // extract animated node ids: remove empty items and convert to integer
    this._allIds = this.data.ids.split(';').filter(Boolean).map(s => parseInt(s));
    this._labelsIds = typeof this.data.labelsIds === 'undefined' ? [] : this.data.labelsIds.split(';').filter(Boolean).map(s => parseInt(s));

    // generate keyFrames to speed up the navigation.
    this._keyFrames = new Map();
    const stepSize = 1000; // Generate a keyFrame each 1000 timesteps. It is an empirical value.
    if (this.data.frames.length > 1000) {
      const numberOfKeyFrames = Math.floor(this.data.frames.length / 1000);
      // let poses = this.data.frames[0].poses;
      // poses = new Map(poses.map(pose => [pose.id, pose]));
      // this.data.frames[0].poses = poses;
      const allPoses = new Map();
      const allLabels = new Map();

      for (let i = 0; i < numberOfKeyFrames; i++) {
        for (let j = (i + 1) * stepSize; j > i * stepSize; j--) {
          const poses = this.data.frames[i].poses;
          // poses = new Map(poses.map(pose => [pose.id, pose])); // Convert to map for easier manipulation
          // this.data.frames[i].poses = poses;
          if (poses) {
            for (let k = 0; k < poses.length; k++) {
              if (!allPoses.has(poses[i].id))
                allPoses.set(poses[i].id, poses[i]);
            }
          }

          const labels = this.data.frames[i].labels;
          if (labels) {
            for (let k = 0; k < labels.length; k++) {
              if (!allLabels.has(labels[i].id))
                allLabels.set(labels[i].id, labels[i]);
            }
          }

          if (allPoses.size === this._allIds && allLabels.size === this._labelsIds.length)
            break;
        }

        if (i === 0) {
          let poses = this.data.frames[i].poses;
          for (let k = 0; k < poses.length; k++) {
            if (!allPoses.has(poses[i].id))
              allPoses.set(poses[i].id, poses[i]);
          }
        }

        this._keyFrames.set(i, {poses: new Map(allPoses), labels: new Map(allLabels)});
      }
    }
    console.log(this._keyFrames)
    // Initialize animation data.
    this.start = new Date().getTime();
    this.step = 0;
    this._previousStep = 0;
    this.updateAnimation();

    // Notify creation completed.
    if (typeof this._onReady === 'function')
      this._onReady();
  }

  _elapsedTime() {
    const end = new Date().getTime();
    return end - this.start;
  }

  updateAnimationState(requestedStep = undefined) {
    const automaticMove = typeof requestedStep === 'undefined';
    if (automaticMove) {
      requestedStep = Math.floor(this._elapsedTime() * this.speed / this.data.basicTimeStep);
      if (requestedStep < 0 || requestedStep >= this.data.frames.length) {
        if (this._loop) {
          if (requestedStep > this.data.frames.length) {
            requestedStep = 0;
            this._previousStep = 0;
            this.start = new Date().getTime();
          } else
            return;
        } else if (this.gui === 'real-time') {
          this._triggerPlayPauseButton();
          return;
        } else
          return;
      }
    }
    if (requestedStep !== this.step) {
      this.step = requestedStep;

      const appliedIds = [];
      const appliedLabelsIds = [];

      if (this.data.frames[this.step].hasOwnProperty('poses')) {
        const poses = this.data.frames[this.step].poses;
        for (let p = 0; p < poses.length; p++)
          appliedIds[poses[p].id] = this._scene.applyPose(poses[p], undefined);
        WbWorld.instance.tracks.forEach(track => {
          if (track.linearSpeed !== 0) {
            track.animateMesh();
            track.linearSpeed = 0;
          }
        });
      }

      if (this.data.frames[this.step].hasOwnProperty('labels')) {
        const labels = this.data.frames[this.step].labels;
        for (let i = 0; i < labels.length; i++) {
          this._scene.applyLabel(labels[i], this._view);
          appliedLabelsIds.push(labels[i].id);
        }
      }

      // lookback mechanism: search in history
      if (this.step !== this._previousStep + 1) {
        let previousPoseStep;
        if (this.step > this._previousStep)
          // in forward animation check only the changes since last pose
          previousPoseStep = this._previousStep;
        else
          previousPoseStep = 0;
        for (let i in this._allIds) {
          const id = this._allIds[i];
          let appliedFields = appliedIds[id];
          for (let f = this.step - 1; f >= previousPoseStep; f--) {
            if (this.data.frames[f].poses) {
              for (let p = 0; p < this.data.frames[f].poses.length; p++) {
                if (this.data.frames[f].poses[p].id === id)
                  appliedFields = this._scene.applyPose(this.data.frames[f].poses[p], appliedFields);
              }
            }
          }
        }

        for (let id of this._labelsIds) {
          for (let f = this.step - 1; f >= previousPoseStep; f--) {
            if (this.data.frames[f].labels) {
              for (let p = 0; p < this.data.frames[f].labels.length; p++) {
                if (this.data.frames[f].labels[p].id === id) {
                  if (!appliedLabelsIds.includes(id)) {
                    this._scene.applyLabel(this.data.frames[f].labels[p], this._view);
                    appliedLabelsIds.push(id);
                  }
                }
              }
            }
          }
        }
      }

      if (automaticMove) {
        let timeSlider = document.getElementById('timeSlider');
        if (timeSlider)
          timeSlider.setValue(100 * this.step / this.data.frames.length);
      }

      this._previousStep = this.step;
      this._view.time = this.data.frames[this.step].time;
      let currentTime = document.getElementById('currentTime');
      if (currentTime)
        currentTime.innerHTML = this._formatTime(this._view.time);
      WbWorld.instance.viewpoint.updateFollowUp(this._view.time, !automaticMove || this.step === 0);
      this._scene.render();
    }
  }

  updateAnimation() {
    if (this.gui === 'real-time')
      this.updateAnimationState();

    window.requestAnimationFrame(() => this.updateAnimation());
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
      const maxTime = this.data.frames[this.data.frames.length - 1].time;
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
}

Animation.sliderDefined = false;
