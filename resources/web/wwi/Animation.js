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
    this.keyFrameStepSize = 1000; // Generate a keyFrame each 1000 timesteps. It is an empirical value.
    if (this.data.frames.length > 1000) {
      this.numberOfKeyFrames = Math.floor(this.data.frames.length / 1000);
      // let poses = this.data.frames[0].poses;
      // poses = new Map(poses.map(pose => [pose.id, pose]));
      // this.data.frames[0].poses = poses;
      const allPoses = new Map();
      const allLabels = new Map();

      for (let i = 0; i < this.numberOfKeyFrames; i++) {
        for (let j = (i + 1) * this.keyFrameStepSize; j > i * this.keyFrameStepSize; j--) {
          const poses = this.data.frames[j].poses;
          // poses = new Map(poses.map(pose => [pose.id, pose])); // Convert to map for easier manipulation
          // this.data.frames[i].poses = poses;
          if (poses) {
            for (let k = 0; k < poses.length; k++) {
              let currentIdFields = allPoses.has(poses[k].id) ? allPoses.get(poses[k].id) : new Map();
              for (let field in poses[k]) {
                if (field === 'id' || currentIdFields.has(field))
                  continue;
                else
                  currentIdFields.set(field, {'id': poses[k].id, [field]: poses[k][field]});
              }
              allPoses.set(poses[k].id, currentIdFields);
            }
          }

          if (allLabels.size === this._labelsIds.length) // We already have all the update we need for the labels
            continue;

          const labels = this.data.frames[i].labels;
          if (labels) {
            for (let k = 0; k < labels.length; k++) {
              if (!allLabels.has(labels[i].id))
                allLabels.set(labels[i].id, labels[i]);
            }
          }
        }

        if (i === 0) {
          const poses = this.data.frames[0].poses;
          if (poses) {
            for (let j = 0; j < poses.length; j++) {
              let currentIdFields = allPoses.has(poses[j].id) ? allPoses.get(poses[j].id) : new Map();

              for (let field in poses[j]) {
                if (field === 'id' || currentIdFields.has(field))
                  continue;
                else
                  currentIdFields.set(field, {'id': poses[j].id, [field]: poses[j][field]});
              }
              allPoses.set(poses[j].id, currentIdFields);
            }
          }
        }
        this._keyFrames.set(i, {poses: new Map(allPoses), labels: new Map(allLabels)});
      }
      this.numberOfFields = new Map();
      for (let [key, value] of this._keyFrames.get(this.numberOfKeyFrames - 1).poses)
        this.numberOfFields.set(key, value.size);
    }

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

      // lookback mechanism: search in history
      if (this.step !== this._previousStep + 1) {
        let previousPoseStep;
        let closestKeyFrame = Math.floor(this.step / this.keyFrameStepSize);

        if (this.step > this._previousStep && this._previousStep > closestKeyFrame * this.keyFrameStepSize)
          previousPoseStep = this._previousStep;
        else
          previousPoseStep = closestKeyFrame * this.keyFrameStepSize;

        let completeIds = new Set();
        let appliedFieldsByIds = new Map();
        for (let i = this.step; i > previousPoseStep; i--) { // Iterate through each step until the nearest keyFrame is reached or all necessary updates have been applied. Go in decreasing order to minize the number of step.
          if (this.data.frames[i].poses) {
            for (let j = 0; j < this.data.frames[i].poses.length; j++) { // At each frame, apply all poses
              let id = this.data.frames[i].poses[j].id;
              if (!completeIds.has(id)) { // Try to apply some updates to a node only if it is missing some
                for (let field in this.data.frames[i]) {
                  if (!appliedFieldsByIds.has(id)) {
                    appliedFieldsByIds.set(id, new Set());
                    appliedFieldsByIds.get(id).add('id');
                  }

                  if (appliedFieldsByIds.get(id).has(field)) // we want to update each field only once
                    continue;
                  else {
                    this._scene.applyPose({'id': id, [field]: this.data.frames[i].poses[j][field]});
                    appliedFieldsByIds.get(id).add(field);
                    if (appliedFieldsByIds.size === this.numberOfFields.get(id))
                      completeIds.add(id);
                  }
                }
              }
            }
          }
        }

        // TODO
        for (let id of this._labelsIds) {
          for (let f = this.step; f > previousPoseStep; f--) {
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
      } else {
        console.log("normal update")
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
