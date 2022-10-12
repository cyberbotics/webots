'use strict';
import WbWorld from './nodes/WbWorld.js';

export default class Animation {
  #jsonPromise;
  #keyFrames;
  #labelsIds;
  #loop;
  #onReady;
  #previousStep;
  #scene;
  #unusedPrefix;
  #view;
  constructor(jsonPromise, scene, view, gui, loop) {
    this.#jsonPromise = jsonPromise;
    this.#scene = scene;
    this.#view = view;
    this.gui = typeof gui === 'undefined' || gui === 'play' ? 'real-time' : 'pause';
    this.#loop = typeof loop === 'undefined' ? true : loop;
    this.speed = 1;
  };

  init(onReady) {
    this.#onReady = onReady;
    this.#jsonPromise.then(json => this.#setup(json))
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
  #setup(data) {
    this.data = data;
    this.#labelsIds = typeof this.data.labelsIds === 'undefined' ? [] : this.data.labelsIds.split(';')
      .filter(Boolean).map(s => parseInt(s));

    const firstFrame = this.data.frames[0];
    if (firstFrame && this.#labelsIds.length > 0) {
      const labelsIdsAtStart = new Set();
      if (firstFrame.labels) {
        firstFrame.labels.forEach((label) => {
          labelsIdsAtStart.add(label.id);
        });
      } else
        firstFrame.labels = [];

      this.#labelsIds.forEach((labelId) => {
        if (!labelsIdsAtStart.has(labelId)) {
          const newlabel = {
            id: labelId,
            rgba: '0, 0, 0, 0'
          };
          firstFrame.labels.push(newlabel);
        }
      });
    }

    // generate keyFrames to speed up the navigation.
    this.#keyFrames = new Map();
    this.keyFrameStepSize = 1000; // Generate a keyFrame each 1000 timesteps. It is an empirical value.
    if (this.data.frames.length > 1000) {
      this.numberOfKeyFrames = Math.floor(this.data.frames.length / 1000);
      for (let i = 0; i < this.numberOfKeyFrames; i++) {
        const allPoses = new Map();
        const allLabels = new Map();
        for (let j = (i + 1) * this.keyFrameStepSize; j > i * this.keyFrameStepSize; j--) {
          const poses = this.data.frames[j].poses;
          if (poses) {
            for (let k = 0; k < poses.length; k++) {
              const currentIdFields = allPoses.has(poses[k].id) ? allPoses.get(poses[k].id) : new Map();
              for (let field in poses[k]) {
                if (field === 'id' || currentIdFields.has(field))
                  continue;
                else
                  currentIdFields.set(field, {'id': poses[k].id, [field]: poses[k][field]});
              }
              allPoses.set(poses[k].id, currentIdFields);
            }
          }

          if (allLabels.size === this.#labelsIds.length) // We already have all the update we need for the labels
            continue;

          const labels = this.data.frames[j].labels;
          if (labels) {
            for (let k = 0; k < labels.length; k++) {
              if (!allLabels.has(labels[k].id))
                allLabels.set(labels[k].id, labels[k]);
            }
          }
        }

        if (i === 0) {
          const poses = this.data.frames[0].poses; // No need to check the labels because they are defined in the second frames.
          if (poses) {
            for (let j = 0; j < poses.length; j++) {
              const currentIdFields = allPoses.has(poses[j].id) ? allPoses.get(poses[j].id) : new Map();

              for (let field in poses[j]) {
                if (field === 'id' || currentIdFields.has(field))
                  continue;
                else
                  currentIdFields.set(field, {'id': poses[j].id, [field]: poses[j][field]});
              }
              allPoses.set(poses[j].id, currentIdFields);
            }
          }
        } else { // Check the previous keyFrame to get missing updates
          const poses = this.#keyFrames.get(i - 1).poses;
          for (let element of poses) {
            const id = element[0];
            const currentIdFields = allPoses.has(id) ? allPoses.get(id) : new Map();

            for (let field of element[1]) {
              if (currentIdFields.has(field[0])) // we want to update each field only once
                continue;
              else
                currentIdFields.set(field[0], field[1]);
            }
            allPoses.set(id, currentIdFields);
          }

          const labels = this.#keyFrames.get(i - 1).labels;
          for (let label of labels) {
            const id = label[0];
            if (!allLabels.has(id))
              allLabels.set(id, label[1]);
          }
        }

        this.#keyFrames.set(i, {poses: new Map(allPoses), labels: new Map(allLabels)});
      }
      this.numberOfFields = new Map();
      for (let [key, value] of this.#keyFrames.get(this.numberOfKeyFrames - 1).poses)
        this.numberOfFields.set(key, value.size);
    }

    // Initialize animation data.
    this.start = new Date().getTime();
    this.step = 0;
    this.#previousStep = 0;
    this.updateAnimation();

    // Notify creation completed.
    if (typeof this.#onReady === 'function')
      this.#onReady();
  }

  #elapsedTime() {
    const end = new Date().getTime();
    return end - this.start;
  }

  updateAnimationState(requestedStep = undefined) {
    const automaticMove = typeof requestedStep === 'undefined';
    if (automaticMove) {
      requestedStep = Math.floor(this.#elapsedTime() * this.speed / this.data.basicTimeStep);
      if (requestedStep < 0 || requestedStep >= this.data.frames.length) {
        if (this.#loop) {
          if (requestedStep > this.data.frames.length) {
            requestedStep = 0;
            this.#previousStep = 0;
            this.start = new Date().getTime();
          } else
            return;
        } else
          return;
      }
    }
    if (requestedStep !== this.step) {
      this.step = requestedStep;

      // lookback mechanism: search in history
      if (this.step !== this.#previousStep + 1) {
        let previousPoseStep;
        const closestKeyFrame = Math.floor(this.step / this.keyFrameStepSize) - 1;

        let previousStepIsAKeyFrame = false;
        if (this.step > this.#previousStep && this.#previousStep > (closestKeyFrame + 1) * this.keyFrameStepSize)
          previousPoseStep = this.#previousStep;
        else {
          previousPoseStep = (closestKeyFrame + 1) * this.keyFrameStepSize;
          if (this.#keyFrames.size > 0)
            previousStepIsAKeyFrame = true;
        }

        const completeIds = new Set();
        const appliedFieldsByIds = new Map();
        const appliedLabelsIds = new Set();

        // We do not want to include the previousPoseStep in the loop as its updates are in the keyFrame.
        // However, we need to include it if there is no keyFrames or if it is the step 0 as there is no keyFrame for it
        if (previousStepIsAKeyFrame && previousPoseStep !== 0)
          previousPoseStep++;

        // Iterate through each step until the nearest keyFrame is reached or all necessary updates have been applied.
        // Go in decreasing order to minize the number of steps.
        for (let i = this.step; i >= previousPoseStep; i--) {
          if (this.data.frames[i].poses) {
            for (let j = 0; j < this.data.frames[i].poses.length; j++) { // At each frame, apply all poses
              const id = this.data.frames[i].poses[j].id;
              if (!completeIds.has(id)) { // Try to apply some updates to a node only if it is missing some
                for (let field in this.data.frames[i].poses[j]) {
                  if (!appliedFieldsByIds.has(id)) {
                    appliedFieldsByIds.set(id, new Set());
                    appliedFieldsByIds.get(id).add('id');
                  }

                  if (appliedFieldsByIds.get(id).has(field)) // we want to update each field only once
                    continue;
                  else {
                    this.#scene.applyPose({'id': id, [field]: this.data.frames[i].poses[j][field]});
                    appliedFieldsByIds.get(id).add(field);

                    if (typeof this.numberOfFields !== 'undefined' && appliedFieldsByIds.size === this.numberOfFields.get(id))
                      completeIds.add(id);
                  }
                }
              }
            }
          }

          const labels = this.data.frames[i].labels;
          if (labels) {
            for (let label of labels) {
              if (!appliedLabelsIds.has(label.id)) {
                this.#scene.applyLabel(label, this.#view);
                appliedLabelsIds.add(label.id);
              }
            }
          }
        }

        if (previousStepIsAKeyFrame && closestKeyFrame >= 0) { // Get the missing updates from the closest keyFrame
          const poses = this.#keyFrames.get(closestKeyFrame).poses;
          for (let element of poses) {
            const id = element[0];
            if (!completeIds.has(id)) { // Try to apply some updates to a node only if it is missing some
              for (let field of element[1]) {
                if (!appliedFieldsByIds.has(id)) {
                  appliedFieldsByIds.set(id, new Set());
                  appliedFieldsByIds.get(id).add('id');
                }
                if (appliedFieldsByIds.get(id).has(field[0])) // we want to update each field only once
                  continue;
                else {
                  this.#scene.applyPose(field[1]);
                  appliedFieldsByIds.get(id).add(field[0]);
                  if (typeof this.numberOfFields !== 'undefined' && appliedFieldsByIds.size === this.numberOfFields.get(id))
                    completeIds.add(id);
                }
              }
            }
          }

          if (this.#labelsIds.length !== appliedLabelsIds.size) {
            const labels = this.#keyFrames.get(closestKeyFrame).labels;
            for (let label of labels) {
              if (!appliedLabelsIds.has(label[0]))
                this.#scene.applyLabel(label[1], this.#view);
            }
          }
        }
      } else {
        if (this.data.frames[this.step].hasOwnProperty('poses')) {
          const poses = this.data.frames[this.step].poses;
          for (let p = 0; p < poses.length; p++)
            this.#scene.applyPose(poses[p], undefined);
          WbWorld.instance.tracks.forEach(track => {
            if (track.linearSpeed !== 0) {
              track.animateMesh();
              track.linearSpeed = 0;
            }
          });
        }

        if (this.data.frames[this.step].hasOwnProperty('labels')) {
          const labels = this.data.frames[this.step].labels;
          for (let i = 0; i < labels.length; i++)
            this.#scene.applyLabel(labels[i], this.#view);
        }
      }

      if (automaticMove) {
        const timeSlider = document.getElementById('timeSlider');
        if (timeSlider)
          timeSlider.setValue(100 * this.step / this.data.frames.length);
      }

      this.#previousStep = this.step;
      this.#view.time = this.data.frames[this.step].time;
      const currentTime = document.getElementById('currentTime');
      if (currentTime)
        currentTime.innerHTML = this.#formatTime(this.#view.time);
      WbWorld.instance.viewpoint.updateFollowUp(this.#view.time, !automaticMove || this.step === 0);
      this.#scene.render();
    }

    if (typeof this.stepCallback === 'function')
      this.stepCallback(this.#view.time);
  }

  updateAnimation() {
    if (this.gui === 'real-time') {
      this.updateAnimationState();
      window.requestAnimationFrame(() => this.updateAnimation());
    }
  }

  #parseMillisecondsIntoReadableTime(milliseconds) {
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

  #formatTime(time) {
    if (typeof this.#unusedPrefix === 'undefined') {
      const maxTime = this.data.frames[this.data.frames.length - 1].time;
      if (maxTime < 60000)
        this.#unusedPrefix = 6;
      else if (maxTime < 600000)
        this.#unusedPrefix = 4;
      else if (maxTime < 3600000)
        this.#unusedPrefix = 3;
      else if (maxTime < 36000000)
        this.#unusedPrefix = 1;
    }

    return this.#parseMillisecondsIntoReadableTime(time).substring(this.#unusedPrefix);
  }
}

Animation.sliderDefined = false;
