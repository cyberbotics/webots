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
    if (this.#jsonPromise instanceof Promise) {
      this.#jsonPromise.then(json => this.#setup(json))
        .catch(error => console.error(error));
    } else
      this.#setup(this.#jsonPromise);
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
        const allUpdates = new Map();
        const allLabels = new Map();
        for (let j = (i + 1) * this.keyFrameStepSize; j > i * this.keyFrameStepSize; j--) {
          const updates = this.data.frames[j].updates;
          if (updates) {
            for (let k = 0; k < updates.length; k++) {
              const currentIdFields = allUpdates.has(updates[k].id) ? allUpdates.get(updates[k].id) : new Map();
              for (let field in updates[k]) {
                if (field === 'id' || currentIdFields.has(field))
                  continue;
                else
                  currentIdFields.set(field, {'id': updates[k].id, [field]: updates[k][field]});
              }
              allUpdates.set(updates[k].id, currentIdFields);
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
          const updates = this.data.frames[0].updates;
          if (updates) {
            for (let j = 0; j < updates.length; j++) {
              const currentIdFields = allUpdates.has(updates[j].id) ? allUpdates.get(updates[j].id) : new Map();

              for (let field in updates[j]) {
                if (field === 'id' || currentIdFields.has(field))
                  continue;
                else
                  currentIdFields.set(field, {'id': updates[j].id, [field]: updates[j][field]});
              }
              allUpdates.set(updates[j].id, currentIdFields);
            }
          }
          // add an empty, transparent label for labels that are initialized later.
          if (allLabels.size !== this.#labelsIds.length) {
            for (const labelId of this.#labelsIds) {
              if (!allLabels.has(labelId))
                allLabels.set(labelId, {'id': labelId, 'rgba': '0,0,0,0', 'size': 0, 'x': 0, 'y': 0, 'text': ''});
            }
          }
        } else { // Check the previous keyFrame to get missing updates
          const updates = this.#keyFrames.get(i - 1).updates;
          for (let element of updates) {
            const id = element[0];
            const currentIdFields = allUpdates.has(id) ? allUpdates.get(id) : new Map();

            for (let field of element[1]) {
              if (currentIdFields.has(field[0])) // we want to update each field only once
                continue;
              else
                currentIdFields.set(field[0], field[1]);
            }
            allUpdates.set(id, currentIdFields);
          }

          const labels = this.#keyFrames.get(i - 1).labels;
          for (let label of labels) {
            const id = label[0];
            if (!allLabels.has(id))
              allLabels.set(id, label[1]);
          }
        }

        this.#keyFrames.set(i, {updates: new Map(allUpdates), labels: new Map(allLabels)});
      }
      this.numberOfFields = new Map();
      for (let [key, value] of this.#keyFrames.get(this.numberOfKeyFrames - 1).updates)
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
        let previousUpdateStep;
        const closestKeyFrame = Math.floor(this.step / this.keyFrameStepSize) - 1;

        let previousStepIsAKeyFrame = false;
        if (this.step > this.#previousStep && this.#previousStep > (closestKeyFrame + 1) * this.keyFrameStepSize)
          previousUpdateStep = this.#previousStep;
        else {
          previousUpdateStep = (closestKeyFrame + 1) * this.keyFrameStepSize;
          if (this.#keyFrames.size > 0)
            previousStepIsAKeyFrame = true;
        }

        const completeIds = new Set();
        const appliedFieldsByIds = new Map();
        const appliedLabelsIds = new Set();

        // We do not want to include the previousUpdateStep in the loop as its updates are in the keyFrame.
        // However, we need to include it if there is no keyFrames or if it is the step 0 as there is no keyFrame for it
        if (previousStepIsAKeyFrame && previousUpdateStep !== 0)
          previousUpdateStep++;

        // Iterate through each step until the nearest keyFrame is reached or all necessary updates have been applied.
        // Go in decreasing order to minize the number of steps.
        for (let i = this.step; i >= previousUpdateStep; i--) {
          if (this.data.frames[i].updates) {
            for (let j = 0; j < this.data.frames[i].updates.length; j++) { // At each frame, apply all updates
              const id = this.data.frames[i].updates[j].id;
              if (!completeIds.has(id)) { // Try to apply some updates to a node only if it is missing some
                for (let field in this.data.frames[i].updates[j]) {
                  if (!appliedFieldsByIds.has(id)) {
                    appliedFieldsByIds.set(id, new Set());
                    appliedFieldsByIds.get(id).add('id');
                  }

                  if (appliedFieldsByIds.get(id).has(field)) // we want to update each field only once
                    continue;
                  else {
                    this.#scene.applyUpdate({'id': id, [field]: this.data.frames[i].updates[j][field]});
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
          const updates = this.#keyFrames.get(closestKeyFrame).updates;
          for (let element of updates) {
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
                  this.#scene.applyUpdate(field[1]);
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
        if (this.data.frames[this.step].hasOwnProperty('updates')) {
          const updates = this.data.frames[this.step].updates;
          for (let p = 0; p < updates.length; p++)
            this.#scene.applyUpdate(updates[p], undefined);
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
