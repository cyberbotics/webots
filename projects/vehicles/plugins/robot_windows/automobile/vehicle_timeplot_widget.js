/* global TimeplotWidget: false */
/* global roundLabel: false */
/* global isNumber: false */

// @param container: DOM element in which to create the plot.
// @param basicTimeStep: world basicTimeStep.
// @param autoRange: cf. AutoRangeType.
// @param yRange: initial range. format `{'min': -1.0, 'max': 1.0}`.
// @param labels: format `{'x': "x-axis", 'y': "y-axis"}`.
// @param device: attached device reference.
// @param decimals: number of decimals used in the pl0t.
function VehicleTimeplotWidget(container, basicTimeStep, autoRange, yRange, labels, device, decimals = 3) {
  this.timeplot = new TimeplotWidget(container, basicTimeStep, autoRange, yRange, labels, device, decimals);
  this.timeplot.stylePrefix = 'vehicle-';
  this.firstUpdate = true;
  this.device = device;
  this.refreshLabelsRate = 3; // Hz

  var that = this;
  setInterval(function() { that.refreshLabels(); }, 1000 / that.refreshLabelsRate);
}

VehicleTimeplotWidget.prototype.applyVehicleStyle = function(container) {
  const legend = this.timeplot.labels['legend'];
  const labelsHeight = legend ? 20 * legend.length : 0;
  const classPrefix = 'vehicle-';
  let classes = [
    {'name': '.vehicle-device', 'heightOffset': labelsHeight},
    {'name': '.vehicle-device-content', 'heightOffset': labelsHeight},
    {'name': '.plot-canvas', 'topOffset': labelsHeight - 10},
    {'name': '.plot-axis-label-x', 'topOffset': labelsHeight},
    {'name': '.plot-axis-label-x-min', 'topOffset': labelsHeight},
    {'name': '.plot-axis-label-x-max', 'topOffset': labelsHeight},
    {'name': '.plot-axis-label-y', 'topOffset': labelsHeight},
    {'name': '.plot-axis-label-y-min', 'topOffset': labelsHeight},
    {'name': '.plot-axis-label-y-max', 'topOffset': labelsHeight}
  ];
  const layout = document.getElementById(this.device.name + '-layout');
  for (let c = 0; c < classes.length; c++) {
    let element = layout.querySelector(classes[c].name);
    if (!classes[c].name.startsWith('.' + classPrefix))
      element.classList.add(classPrefix + classes[c].name.substring(1));
    if (classes[c].heightOffset) {
      const height = parseFloat(window.getComputedStyle(element).getPropertyValue('height').replace(/px$/g, ''));
      element.style.height = (height + classes[c].heightOffset) + 'px';
    }
    if (classes[c].topOffset) {
      const top = parseFloat(window.getComputedStyle(element).getPropertyValue('top').replace(/px$/g, ''));
      element.style.top = (top + classes[c].topOffset) + 'px';
    }
  }
};

VehicleTimeplotWidget.prototype.initialize = function() {
  this.timeplot.initialize();
  this.applyVehicleStyle();
  this.timeplot.computeCanvasSize();
};

VehicleTimeplotWidget.prototype.refresh = function() {
  // Do nothing if the div is hidden.
  if (this.timeplot.container.offsetParent === null)
    return;

  // Initialize if required.
  if (!this.timeplot.initialized)
    this.initialize();

  this.timeplot.refresh();
};

VehicleTimeplotWidget.prototype.refreshLabels = function() {
  // Refresh the labels only if the container is visible.
  if (!this.timeplot.initialized || !this.timeplot.container.offsetParent)
    return;

  this.timeplot.refreshLabels();

  const v = this.timeplot.lastY;
  if (this.label && Array.isArray(v)) {
    const legend = this.timeplot.labels['legend'];
    let text = ': [';
    for (let i = 0; i < v.length; i++) {
      if (i > 0)
        text += ', ';
      if (legend)
        text += '<br>&emsp;&emsp;';
      text += '<span style="color:' + this.timeplot.labelColorByIndex(i) + '">';
      if (legend && legend.length > i)
        text += ' ' + legend[i] + ': ';
      text += roundLabel(v[i], this.timeplot.decimals) + '</span>';
    }
    if (legend)
      text += '<br>&emsp;&emsp;';
    this.timeplot.label.innerHTML = text + ']';
  }
};

VehicleTimeplotWidget.prototype.setLabel = function(label) {
  this.timeplot.setLabel(label);
};

// @param value: format `{'x': 0.1, 'y': 0.2}` or `{'x': 0.1, 'y': [0.2, 0.3, 0.5]}`.
VehicleTimeplotWidget.prototype.addValue = function(value) {
  this.timeplot.addValue(value);
};

VehicleTimeplotWidget.prototype.resize = function() {
  if (this.timeplot && this.timeplot.resize === 'function')
    this.timeplot.resize();
};

// Jump to a new y range based on the initial range and the new y value.
TimeplotWidget.prototype.jumpToRange = function(y) {
  let value;
  if (Array.isArray(y)) {
    let max = y[0];
    let min = max;
    for (let i = 1; i < y.length; i++) {
      if (y[i] < min)
        min = y[i];
      else if (y[i] > max)
        max = y[i];
    }
    if (max > this.yRange['max'])
      value = max;
    else if (min < this.yRange['min'])
      value = min;
    if (value) {
      if (this.delta < (max - min))
        this.delta = (max - min) * 2;
    }
  } else {
    console.assert(isNumber(y));
    if (y > this.yRange['max'] || y < this.yRange['min'])
      value = y;
  }
  if (value) {
    const rangeLevel = Math.floor(value / (0.5 * this.delta) + 0.5);
    this.yRange['min'] = 0.5 * this.delta * (rangeLevel - 1.0);
    this.yRange['max'] = 0.5 * this.delta * (rangeLevel + 1.0);
    this.updateRange();
  }
};
