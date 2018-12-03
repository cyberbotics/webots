// @param container: DOM element in which to create the plot.
// @param basicTimeStep: world basicTimeStep.
// @param autoRange: cf. AutoRangeType.
// @param yRange: initial range. format `{'min': -1.0, 'max': 1.0}`.
// @param labels: format `{'x': "x-axis", 'y': "y-axis"}`.
// @param device: attached device reference.

/* global basicTimeStep: false */

function TimeplotWidget(container, basicTimeStep, autoRange, yRange, labels, device) {
  this.container = container;
  this.basicTimeStep = basicTimeStep;
  this.autoRange = autoRange;
  this.yRange = yRange;
  this.initialYRange = yRange;
  this.labels = labels;
  this.device = device;

  this.slider = null;
  this.label = null;

  // Should be hard coded correctly!
  this.canvasWidth = 320;
  this.canvasHeight = 200;

  this.values = [];
  this.lastX = 0;
  this.lastY = 0;
  this.initialized = false;
  this.shown = true;
  this.canvas = null;
  this.canvasContext = null;
  this.refreshLabelsRate = 3; // [Hz]
  this.lastLabelRefresh = 0; // [simulated seconds]
  this.blockSliderUpdateFlag = false;

  // Compute y-axis offset.
  // This is used to avoid that points touches the top or bottom bounds of the plot in order to improve the plot readability.
  this.yOffset = {};
  this.yOffset['offset'] = 6;
  this.yOffset['halfOffset'] = this.yOffset['offset'] / 2;
  this.yOffset['ratio'] = (this.canvasHeight - this.yOffset['offset']) / this.canvasHeight;

  // Compute vertical grid steps.
  this.updateGridConstants();

  var that = this;
  setInterval(function() { that.refreshLabels(); }, 1000 / that.refreshLabelsRate);
}

TimeplotWidget.prototype.AutoRangeType = {
  NONE: 1, // Fixed range whatever the input data.
  STRETCH: 2, // If the input data is out of the range, then strech the range to see everything.
  JUMP: 3 // If the input data is out of the range, then the range will jump to the closest range having the same size as the initial range.
};

// @param value: format `{'x': 0.1, 'y': 0.2}` or `{'x': 0.1, 'y': [0.2, 0.3, 0.5]}`.
TimeplotWidget.prototype.addValue = function(value) {
  this.values.push(value);
  if (this.values.length > this.canvasWidth)
    this.values.shift();

  if (this.autoRange === this.AutoRangeType.STRETCH)
    this.stretchRange(value.y);
  else if (this.autoRange === this.AutoRangeType.JUMP)
    this.jumpToRange(value.y);
};

TimeplotWidget.prototype.setSlider = function(slider) {
  this.slider = slider;
};

TimeplotWidget.prototype.setLabel = function(label) {
  this.label = label;
};

TimeplotWidget.prototype.blockSliderUpdate = function(block) {
  this.blockSliderUpdateFlag = block;
};

TimeplotWidget.prototype.initialize = function() {
  var id = this.container.getAttribute('id');

  this.canvas = this.appendChildToContainer('<canvas id="' + id + '-canvas" class="plot-canvas" />');

  // Let the canvas size match with the element size (-2 because of the border), otherwise
  // the sizes are not matching causing a aliased zoom-like effect.
  this.canvas.width = this.canvas.offsetWidth - 2;
  this.canvas.height = this.canvas.offsetHeight - 2;

  this.canvasContext = this.canvas.getContext('2d');
  console.assert(this.canvasWidth === this.canvas.width);
  console.assert(this.canvasHeight === this.canvas.height);

  this.xLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-x">' + this.labels['x'] + '</p>');
  this.xMinLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-x-min">0.0</p>');
  this.xMaxLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-x-max">0.0</p>');

  this.yLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-y">' + this.labels['y'] + '</p>');
  this.yMinLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-y-min">' + roundLabel(this.yRange['min']) + '</p>');
  this.yMaxLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-y-max">' + roundLabel(this.yRange['max']) + '</p>');

  if (this.slider) {
    this.slider.setAttribute('min', this.yRange['min']);
    this.slider.setAttribute('max', this.yRange['max']);
  }

  // fill the grid.
  this.displayHorizontalGrid(0, this.canvasWidth);

  this.initialized = true;

  this.show(this.shown);
};

TimeplotWidget.prototype.refresh = function() {
  // Do nothing if the div is hidden.
  if (this.container.offsetParent === null)
    return;

  // Initialize if required.
  if (!this.initialized)
    this.initialize();

  while (this.values.length > 0) { // foreach point to draw.
    var value = this.values.shift(); // pop first item
    var skip = Math.round((value.x - this.lastX) / basicTimeStep); // number of pixels to skip.

    // blit
    var imageData = this.canvasContext.getImageData(skip, 0, this.canvasWidth - skip, this.canvasHeight);
    this.canvasContext.putImageData(imageData, 0, 0);
    this.canvasContext.clearRect(this.canvasWidth - skip, 0, skip, this.canvasHeight);

    // draw the grid on the blit area.
    this.displayHorizontalGrid(this.canvasWidth - skip, skip);

    // draw the new coordinate.
    this.canvasContext.fillStyle = '#059';
    if (Array.isArray(value.y)) {
      for (var j = 0; j < value.y.length; ++j) {
        this.canvasContext.fillStyle = (j === 0) ? '#A44' : ((j === 1) ? '#4A4' : '#44A');
        this.canvasContext.beginPath();
        this.canvasContext.arc(this.canvasWidth - 1, this.convertYCoordToCanvas(value.y[j]), 1.25, 0, 2.0 * Math.PI);
        this.canvasContext.fill();
      }
    } else {
      if (this.device) {
        this.canvasContext.beginPath();
        this.canvasContext.arc(this.canvasWidth - 1, this.convertYCoordToCanvas(value.y), 1.25, 0, 2.0 * Math.PI);
        this.canvasContext.fill();
      }
    }

    this.lastX = value.x;
    this.lastY = value.y;
  }
};

// Jump to a new y range based on the initial range and the new y value.
TimeplotWidget.prototype.jumpToRange = function(y) {
  console.assert(isNumber(y));
  if (y > this.yRange['max'] || y < this.yRange['min']) {
    var delta = this.initialYRange['max'] - this.initialYRange['min'];
    var rangeLevel = Math.floor(0.5 * Math.floor(y / (0.5 * delta) + 1.0));
    this.yRange['min'] = delta * rangeLevel - 0.5 * delta;
    this.yRange['max'] = delta * rangeLevel + 0.5 * delta;
    this.updateRange();
  }
};

// Stretch the y range based on a new y list.
TimeplotWidget.prototype.stretchRange = function(y) {
  var increaseFactor = 1.5; // Increase 50% more than targeted to reduce plot redraws.
  var changed = false;
  if (Array.isArray(y)) {
    for (var i = 0; i < y.length; ++i) {
      var v = y[i];
      if (v < this.yRange['min']) {
        this.yRange['min'] = increaseFactor * v;
        changed = true;
      } else if (v > this.yRange['max']) {
        this.yRange['max'] = increaseFactor * v;
        changed = true;
      }
    }
  } else {
    console.assert(isNumber(y));
    if (y < this.yRange['min']) {
      this.yRange['min'] = increaseFactor * y;
      changed = true;
    } else if (y > this.yRange['max']) {
      this.yRange['max'] = increaseFactor * y;
      changed = true;
    }
  }
  if (changed)
    this.updateRange();
};

// Propagate a modification applied on this.yRange.
TimeplotWidget.prototype.updateRange = function() {
  if (!this.initialized)
    return;

  this.updateGridConstants();

  if (this.yMinLabel && this.yMaxLabel) {
    this.yMinLabel.textContent = roundLabel(this.yRange['min']);
    this.yMaxLabel.textContent = roundLabel(this.yRange['max']);
  }

  this.canvasContext.clearRect(0, 0, this.canvasWidth, this.canvasHeight);
  this.displayHorizontalGrid(0, this.canvasWidth);

  if (this.slider) {
    this.slider.setAttribute('min', this.yRange['min']);
    this.slider.setAttribute('max', this.yRange['max']);
  }
};

TimeplotWidget.prototype.updateGridConstants = function() {
  var delta = this.yRange['max'] - this.yRange['min'];
  delta *= 0.999; // in order to decrease the order of magnitude if delta is a perfect divider of the increment
  var orderOfMagnitude = Math.floor(Math.log(delta) / Math.LN10);
  this.verticalGridSteps = Math.pow(10, orderOfMagnitude);
};

TimeplotWidget.prototype.show = function(show) {
  this.shown = show;
  if (!this.initialized)
    return;
  if (show) {
    this.canvas.style.visibility = 'visible';
    this.xLabel.style.visibility = 'visible';
    this.xMinLabel.style.visibility = 'visible';
    this.xMaxLabel.style.visibility = 'visible';
    this.yLabel.style.visibility = 'visible';
    this.yMinLabel.style.visibility = 'visible';
    this.yMaxLabel.style.visibility = 'visible';
    if (this.slider)
      this.slider.style.visibility = 'visible';
  } else {
    this.canvas.style.visibility = 'hidden';
    this.xLabel.style.visibility = 'hidden';
    this.xMinLabel.style.visibility = 'hidden';
    this.xMaxLabel.style.visibility = 'hidden';
    this.yLabel.style.visibility = 'hidden';
    this.yMinLabel.style.visibility = 'hidden';
    this.yMaxLabel.style.visibility = 'hidden';
    if (this.slider)
      this.slider.style.visibility = 'hidden';
  }
};

TimeplotWidget.prototype.refreshLabels = function() {
  // Refresh the labels only if the container is visible.
  if (!this.initialized || !this.container.offsetParent)
    return;

  if (this.xMinLabel.textContent === '0.0') // Blitting didn't started: the labels are constant.
    this.xMinLabel.textContent = roundLabel(-this.basicTimeStep * this.canvasWidth);
  if (this.lastLabelRefresh !== this.lastX) {
    // Blitting started: update the x labels.
    this.xMinLabel.textContent = roundLabel(this.lastX - this.basicTimeStep * this.canvasWidth);
    this.xMaxLabel.textContent = roundLabel(this.lastX);
  }
  if (this.slider && !this.blockSliderUpdateFlag)
    this.slider.value = this.lastY;
  if (this.label) {
    var v = this.lastY;
    if (Array.isArray(v))
      this.label.textContent = ': [' + roundLabel(v[0]) + ', ' + roundLabel(v[1]) + ', ' + roundLabel(v[2]) + ']';
    else
      this.label.textContent = ': ' + roundLabel(v);
  }
  this.lastLabelRefresh = this.lastX;
};

TimeplotWidget.prototype.appendChildToContainer = function(child) {
  var tmp = document.createElement('tmp');
  tmp.innerHTML = child;
  this.container.appendChild(tmp.firstChild);
  return this.container.childNodes[this.container.childNodes.length - 1];
};

TimeplotWidget.prototype.convertYCoordToCanvas = function(y) {
  return Math.round(-this.yOffset['halfOffset'] + this.canvasHeight + (y - this.yRange['min']) / (this.yRange['min'] - this.yRange['max']) * this.canvasHeight * this.yOffset['ratio']);
};

TimeplotWidget.prototype.displayHorizontalGrid = function(fromX, nX) {
  for (var i = Math.ceil(this.yRange['min'] / this.verticalGridSteps); i < Math.ceil(this.yRange['max'] / this.verticalGridSteps) + 1; ++i) {
    this.canvasContext.fillStyle = (i === 0) ? '#AAAAAA' : '#DDDDDD';
    this.canvasContext.fillRect(fromX, this.convertYCoordToCanvas(i * this.verticalGridSteps), nX, 1);
  }
};

function roundLabel(value) {
  console.assert(isNumber(value) || value === 'Inf' || value === '-Inf' || value === 'NaN');
  if (isNumber(value))
    return Math.round(value * 100) / 100; // keeps 2 decimals at max.
  else
    return value;
}

function isNumber(n) {
  if (n === 'Inf' || n === '-Inf' || n === 'NaN')
    return true;
  return parseFloat(n) === n;
}
