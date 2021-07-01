// @param container: DOM element in which to create the plot.
// @param autoRange: if true, the ranges are doubled each time a value exceed the range.
// @param indices: indices to be taken from the input data. format: `{'x': 0, 'y': 2}`
// @param x/yRange: initial ranges. format `{'min': -1.0, 'max': 1.0}`.
// @param labels: format `{'x': "x-axis", 'y': "y-axis"}`.
// @param device: attached device reference.

/* global roundLabel: false */

function PlotWidget(container, autoRange, indices, xRange, yRange, labels, device) {
  this.container = container;
  this.autoRange = autoRange;
  this.indices = indices;
  this.xRange = xRange;
  this.yRange = yRange;
  this.labels = labels;
  this.device = device;

  // Should be hard coded correctly!
  this.canvasWidth = 320;
  this.canvasHeight = 200;

  this.values = [];

  this.initialized = false;
  this.shown = true;
  this.canvas = null;
  this.canvasContext = null;
}

PlotWidget.recordDataInBackground = false;

// @param value: format `{'x': 0.1, 'y': [0.2, 0.3, 0.5]}`.
PlotWidget.prototype.addValue = function(value) {
  if (!PlotWidget.recordDataInBackground && this.container.offsetParent === null)
    return;

  const x = value.y[this.indices['x']];
  const y = value.y[this.indices['y']];
  this.values.push([x, y]);
  while (this.autoRange && (x > this.xRange['max'] || x < this.xRange['min'] || y > this.yRange['max'] || y < this.yRange['min']))
    this.doubleRange();
};

PlotWidget.prototype.initialize = function() {
  const id = this.container.getAttribute('id');

  this.canvas = this.appendChildToContainer('<canvas id="' + id + '-canvas" class="plot-canvas plot-canvas-background" />');
  this.xLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-x">' + this.labels['x'] + '</p>');
  this.xMinLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-x-min">' + roundLabel(this.xRange['min']) + '</p>');
  this.xMaxLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-x-max">' + roundLabel(this.xRange['max']) + '</p>');
  this.yLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-y">' + this.labels['y'] + '</p>');
  this.yMinLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-y-min">' + roundLabel(this.yRange['min']) + '</p>');
  this.yMaxLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-y-max">' + roundLabel(this.yRange['max']) + '</p>');

  // Let the canvas size match with the element size (-2 because of the border), otherwise
  // the sizes are not matching causing a aliased zoom-like effect.
  this.canvas.width = this.canvas.offsetWidth - 2;
  this.canvas.height = this.canvas.offsetHeight - 2;

  this.canvasContext = this.canvas.getContext('2d');
  console.assert(this.canvasWidth === this.canvas.width);
  console.assert(this.canvasHeight === this.canvas.height);

  this.initialized = true;

  this.show(this.shown);
};

PlotWidget.prototype.refresh = function() {
  // Do nothing if the div is hidden.
  if (!this.shown || this.container.offsetParent === null)
    return;

  // Initialize if required.
  if (!this.initialized)
    this.initialize();

  while (this.values.length > 0) { // foreach point to draw.
    var value = this.values.shift(); // pop first item
    this.canvasContext.beginPath();
    this.canvasContext.arc(this.convertXCoordToCanvas(value[0]), this.convertYCoordToCanvas(value[1]), 1.25, 0, 2.0 * Math.PI);
    this.canvasContext.fill();
  }
};

PlotWidget.prototype.doubleRange = function() {
  this.xRange['min'] *= 2.0;
  this.xRange['max'] *= 2.0;
  this.yRange['min'] *= 2.0;
  this.yRange['max'] *= 2.0;

  if (!this.initialized)
    return;

  this.xMinLabel.textContent = roundLabel(this.xRange['min']);
  this.xMaxLabel.textContent = roundLabel(this.xRange['max']);
  this.yMinLabel.textContent = roundLabel(this.yRange['min']);
  this.yMaxLabel.textContent = roundLabel(this.yRange['max']);

  // downscale the current image at the center.
  const srcWidth = this.canvasWidth;
  const srcImg = this.canvasContext.getImageData(0, 0, this.canvasWidth, this.canvasHeight);
  const dstWidth = 0.5 * this.canvasWidth;
  const dstHeight = 0.5 * this.canvasHeight;
  let dstImg = this.canvasContext.createImageData(dstWidth, dstHeight);
  for (let y = 0; y < dstHeight; ++y) {
    for (let x = 0; x < dstWidth; ++x) {
      for (let c = 0; c < 4; ++c)
        dstImg.data[4 * (x + y * dstWidth) + c] = srcImg.data[8 * (x + y * srcWidth) + c];
    }
  }

  this.canvasContext.clearRect(0, 0, this.canvasWidth, this.canvasHeight);

  this.canvasContext.putImageData(dstImg, this.canvasWidth / 4.0, this.canvasHeight / 4.0);
};

PlotWidget.prototype.show = function(show) {
  this.shown = show;
  if (!this.initialized)
    return;
  const visibility = this.shown ? 'visible' : 'hidden';
  this.canvas.style.visibility = visibility;
  this.xLabel.style.visibility = visibility;
  this.xMinLabel.style.visibility = visibility;
  this.xMaxLabel.style.visibility = visibility;
  this.yLabel.style.visibility = visibility;
  this.yMinLabel.style.visibility = visibility;
  this.yMaxLabel.style.visibility = visibility;
};

PlotWidget.prototype.convertXCoordToCanvas = function(x) {
  return Math.round(-(x - this.xRange['min']) / (this.xRange['min'] - this.xRange['max']) * this.canvasWidth);
};

PlotWidget.prototype.convertYCoordToCanvas = function(y) {
  return Math.round(this.canvasHeight + (y - this.yRange['min']) / (this.yRange['min'] - this.yRange['max']) * this.canvasHeight);
};

PlotWidget.prototype.appendChildToContainer = function(child) {
  let tmp = document.createElement('tmp');
  tmp.innerHTML = child;
  this.container.appendChild(tmp.firstChild);
  return this.container.childNodes[this.container.childNodes.length - 1];
};
