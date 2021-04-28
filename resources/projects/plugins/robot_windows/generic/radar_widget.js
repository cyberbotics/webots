// @param container: DOM element in which to create the widget.
function RadarWidget(container, device) {
  this.container = container;
  this.device = device;
  this.fieldOfView = device.fieldOfView;
  this.maxRange = device.maxRange;

  // Should be hard coded correctly!
  this.canvasWidth = 320;
  this.canvasHeight = 200;

  this.Targets = [];

  this.initialized = false;
  this.canvas = null;
  this.canvasContext = null;
}

RadarWidget.recordDataInBackground = false;

RadarWidget.prototype.addTarget = function(x, y) {
  if (!RadarWidget.recordDataInBackground && this.container.offsetParent === null)
    return;

  this.Targets.push([x, y]);
};

RadarWidget.prototype.initialize = function() {
  const id = this.container.getAttribute('id');

  this.canvas = this.appendChildToContainer('<canvas id="' + id + '-canvas" class="plot-canvas plot-canvas-background" />');

  // Let the canvas size match with the element size (-2 because of the border), otherwise
  // the sizes are not matching causing a aliased zoom-like effect.
  this.canvas.width = this.canvas.offsetWidth - 2;
  this.canvas.height = this.canvas.offsetHeight - 2;

  this.canvasContext = this.canvas.getContext('2d');
  console.assert(this.canvasWidth === this.canvas.width);
  console.assert(this.canvasHeight === this.canvas.height);

  this.initialized = true;
};

RadarWidget.prototype.drawBackground = function() {
  this.canvasContext.fillStyle = '#000';
  const radius = 0.5 * Math.min(this.canvasWidth, this.canvasHeight);
  const centerX = 0.5 * this.canvasWidth;
  const centerY = 0.5 * this.canvasHeight;

  this.canvasContext.clearRect(0, 0, this.canvasWidth, this.canvasHeight);

  this.canvasContext.beginPath();
  this.canvasContext.moveTo(centerX + radius * Math.sin(0.5 * this.fieldOfView), centerY - radius * Math.cos(0.5 * this.fieldOfView));
  this.canvasContext.lineTo(centerX, centerY);
  this.canvasContext.lineTo(centerX - radius * Math.sin(0.5 * this.fieldOfView), centerY - radius * Math.cos(0.5 * this.fieldOfView));
  this.canvasContext.arc(
    centerX,
    centerY,
    radius,
    1.5 * Math.PI - 0.5 * this.fieldOfView,
    1.5 * Math.PI + 0.5 * this.fieldOfView
  );
  this.canvasContext.stroke();
};

RadarWidget.prototype.refreshTargets = function(targets) {
  // Do nothing if the div is hidden.
  if (this.container.offsetParent === null)
    return;

  // Initialize if required.
  if (!this.initialized)
    this.initialize();

  this.drawBackground();

  const centerX = 0.5 * this.canvasWidth;
  const centerY = 0.5 * this.canvasHeight;
  const radius = 0.5 * Math.min(this.canvasWidth, this.canvasHeight);
  this.canvasContext.fillStyle = '#059';
  for (let t = 0; t < targets.length; t++) {
    const target = targets[t];
    this.canvasContext.beginPath();
    this.canvasContext.arc(
      centerX + radius * Math.sin(target.azimuth) * target.distance / this.maxRange,
      centerY - radius * Math.cos(target.azimuth) * target.distance / this.maxRange,
      2.0, 0, 2.0 * Math.PI
    );
    this.canvasContext.fill();
  }
};

RadarWidget.prototype.refresh = function() {
  // Do nothing if the div is hidden.
  if (this.container.offsetParent === null)
    return;

  // Initialize if required.
  if (!this.initialized) {
    this.initialize();
    this.drawBackground();
  }
};

RadarWidget.prototype.appendChildToContainer = function(child) {
  let tmp = document.createElement('tmp');
  tmp.innerHTML = child;
  this.container.appendChild(tmp.firstChild);
  return this.container.childNodes[this.container.childNodes.length - 1];
};
