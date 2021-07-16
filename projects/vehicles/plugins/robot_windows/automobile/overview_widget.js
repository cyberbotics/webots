/* global appendNewElement: false */
/* global roundLabel: false */

function OverviewWidget(container) {
  this.device = {'type': 'overview'};
  this.canvas = document.getElementById('overview-canvas');
  this.container = container;
  this.initialized = false;
  this.enabled = false;
  this.isTorqueControl = false;
  this.steering = [0, 0, 0];
  this.isUndefinedMode = false;
  this.isTorqueMode = false;

  let labels = document.getElementById('overview-labels');
  this.fontSize = parseFloat(window.getComputedStyle(labels, null).getPropertyValue('font-size'));
  // Show overview div
  let overview = document.getElementById('overview-section');
  overview.style.display = '';
  // Add overview menu button
  appendNewElement('menu',
    '<button id="overview-menu-button" class="menu-button tablink" onclick="menuTabCallback(\'overview\')">Overview</button>'
  );

  this.modified = true;
}

OverviewWidget.prototype.refreshLabels = function() {
  this.update();
};

OverviewWidget.prototype.refresh = function() {
  this.paint();
};

OverviewWidget.prototype.initialize = function() {
  this.initialized = true;
  this.resize();
  let labels = document.getElementById('overview-layout');
  labels.style.display = 'block'; // use display none to hide labels before they are positioned
};

OverviewWidget.prototype.resize = function() {
  if (!this.initialized) {
    this.initialize();
    return;
  }

  // Set minimum size
  this.canvas.width = Math.max(300, this.canvas.parentNode.offsetWidth - 100);
  this.canvas.height = 400;

  // Compute positions
  this.wheelWidth = 20;
  this.wheelHeight = 50;
  this.centerPosition = [this.canvas.width / 2 + 50, 80 + (this.canvas.height - 80) / 2];
  this.frontAxisCenterPosition = [this.centerPosition[0], this.centerPosition[1] - this.canvas.height / 3];
  this.rearAxisCenterPosition = [this.centerPosition[0], this.centerPosition[1] + this.canvas.height / 3];
  this.frontAxisRightPosition = [this.frontAxisCenterPosition[0] + this.canvas.width / 6, this.frontAxisCenterPosition[1]];
  this.frontAxisLeftPosition = [this.frontAxisCenterPosition[0] - this.canvas.width / 6, this.frontAxisCenterPosition[1]];
  this.rearAxisRightPosition = [this.rearAxisCenterPosition[0] + this.canvas.width / 6, this.rearAxisCenterPosition[1]];
  this.rearAxisLeftPosition = [this.rearAxisCenterPosition[0] - this.canvas.width / 6, this.rearAxisCenterPosition[1]];

  // Initialize labels and positions
  var label = document.getElementById('overview-wheel1-radius-label');
  label.parentNode.style.top = (this.frontAxisRightPosition[1] + this.wheelHeight / 2 + 2) + 'px';
  label.parentNode.style.left = (this.frontAxisRightPosition[0] - this.wheelWidth - 10) + 'px';

  label = document.getElementById('overview-wheel2-radius-label');
  label.parentNode.style.top = (this.frontAxisLeftPosition[1] + this.wheelHeight / 2 + 2) + 'px';
  label.parentNode.style.left = (this.frontAxisLeftPosition[0] - this.wheelWidth - 40) + 'px';

  label = document.getElementById('overview-wheel3-radius-label');
  label.parentNode.style.top = (this.rearAxisRightPosition[1] - this.wheelHeight / 2 - this.fontSize - 2) + 'px';
  label.parentNode.style.left = (this.rearAxisRightPosition[0] - this.wheelWidth - 10) + 'px';

  label = document.getElementById('overview-wheel4-radius-label');
  label.parentNode.style.top = (this.rearAxisLeftPosition[1] - this.wheelHeight / 2 - this.fontSize - 2) + 'px';
  label.parentNode.style.left = (this.rearAxisLeftPosition[0] - this.wheelWidth - 40) + 'px';

  label = document.getElementById('overview-engine-label');
  label.parentNode.parentNode.style.top = this.centerPosition[1] + 'px';
  label.parentNode.parentNode.style.left = ((this.centerPosition[0] + 2 * this.frontAxisRightPosition[0]) / 3) + 'px';

  label = document.getElementById('overview-rpm-label');
  label.parentNode.parentNode.style.top = this.centerPosition[1] + 'px';
  label.parentNode.parentNode.style.left = (this.centerPosition[0] - 200) + 'px';

  label = document.getElementById('overview-front-track-label');
  label.parentNode.style.top = (this.frontAxisCenterPosition[1] + 5) + 'px';
  label.parentNode.style.left = (this.frontAxisCenterPosition[0] + 5) + 'px';

  label = document.getElementById('overview-rear-track-label');
  label.parentNode.style.top = (this.rearAxisCenterPosition[1] + 5) + 'px';
  label.parentNode.style.left = (this.rearAxisCenterPosition[0] - 20) + 'px';

  label = document.getElementById('overview-wheelbase-label');
  label.parentNode.style.top = (this.centerPosition[1] - 30) + 'px';
  label.parentNode.style.left = this.centerPosition[0] + 'px';

  label = document.getElementById('overview-speed-label');
  label.parentNode.style.top = '0 px';
  label.parentNode.style.left = (this.centerPosition[0] - 150) + 'px';

  label = document.getElementById('overview-steering-label');
  label.parentNode.style.top = '0 px';
  label.parentNode.style.left = (this.centerPosition[0] + 40) + 'px';

  label = document.getElementById('overview-wheel1-speed-label');
  label.parentNode.parentNode.style.top = (this.frontAxisRightPosition[1] - 30) + 'px';
  label.parentNode.parentNode.style.left = (this.frontAxisRightPosition[0] + 40) + 'px';

  label = document.getElementById('overview-wheel2-speed-label');
  label.parentNode.parentNode.style.top = (this.frontAxisLeftPosition[1] - 30) + 'px';
  label.parentNode.parentNode.style.left = (this.frontAxisLeftPosition[0] - 150) + 'px';

  label = document.getElementById('overview-wheel3-speed-label');
  label.parentNode.parentNode.style.top = (this.rearAxisRightPosition[1] - 15) + 'px';
  label.parentNode.parentNode.style.left = (this.rearAxisRightPosition[0] + 40) + 'px';

  label = document.getElementById('overview-wheel4-speed-label');
  label.parentNode.parentNode.style.top = (this.rearAxisLeftPosition[1] - 15) + 'px';
  label.parentNode.parentNode.style.left = (this.rearAxisLeftPosition[0] - 150) + 'px';

  this.paint();
};

OverviewWidget.prototype.paint = function() {
  // Do nothing if the div is hidden.
  if (this.container.offsetParent === null)
    return;

  if (!this.initialized)
    this.initialize();

  const ctx = this.canvas.getContext('2d');
  ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

  // Draw black vehicle axes
  this.drawLine(ctx, this.frontAxisCenterPosition[0], this.frontAxisCenterPosition[1], this.rearAxisCenterPosition[0], this.rearAxisCenterPosition[1]);
  this.drawLine(ctx, this.frontAxisLeftPosition[0], this.frontAxisLeftPosition[1], this.frontAxisRightPosition[0], this.frontAxisRightPosition[1]);
  this.drawLine(ctx, this.rearAxisLeftPosition[0], this.rearAxisLeftPosition[1], this.rearAxisRightPosition[0], this.rearAxisRightPosition[1]);
  // Draw steering dashed arrow
  this.drawDashedArrow(ctx, this.frontAxisCenterPosition[0], this.frontAxisCenterPosition[1], 0, -75, this.steering[0]);
  // Draw a wheel rectangles
  const startX = -this.wheelWidth / 2;
  const startY = -this.wheelHeight / 2;
  this.drawRotatedRectangle(ctx, this.frontAxisRightPosition[0] + startX, this.frontAxisRightPosition[1] + startY, 20, 50, this.steering[1]);
  this.drawRotatedRectangle(ctx, this.frontAxisLeftPosition[0] + startX, this.frontAxisLeftPosition[1] + startY, 20, 50, this.steering[2]);
  this.drawRotatedRectangle(ctx, this.rearAxisRightPosition[0] + startX, this.rearAxisRightPosition[1] + startY, 20, 50, 0);
  this.drawRotatedRectangle(ctx, this.rearAxisLeftPosition[0] + startX, this.rearAxisLeftPosition[1] + startY, 20, 50, 0);
};

OverviewWidget.prototype.setStaticInformation = function(data) {
  document.getElementById('overview-wheel1-radius-label').textContent = roundLabel(data['front-wheel-radius'], 3);
  document.getElementById('overview-wheel2-radius-label').textContent = roundLabel(data['front-wheel-radius'], 3);
  document.getElementById('overview-wheel3-radius-label').textContent = roundLabel(data['rear-wheel-radius'], 3);
  document.getElementById('overview-wheel4-radius-label').textContent = roundLabel(data['rear-wheel-radius'], 3);
  document.getElementById('overview-engine-label').textContent = data.engine;
  document.getElementById('overview-transmission-label').textContent = data.transmission;
  document.getElementById('overview-gear-number-label').textContent = roundLabel(data['gear-number'], 1);
  document.getElementById('overview-front-track-label').textContent = roundLabel(data['front-track'], 3);
  document.getElementById('overview-rear-track-label').textContent = roundLabel(data['rear-track'], 3);
  document.getElementById('overview-wheelbase-label').textContent = roundLabel(data['wheelbase'], 3);
};

OverviewWidget.prototype.updateInformation = function(data) {
  this.data = data;
  this.steering = this.data.steering;
  this.modified = true;
};

OverviewWidget.prototype.update = function() {
  if (!this.modified || !this.data)
    return;
  if (!this.initialized)
    this.initialize();

  if (this.isUndefinedMode) {
    document.getElementById('overview-speed-label').textContent = '-';
    document.getElementById('overview-target-speed-label').textContent = '-';
  } else {
    document.getElementById('overview-speed-label').textContent = roundLabel(this.data.speed, 2);
    document.getElementById('overview-target-speed-label').textContent = roundLabel(this.data['target-speed'], 3);
  }
  document.getElementById('overview-steering-label').textContent = roundLabel(this.data.steering[0], 4);
  document.getElementById('overview-rpm-label').textContent = roundLabel(this.data.rpm, 3);
  document.getElementById('overview-gearbox-label').textContent = roundLabel(this.data.gearbox, 1);
  document.getElementById('overview-wheel1-speed-label').textContent = roundLabel(this.data.wheel1.speed, 1);
  document.getElementById('overview-wheel1-encoder-label').textContent = this.data.wheel1.encoder.toExponential(1);
  document.getElementById('overview-wheel1-angle-label').textContent = roundLabel(this.data.steering[1], 4);
  document.getElementById('overview-wheel2-speed-label').textContent = roundLabel(this.data.wheel2.speed, 1);
  document.getElementById('overview-wheel2-encoder-label').textContent = this.data.wheel2.encoder.toExponential(1);
  document.getElementById('overview-wheel2-angle-label').textContent = roundLabel(this.data.steering[2], 4);
  document.getElementById('overview-wheel3-speed-label').textContent = roundLabel(this.data.wheel3.speed, 1);
  document.getElementById('overview-wheel3-encoder-label').textContent = this.data.wheel3.encoder.toExponential(1);
  document.getElementById('overview-wheel4-speed-label').textContent = roundLabel(this.data.wheel4.speed, 1);
  document.getElementById('overview-wheel4-encoder-label').textContent = this.data.wheel4.encoder.toExponential(1);
  this.modified = false;
  this.paint();
};

OverviewWidget.prototype.updateControlMode = function(isSpeedMode, isTorqueMode) {
  this.isUndefinedMode = !isTorqueMode && !isSpeedMode;
  this.isTorqueMode = isTorqueMode;
  if (isTorqueMode) {
    document.getElementById('overview-rpm-label').parentNode.style.display = 'block';
    document.getElementById('overview-gearbox-label').parentNode.style.display = 'block';
    document.getElementById('overview-target-speed-label').parentNode.style.display = 'none';
  } else {
    document.getElementById('overview-rpm-label').parentNode.style.display = 'none';
    document.getElementById('overview-gearbox-label').parentNode.style.display = 'none';
    document.getElementById('overview-target-speed-label').parentNode.style.display = 'block';
  }
};

OverviewWidget.prototype.drawRotatedRectangle = function(ctx, x, y, width, height, angle) {
  // Save the untranslated/unrotated context
  ctx.save();

  ctx.beginPath();
  ctx.translate(x + width / 2, y + height / 2);
  ctx.rotate(angle); // rad
  ctx.rect(-width / 2, -height / 2, width, height);
  ctx.lineWidth = '2';
  ctx.strokeStyle = 'blue';
  ctx.stroke();

  // Restore the context to its untranslated/unrotated state
  ctx.restore();
};

OverviewWidget.prototype.drawLine = function(ctx, x1, y1, x2, y2, color) {
  // Save the untranslated/unrotated context
  ctx.save();

  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.lineWidth = '4';
  ctx.strokeStyle = 'black';
  ctx.stroke();

  // Restore the context to its untranslated/unrotated state
  ctx.restore();
};

OverviewWidget.prototype.drawDashedArrow = function(ctx, x1, y1, x2, y2, angle) {
  // Save the untranslated/unrotated context
  ctx.save();

  ctx.setLineDash([7, 3]); // dashes are 7px and spaces are 3px
  ctx.lineWidth = '2';
  ctx.strokeStyle = 'red';
  ctx.beginPath();
  ctx.translate(x1, y1);
  ctx.rotate(angle); // rad
  // Draw line
  ctx.moveTo(0, 0);
  ctx.lineTo(x2, y2);
  // Draw arrow
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 + 8, y2 + 12);
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - 8, y2 + 12);
  ctx.stroke();

  // Restore the context to its untranslated/unrotated state
  ctx.restore();
};
