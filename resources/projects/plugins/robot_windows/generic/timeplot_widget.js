// @param container: DOM element in which to create the plot.
// @param basicTimeStep: world basicTimeStep.
// @param autoRange: cf. AutoRangeType.
// @param yRange: initial range. format `{'min': -1.0, 'max': 1.0}`.
// @param labels: format `{'x': "x-axis", 'y': "y-axis"}`.
// @param device: attached device reference.

function TimeplotLine(color, maxNumberOfPoints) {
  this.color = color;
  this.maxNumberOfPoints = maxNumberOfPoints;
  this.nextIndex = 0;
  this.size = 0;
  this.xy = new Float32Array(2 * this.maxNumberOfPoints);
};

TimeplotLine.prototype.addPoint = function(x, y) {
  if (this.nextIndex >= this.maxNumberOfPoints)
    this.shift();
  this.xy[this.nextIndex * 2] = x;
  this.xy[this.nextIndex * 2 + 1] = y;
  this.size += 1;
  this.nextIndex += 1;
};

TimeplotLine.prototype.shift = function() {
  const maxIndex = this.size * 2;
  for (let i = 2; i < maxIndex; i += 2) {
    this.xy[i - 2] = this.xy[i];
    this.xy[i - 1] = this.xy[i + 1];
  }
  this.nextIndex -= 1;
  this.size -= 1;
};

TimeplotLine.prototype.lastX = function() {
  if (this.nextIndex <= 0)
    return null;
  return this.xy[this.nextIndex * 2 - 2];
};

TimeplotLine.prototype.lastY = function() {
  if (this.nextIndex <= 0)
    return null;
  return this.xy[this.nextIndex * 2 - 1];
};

TimeplotLine.prototype.resize = function(maxNumberOfPoints) {
  if (this.maxNumberOfPoints >= maxNumberOfPoints)
    return;
  this.maxNumberOfPoints = maxNumberOfPoints;
  const previousArray = this.xy;
  this.xy = new Float32Array(2 * this.maxNumberOfPoints);
  const diff = this.size - maxNumberOfPoints;
  let j = 0;
  if (diff > 0) {
    j = diff * 2;
    this.size = maxNumberOfPoints;
  }
  const nPoints = this.size * 2;
  for (let i = 0; i < nPoints; ++i, ++j)
    this.xy[i] = previousArray[j];
};

function TimeplotWidget(container, basicTimeStep, autoRange, yRange, labels, device, decimals = 3) {
  this.container = container;
  this.basicTimeStep = basicTimeStep;
  this.autoRange = autoRange;
  this.yRange = yRange;
  this.initialYRange = yRange;
  this.labels = labels;
  this.device = device;
  this.decimals = decimals;

  this.initialized = false;
  this.shown = true;
  this.values = []; // store values before the plot is initialized

  // Refresh parameters
  this.lastX = 0;
  this.lastY = [];
  this.refreshLabelsRate = 3; // [Hz]
  this.lastLabelRefresh = 0; // [simulated seconds]
  this.blockSliderUpdateFlag = false;
  this.pendingSliderPositionUpdate = false;

  this.canvas = null;
  // Canvas size computed during initialization
  this.canvasWidth = undefined;
  this.canvasHeight = undefined;
  // WebGL
  this.glContext = null;
  this.shaderProgram = null;
  this.scaleUniformLocation = null;
  this.offsetUniformLocation = null;
  this.pointSizeUniformLocation = null;
  this.colorUniformLocation = null;

  // Plot
  this.lines = []; // array of TimeplotLine
  this.horizontalGrid = null; // dictionary representing the horizontal grid lines
  this.xOffset = 0;
  this.xRangeSize = 0;

  this.slider = null;
  this.label = null;

  // Compute vertical grid steps.
  this.updateGridConstants();

  var that = this;
  setInterval(function() { that.refreshLabels(); }, 1000 / that.refreshLabelsRate);
}

TimeplotWidget.recordDataInBackground = false;

TimeplotWidget.prototype.AutoRangeType = {
  NONE: 1, // Fixed range whatever the input data.
  STRETCH: 2, // If the input data is out of the range, then strech the range to see everything.
  JUMP: 3 // If the input data is out of the range, then the range will jump to the closest range having the same size as the initial range.
};

TimeplotWidget.prototype.FRAGMENT_SHADER = `
  precision mediump float;
  uniform vec3 color;
  void main(void) {
    gl_FragColor = vec4(color, 1.0);
  }`;

TimeplotWidget.prototype.VERTEX_SHADER = `
  attribute vec2 coordinates;
  uniform mat2 scale;
  uniform vec2 offset;
  uniform float pointSize;
  void main() {
    gl_Position = vec4(scale * (coordinates - offset) - vec2(1.0, 1.0),  0.0, 1.0);
    gl_PointSize = pointSize;
  }`;

// @param value: format `{'x': 0.1, 'y': 0.2}` or `{'x': 0.1, 'y': [0.2, 0.3, 0.5]}`.
TimeplotWidget.prototype.addValue = function(value) {
  if (!TimeplotWidget.recordDataInBackground && this.container.offsetParent === null)
    return;

  if (this.initialized) {
    const isArray = Array.isArray(value.y);
    const size = isArray ? value.y.length : 1;
    if (this.lines.length < size)
      this.createLines(size);

    const y = isArray ? value.y : [value.y];
    for (let i = 0; i < size; ++i)
      this.lines[i].addPoint(value.x, y[i]);
  } else {
    this.values.push(value);
    if (this.values.length > this.canvasWidth)
      this.values.shift();
  }

  const newOffset = value.x - this.xRangeSize;
  if (newOffset > this.xOffset)
    this.xOffset = newOffset;

  if (this.autoRange === this.AutoRangeType.STRETCH)
    this.stretchRange(value.y);
  else if (this.autoRange === this.AutoRangeType.JUMP)
    this.jumpToRange(value.y);
};

TimeplotWidget.prototype.setSlider = function(slider) {
  if (slider !== null && this.slider === null) {
    let that = this;
    window.addEventListener('resize', function() { that.resize(); });
  }
  this.slider = slider;
};

TimeplotWidget.prototype.setLabel = function(label) {
  this.label = label;
};

TimeplotWidget.prototype.blockSliderUpdate = function(block) {
  this.blockSliderUpdateFlag = block;
};

TimeplotWidget.prototype.loadShader = function(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    alert('An error occurred compiling the shaders: ' + gl.getShaderInfoLog(shader));
    gl.deleteShader(shader);
    return null;
  }
  return shader;
};

TimeplotWidget.prototype.createLines = function(size) {
  for (let i = this.lines.length; i < size; ++i) {
    this.lines.push(new TimeplotLine(this.decimalColorByIndex(size === 1 ? -1 : i), this.canvasWidth));
    this.lines[i]._vbo = this.glContext.createBuffer();
    this.glContext.bindBuffer(this.glContext.ARRAY_BUFFER, this.lines[i]._vbo);
    this.glContext.bufferData(this.glContext.ARRAY_BUFFER, this.lines[i].xy, this.glContext.STREAM_DRAW);
    this.glContext.bindBuffer(this.glContext.ARRAY_BUFFER, this.lines[i]._vbo);
    this.lines[i]._coord = this.glContext.getAttribLocation(this.shaderProgram, 'coordinates');
    this.glContext.vertexAttribPointer(this.lines[i]._coord, 2, this.glContext.FLOAT, false, 0, 0);
    this.glContext.enableVertexAttribArray(this.lines[i]._coord);

    for (let j = 0; j < this.values.length; ++j) {
      const isArray = Array.isArray(this.values[j]);
      if (isArray)
        this.lines[j].addPoint(this.values[j].x, this.values[j].y[i]);
      else if (i === 0)
        this.lines[j].addPoint(this.values[j].x, this.values[j].y);
    }
  }
};

TimeplotWidget.prototype.initialize = function() {
  if (!window.WebGLRenderingContext) {
    console.error('This page requires a browser that supports WebGL.');
    return null;
  }

  if (this.initialized)
    return;

  const id = this.container.getAttribute('id');

  this.canvas = this.appendChildToContainer('<canvas id="' + id + '-canvas" class="plot-canvas" />');
  this.computeCanvasSize();
  console.assert(this.canvasWidth === this.canvas.width);
  console.assert(this.canvasHeight === this.canvas.height);

  this.initializeGlContext();

  this.xLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-x">' + this.labels['x'] + '</p>');
  this.xMinLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-x-min">0.0</p>');
  this.xMaxLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-x-max">0.0</p>');
  this.xMinLabel.textContent = roundLabel(0.0, this.decimals);
  this.xMaxLabel.textContent = roundLabel(this.xRangeSize, this.decimals);

  this.yLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-y">' + this.labels['y'] + '</p>');
  this.yMinLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-y-min">' + roundLabel(this.yRange['min'], this.decimals) + '</p>');
  this.yMaxLabel = this.appendChildToContainer('<p class="plot-axis-label plot-axis-label-y-max">' + roundLabel(this.yRange['max'], this.decimals) + '</p>');

  if (this.slider) {
    this.slider.setAttribute('min', this.yRange['min']);
    this.slider.setAttribute('max', this.yRange['max']);
  }

  if (this.values && this.values.length > 0) {
    const linesCount = Array.isArray(this.values[0].y) ? this.values[0].y.length : 1;
    if (this.lines.length < linesCount)
      this.createLines(linesCount);
    this.values = [];
  }
  this.initialized = true;

  this.show(this.shown);
};

TimeplotWidget.prototype.computeCanvasSize = function() {
  // Let the canvas size match with the element size (-2 because of the border), otherwise
  // the sizes are not matching causing a aliased zoom-like effect.
  this.canvas.width = this.canvas.offsetWidth - 2;
  this.canvas.height = this.canvas.offsetHeight - 2;
  this.canvasWidth = this.canvas.width;
  this.canvasHeight = this.canvas.height;
  this.xRangeSize = this.canvasWidth * this.basicTimeStep;

  // Update lines and grid
  for (let i = 0; i < this.lines.length; ++i)
    this.lines[i].resize(this.canvasWidth);

  if (this.horizontalGrid && this.horizontalGrid.maxSize < this.canvasHeight) {
    const diff = this.horizontalGrid.size - this.canvasHeight;
    let j = 0;
    if (diff > 0) {
      j = diff * 4;
      this.horizontalGrid.size = this.canvasHeight;
    }
    const pointSize = this.horizontalGrid.size * 4;
    const previousArray = this.horizontalGrid.xy;
    this.horizontalGrid.xy = new Float32Array(this.canvasHeight * 4);
    for (let i = 0; i < pointSize; ++i, ++j)
      this.horizontalGrid.xy[i] = previousArray[j];
  }

  if (this.initialized)
    this.initializeGlContext();
};

TimeplotWidget.prototype.initializeGlContext = function() {
  var webglNames = ['webgl', 'experimental-webgl'];
  for (var i = 0; i < webglNames.length; ++i) {
    this.glContext = this.canvas.getContext(webglNames[i], { antialias: false });
    if (this.glContext !== null)
      break;
  }
  if (this.glContext === null) {
    console.error('Unable to initialize WebGL. Your browser or machine may not support it.');
    return;
  }

  // Initialize shader program
  const vertexShader = this.loadShader(this.glContext, this.glContext.VERTEX_SHADER, this.VERTEX_SHADER);
  const fragmentShader = this.loadShader(this.glContext, this.glContext.FRAGMENT_SHADER, this.FRAGMENT_SHADER);
  this.shaderProgram = this.glContext.createProgram();
  this.glContext.attachShader(this.shaderProgram, vertexShader);
  this.glContext.attachShader(this.shaderProgram, fragmentShader);
  this.glContext.linkProgram(this.shaderProgram);
  if (!this.glContext.getProgramParameter(this.shaderProgram, this.glContext.LINK_STATUS)) {
    alert('Unable to initialize the shader program: ' + this.glContext.getProgramInfoLog(this.shaderProgram));
    return;
  }

  this.clearCanvas();

  // Set viewport
  this.glContext.useProgram(this.shaderProgram);
  this.glContext.viewport(0, 0, this.canvas.width, this.canvas.height);

  // Set coordinates scale and offset
  this.scaleUniformLocation = this.glContext.getUniformLocation(this.shaderProgram, 'scale');
  this.glContext.uniformMatrix2fv(this.scaleUniformLocation, false, [ 2.0 / this.xRangeSize, 0.0, 0.0, 2.0 / (this.yRange['max'] - this.yRange['min']) ]);
  this.offsetUniformLocation = this.glContext.getUniformLocation(this.shaderProgram, 'offset');
  this.pointSizeUniformLocation = this.glContext.getUniformLocation(this.shaderProgram, 'pointSize');
  this.colorUniformLocation = this.glContext.getUniformLocation(this.shaderProgram, 'color');
}

TimeplotWidget.prototype.drawLine = function(line, color, pointSize, mode) {
  if (line.size <= 0)
    return;

  const gl = this.glContext;
  gl.uniform1f(this.pointSizeUniformLocation, pointSize);
  gl.uniform3fv(this.colorUniformLocation, color);
  gl.bufferData(gl.ARRAY_BUFFER, line.xy, gl.STREAM_DRAW);
  gl.drawArrays(mode, 0, line.size);
};

TimeplotWidget.prototype.clearCanvas = function() {
  this.glContext.clearColor(1.0, 1.0, 1.0, 1.0);
  this.glContext.clear(this.glContext.COLOR_BUFFER_BIT);
};

TimeplotWidget.prototype.refresh = function() {
  // Do nothing if the div is hidden.
  if (this.container.offsetParent === null)
    return;

  // Initialize if required.
  if (!this.initialized)
    this.initialize();

  this.clearCanvas();

  this.displayHorizontalGrid();

  if (this.lines.length > 0 && this.lines[0].size > 0) {
    const gl = this.glContext;
    gl.uniform1f(this.pointSizeUniformLocation, 0.2);
    gl.uniform2fv(this.offsetUniformLocation, [this.xOffset, this.yRange['min']]);
    this.lastX = this.lines[0].lastX();
    this.lastY = [];
    for (let i = 0; i < this.lines.length; ++i) {
      this.drawLine(this.lines[i], this.lines[i].color, 2.0, gl.POINTS);
      this.lastY.push(this.lines[i].lastY());
    }
  }

  if (this.pendingSliderPositionUpdate)
    this.updateSliderPosition();
};

// Jump to a new y range based on the initial range and the new y value.
TimeplotWidget.prototype.jumpToRange = function(y) {
  console.assert(isNumber(y));
  if (y > this.yRange['max'] || y < this.yRange['min']) {
    const delta = this.initialYRange['max'] - this.initialYRange['min'];
    const rangeLevel = Math.floor(0.5 * Math.floor(y / (0.5 * delta) + 1.0));
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
    for (let i = 0; i < y.length; ++i) {
      const v = y[i];
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
    this.yMinLabel.textContent = roundLabel(this.yRange['min'], this.decimals);
    this.yMaxLabel.textContent = roundLabel(this.yRange['max'], this.decimals);
  }

  if (this.slider) {
    this.slider.setAttribute('min', this.yRange['min']);
    this.slider.setAttribute('max', this.yRange['max']);
  }

  this.glContext.uniformMatrix2fv(this.scaleUniformLocation, false, [ 2.0 / this.xRangeSize, 0.0, 0.0, 2.0 / (this.yRange['max'] - this.yRange['min']) ]);
};

TimeplotWidget.prototype.updateGridConstants = function() {
  let delta = this.yRange['max'] - this.yRange['min'];
  delta *= 0.999; // in order to decrease the order of magnitude if delta is a perfect divider of the increment
  const minStep = delta / 10; // initial step size
  const orderOfMagnitude = Math.floor(Math.log(minStep) / Math.LN10);
  const magnitude = Math.pow(10, orderOfMagnitude);
  // Calculate most significant digit of the new step size
  let residual = Math.round(minStep / magnitude + 0.5);
  if (residual > 5.0)
    residual = 10.0;
  else if (residual > 2.0)
    residual = 5.0;
  else if (residual > 1.0)
    residual = 2.0;

  this.verticalGridSteps = magnitude * residual;
  if (this.horizontalGrid)
    this.horizontalGrid.changed = true;
};

TimeplotWidget.prototype.show = function(show) {
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
  if (this.slider)
    this.slider.style.visibility = visibility;
};

TimeplotWidget.prototype.refreshLabels = function() {
  // Refresh the labels only if the container is visible.
  if (!this.initialized || !this.container.offsetParent)
    return;

  if (this.lastLabelRefresh !== this.lastX) {
    // Update the x labels.
    this.xMinLabel.textContent = roundLabel(this.xOffset, this.decimals);
    this.xMaxLabel.textContent = roundLabel(this.xOffset + this.xRangeSize, this.decimals);
  }
  if (this.slider && !this.blockSliderUpdateFlag)
    this.slider.value = this.lastY;
  if (this.label) {
    const v = this.lastY;
    if (v.length > 1) {
      const legend = this.labels['legend'];
      let text = ':' + (legend ? ' ' : '<br>&emsp;&emsp;') + '[ ';
      for (let i = 0; i < v.length; i++) {
        if (i > 0)
          text += ', ';
        if (legend)
          text += '<br>&emsp;&emsp;';
        text += '<span style="color:' + this.labelColorByIndex(i) + '">';
        if (legend && legend.length > i)
          text += ' ' + legend[i] + ': ';
        text += roundLabel(v[i], this.decimals) + '</span>';
      }
      this.label.innerHTML = text + (legend ? '<br>&emsp;&emsp;]' : ' ]');
    } else if (v.length > 0)
      this.label.textContent = ': ' + roundLabel(v[0], this.decimals);
  }
  this.lastLabelRefresh = this.lastX;
};

TimeplotWidget.prototype.appendChildToContainer = function(child) {
  let tmp = document.createElement('tmp');
  tmp.innerHTML = child;
  this.container.appendChild(tmp.firstChild);
  return this.container.childNodes[this.container.childNodes.length - 1];
};

TimeplotWidget.prototype.displayHorizontalGrid = function() {
  if (!this.horizontalGrid) {
    this.horizontalGrid = {
      'vbo': this.glContext.createBuffer(),
      'coord': null,
      'xy': new Float32Array(this.canvasHeight * 4), // TODO
      'maxSize': this.canvasHeight,
      'size': 0,
      'changed': true
    };
    this.glContext.bindBuffer(this.glContext.ARRAY_BUFFER, this.horizontalGrid.vbo);
    this.glContext.bufferData(this.glContext.ARRAY_BUFFER, this.horizontalGrid.xy, this.glContext.STREAM_DRAW);
    this.horizontalGrid.coord = this.glContext.getAttribLocation(this.shaderProgram, 'coordinates');
    this.glContext.vertexAttribPointer(this.horizontalGrid.coord, 2, this.glContext.FLOAT, false, 0, 0);
    this.glContext.enableVertexAttribArray(this.horizontalGrid.coord);
  }
  if (this.horizontalGrid.changed) {
    let max = Math.ceil(this.yRange['max'] / this.verticalGridSteps) + 1;
    if (max > this.horizontalGrid.maxSize)
      max = this.horizontalGrid.maxSize;
    const maxX = this.xRangeSize;
    let i = 0;
    for (let y = Math.ceil(this.yRange['min'] / this.verticalGridSteps); y < max; ++y, i += 4) {
      this.horizontalGrid.xy[i] = 0.0; // x1
      this.horizontalGrid.xy[i + 1] = y * this.verticalGridSteps; // y1
      this.horizontalGrid.xy[i + 2] = maxX; // x2
      this.horizontalGrid.xy[i + 3] = this.horizontalGrid.xy[i + 1]; // y2
    }
    this.horizontalGrid.size = i / 2;
    this.horizontalGrid.changed = false;
  }
  this.glContext.uniform2fv(this.offsetUniformLocation, [0, this.yRange['min']]);
  this.drawLine(this.horizontalGrid, [0.867, 0.867, 0.867], 1.0, this.glContext.LINES);
};

TimeplotWidget.prototype.decimalColorByIndex = function(i) {
  if (i < 0) // default for single line plot
    return [0.0, 0.3333, 0.6];
  if (i === 0)
    return [0.6667, 0.2667, 0.2667];
  if (i === 1)
    return [0.2667, 0.6667, 0.2667];
  if (i === 2)
    return [0.2667, 0.2667, 0.6667];
  return [0.2667, 0.2667, 0.2667];
};

TimeplotWidget.prototype.labelColorByIndex = function(i) {
  if (i === 0)
    return 'red';
  if (i === 1)
    return 'green';
  if (i === 2)
    return 'blue';
  return 'black';
};

TimeplotWidget.prototype.updateSliderPosition = function() {
  let parentSection = this.container;
  while (parentSection.className !== 'devices-layout') {
    parentSection = parentSection.parentElement;
    if (!parentSection)
      return;
  }
  if (parentSection.offsetWidth < (this.canvasWidth + 30))
    this.slider.classList.add('motor-slider-right');
  else
    this.slider.classList.remove('motor-slider-right');
  this.pendingSliderPositionUpdate = false;
};

TimeplotWidget.prototype.resize = function() {
  if (!this.slider)
    return;
  // Mode slider immediately only if the container is visible.
  if (!this.container.offsetParent)
    this.pendingSliderPositionUpdate = true;
  else
    this.updateSliderPosition();
};

function roundLabel(value, decimals = 3) {
  console.assert(isNumber(value));
  if (isNumber(value))
    return parseFloat(value).toFixed(decimals); // select number of decimals
  return value;
}

function isNumber(n) {
  if (n === 'Inf' || n === '-Inf' || n === 'NaN')
    return true;
  return parseFloat(n) === n;
}
