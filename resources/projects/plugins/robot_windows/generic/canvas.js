function Canvas() {
  if (!Canvas._instance) {
    Canvas._instance = this;
    this.webglContext = null;
    this.shaderProgram = null;
    this.scaleUniformLocation = null;
    this.offsetUniformLocation = null;
    this.pointSizeUniformLocation = null;
    this.colorUniformLocation = null;
    this.canvasWidth = 0;
    this.canvasHeight = 0;
  }
  return Canvas._instance;
}

Canvas.prototype.getWebglContext = function() {
  if (!this.webglContext) {
    const webglNames = ['webgl', 'experimental-webgl'];
    for (let i = 0; i < webglNames.length; ++i) {
      const canvas = document.getElementById('canvas');
      this.webglContext = canvas.getContext(webglNames[i], { antialias: false });
      if (this.webglContext !== null)
        break;
    }
    if (this.webglContext === null)
      throw new Error('Unable to initialize WebGL. Your browser or machine may not support it.');

    // Initialize shader program
    const gl = this.webglContext;
    const vertexShader = this.loadShader(gl, gl.VERTEX_SHADER, Canvas.VERTEX_SHADER);
    const fragmentShader = this.loadShader(gl, gl.FRAGMENT_SHADER, Canvas.FRAGMENT_SHADER);
    this.shaderProgram = gl.createProgram();
    gl.attachShader(this.shaderProgram, vertexShader);
    gl.attachShader(this.shaderProgram, fragmentShader);
    gl.linkProgram(this.shaderProgram);
    if (!gl.getProgramParameter(this.shaderProgram, gl.LINK_STATUS))
      throw new Error('Unable to initialize the shader program: ' + gl.getProgramInfoLog(this.shaderProgram));
    gl.useProgram(this.shaderProgram);
    this.scaleUniformLocation = gl.getUniformLocation(this.shaderProgram, 'scale');
    this.offsetUniformLocation = gl.getUniformLocation(this.shaderProgram, 'offset');
    this.pointSizeUniformLocation = gl.getUniformLocation(this.shaderProgram, 'pointSize');
    this.colorUniformLocation = gl.getUniformLocation(this.shaderProgram, 'color');
    this.resizeCanvas();
  }
  return this.webglContext;
};

Canvas.prototype.clearCanvas = function() {
  const gl = this.getWebglContext();
  gl.disable(gl.SCISSOR_TEST);
  gl.viewport(0, 0, this.canvasWidth, this.canvasHeight);
  gl.clearColor(0.0, 0.0, 0.0, 0.0);
  gl.clear(gl.COLOR_BUFFER_BIT);
};

Canvas.prototype.resizeCanvas = function() {
  let canvas = document.getElementById('canvas');
  const newWidth = canvas.clientWidth;
  const newHeight = canvas.clientHeight;
  // Check if the canvas is not the same size.
  const needResize = canvas.width !== newWidth || canvas.height !== newHeight;
  if (needResize) {
    // Make the canvas the same size
    canvas.width = newWidth;
    canvas.height = newHeight;
    this.canvasWidth = newWidth;
    this.canvasHeight = newHeight;
    this.webglContext = null; // better clear the whole webgl resources
    this.getWebglContext();
  }
  return needResize;
};

// Private members and methods

Canvas._instance = null;

Canvas.FRAGMENT_SHADER = `
  precision mediump float;
  uniform vec3 color;
  void main(void) {
    gl_FragColor = vec4(color, 1.0);
  }`;

Canvas.VERTEX_SHADER = `
  attribute vec2 coordinates;
  uniform mat2 scale;
  uniform vec2 offset;
  uniform float pointSize;
  void main() {
    gl_Position = vec4(scale * (coordinates - offset) - vec2(1.0, 1.0),  0.0, 1.0);
    gl_PointSize = pointSize;
  }`;

Canvas.prototype.loadShader = function(gl, type, source) {
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
