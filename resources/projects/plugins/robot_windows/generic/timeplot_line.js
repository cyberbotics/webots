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
