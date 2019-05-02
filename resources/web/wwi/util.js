/* global THREE */
/* eslint no-extend-native: ["error", { "exceptions": ["String"] }] */

var webots = window.webots || {};

webots.quaternionToAxisAngle = (quaternion) => {
  var angle, axis;
  var q = quaternion.clone();
  if (q.w > 1.0)
    q.normalize();
  if (q.w >= 1.0)
    angle = 0.0;
  else if (q.w <= -1.0)
    angle = 2.0 * Math.PI;
  else
    angle = 2.0 * Math.acos(q.w);
  if (angle < 0.001) {
    // if the angle is close to zero, then the direction of the axis is not important
    axis = new THREE.Vector3(0.0, 1.0, 0.0);
    angle = 0;
  } else {
    // normalize axes
    var inv = 1.0 / Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    axis = new THREE.Vector3(q.x * inv, q.y * inv, q.z * inv);
  }
  return { 'axis': axis, 'angle': angle};
};

webots.parseMillisecondsIntoReadableTime = (milliseconds) => {
  var hours = (milliseconds + 0.9) / (1000 * 60 * 60);
  var absoluteHours = Math.floor(hours);
  var h = absoluteHours > 9 ? absoluteHours : '0' + absoluteHours;
  var minutes = (hours - absoluteHours) * 60;
  var absoluteMinutes = Math.floor(minutes);
  var m = absoluteMinutes > 9 ? absoluteMinutes : '0' + absoluteMinutes;
  var seconds = (minutes - absoluteMinutes) * 60;
  var absoluteSeconds = Math.floor(seconds);
  var s = absoluteSeconds > 9 ? absoluteSeconds : '0' + absoluteSeconds;
  var ms = Math.floor((seconds - absoluteSeconds) * 1000);
  if (ms < 10)
    ms = '00' + ms;
  else if (ms < 100)
    ms = '0' + ms;
  return h + ':' + m + ':' + s + ':' + ms;
};

// add startsWith() and endsWith() functions to the String prototype
if (typeof String.prototype.startsWith !== 'function') {
  String.prototype.startsWith = (prefix) => {
    return this.slice(0, prefix.length) === prefix;
  };
}

if (typeof String.prototype.endsWith !== 'function') {
  String.prototype.endsWith = (suffix) => {
    return this.indexOf(suffix, this.length - suffix.length) !== -1;
  };
}
