/* global THREE */
/* eslint no-extend-native: ["error", { "exceptions": ["String"] }] */

var webots = window.webots || {};

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
