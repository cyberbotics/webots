"use strict";

%import%

let console = {
  log: function() {
    stdout.push(this.digest.apply(this, arguments));
  },

  info: function() {
    stdout.push(this.digest.apply(this, arguments));
  },

  warn: function() {
    stdout.push(this.digest.apply(this, arguments));
  },

  debug: function() {
    stdout.push(this.digest.apply(this, arguments));
  },

  error: function() {
    stderr.push(this.digest.apply(this, arguments));
  },

  digest: function() {
    var args = Array.prototype.slice.call(arguments);
    let entry = '';
    for (let i = 0; i < args.length; ++i){
      if (typeof args[i] === 'object')
        entry += JSON.stringify(args[i], null, 1);
      else
        entry += args[i];
    }
    return entry;
  }
}

function render(text) {
  return text;
};

export function generateVrml() {
  let ___vrml = '';
  let ___tmp;

  const context = { %context% };

  const fields = { %fields% };

  %body%

  return ___vrml;
};
