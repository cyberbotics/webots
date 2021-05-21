"use strict";

%import%

function render(text) {
  return text;
};

let console = {
  log: function() {
    var args = Array.prototype.slice.call(arguments);
    let entry = '';
    for (let i = 0; i < args.length; ++i){
      if (typeof args[i] == 'object')
        entry += JSON.stringify(args[i])
      else
        entry += args[i]
    }
    stdout.push(entry);
  },

  error: function() {
    var args = Array.prototype.slice.call(arguments);
    let entry = '';
    for (let i = 0; i < args.length; ++i){
      if (typeof args[i] == 'object')
        entry += JSON.stringify(args[i])
      else
        entry += args[i]
    }
    stderr.push(entry);
  }
}


export function main() {
  let result = '';

  const context = { %context% };

  const fields = { %fields% };

  %body%

  /*
  console.log("something");
  console.log("1 + 1 = ", 1+1);
  var a = 2;
  console.log(a);
  console.log({x:1, y:1})
  function func() { return (5 * 19); }
  console.log(func());
  console.log("The value of a is " + a);
  var players = ['Jim', 'Shawna', 'Andrew', 'Lora', 'Aimee', 'Nick'];
  console.log(players);
  var b = {x: 1, y: 2};
  console.log(b);
  console.error("ASD!", players[2])
  */

  return result;
};
