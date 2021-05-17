%import%

//import console from './console.js';

export function render(text) {
  return text
}

// function main() {
export function main() {
  let result = '';

  let context = { %context% };

  let fields = { %fields% };

  %body%

  //console.log(1);

  return result;
}

//main()
