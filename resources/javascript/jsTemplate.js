%import%

export function render(text) {
  return text
}

// function main() {
export function main() {
  let result = '';

  let context = { %context% }

  let fields = { %fields% }

  %body%

  return result;
}

//main()
