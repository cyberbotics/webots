function render(text) {
  return text
}

function compile() {
  var result = "";

  %context%

  %fields%

  %body%

  return result;
}

compile()
