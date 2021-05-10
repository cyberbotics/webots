function render(text) {
  return text
}

function compile() {
  var result = ""; 
  
  %fields%

  %body%

  return result;
}

compile()
