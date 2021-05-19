"use strict";

%import%

function render(text) {
  return text;
};

export function main() {
  let result = '';

  const context = { %context% };

  const fields = { %fields% };

  %body%

  return result;
};
