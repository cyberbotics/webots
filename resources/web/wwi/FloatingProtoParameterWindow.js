import FloatingWindow from './FloatingWindow.js';

export default class FloatingProtoParameterWindow extends FloatingWindow {
  #protoManager;
  constructor(parentNode, protoManager) {
    super(parentNode, 'proto-parameter');
    this.floatingWindow.style.zIndex = '2';
    this.headerText.innerHTML = 'Proto parameter window';
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-content';
    this.floatingWindowContent.appendChild(this.frame);
    this.#protoManager = protoManager;
  }

  populateProtoParameterWindow() {
    const contentDiv = document.getElementById('proto-parameter-content');
    if (contentDiv) {
      contentDiv.innerHTML = '';
      const keys = this.#protoManager.exposedParameters.keys();
      for (let key of keys) {
        const parameter = this.#protoManager.exposedParameters.get(key);

        if (parameter.type === 1)
          this.#createBoolField(key, contentDiv);
        else
          this.#createColorField(key, contentDiv);
      }
    }
  }

  #createColorField(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);
    const span = document.createElement('span');
    span.innerHTML = key + ': ';
    span.inputs = [];
    span.parameter = parameter;
    span.inputs.push(this.#createColorInput('r', parameter.value.value.r, span, this.#colorOnChange));
    span.inputs.push(this.#createColorInput(' g', parameter.value.value.g, span, this.#colorOnChange));
    span.inputs.push(this.#createColorInput(' b', parameter.value.value.b, span, this.#colorOnChange));

    parent.appendChild(span);
  }

  #createColorInput(name, initialValue, parent, callback) {
    const span = document.createElement('span');
    span.innerHTML = name + ': ';
    const input = document.createElement('input');
    input.type = 'number';
    input.value = initialValue;
    input.step = 0.1;
    input.min = 0;
    input.max = 1;
    input.style.width = '50px';
    input.onchange = () => callback(parent);

    span.appendChild(input);
    parent.appendChild(span);

    return input;
  }

  #colorOnChange(node) {
    let object = {'r': node.inputs[0].value, 'g': node.inputs[1].value, 'b': node.inputs[2].value};
    node.parameter.setValueFromJavaScript(object);
  }

  #createBoolField(key, parent) {
    const parameter = this.parentNode.protoManager.exposedParameters.get(key);
    const span = document.createElement('span');
    span.innerHTML = key + ': ';
    span.parameter = parameter;
    const input = document.createElement('input');

    input.type = 'checkbox';
    if (parameter.value.value)
      input.checked = true;

    input.onchange = () => this.#boolOnChange(span);
    span.input = input;
    span.appendChild(input);
    parent.appendChild(span);
  }

  #boolOnChange(node) {
    node.parameter.setValueFromJavaScript(node.input.checked);
  }
}
