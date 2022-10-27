import FloatingWindow from './FloatingWindow.js';
import {VRML} from './protoVisualizer/vrml_type.js';

export default class FloatingProtoParameterWindow extends FloatingWindow {
  #protoManager;
  #view;
  constructor(parentNode, protoManager, view) {
    super(parentNode, 'proto-parameter');
    this.floatingWindow.style.zIndex = '2';
    this.headerText.innerHTML = 'Proto parameter window';
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-content';
    this.floatingWindowContent.appendChild(this.frame);
    this.#protoManager = protoManager;
    this.#view = view;
  }

  populateProtoParameterWindow() {
    const contentDiv = document.getElementById('proto-parameter-content');
    if (contentDiv) {
      contentDiv.innerHTML = '';
      const keys = this.#protoManager.exposedParameters.keys();
      for (let key of keys) {
        const parameter = this.#protoManager.exposedParameters.get(key);
        if (parameter.type === VRML.SFColor)
          this.#createBoolField(key, contentDiv);
        else if (parameter.type === VRML.SFVec3f)
          this.#createSFVec3Field(key, contentDiv);
        else if (parameter.type === VRML.SFBool)
          this.#createColorField(key, contentDiv);
        else if (parameter.type === VRML.SFRotation)
          this.#createSFRotation(key, contentDiv);
        else if (parameter.type === VRML.SFString)
          this.#createSFStringField(key, contentDiv);
        else if (parameter.type === VRML.SFFloat)
          this.#createSFFloatField(key, contentDiv);
      }
    }
  }

  #createSFVec3Field(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);
    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.inputs = [];
    p.parameter = parameter;
    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, p, () => this.#vector3OnChange(p)));
    p.inputs.push(this.#createVectorInput(' y', parameter.value.value.y, p, () => this.#vector3OnChange(p)));
    p.inputs.push(this.#createVectorInput(' z', parameter.value.value.z, p, () => this.#vector3OnChange(p)));

    parent.appendChild(p);
  }

  #createSFRotation(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);
    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.inputs = [];
    p.parameter = parameter;
    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, p, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' y', parameter.value.value.y, p, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' z', parameter.value.value.z, p, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' a', parameter.value.value.a, p, () => this.#rotationOnChange(p)));

    parent.appendChild(p);
  }

  #createVectorInput(name, initialValue, parent, callback) {
    const span = document.createElement('span');
    span.innerHTML = name + ': ';
    const input = document.createElement('input');
    input.type = 'number';
    input.value = initialValue;
    input.step = 0.1;
    input.style.width = '40px';
    input.onchange = () => callback(parent);

    span.appendChild(input);
    parent.appendChild(span);

    return input;
  }

  #rotationOnChange(node) {
    let object = {'x': node.inputs[0].value, 'y': node.inputs[1].value, 'z': node.inputs[2].value, 'a': node.inputs[3].value};
    node.parameter.setValueFromJavaScript(this.#view, object);
  }

  #vector3OnChange(node) {
    let object = {'x': node.inputs[0].value, 'y': node.inputs[1].value, 'z': node.inputs[2].value};
    node.parameter.setValueFromJavaScript(this.#view, object);
  }

  #createColorField(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);
    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.inputs = [];
    p.parameter = parameter;
    p.inputs.push(this.#createColorInput('r', parameter.value.value.r, p, this.#colorOnChange));
    p.inputs.push(this.#createColorInput(' g', parameter.value.value.g, p, this.#colorOnChange));
    p.inputs.push(this.#createColorInput(' b', parameter.value.value.b, p, this.#colorOnChange));

    parent.appendChild(p);
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
    const parameter = this.#protoManager.exposedParameters.get(key);
    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.parameter = parameter;
    const input = document.createElement('input');

    input.type = 'checkbox';
    if (parameter.value.value)
      input.checked = true;

    input.onchange = () => this.#boolOnChange(p);
    p.input = input;
    p.appendChild(input);
    parent.appendChild(p);
  }

  #boolOnChange(node) {
    node.parameter.setValueFromJavaScript(node.input.checked);
  }

  #createSFStringField(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);
    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.parameter = parameter;
    const input = document.createElement('input');

    input.type = 'text';
    input.value = parameter.value.value;

    input.onchange = () => this.#stringOnChange(p);
    p.input = input;
    p.appendChild(input);
    parent.appendChild(p);
  }

  #stringOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.value);
  }

  #createSFFloatField(key, parent) {
    const parameter = this.#protoManager.exposedParameters.get(key);
    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.parameter = parameter;
    const input = document.createElement('input');

    input.type = 'number';
    input.step = 0.1;
    input.value = parameter.value.value;
    input.style.width = '40px';

    input.onchange = () => this.#floatOnChange(p);
    p.input = input;
    p.appendChild(input);
    parent.appendChild(p);
  }

  #floatOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.value);
  }
}
