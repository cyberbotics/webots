import FloatingWindow from './FloatingWindow.js';
import {VRML} from './protoVisualizer/vrml_type.js';

export default class FloatingProtoParameterWindow extends FloatingWindow {
  #protoManager;
  #view;
  constructor(parentNode, protoManager, view, proto) {
    super(parentNode, 'proto-parameter');
    this.floatingWindow.style.zIndex = '2';
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-content';
    this.floatingWindowContent.appendChild(this.frame);
    this.#protoManager = protoManager;
    this.proto = proto;
    this.headerText.innerHTML = this.proto.name;
    this.#view = view;

    this.setupNodeSelector(parentNode);
  }

  populateProtoParameterWindow() {
    const contentDiv = document.getElementById('proto-parameter-content');
    if (contentDiv) {
      this.headerText.innerHTML = this.proto.name;

      contentDiv.innerHTML = '';
      let row = 1;
      if (!this.proto.isRoot)
        this.#createBackButton(contentDiv, row++);

      const keys = this.proto.parameters.keys();
      for (let key of keys) {
        const parameter = this.proto.parameters.get(key);
        if (parameter.type === VRML.SFVec3f)
          this.#createSFVec3Field(key, contentDiv, row++);
        else if (parameter.type === VRML.SFRotation)
          this.#createSFRotation(key, contentDiv, row++);
        else if (parameter.type === VRML.SFString)
          this.#createSFStringField(key, contentDiv, row++);
        else if (parameter.type === VRML.SFFloat)
          this.#createSFFloatField(key, contentDiv, row++);
        else if (parameter.type === VRML.SFNode)
          this.#createSFNodeField(key, contentDiv, row++);
      }

      if (this.proto.isRoot)
        this.#createDownloadButton(contentDiv, row);
    }
  }

  setupNodeSelector(parentNode) {
    let div = document.createElement('div');
    div.className = 'node-library';
    div.setAttribute('id', 'node-library');
    parentNode.appendChild(div);
  }

  #createBackButton(parent, row) {
    const buttonContainer = document.createElement('span');
    buttonContainer.style.gridRow = '' + row + ' / ' + row;
    buttonContainer.style.gridColumn = '1 / 1';

    const backButton = document.createElement('button');
    backButton.innerHTML = 'Back';
    backButton.title = 'Return to the previous PROTO';
    backButton.onclick = () => {
      console.log('back.');
      this.proto = this.#protoManager.proto; // TODO: go one layer up, not back to root
      this.populateProtoParameterWindow();
    };
    buttonContainer.appendChild(backButton);

    parent.appendChild(buttonContainer);
  }

  #createDownloadButton(parent, row) {
    const buttonContainer = document.createElement('span');
    buttonContainer.className = 'value-parameter';
    buttonContainer.style.gridRow = '' + row + ' / ' + row;
    buttonContainer.style.gridColumn = '3 / 3';

    const input = document.createElement('input');
    input.type = 'text';
    input.title = 'New proto name';
    input.value = 'My' + this.proto.name;
    buttonContainer.appendChild(input);

    const downloadButton = document.createElement('button');
    downloadButton.innerHTML = 'Download';
    downloadButton.title = 'Download the new proto';
    downloadButton.onclick = () => {
      const fields = document.getElementsByClassName('key-parameter');
      let fieldsToExport = new Set();
      if (fields) {
        for (let i = 0; i < fields.length; i++) {
          if (fields[i].checkbox.checked)
            fieldsToExport.add(fields[i].key);
        }
      }

      const data = this.#protoManager.exportProto(input.value, fieldsToExport);
      const downloadLink = document.createElement('a');
      downloadLink.download = input.value + '.proto';
      const file = new Blob([data], {
        type: 'text/plain'
      });
      downloadLink.href = window.URL.createObjectURL(file);
      downloadLink.click();
    };
    buttonContainer.appendChild(downloadButton);
    parent.appendChild(buttonContainer);
  }

  #createSFVec3Field(key, parent, row) {
    const parameter = this.proto.parameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.key = key;
    p.inputs = [];
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';
    p.parameter = parameter;
    p.className = 'key-parameter';

    if (this.proto.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, row);
      p.checkbox = exportCheckbox;
    }

    const values = document.createElement('p');
    values.style.gridRow = '' + row + ' / ' + row;
    values.style.gridColumn = '3 / 3';
    values.className = 'value-parameter';
    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, values, () => this.#vector3OnChange(p)));
    p.inputs.push(this.#createVectorInput(' y', parameter.value.value.y, values, () => this.#vector3OnChange(p)));
    p.inputs.push(this.#createVectorInput(' z', parameter.value.value.z, values, () => this.#vector3OnChange(p)));

    const resetButton = this.#createResetButton(values);
    resetButton.onclick = () => {
      p.inputs[0].value = parameter.defaultValue.value.x;
      p.inputs[1].value = parameter.defaultValue.value.y;
      p.inputs[2].value = parameter.defaultValue.value.z;
      this.#vector3OnChange(p);
    };

    parent.appendChild(p);
    parent.appendChild(values);
  }

  #createSFRotation(key, parent, row) {
    const parameter = this.proto.parameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.inputs = [];
    p.key = key;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';
    p.parameter = parameter;
    p.className = 'key-parameter';

    if (this.proto.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, row);
      p.checkbox = exportCheckbox;
    }

    const values = document.createElement('p');
    values.style.gridRow = '' + row + ' / ' + row;
    values.style.gridColumn = '3 / 3';
    values.className = 'value-parameter';
    p.inputs.push(this.#createVectorInput('x', parameter.value.value.x, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' y', parameter.value.value.y, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' z', parameter.value.value.z, values, () => this.#rotationOnChange(p)));
    p.inputs.push(this.#createVectorInput(' a', parameter.value.value.a, values, () => this.#rotationOnChange(p)));
    const resetButton = this.#createResetButton(values);
    resetButton.onclick = () => {
      p.inputs[0].value = parameter.defaultValue.value.x;
      p.inputs[1].value = parameter.defaultValue.value.y;
      p.inputs[2].value = parameter.defaultValue.value.z;
      p.inputs[3].value = parameter.defaultValue.value.a;
      this.#rotationOnChange(p);
    };
    parent.appendChild(p);
    parent.appendChild(values);
  }

  #createVectorInput(name, initialValue, parent, callback) {
    const span = document.createElement('span');
    span.innerHTML = name + ': ';
    const input = document.createElement('input');
    input.type = 'number';
    input.value = initialValue;
    input.step = 0.1;
    input.style.width = '50px';
    input.onchange = () => callback(parent);

    span.appendChild(input);
    parent.appendChild(span);

    return input;
  }

  #rotationOnChange(node) {
    const object = {'x': node.inputs[0].value, 'y': node.inputs[1].value, 'z': node.inputs[2].value, 'a': node.inputs[3].value};
    node.parameter.setValueFromJavaScript(this.#view, object);
  }

  #vector3OnChange(node) {
    const object = {'x': node.inputs[0].value, 'y': node.inputs[1].value, 'z': node.inputs[2].value};
    node.parameter.setValueFromJavaScript(this.#view, object);
  }

  #createSFStringField(key, parent, row) {
    const parameter = this.proto.parameters.get(key);

    const p = document.createElement('p');
    p.innerHTML = key + ': ';
    p.parameter = parameter;
    p.key = key;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';
    p.className = 'key-parameter';

    if (this.proto.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, row);
      p.checkbox = exportCheckbox;
    }

    const value = document.createElement('p');
    value.style.gridRow = '' + row + ' / ' + row;
    value.style.gridColumn = '3 / 3';
    value.className = 'value-parameter';

    const input = document.createElement('input');
    input.type = 'text';

    const string = parameter.value.value;

    input.value = this.#stringRemoveQuote(string);
    input.style.height = '20px';

    input.onchange = () => this.#stringOnChange(p);
    p.input = input;
    value.appendChild(input);

    const resetButton = this.#createResetButton(value);
    resetButton.onclick = () => {
      input.value = this.#stringRemoveQuote(parameter.defaultValue.value);
      this.#stringOnChange(p);
    };

    parent.appendChild(p);
    parent.appendChild(value);
  }

  #stringOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.value);
  }

  #stringRemoveQuote(string) {
    if (typeof string !== 'undefined' && string.length > 0) {
      string = string?.trim();
      if (string[0] === '"')
        string = string.substring(1);
      if (string[string.length - 1] === '"')
        string = string.slice(0, string.length - 1);
    }
    return string;
  }

  #createSFFloatField(key, parent, row) {
    const parameter = this.proto.parameters.get(key);

    const p = document.createElement('p');
    p.className = 'key-parameter';
    p.innerHTML = key + ': ';
    p.key = key;
    p.parameter = parameter;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';

    if (this.proto.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, row);
      p.checkbox = exportCheckbox;
    }

    const value = document.createElement('p');
    value.className = 'value-parameter';
    value.style.gridRow = '' + row + ' / ' + row;
    value.style.gridColumn = '3 / 3';

    const input = document.createElement('input');
    input.type = 'number';
    input.step = 0.1;
    input.value = parameter.value.value;
    input.style.width = '50px';

    input.onchange = () => this.#floatOnChange(p);
    p.input = input;
    value.appendChild(input);

    const resetButton = this.#createResetButton(value);
    resetButton.onclick = () => {
      input.value = parameter.defaultValue.value;
      this.#floatOnChange(p);
    };
    parent.appendChild(p);
    parent.appendChild(value);
  }

  #createSFNodeField(key, parent, row) {
    const parameter = this.proto.parameters.get(key);

    const p = document.createElement('p');
    p.className = 'key-parameter';
    p.innerHTML = key + ': ';
    p.key = key;
    p.parameter = parameter;
    p.style.gridRow = '' + row + ' / ' + row;
    p.style.gridColumn = '2 / 2';

    if (this.proto.isRoot) {
      const exportCheckbox = this.#createCheckbox(parent, row);
      p.checkbox = exportCheckbox;
    }

    const value = document.createElement('p');
    value.className = 'value-parameter';
    value.style.gridRow = '' + row + ' / ' + row;
    value.style.gridColumn = '3 / 3';

    const configureButton = document.createElement('button');
    configureButton.innerHTML = 'configure';
    configureButton.onclick = async() => {
      console.log('configure.');
      this.proto = parameter.value.value;
      this.populateProtoParameterWindow();
    };

    const nodeButton = document.createElement('button');
    nodeButton.title = 'Select a node to insert';
    nodeButton.onclick = async() => {
      console.log('clicked.');
      return new Promise((resolve, reject) => {
        const xmlhttp = new XMLHttpRequest();
        xmlhttp.open('GET', '../../proto-list.xml', true);
        xmlhttp.onreadystatechange = async() => {
          if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
            resolve(xmlhttp.responseText);
        };
        xmlhttp.send();
      }).then(text => {
        this.#populateNodeLibrary(text, parameter, nodeButton, configureButton);
      });
    };

    if (parameter.value.value === null) {
      configureButton.style.display = 'none';
      nodeButton.innerHTML = 'NULL';
    } else {
      configureButton.style.display = 'block';
      configureButton.title = 'Configure ' + parameter.value.value.name + ' node';
      nodeButton.innerHTML = parameter.value.value.name;
    }

    value.appendChild(nodeButton);
    value.appendChild(configureButton);

    const resetButton = this.#createResetButton(value);
    resetButton.onclick = () => {
      throw new Error('TODO: implement reset.');
    };
    parent.appendChild(p);
    parent.appendChild(value);
  }

  async #populateNodeLibrary(protoList, parameter, nodeButton, configureButton) {
    let panel = document.getElementById('node-library');
    panel.innerHTML = '';
    panel.style.display = 'block';

    // TODO: filter clicks inside the panel
    // window.addEventListener('click', _ => {
    //   console.log('close')
    //   panel.style.display = 'none';
    // });

    const container = document.createElement('div');

    let protoNodes = [{name: 'NULL', url: null}];

    const parser = new DOMParser();
    const xml = parser.parseFromString(protoList, 'text/xml').firstChild;
    for (const proto of xml.getElementsByTagName('proto')) {
      const info = {};
      info['name'] = proto.getElementsByTagName('name')[0].innerHTML;
      info['url'] = proto.getElementsByTagName('url')[0].innerHTML;
      protoNodes.push(info);
    }

    const nodeList = document.createElement('div');
    nodeList.className = 'node-list';
    let ol = document.createElement('ol');
    for (const node of protoNodes) {
      const item = document.createElement('li');
      const button = document.createElement('button');
      button.innerText = node['name'];
      button.value = node['url'];
      item.appendChild(button);

      // button.onclick = (element) => {
      //   const url = element.target.value;
      //   console.log('selected:', url);
      // }
      button.ondblclick = async(element) => {
        const url = element.target.value;
        console.log('inserting:', url);

        // const protoManager = new ProtoManager(this.#view);
        // await protoManager.loadProto(url);
        // const x3d = new XMLSerializer().serializeToString(protoManager.proto.toX3d());
        // console.log(x3d);

        if (url === 'null')
          parameter.setValueFromJavaScript(this.#view, null);
        else {
          const node = await this.#protoManager.generateNodeFromUrl(url);
          // const x3d = new XMLSerializer().serializeToString(node.toX3d());
          // console.log('Grafted x3d:', x3d);
          // console.log(parameter);

          // const sfnode = new SFNode();
          // sfnode.setValue(node);

          parameter.setValueFromJavaScript(this.#view, node);
        }

        // close library panel
        panel.style.display = 'none';

        // update button name
        if (parameter.value.value === null) {
          nodeButton.innerHTML = 'NULL';
          configureButton.style.display = 'none';
        } else {
          nodeButton.innerHTML = parameter.value.value.name;
          configureButton.style.display = 'block';
        }
      };

      ol.appendChild(item);
    }
    nodeList.appendChild(ol);

    const nodeInfo = document.createElement('div');
    nodeInfo.className = 'node-info';

    const img = document.createElement('img');
    img.setAttribute('draggable', false);
    img.setAttribute('src', './protoVisualizer/red_texture.jpg');
    nodeInfo.appendChild(img);

    container.appendChild(nodeList);
    container.appendChild(nodeInfo);
    panel.appendChild(container);
  }

  #floatOnChange(node) {
    node.parameter.setValueFromJavaScript(this.#view, node.input.value);
  }

  #createResetButton(parentNode) {
    const resetButton = document.createElement('button');
    resetButton.className = 'reset-field-button';
    resetButton.title = 'Reset to initial value';

    parentNode.appendChild(resetButton);
    return resetButton;
  }

  #createCheckbox(parent, row) {
    const exportCheckbox = document.createElement('input');
    exportCheckbox.type = 'checkbox';
    exportCheckbox.className = 'export-checkbox';
    exportCheckbox.title = 'Field to be exposed';
    exportCheckbox.style.gridRow = '' + row + ' / ' + row;
    exportCheckbox.style.gridColumn = '1 / 1';
    parent.appendChild(exportCheckbox);

    return exportCheckbox;
  }
}
