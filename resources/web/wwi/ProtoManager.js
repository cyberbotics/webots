import Node from './protoVisualizer/Node.js';
import { MFNode, SFNode, stringifyType } from './protoVisualizer/Vrml.js';
import {getAnId} from './nodes/utils/id_provider.js';

export default class ProtoManager {
  #view;
  constructor(view) {
    this.#view = view;
    this.exposedParameters = new Map();
  }

  async loadProto(url, parentId) {
    this.url = url;
    this.parentId = parentId;
    return new Promise((resolve, reject) => {
      const xmlhttp = new XMLHttpRequest();
      xmlhttp.open('GET', url, true);
      xmlhttp.overrideMimeType('plain/text');
      xmlhttp.onreadystatechange = async() => {
        if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
          resolve(xmlhttp.responseText);
      };
      xmlhttp.send();
    }).then(async text => {
      console.log('Load PROTO from URL: ' + url);
      this.proto = new Node(url, text, true);
      await this.proto.generateInterface();
      this.proto.parseBody();
      this.loadX3d();
      // generate list of exposed parameters (that will be tied to interface elements)
      console.log('EXPOSED PARAMETERS ARE:');
      for (const [parameterName, parameter] of this.proto.parameters) {
        console.log('> ', parameterName);
        parameter._view = this.#view;
        this.exposedParameters.set(parameterName, parameter); // TODO: change key to parameter id ?
      }
    });
  }

  async generateNodeFromUrl(url) {
    return new Promise((resolve, reject) => {
      const xmlhttp = new XMLHttpRequest();
      xmlhttp.open('GET', url, true);
      xmlhttp.overrideMimeType('plain/text');
      xmlhttp.onreadystatechange = async() => {
        if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
          resolve(xmlhttp.responseText);
      };
      xmlhttp.send();
    }).then(async text => {
      console.log('Load PROTO from URL: ' + url);
      const node = new Node(url, text);
      await node.generateInterface();
      node.parseBody();
      return node;
    });
  }

  async loadX3d() {
    let xml = this.getXmlOfMinimalScene();
    const scene = xml.getElementsByTagName('Scene')[0];
    scene.appendChild(this.proto.toX3d());
    const x3d = new XMLSerializer().serializeToString(xml);
    this.#view.prefix = this.url.substr(0, this.url.lastIndexOf('/') + 1);
    this.#view.open(x3d, 'x3d', '', true);
  }

  async demoRegeneration() {
    const parameterName = 'flag2'; // parameter to change
    const newValue = true; // new value to set

    // get reference to the parameter being changed
    const parameter = this.exposedParameters.get(parameterName);
    parameter.setValueFromJavaScript(this.#view, newValue);
  }

  exportProto(name, fieldsToExport) {
    function indent(depth) {
      return ' '.repeat(depth);
    }

    function listExternProto(node, list) {
      for (const parameter of node.parameters.values()) {
        const currentValue = parameter.value;
        if (currentValue instanceof SFNode && currentValue.value !== null) {
          if (currentValue.value.isProto && !list.includes(currentValue.value.url)) {
            list.push(currentValue.value.url);
            listExternProto(currentValue.value, list);
          }
        }

        if (parameter.value instanceof MFNode && currentValue.value.length > 0) {
          for (const item of currentValue.value) {
            if (item.value.isProto && !list.includes(item.value.url)) {
              list.push(item.value.url);
              listExternProto(item.value, list);
            }
          }
        }
      }
    }

    // write PROTO contents
    let s = '';
    s += '#VRML_SIM R2023b utf8\n';
    s += '\n';

    const externProto = [this.proto.url];
    listExternProto(this.proto, externProto);
    for (const item of externProto)
      s += `EXTERNPROTO "${item}"\n`;

    s += '\n';
    s += `PROTO ${name} [\n`;

    for (const parameter of this.proto.parameters.values()) {
      if (fieldsToExport.has(parameter.name))
        s += `${indent(2)}field ${stringifyType(parameter.type)} ${parameter.name} ${parameter.value.toVrml()}\n`;
    }

    s += ']\n';
    s += '{\n';

    s += `${indent(2)}${this.proto.name} {\n`;
    for (const parameter of this.proto.parameters.values()) {
      if (fieldsToExport.has(parameter.name))
        s += `${indent(4)}${parameter.name} IS ${parameter.name}\n`;
      else if (!parameter.isDefault())
        s += `${indent(4)}${parameter.name} ${parameter.value.toVrml()}\n`;
    }
    s += `${indent(2)}}\n`;
    s += '}\n';

    console.log(s);
    return s;
  }

  getXmlOfMinimalScene() {
    const xml = document.implementation.createDocument('', '', null);
    const scene = xml.createElement('Scene');

    const worldinfo = xml.createElement('WorldInfo');
    worldinfo.setAttribute('id', getAnId());
    worldinfo.setAttribute('basicTimeStep', '32');

    const viewpoint = xml.createElement('Viewpoint');
    viewpoint.setAttribute('id', getAnId());
    viewpoint.setAttribute('position', '2 0 0.6');
    viewpoint.setAttribute('orientation', '-0.125 0 0.992 -3.12');
    viewpoint.setAttribute('exposure', '1');
    viewpoint.setAttribute('bloomThreshold', '21');
    viewpoint.setAttribute('zNear', '0.05');
    viewpoint.setAttribute('zFar', '0');
    viewpoint.setAttribute('ambientOcclusionRadius', '2');

    const background = xml.createElement('Background');
    background.setAttribute('id', getAnId());
    background.setAttribute('skyColor', '0.7 0.7 0.7');
    background.setAttribute('luminosity', '0.8');

    const directionalLight = xml.createElement('DirectionalLight');
    directionalLight.setAttribute('id', getAnId());
    directionalLight.setAttribute('direction', '0.55 -0.6 -1');
    directionalLight.setAttribute('intensity', '2.7');
    directionalLight.setAttribute('ambientIntensity', '1');
    directionalLight.setAttribute('castShadows', 'true');

    const floor = xml.createElement('Transform');
    floor.setAttribute('id', getAnId());
    const shape = xml.createElement('Shape');
    shape.setAttribute('id', getAnId());
    shape.setAttribute('castShadows', 'false');
    const appearance = xml.createElement('PBRAppearance');
    appearance.setAttribute('id', getAnId());
    appearance.setAttribute('roughness', '1');
    appearance.setAttribute('metalness', '0');
    appearance.setAttribute('baseColor', '0.8 0.8 0.8');
    const imageTexture = xml.createElement('ImageTexture');
    imageTexture.setAttribute('id', getAnId());
    imageTexture.setAttribute('url', 'https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/default/worlds/textures/grid.png');
    imageTexture.setAttribute('role', 'baseColor');
    appearance.appendChild(imageTexture);
    const textureTransform = xml.createElement('TextureTransform');
    textureTransform.setAttribute('id', getAnId());
    textureTransform.setAttribute('scale', '50 50');
    appearance.appendChild(textureTransform);
    const geometry = xml.createElement('Plane');
    geometry.setAttribute('id', getAnId());
    geometry.setAttribute('size', '10 10');

    shape.appendChild(appearance);
    shape.appendChild(geometry);
    floor.appendChild(shape);

    scene.appendChild(worldinfo);
    scene.appendChild(viewpoint);
    scene.appendChild(background);
    scene.appendChild(directionalLight);
    scene.appendChild(floor);
    xml.appendChild(scene);

    return xml;
  }
}
