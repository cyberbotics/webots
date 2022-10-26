import Node from './protoVisualizer/Node.js';

import {getAnId} from './nodes/utils/id_provider.js';

export default class ProtoManager {
  #view;
  constructor(view) {
    this.#view = view;
    this.exposedParameters = new Map();
  }

  get view() {
    return this.#view;
  }

  set view(v) {
    this.#view = v;
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
      this.proto = new Node(url, text);
      await this.proto.generateInterface();
      this.proto.parseBody();
      this.loadX3d();
      // generate list of exposed parameters (that will be tied to interface elements)
      console.log('EXPOSED PARAMETERS ARE:');
      for (const [parameterName, parameter] of this.proto.parameters) {
        console.log('> ', parameterName);
        this.exposedParameters.set(parameterName, parameter); // TODO: change key to parameter id ?
      }

      // test this using the world: DemoRegeneration.proto in the html
      // setTimeout(() => this.demoRegeneration(), 2000);

      // test this using the world: DemoFieldChange.proto in the html
      setTimeout(() => this.demoFieldChange('color', {r: 0, g: 0, b: 1}), 2000);
      setTimeout(() => this.demoFieldChange('translation', {x: 0, y: 0, z: 0.5}), 2000);
      setTimeout(() => this.demoFieldChange('rotation', {x: 0, y: 0, z: 1, a: 0.785}), 2000);

      // not implemented yet JS side
      // setTimeout(() => this.demoFieldChange('radius', 0.2), 2000);
      // setTimeout(() => this.demoFieldChange('subdivision', 5), 2000);
    });
  }

  loadX3d() {
    const x3d = new XMLSerializer().serializeToString(this.proto.toX3d());
    console.log(x3d);
    this.#view.prefix = this.url.substr(0, this.url.lastIndexOf('/') + 1);
    this.#view.x3dScene.loadObject('<nodes>' + x3d + '</nodes>', this.parentId);
  }

  async demoRegeneration() {
    const parameterName = 'flag2'; // parameter to change
    const newValue = true; // new value to set

    // get reference to the parameter being changed
    const parameter = this.exposedParameters.get(parameterName);
    parameter.setValueFromJavaScript(this.view, newValue);
  }

  async demoFieldChange(parameterName, newValue) {
    // get reference to the parameter being changed
    const parameter = this.exposedParameters.get(parameterName);
    parameter.setValueFromJavaScript(this.view, newValue);
  }

  loadMinimalScene() {
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

    const x3d = new XMLSerializer().serializeToString(xml);
    this.#view.open(x3d, 'x3d', '', true);
  }
}
