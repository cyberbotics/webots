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
      this.proto = await Node.createNode(url, undefined, true);
      this.loadX3d();
    });
  }

  async loadX3d() {
    const xml = this.getXmlOfMinimalScene();
    const scene = xml.getElementsByTagName('Scene')[0];
    if (this.proto.isRoot && ['PBRAppearance', 'Appearance'].includes(this.proto.getBaseNode().name)) {
      const wrapper = this.#createAppearanceWrapper();
      scene.appendChild(wrapper);
    } else if (this.proto.isRoot && ['Box', 'Capsule', 'Cone', 'Cylinder', 'ElevationGrid', 'IndexedFaceSet', 'IndexedLineSet',
      'Mesh', 'Plane', 'PointSet', 'Sphere'].includes(this.proto.getBaseNode().name)) {
      const wrapper = this.#createGeometryWrapper();
      scene.appendChild(wrapper);
    } else
      scene.appendChild(this.proto.toX3d());

    const x3d = new XMLSerializer().serializeToString(xml);

    this.#view.prefix = this.url.substr(0, this.url.lastIndexOf('/') + 1);
    this.#view.open(x3d, 'x3d', '', true);
  }

  exportProto(name, fieldsToExport) {
    function indent(depth) {
      return ' '.repeat(depth);
    }

    function listExternProto(node, list) {
      for (const parameter of node.parameters.values()) {
        if (parameter.isDefault())
          continue;
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

    const floor = xml.createElement('Pose');
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
    imageTexture.setAttribute('role', 'baseColorMap');
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

  #createAppearanceWrapper() {
    const xml = document.implementation.createDocument('', '', null);
    const pose = xml.createElement('Pose');
    pose.setAttribute('id', getAnId);
    pose.setAttribute('translation', '0 0 0.1');

    const shape = xml.createElement('Shape');
    shape.setAttribute('id', getAnId());

    const sphere = xml.createElement('Sphere');
    sphere.setAttribute('id', getAnId());
    sphere.setAttribute('radius', 0.1);
    sphere.setAttribute('ico', 'FALSE');

    shape.appendChild(sphere);
    shape.appendChild(this.proto.toX3d());
    pose.appendChild(shape);
    return pose;
  }

  #createGeometryWrapper() {
    const xml = document.implementation.createDocument('', '', null);
    const pose = xml.createElement('Pose');
    pose.setAttribute('id', getAnId);
    pose.setAttribute('translation', '0 0 0.05');

    const shape = xml.createElement('Shape');
    shape.setAttribute('id', getAnId());

    const appearance = xml.createElement('PBRAppearance');
    appearance.setAttribute('id', getAnId());
    appearance.setAttribute('roughness', 0.5);
    appearance.setAttribute('metalness', 0);

    const imageTexture = xml.createElement('ImageTexture');
    imageTexture.setAttribute('url', 'https://raw.githubusercontent.com/cyberbotics/webots/released/projects/default/worlds/textures/tagged_wall.jpg');
    imageTexture.setAttribute('role', 'baseColorMap');
    appearance.appendChild(imageTexture);

    shape.appendChild(appearance);
    shape.appendChild(this.proto.toX3d());
    pose.appendChild(shape);
    return pose;
  }
}
