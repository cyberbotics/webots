/* global THREE, ActiveXObject, TextureLoader, TextureData */
'use strict';

// Inspiration: https://github.com/lkolbly/threejs-x3dloader/blob/master/X3DLoader.js

THREE.X3DLoader = class X3DLoader {
  constructor(scene) {
    this.scene = scene;
    this.parsedObjects = [];
    this.directionalLights = [];

    this.enableHDRReflections = true;
  };

  load(url, onLoad, onProgress, onError) {
    console.log('X3D: Loading ' + url);
    let scope = this;
    let loader = new THREE.FileLoader(scope.manager);
    loader.load(url, (text) => {
      if (typeof onLoad !== 'undefined')
        onLoad(scope.parse(text));
    });
  }

  parse(text, parentObject = undefined) {
    this.directionalLights = [];
    let object;

    let xml = null;
    if (window.DOMParser) {
      let parser = new DOMParser();
      xml = parser.parseFromString(text, 'text/xml');
    } else { // Internet Explorer
      xml = new ActiveXObject('Microsoft.XMLDOM');
      xml.async = false;
      xml.loadXML(text);
    }

    // Parse scene.
    let scene = xml.getElementsByTagName('Scene')[0];
    if (typeof scene !== 'undefined') {
      object = new THREE.Group();
      object.userData.x3dType = 'Group';
      object.name = 'n0';
      this.parsedObjects.push(object); // push before parsing to let _getDefNode work correctly
      this.parseNode(object, scene);
      return this.parsedObjects;
    }

    // Parse objects.
    let rootObjects = [];
    xml.childNodes.forEach((n) => {
      if (n.tagName === 'nodes')
        n.childNodes.forEach((child) => { rootObjects.push(child); });
      else
        rootObjects.push(n);
    });
    while (rootObjects.length > 0) {
      let node = rootObjects.shift(); // get and remove first item
      if (parentObject)
        object = parentObject;
      else
        object = new THREE.Group();
      this.parsedObjects.push(object); // push before parsing
      this.parseNode(object, node);
    }
    return this.parsedObjects;
  }

  parseNode(parentObject, node) {
    let object = this._getDefNode(node);
    if (typeof object !== 'undefined') {
      let useObject = object.clone();
      this._setCustomId(node, useObject, object);
      parentObject.add(useObject);
      return;
    }

    let hasChildren = false;
    let helperNodes = [];
    if (node.tagName === 'Transform') {
      object = this.parseTransform(node);
      hasChildren = true;
    } else if (node.tagName === 'Shape')
      object = this.parseShape(node);
    else if (node.tagName === 'DirectionalLight')
      object = this.parseDirectionalLight(node);
    else if (node.tagName === 'PointLight')
      object = this.parsePointLight(node);
    else if (node.tagName === 'SpotLight')
      object = this.parseSpotLight(node, helperNodes);
    else if (node.tagName === 'Group') {
      object = new THREE.Object3D();
      object.userData.x3dType = 'Group';
      hasChildren = true;
    } else if (node.tagName === 'Switch') {
      object = new THREE.Object3D();
      object.visible = getNodeAttribute(node, 'whichChoice', '-1') !== '-1';
      object.userData.x3dType = 'Switch';
      hasChildren = true;
    } else if (node.tagName === 'Fog')
      this.parseFog(node);
    else if (node.tagName === 'Viewpoint')
      object = this.parseViewpoint(node);
    else if (node.tagName === 'Background')
      object = this.parseBackground(node);
    else if (node.tagName === 'WorldInfo') {
      this.parseWorldInfo(node);
      return;
    } else if (node.tagName === 'Appearance') {
      if (!parentObject.isMesh) {
        console.error("X3DLoader:parsenode: cannot add 'Appearance' node to '" + parentObject.userData.x3dType + "' parent node.");
        return;
      }
      let material = this.parseAppearance(node);
      if (typeof material !== 'undefined')
        parentObject.material = material;
    } else if (node.tagName === 'PBRAppearance') {
      if (!parentObject.isMesh) {
        console.error("X3DLoader:parsenode: cannot add 'Appearance' node to '" + parentObject.userData.x3dType + "' parent node.");
        return;
      }
      let material = this.parsePBRAppearance(node);
      if (typeof material !== 'undefined')
        parentObject.material = material;
    } else if (node.tagName === 'TextureTransform')
      X3DLoader.applyTextureTransformToMaterial(parentObject, this.parseTextureTransform(node));
    else {
      let geometry = this.parseGeometry(node);
      if (typeof geometry !== 'undefined') {
        if (!parentObject.isMesh) {
          console.error("X3DLoader:parsenode: cannot add 'Appearance' node to '" + parentObject.userData.x3dType + "' parent node.");
          return;
        }
        parentObject.geometry = geometry;
      } else {
        // generic node type
        this.parseChildren(node, parentObject);
        return;
      }
    }

    if (typeof object !== 'undefined') {
      if (object.isObject3D) {
        let isInvisible = getNodeAttribute(node, 'render', 'true').toLowerCase() === 'false';
        if (isInvisible && object.visible)
          object.visible = false;
        this._setCustomId(node, object);
        parentObject.add(object);
      }
      let docUrl = getNodeAttribute(node, 'docUrl', '');
      if (docUrl)
        object.userData.docUrl = docUrl;
    }

    if (helperNodes.length > 0) {
      helperNodes.forEach((o) => {
        parentObject.add(o);
      });
    }

    if (hasChildren)
      this.parseChildren(node, object);
  }

  parseChildren(node, currentObject) {
    for (let i = 0; i < node.childNodes.length; i++) {
      let child = node.childNodes[i];
      if (typeof child.tagName !== 'undefined')
        this.parseNode(currentObject, child);
    }
  }

  parseTransform(transform) {
    let object = new THREE.Object3D();
    object.userData.x3dType = 'Transform';
    object.userData.solid = getNodeAttribute(transform, 'solid', 'false').toLowerCase() === 'true';
    object.userData.window = getNodeAttribute(transform, 'window', '');
    let controller = getNodeAttribute(transform, 'controller', undefined);
    if (typeof controller !== 'undefined')
      object.userData.controller = controller;
    object.userData.name = getNodeAttribute(transform, 'name', '');

    let position = convertStringToVec3(getNodeAttribute(transform, 'translation', '0 0 0'));
    object.position.copy(position);
    let scale = convertStringToVec3(getNodeAttribute(transform, 'scale', '1 1 1'));
    object.scale.copy(scale);
    let quaternion = convertStringToQuaternion(getNodeAttribute(transform, 'rotation', '0 1 0 0'));
    object.quaternion.copy(quaternion);

    return object;
  }

  parseShape(shape) {
    let geometry;
    let material;

    for (let i = 0; i < shape.childNodes.length; i++) {
      let child = shape.childNodes[i];
      if (typeof child.tagName === 'undefined')
        continue;

      // Check if USE node and return the DEF node.
      let defObject = this._getDefNode(child);
      if (typeof defObject !== 'undefined') {
        if (defObject.isGeometry || defObject.isBufferGeometry)
          geometry = defObject;
        else if (defObject.isMaterial)
          material = defObject;
        // else error
        continue;
      }

      if (typeof material === 'undefined') {
        if (child.tagName === 'Appearance') {
          // If a sibling PBRAppearance is detected, prefer it.
          let pbrAppearanceChild = false;
          for (let j = 0; j < shape.childNodes.length; j++) {
            let child0 = shape.childNodes[j];
            if (child0.tagName === 'PBRAppearance') {
              pbrAppearanceChild = true;
              break;
            }
          }
          if (pbrAppearanceChild)
            continue;
          material = this.parseAppearance(child);
        } else if (child.tagName === 'PBRAppearance')
          material = this.parsePBRAppearance(child);
        if (typeof material !== 'undefined')
          continue;
      }

      if (typeof geometry === 'undefined') {
        geometry = this.parseGeometry(child);
        if (typeof geometry !== 'undefined')
          continue;
      }

      console.log('X3dLoader: Unknown node: ' + child.tagName);
    }

    // Apply default geometry and/or material.
    if (typeof geometry === 'undefined')
      geometry = createDefaultGeometry();
    if (typeof material === 'undefined')
      material = createDefaultMaterial(geometry);

    let mesh;
    if (geometry.userData.x3dType === 'IndexedLineSet')
      mesh = new THREE.LineSegments(geometry, material);
    else if (geometry.userData.x3dType === 'PointSet')
      mesh = new THREE.Points(geometry, material);
    else
      mesh = new THREE.Mesh(geometry, material);
    mesh.userData.x3dType = 'Shape';

    if (!material.transparent && !material.userData.hasTransparentTexture)
      // Webots transparent object don't cast shadows.
      mesh.castShadow = getNodeAttribute(shape, 'castShadows', 'false').toLowerCase() === 'true';
    mesh.receiveShadow = true;
    mesh.userData.isPickable = getNodeAttribute(shape, 'isPickable', 'true').toLowerCase() === 'true';
    return mesh;
  }

  parseGeometry(node) {
    let geometry;
    if (node.tagName === 'Box')
      geometry = this.parseBox(node);
    else if (node.tagName === 'Cone')
      geometry = this.parseCone(node);
    else if (node.tagName === 'Cylinder')
      geometry = this.parseCylinder(node);
    else if (node.tagName === 'IndexedFaceSet')
      geometry = this.parseIndexedFaceSet(node);
    else if (node.tagName === 'Sphere')
      geometry = this.parseSphere(node);
    else if (node.tagName === 'Plane')
      geometry = this.parsePlane(node);
    else if (node.tagName === 'ElevationGrid')
      geometry = this.parseElevationGrid(node);
    else if (node.tagName === 'IndexedLineSet')
      geometry = this.parseIndexedLineSet(node);
    else if (node.tagName === 'PointSet')
      geometry = this.parsePointSet(node);

    if (typeof geometry !== 'undefined')
      this._setCustomId(node, geometry);
    return geometry;
  }

  parseAppearance(appearance) {
    let mat = new THREE.MeshBasicMaterial({color: 0xffffff});
    mat.userData.x3dType = 'Appearance';

    // Get the Material tag.
    let material = appearance.getElementsByTagName('Material')[0];

    let materialSpecifications = {};
    if (typeof material !== 'undefined') {
      let defMaterial = this._getDefNode(material);
      if (typeof defMaterial !== 'undefined') {
        materialSpecifications = {
          'color': defMaterial.color,
          'specular': defMaterial.specular,
          'emissive': defMaterial.emissive,
          'shininess': defMaterial.shininess
        };
      } else {
        // Pull out the standard colors.
        materialSpecifications = {
          'color': convertStringToColor(getNodeAttribute(material, 'diffuseColor', '0.8 0.8 0.8'), false),
          'specular': convertStringToColor(getNodeAttribute(material, 'specularColor', '0 0 0'), false),
          'emissive': convertStringToColor(getNodeAttribute(material, 'emissiveColor', '0 0 0'), false),
          'shininess': parseFloat(getNodeAttribute(material, 'shininess', '0.2')),
          'transparent': getNodeAttribute(appearance, 'sortType', 'auto') === 'transparent'
        };
      }
    }

    // Check to see if there is a texture.
    let imageTexture = appearance.getElementsByTagName('ImageTexture');
    let colorMap;
    if (imageTexture.length > 0) {
      colorMap = this.parseImageTexture(imageTexture[0], appearance.getElementsByTagName('TextureTransform'));
      if (typeof colorMap !== 'undefined') {
        materialSpecifications.map = colorMap;
        if (colorMap.userData.isTransparent) {
          materialSpecifications.transparent = true;
          materialSpecifications.alphaTest = 0.5; // FIXME needed for example for the target.png in robot_programming.wbt
        }
      }
    }

    mat = new THREE.MeshPhongMaterial(materialSpecifications);
    mat.userData.x3dType = 'Appearance';
    mat.userData.hasTransparentTexture = colorMap && colorMap.userData.isTransparent;
    if (typeof material !== 'undefined')
      this._setCustomId(material, mat);
    this._setCustomId(appearance, mat);
    return mat;
  }

  parsePBRAppearance(pbrAppearance) {
    const roughnessFactor = 2.0; // This factor has been empirically found to match the Webots rendering.

    let isTransparent = false;

    let baseColor = convertStringToColor(getNodeAttribute(pbrAppearance, 'baseColor', '1 1 1'));
    let roughness = parseFloat(getNodeAttribute(pbrAppearance, 'roughness', '0')) * roughnessFactor;
    let metalness = parseFloat(getNodeAttribute(pbrAppearance, 'metalness', '1'));
    let emissiveColor = convertStringToColor(getNodeAttribute(pbrAppearance, 'emissiveColor', '0 0 0'));
    let transparency = parseFloat(getNodeAttribute(pbrAppearance, 'transparency', '0'));
    let materialSpecifications = {
      color: baseColor,
      roughness: roughness,
      metalness: metalness,
      emissive: emissiveColor
    };

    if (transparency) {
      materialSpecifications.opacity = 1.0 - transparency;
      isTransparent = true;
    }

    let textureTransform = pbrAppearance.getElementsByTagName('TextureTransform');
    let imageTextures = pbrAppearance.getElementsByTagName('ImageTexture');
    for (let t = 0; t < imageTextures.length; t++) {
      let imageTexture = imageTextures[t];
      let type = getNodeAttribute(imageTexture, 'type', undefined);
      if (type === 'baseColor') {
        materialSpecifications.map = this.parseImageTexture(imageTexture, textureTransform);
        if (typeof materialSpecifications.map !== 'undefined' && materialSpecifications.map.userData.isTransparent) {
          isTransparent = true;
          materialSpecifications.alphaTest = 0.5; // FIXME needed for example for the target.png in robot_programming.wbt
        }
      } else if (type === 'roughness') {
        materialSpecifications.roughnessMap = this.parseImageTexture(imageTexture, textureTransform);
        if (roughness <= 0.0)
          materialSpecifications.roughness = roughnessFactor;
      } else if (type === 'metalness')
        materialSpecifications.metalnessMap = this.parseImageTexture(imageTexture, textureTransform);
      else if (type === 'normal')
        materialSpecifications.normalMap = this.parseImageTexture(imageTexture, textureTransform);
      else if (type === 'emissiveColor') {
        materialSpecifications.emissiveMap = this.parseImageTexture(imageTexture, textureTransform);
        materialSpecifications.emissive = new THREE.Color(0xffffff);
      }
      /* Ambient occlusion not fully working
      else if (type === 'occlusion')
        materialSpecifications.aoMap = this.parseImageTexture(imageTexture, textureTransform);
      */
    }

    let mat = new THREE.MeshStandardMaterial(materialSpecifications);
    mat.userData.x3dType = 'PBRAppearance';
    if (isTransparent)
      mat.transparent = true;
    mat.userData.hasTransparentTexture = materialSpecifications.map && materialSpecifications.map.userData.isTransparent;
    this._setCustomId(pbrAppearance, mat);
    return mat;
  }

  parseImageTexture(imageTexture, textureTransform, mat) {
    // Issues with DEF and USE image texture with different image transform.
    let texture = this._getDefNode(imageTexture);
    if (typeof texture !== 'undefined')
      return texture;

    let filename = getNodeAttribute(imageTexture, 'url', '');
    filename = filename.split(/['"\s]/).filter((n) => { return n; });
    if (filename[0] == null)
      return undefined;

    let transformData;
    if (textureTransform && textureTransform[0]) {
      let defTexture = this._getDefNode(textureTransform[0]);
      if (typeof defTexture !== 'undefined')
        transformData = defTexture.userData.transform;
      else
        transformData = this.parseTextureTransform(textureTransform[0]);
    }

    // Map ImageTexture.TextureProperties.anisotropicDegree to THREE.Texture.anisotropy.
    let anisotropy = 8; // matches with the default value: `ImageTexture.filtering = 4`
    let textureProperties = imageTexture.getElementsByTagName('TextureProperties');
    if (textureProperties.length > 0)
      anisotropy = parseFloat(getNodeAttribute(textureProperties[0], 'anisotropicDegree', '8'));

    texture = TextureLoader.createOrRetrieveTexture(filename[0], new TextureData(
      getNodeAttribute(imageTexture, 'isTransparent', 'false').toLowerCase() === 'true',
      { 's': getNodeAttribute(imageTexture, 'repeatS', 'true').toLowerCase(),
        't': getNodeAttribute(imageTexture, 'repeatT', 'true').toLowerCase() },
      anisotropy,
      transformData
    ));

    if (textureTransform && textureTransform[0])
      this._setCustomId(textureTransform[0], texture);
    this._setCustomId(imageTexture, texture);
    return texture;
  }

  parseTextureTransform(textureTransform, textureObject = undefined) {
    let transformData = {
      'center': convertStringToVec2(getNodeAttribute(textureTransform, 'center', '0 0')),
      'rotation': parseFloat(getNodeAttribute(textureTransform, 'rotation', '0')),
      'scale': convertStringToVec2(getNodeAttribute(textureTransform, 'scale', '1 1')),
      'translation': convertStringToVec2(getNodeAttribute(textureTransform, 'translation', '0 0'))
    };
    if (typeof textureObject !== 'undefined' && textureObject.isTexture)
      TextureLoader.applyTextureTransform(textureObject, transformData);
    return transformData;
  }

  parseIndexedFaceSet(ifs) {
    let coordinate = ifs.getElementsByTagName('Coordinate')[0];
    let textureCoordinate = ifs.getElementsByTagName('TextureCoordinate')[0];
    let normal = ifs.getElementsByTagName('Normal')[0];

    if (typeof coordinate !== 'undefined' && 'USE' in coordinate.attributes) {
      console.error("X3DLoader:parseIndexedFaceSet: USE 'Coordinate' node not supported.");
      coordinate = undefined;
    }
    if (typeof textureCoordinate !== 'undefined' && 'USE' in textureCoordinate.attributes) {
      console.error("X3DLoader:parseIndexedFaceSet: USE 'TextureCoordinate' node not supported.");
      textureCoordinate = undefined;
    }
    if (typeof normal !== 'undefined' && 'USE' in normal.attributes) {
      console.error("X3DLoader:parseIndexedFaceSet: USE 'Normal' node not supported.");
      normal = undefined;
    }

    let geometry = new THREE.Geometry();
    let x3dType = getNodeAttribute(ifs, 'x3dType', undefined);
    geometry.userData = { 'x3dType': (typeof x3dType === 'undefined' ? 'IndexedFaceSet' : x3dType) };
    if (typeof coordinate === 'undefined')
      return geometry;

    let indicesStr = getNodeAttribute(ifs, 'coordIndex', '').split(/\s/);
    let verticesStr = getNodeAttribute(coordinate, 'point', '').split(/\s/);
    let hasTexCoord = 'texCoordIndex' in ifs.attributes;
    let texcoordIndexStr = hasTexCoord ? getNodeAttribute(ifs, 'texCoordIndex', '') : '';
    let texcoordsStr = hasTexCoord ? getNodeAttribute(textureCoordinate, 'point', '') : '';

    for (let i = 0; i < verticesStr.length; i += 3) {
      let v = new THREE.Vector3();
      v.x = parseFloat(verticesStr[i + 0]);
      v.y = parseFloat(verticesStr[i + 1]);
      v.z = parseFloat(verticesStr[i + 2]);
      geometry.vertices.push(v);
    }

    let normalArray, normalIndicesStr;
    if (typeof normal !== 'undefined') {
      let normalStr = getNodeAttribute(normal, 'vector', '').split(/[\s,]+/);
      normalIndicesStr = getNodeAttribute(ifs, 'normalIndex', '').split(/\s/);
      normalArray = [];
      for (let i = 0; i < normalStr.length; i += 3) {
        normalArray.push(new THREE.Vector3(
          parseFloat(normalStr[i + 0]),
          parseFloat(normalStr[i + 1]),
          parseFloat(normalStr[i + 2])));
      }
    }

    if (hasTexCoord) {
      let isDefaultMapping = getNodeAttribute(ifs, 'defaultMapping', 'false').toLowerCase() === 'true';
      let texcoords = texcoordsStr.split(/\s/);
      let uvs = [];
      for (let i = 0; i < texcoords.length; i += 2) {
        let v = new THREE.Vector2();
        v.x = parseFloat(texcoords[i + 0]);
        v.y = parseFloat(texcoords[i + 1]);
        if (isDefaultMapping) {
          // add small offset to avoid using the exact same texture coordinates for a face
          // (i.e. mapping to a line or a point) that is causing a rendering issue
          // https://github.com/cyberbotics/webots/issues/752
          v.x += 0.01 * Math.random();
          v.y += 0.01 * Math.random();
        }
        uvs.push(v);
      }
    }

    // Now pull out the face indices.
    if (hasTexCoord)
      let texIndices = texcoordIndexStr.split(/\s/);
    for (let i = 0; i < indicesStr.length; i++) {
      let faceIndices = [];
      let uvIndices = [];
      let normalIndices = [];
      while (parseFloat(indicesStr[i]) >= 0) {
        faceIndices.push(parseFloat(indicesStr[i]));
        if (hasTexCoord)
          uvIndices.push(parseFloat(texIndices[i]));
        if (typeof normalIndicesStr !== 'undefined')
          normalIndices.push(parseFloat(normalIndicesStr[i]));
        i++;
      }

      let faceNormal;
      while (faceIndices.length > 3) {
        // Take the last three, make a triangle, and remove the
        // middle one (works for convex & continuously wrapped).
        if (hasTexCoord) {
          // Add to the UV layer.
          geometry.faceVertexUvs[0].push([
            uvs[parseFloat(uvIndices[uvIndices.length - 3])].clone(),
            uvs[parseFloat(uvIndices[uvIndices.length - 2])].clone(),
            uvs[parseFloat(uvIndices[uvIndices.length - 1])].clone()
          ]);
          // Remove the second-to-last vertex.
          let tmp = uvIndices[uvIndices.length - 1];
          uvIndices.pop();
          uvIndices[uvIndices.length - 1] = tmp;
        }

        faceNormal = undefined;
        if (typeof normal !== 'undefined') {
          faceNormal = [
            normalArray[normalIndices[faceIndices.length - 3]],
            normalArray[normalIndices[faceIndices.length - 2]],
            normalArray[normalIndices[faceIndices.length - 1]]];
        }

        // Make a face.
        geometry.faces.push(new THREE.Face3(
          faceIndices[faceIndices.length - 3],
          faceIndices[faceIndices.length - 2],
          faceIndices[faceIndices.length - 1],
          faceNormal
        ));
        // Remove the second-to-last vertex.
        tmp = faceIndices[faceIndices.length - 1];
        faceIndices.pop();
        faceIndices[faceIndices.length - 1] = tmp;
      }

      // Make a face with the final three.
      if (faceIndices.length === 3) {
        if (hasTexCoord) {
          geometry.faceVertexUvs[0].push([
            uvs[parseFloat(uvIndices[uvIndices.length - 3])].clone(),
            uvs[parseFloat(uvIndices[uvIndices.length - 2])].clone(),
            uvs[parseFloat(uvIndices[uvIndices.length - 1])].clone()
          ]);
        }

        if (typeof normal !== 'undefined') {
          faceNormal = [
            normalArray[normalIndices[faceIndices.length - 3]],
            normalArray[normalIndices[faceIndices.length - 2]],
            normalArray[normalIndices[faceIndices.length - 1]]];
        }

        geometry.faces.push(new THREE.Face3(
          faceIndices[0], faceIndices[1], faceIndices[2], faceNormal
        ));
      }
    }

    geometry.computeBoundingSphere();
    if (typeof normal === 'undefined')
      geometry.computeVertexNormals();

    this._setCustomId(coordinate, geometry);
    if (hasTexCoord)
      this._setCustomId(textureCoordinate, geometry);

    return geometry;
  }

  parseIndexedLineSet(ils) {
    let coordinate = ils.getElementsByTagName('Coordinate')[0];
    if (typeof coordinate !== 'undefined' && 'USE' in coordinate.attributes) {
      console.error("X3DLoader:parseIndexedLineSet: USE 'Coordinate' node not supported.");
      coordinate = undefined;
    }

    let geometry = new THREE.BufferGeometry();
    geometry.userData = { 'x3dType': 'IndexedLineSet' };
    if (typeof coordinate === 'undefined')
      return geometry;

    let indicesStr = getNodeAttribute(ils, 'coordIndex', '').trim().split(/\s/);
    let verticesStr = getNodeAttribute(coordinate, 'point', '').trim().split(/\s/);

    let positions = new Float32Array(verticesStr.length * 3);
    for (let i = 0; i < verticesStr.length; i += 3) {
      positions[i] = parseFloat(verticesStr[i + 0]);
      positions[i + 1] = parseFloat(verticesStr[i + 1]);
      positions[i + 2] = parseFloat(verticesStr[i + 2]);
    }

    let indices = [];
    for (let i = 0; i < indicesStr.length; i++) {
      while (parseFloat(indicesStr[i]) >= 0) {
        indices.push(parseFloat(indicesStr[i]));
        i++;
      }
    }

    geometry.setIndex(new THREE.BufferAttribute(new Uint16Array(indices), 1));
    geometry.addAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.computeBoundingSphere();

    this._setCustomId(coordinate, geometry);

    return geometry;
  }

  parseElevationGrid(eg) {
    let heightStr = getNodeAttribute(eg, 'height', undefined);
    let xDimension = parseInt(getNodeAttribute(eg, 'xDimension', '0'));
    let xSpacing = parseFloat(getNodeAttribute(eg, 'xSpacing', '1'));
    let zDimension = parseInt(getNodeAttribute(eg, 'zDimension', '0'));
    let zSpacing = parseFloat(getNodeAttribute(eg, 'zSpacing', '1'));

    let width = (xDimension - 1) * xSpacing;
    let depth = (zDimension - 1) * zSpacing;

    let geometry = new THREE.PlaneBufferGeometry(width, depth, xDimension - 1, zDimension - 1);
    geometry.userData = { 'x3dType': 'ElevationGrid' };
    geometry.rotateX(-Math.PI / 2);
    geometry.translate(width / 2, 0, depth / 2); // center located in the corner
    if (typeof heightStr === 'undefined')
      return geometry;

    // Set height and adjust uv mappings.
    let heightArray = heightStr.trim().split(/\s/);
    let vertices = geometry.getAttribute('position').array;
    let uv = geometry.getAttribute('uv').array;
    let maxIndex = heightArray.length;
    let i = 0;
    let v = 1;
    for (let dx = 0; dx < xDimension; dx++) {
      for (let dz = 0; dz < zDimension; dz++) {
        let index = xDimension * dx + dz;
        if (index < maxIndex)
          vertices[i + 1] = parseFloat(heightArray[index]);
        uv[v] = -uv[v];
        i += 3;
        v += 2;
      }
    }

    geometry.computeVertexNormals();

    return geometry;
  }

  parseBox(box) {
    let size = convertStringToVec3(getNodeAttribute(box, 'size', '2 2 2'));
    let boxGeometry = new THREE.BoxBufferGeometry(size.x, size.y, size.z);
    boxGeometry.userData = { 'x3dType': 'Box' };
    return boxGeometry;
  }

  parseCone(cone) {
    let radius = getNodeAttribute(cone, 'bottomRadius', '1');
    let height = getNodeAttribute(cone, 'height', '2');
    let subdivision = getNodeAttribute(cone, 'subdivision', '32');
    let side = getNodeAttribute(cone, 'side', 'true').toLowerCase() === 'true';
    let bottom = getNodeAttribute(cone, 'bottom', 'true').toLowerCase() === 'true';
    // Note: the three.js Cone is created with thetaStart = Math.PI / 2 to match X3D texture mapping.
    let coneGeometry;
    if (side && bottom)
      coneGeometry = new THREE.ConeBufferGeometry(radius, height, subdivision, 1, false, Math.PI / 2);
    else {
      coneGeometry = new THREE.Geometry();
      if (side) {
        let sideGeometry = new THREE.ConeGeometry(radius, height, subdivision, 1, true, Math.PI / 2);
        coneGeometry.merge(sideGeometry);
      }
      if (bottom) {
        let bottomGeometry = new THREE.CircleGeometry(radius, subdivision);
        let bottomMatrix = new THREE.Matrix4();
        bottomMatrix.makeRotationFromEuler(new THREE.Euler(Math.PI / 2, 0, Math.PI / 2));
        bottomMatrix.setPosition(new THREE.Vector3(0, -height / 2, 0));
        coneGeometry.merge(bottomGeometry, bottomMatrix);
      }
    }
    coneGeometry.userData = { 'x3dType': 'Cone' };
    coneGeometry.rotateY(Math.PI / 2);
    return coneGeometry;
  }

  parseCylinder(cylinder) {
    let radius = getNodeAttribute(cylinder, 'radius', '1');
    let height = getNodeAttribute(cylinder, 'height', '2');
    let subdivision = getNodeAttribute(cylinder, 'subdivision', '32');
    let bottom = getNodeAttribute(cylinder, 'bottom', 'true').toLowerCase() === 'true';
    let side = getNodeAttribute(cylinder, 'side', 'true').toLowerCase() === 'true';
    let top = getNodeAttribute(cylinder, 'top', 'true').toLowerCase() === 'true';
    // Note: the three.js Cylinder is created with thetaStart = Math.PI / 2 to match X3D texture mapping.
    let cylinderGeometry;
    if (bottom && side && top)
      cylinderGeometry = new THREE.CylinderBufferGeometry(radius, radius, height, subdivision, 1, false, Math.PI / 2);
    else {
      cylinderGeometry = new THREE.Geometry();
      if (side) {
        let sideGeometry = new THREE.CylinderGeometry(radius, radius, height, subdivision, 1, true, Math.PI / 2);
        cylinderGeometry.merge(sideGeometry);
      }
      if (top) {
        let topGeometry = new THREE.CircleGeometry(radius, subdivision);
        let topMatrix = new THREE.Matrix4();
        topMatrix.makeRotationFromEuler(new THREE.Euler(-Math.PI / 2, 0, -Math.PI / 2));
        topMatrix.setPosition(new THREE.Vector3(0, height / 2, 0));
        cylinderGeometry.merge(topGeometry, topMatrix);
      }
      if (bottom) {
        let bottomGeometry = new THREE.CircleGeometry(radius, subdivision);
        let bottomMatrix = new THREE.Matrix4();
        bottomMatrix.makeRotationFromEuler(new THREE.Euler(Math.PI / 2, 0, Math.PI / 2));
        bottomMatrix.setPosition(new THREE.Vector3(0, -height / 2, 0));
        cylinderGeometry.merge(bottomGeometry, bottomMatrix);
      }
    }
    cylinderGeometry.userData = { 'x3dType': 'Cylinder' };
    cylinderGeometry.rotateY(Math.PI / 2);
    return cylinderGeometry;
  }

  parseSphere(sphere) {
    let radius = getNodeAttribute(sphere, 'radius', '1');
    let subdivision = getNodeAttribute(sphere, 'subdivision', '8,8').split(',');
    let ico = getNodeAttribute(sphere, 'ico', 'false').toLowerCase() === 'true';
    let sphereGeometry;
    if (ico) {
      sphereGeometry = new THREE.IcosahedronBufferGeometry(radius, subdivision[0]);
      sphereGeometry.rotateY(Math.PI / 2);
    } else
      sphereGeometry = new THREE.SphereBufferGeometry(radius, subdivision[0], subdivision[1], -Math.PI / 2); // thetaStart: -Math.PI/2
    sphereGeometry.userData = { 'x3dType': 'Sphere' };
    return sphereGeometry;
  }

  parsePlane(plane) {
    let size = convertStringToVec2(getNodeAttribute(plane, 'size', '1,1'));
    let planeGeometry = new THREE.PlaneBufferGeometry(size.x, size.y);
    planeGeometry.userData = { 'x3dType': 'Plane' };
    planeGeometry.rotateX(-Math.PI / 2);
    return planeGeometry;
  }

  parsePointSet(pointSet) {
    let coordinate = pointSet.getElementsByTagName('Coordinate')[0];
    let geometry = new THREE.BufferGeometry();
    geometry.userData = { 'x3dType': 'PointSet' };
    if (typeof coordinate === 'undefined')
      return geometry;

    let coordStrArray = getNodeAttribute(coordinate, 'point', '').trim().split(/\s/);
    let color = pointSet.getElementsByTagName('Color')[0];

    let count = coordStrArray.length;
    let colorStrArray;
    geometry.userData.isColorPerVertex = false;
    if (typeof color !== 'undefined') {
      colorStrArray = getNodeAttribute(color, 'color', '').trim().split(/\s/);
      if (typeof colorStrArray !== 'undefined') {
        if (count !== colorStrArray.length) {
          count = Math.min(count, colorStrArray.length);
          console.error("X3DLoader:parsePointSet: 'coord' and 'color' fields size doesn't match.");
        }
        geometry.userData.isColorPerVertex = true;
      }
    }

    let positions = new Float32Array(count);
    for (let i = 0; i < count; i++)
      positions[i] = parseFloat(coordStrArray[i]);
    geometry.addAttribute('position', new THREE.BufferAttribute(positions, 3));

    if (geometry.userData.isColorPerVertex) {
      let colors = new Float32Array(count);
      for (let i = 0; i < count; i++)
        colors[i] = parseFloat(colorStrArray[i]);
      geometry.addAttribute('color', new THREE.BufferAttribute(colors, 3));
    }

    geometry.computeBoundingBox();
    return geometry;
  }

  parseDirectionalLight(light) {
    let on = getNodeAttribute(light, 'on', 'true').toLowerCase() === 'true';
    if (!on)
      return;

    let color = convertStringToColor(getNodeAttribute(light, 'color', '1 1 1'), false);
    let direction = convertStringToVec3(getNodeAttribute(light, 'direction', '0 0 -1'));
    let intensity = parseFloat(getNodeAttribute(light, 'intensity', '1'));
    let castShadows = getNodeAttribute(light, 'castShadows', 'false').toLowerCase() === 'true';

    let lightObject = new THREE.DirectionalLight(color.getHex(), intensity);
    if (castShadows) {
      lightObject.castShadow = true;
      let shadowMapSize = parseFloat(getNodeAttribute(light, 'shadowMapSize', '1024'));
      lightObject.shadow.mapSize.width = shadowMapSize;
      lightObject.shadow.mapSize.height = shadowMapSize;
      lightObject.shadow.radius = parseFloat(getNodeAttribute(light, 'shadowsRadius', '1.5'));
      lightObject.shadow.bias = parseFloat(getNodeAttribute(light, 'shadowBias', '0.000001'));
      lightObject.shadow.camera.near = parseFloat(getNodeAttribute(light, 'zNear', '0.001;'));
      lightObject.shadow.camera.far = parseFloat(getNodeAttribute(light, 'zFar', '2000'));
    }
    lightObject.position.set(-direction.x, -direction.y, -direction.z);
    lightObject.userData = { 'x3dType': 'DirectionalLight' };
    // Position of the directional light will be adjusted at the end of the load
    // based on the size of the scene so that all the objects are illuminated by this light.
    this.directionalLights.push(lightObject);
    return lightObject;
  }

  parsePointLight(light) {
    let on = getNodeAttribute(light, 'on', 'true').toLowerCase() === 'true';
    if (!on)
      return;

    let attenuation = convertStringToVec3(getNodeAttribute(light, 'attenuation', '1 0 0'));
    let color = convertStringToColor(getNodeAttribute(light, 'color', '1 1 1'), false);
    let intensity = parseFloat(getNodeAttribute(light, 'intensity', '1'));
    let location = convertStringToVec3(getNodeAttribute(light, 'location', '0 0 0'));
    let radius = parseFloat(getNodeAttribute(light, 'radius', '100'));
    let castShadows = getNodeAttribute(light, 'castShadows', 'false').toLowerCase() === 'true';

    let lightObject = new THREE.PointLight(color.getHex());

    // Tradeoff to let cohabit VRML light attenuation and the three.js light "physically correct mode".
    // - The intensity is attenuated by the total amount of the VRML attenuation.
    // - The biggest attenuation component defines the `decay` "exponent".
    lightObject.intensity = intensity / attenuation.manhattanLength();
    if (attenuation.x > 0)
      lightObject.decay = 0;
    if (attenuation.y > 0)
      lightObject.decay = 1;
    if (attenuation.z > 0)
      lightObject.decay = 2;
    lightObject.distance = radius;

    if (castShadows) {
      lightObject.castShadow = true;
      let shadowMapSize = parseFloat(getNodeAttribute(light, 'shadowMapSize', '512'));
      lightObject.shadow.mapSize.width = shadowMapSize;
      lightObject.shadow.mapSize.height = shadowMapSize;
      lightObject.shadow.radius = parseFloat(getNodeAttribute(light, 'shadowsRadius', '1'));
      lightObject.shadow.bias = parseFloat(getNodeAttribute(light, 'shadowBias', '0'));
      lightObject.shadow.camera.near = parseFloat(getNodeAttribute(light, 'zNear', '0.001;'));
      lightObject.shadow.camera.far = radius;
    }
    lightObject.position.copy(location);
    lightObject.userData = { 'x3dType': 'PointLight' };
    return lightObject;
  }

  parseSpotLight(light, helperNodes) {
    let on = getNodeAttribute(light, 'on', 'true').toLowerCase() === 'true';
    if (!on)
      return;

    let attenuation = convertStringToVec3(getNodeAttribute(light, 'attenuation', '1 0 0'));
    let beamWidth = parseFloat(getNodeAttribute(light, 'beamWidth', '0.785'));
    let color = convertStringToColor(getNodeAttribute(light, 'color', '1 1 1'), false);
    let cutOffAngle = parseFloat(getNodeAttribute(light, 'cutOffAngle', '0.785'));
    let direction = convertStringToVec3(getNodeAttribute(light, 'direction', '0 0 -1'));
    let intensity = parseFloat(getNodeAttribute(light, 'intensity', '1'));
    let location = convertStringToVec3(getNodeAttribute(light, 'location', '0 0 0'));
    let radius = parseFloat(getNodeAttribute(light, 'radius', '100'));
    let castShadows = getNodeAttribute(light, 'castShadows', 'false').toLowerCase() === 'true';

    let lightObject = new THREE.SpotLight(color.getHex());

    lightObject.intensity = intensity / attenuation.manhattanLength();
    if (attenuation.x > 0)
      lightObject.decay = 0;
    if (attenuation.y > 0)
      lightObject.decay = 1;
    if (attenuation.z > 0)
      lightObject.decay = 2;
    lightObject.distance = radius;

    lightObject.angle = cutOffAngle;
    if (beamWidth > cutOffAngle)
      lightObject.penumbra = 0.0;
    else
      lightObject.penumbra = 1.0 - (beamWidth / cutOffAngle);

    if (castShadows) {
      lightObject.castShadow = true;
      let shadowMapSize = parseFloat(getNodeAttribute(light, 'shadowMapSize', '512'));
      lightObject.shadow.mapSize.width = shadowMapSize;
      lightObject.shadow.mapSize.height = shadowMapSize;
      lightObject.shadow.radius = parseFloat(getNodeAttribute(light, 'shadowsRadius', '1'));
      lightObject.shadow.bias = parseFloat(getNodeAttribute(light, 'shadowBias', '0'));
      lightObject.shadow.camera.near = parseFloat(getNodeAttribute(light, 'zNear', '0.001;'));
      lightObject.shadow.camera.far = radius;
    }
    lightObject.position.copy(location);
    lightObject.target = new THREE.Object3D();
    lightObject.target.position.addVectors(lightObject.position, direction);
    lightObject.target.userData.x3dType = 'LightTarget';
    helperNodes.push(lightObject.target);
    lightObject.userData = { 'x3dType': 'SpotLight' };
    return lightObject;
  }

  parseBackground(background) {
    this.scene.scene.background = convertStringToColor(getNodeAttribute(background, 'skyColor', '0 0 0'));
    this.scene.scene.userData.luminosity = parseFloat(getNodeAttribute(background, 'luminosity', '1.0'));
    this.scene.scene.irradiance = undefined;

    let backgroundEnabled = false;
    let irradianceEnabled = false;

    let backgroundFields = ['leftUrl', 'rightUrl', 'topUrl', 'bottomUrl', 'backUrl', 'frontUrl'];
    let irradianceFields = ['leftIrradianceUrl', 'rightIrradianceUrl', 'topIrradianceUrl', 'bottomIrradianceUrl', 'backIrradianceUrl', 'frontIrradianceUrl'];

    let backgroundURLs = [];
    let irradianceURLs = [];
    for (let i = 0; i < 6; i++) {
      let url = getNodeAttribute(background, backgroundFields[i], undefined);
      if (typeof url !== 'undefined') {
        backgroundEnabled = true;
        url = url.split(/['"\s]/).filter((n) => { return n; })[0];
      }
      backgroundURLs.push(url);

      url = getNodeAttribute(background, irradianceFields[i], undefined);
      if (typeof url !== 'undefined') {
        irradianceEnabled = true;
        url = url.split(/['"\s]/).filter((n) => { return n; })[0];
      }
      irradianceURLs.push(url);
    }

    if (backgroundEnabled) {
      let cubeTexture = new THREE.CubeTexture();
      cubeTexture.encoding = THREE.sRGBEncoding;

      for (let i = 0; i < 6; i++) {
        if (typeof backgroundURLs[i] === 'undefined')
          continue;
        // Look for already loaded texture or load the texture in an asynchronous way.
        TextureLoader.loadOrRetrieveImage(backgroundURLs[i], cubeTexture, i);
      }
      this.scene.scene.background = cubeTexture;
      cubeTexture.needsUpdate = true;
    }

    if (irradianceEnabled && this.enableHDRReflections) {
      let cubeTexture = new THREE.CubeTexture();
      cubeTexture.format = THREE.RGBFormat;
      cubeTexture.type = THREE.FloatType;

      for (let i = 0; i < 6; i++) {
        if (typeof irradianceURLs[i] === 'undefined')
          continue;
        // Look for already loaded texture or load the texture in an asynchronous way.
        TextureLoader.loadOrRetrieveImage(irradianceURLs[i], cubeTexture, i);
      }
      this.scene.scene.userData.irradiance = cubeTexture;
      cubeTexture.needsUpdate = true;
    }

    // Light offset: empirically found to match the Webots rendering.
    let ambientLight = new THREE.AmbientLight(0xffffff);
    this.scene.scene.add(ambientLight);

    return undefined;
  }

  parseViewpoint(viewpoint) {
    let fov = parseFloat(getNodeAttribute(viewpoint, 'fieldOfView', '0.785'));
    let near = parseFloat(getNodeAttribute(viewpoint, 'zNear', '0.1'));
    let far = parseFloat(getNodeAttribute(viewpoint, 'zFar', '2000'));
    if (typeof this.scene.viewpoint !== 'undefined') {
      this.scene.viewpoint.camera.near = near;
      this.scene.viewpoint.camera.far = far;
    } else {
      console.log('Parse Viewpoint: error camera');
      // Set default aspect ratio to 1. It will be updated on window resize.
      this.scene.viewpoint.camera = new THREE.PerspectiveCamera(0.785, 1, near, far);
    }

    // camera.fov should be updated at each window resize.
    this.scene.viewpoint.camera.fovX = fov; // radians
    this.scene.viewpoint.camera.fov = THREE.Math.radToDeg(horizontalToVerticalFieldOfView(fov, this.scene.viewpoint.camera.aspect)); // degrees

    if ('position' in viewpoint.attributes) {
      let position = getNodeAttribute(viewpoint, 'position', '0 0 10');
      this.scene.viewpoint.camera.position.copy(convertStringToVec3(position));
    }
    if ('orientation' in viewpoint.attributes) {
      let quaternion = convertStringToQuaternion(getNodeAttribute(viewpoint, 'orientation', '0 1 0 0'));
      this.scene.viewpoint.camera.quaternion.copy(quaternion);
    }
    this.scene.viewpoint.camera.updateProjectionMatrix();

    // Set Webots specific attributes.
    this.scene.viewpoint.camera.userData.x3dType = 'Viewpoint';
    this.scene.viewpoint.camera.userData.followedId = getNodeAttribute(viewpoint, 'followedId', null);
    this.scene.viewpoint.camera.userData.followSmoothness = getNodeAttribute(viewpoint, 'followSmoothness', null);
    this.scene.viewpoint.camera.userData.exposure = parseFloat(getNodeAttribute(viewpoint, 'exposure', '1.0'));
    return undefined;
  }

  parseWorldInfo(worldInfo) {
    this.scene.worldInfo.title = getNodeAttribute(worldInfo, 'title', '');
    this.scene.worldInfo.window = getNodeAttribute(worldInfo, 'window', '');
  }

  parseFog(fog) {
    let colorInt = convertStringToColor(getNodeAttribute(fog, 'color', '1 1 1'), false).getHex();
    let visibilityRange = parseFloat(getNodeAttribute(fog, 'visibilityRange', '0'));

    let fogObject = null;
    let fogType = getNodeAttribute(fog, 'fogType', 'LINEAR');
    if (fogType === 'LINEAR')
      fogObject = new THREE.Fog(colorInt, 0.001, visibilityRange);
    else
      fogObject = new THREE.FogExp2(colorInt, 1.0 / visibilityRange);
    this.scene.scene.fog = fogObject;
    return undefined;
  }

  _setCustomId(node, object, defNode) {
    // Some THREE.js nodes, like the material and IndexedFaceSet, merges multiple X3D nodes.
    // In order to be able to retrieve the node to be updated, we need to assign to the object all the ids of the merged X3D nodes.
    if (!node || !object)
      return;
    let id = getNodeAttribute(node, 'id', undefined);
    if (typeof id !== 'undefined') {
      if (object.name !== '')
        object.name = object.name + ';' + String(id);
      else
        object.name = String(id);
      if (defNode) {
        if (typeof defNode.userData.USE === 'undefined')
          defNode.userData.USE = String(id);
        else
          defNode.userData.USE = defNode.userData.USE + ';' + String(id);
      }
    }
  }

  _getDefNode(node) {
    let useNodeId = getNodeAttribute(node, 'USE', undefined);
    if (typeof useNodeId === 'undefined')
      return undefined;

    // Look for node in previously parsed objects
    let defNode = this.scene.getObjectById(useNodeId, false, this.parsedObjects);
    if (typeof defNode !== 'undefined')
      return defNode;

    // Look for node in the already loaded scene
    defNode = this.scene.getObjectById(useNodeId, false, this.scene.root);
    if (typeof defNode === 'undefined')
      console.error('X3dLoader: no matching DEF node "' + useNodeId + '" node.');
    return defNode;
  }

  static applyTextureTransformToMaterial(material, textureTransform) {
    if (typeof material === 'undefined' || !material.isMaterial) {
      console.error('X3DLoader:parseTextureTransform: invalid parent object.');
      return;
    }
    let maps = [material.map, material.roughnessMap, material.metalnessMap, material.normalMap, material.emissiveMap, material.aoMap];
    maps.forEach((map) => {
      if (map && map.isTexture)
        TextureLoader.applyTextureTransform(map, textureTransform);
    });
  }
};

function getNodeAttribute(node, attributeName, defaultValue) {
  console.assert(node && node.attributes);
  if (attributeName in node.attributes)
    return node.attributes.getNamedItem(attributeName).value;
  return defaultValue;
}

function createDefaultGeometry() {
  let geometry = new THREE.Geometry();
  geometry.userData = { 'x3dType': 'unknown' };
  return geometry;
};

function createDefaultMaterial(geometry) {
  let material;
  if (typeof geometry !== 'undefined' && geometry.userData.x3dType === 'PointSet' && geometry.userData.isColorPerVertex)
    material = new THREE.PointsMaterial({ size: 4, sizeAttenuation: false, vertexColors: THREE.VertexColors });
  else
    material = new THREE.MeshBasicMaterial({color: 0xffffff});
  return material;
};

function convertStringToVec2(s) {
  s = s.split(/\s/);
  let v = new THREE.Vector2(parseFloat(s[0]), parseFloat(s[1]));
  return v;
}

function convertStringToVec3(s) {
  s = s.split(/\s/);
  let v = new THREE.Vector3(parseFloat(s[0]), parseFloat(s[1]), parseFloat(s[2]));
  return v;
}

function convertStringToQuaternion(s) {
  let pos = s.split(/\s/);
  let q = new THREE.Quaternion();
  q.setFromAxisAngle(
    new THREE.Vector3(parseFloat(pos[0]), parseFloat(pos[1]), parseFloat(pos[2])),
    parseFloat(pos[3])
  );
  return q;
}

function convertStringToColor(s, sRGB = true) {
  let v = convertStringToVec3(s);
  let color = new THREE.Color(v.x, v.y, v.z);
  if (sRGB)
    color.convertSRGBToLinear();
  return color;
}

function horizontalToVerticalFieldOfView(hFov, aspectRatio) {
  // Units of the angles: radians.
  // reference: WbViewpoint::updateFieldOfViewY()

  // According to VRML standards, the meaning of mFieldOfView depends on the aspect ratio:
  // the view angle is taken with respect to the largest dimension

  if (aspectRatio < 1.0)
    return hFov;

  return 2.0 * Math.atan(Math.tan(0.5 * hFov) / aspectRatio);
}

THREE.X3DLoader.textures = {};
