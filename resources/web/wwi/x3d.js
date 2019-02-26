/* global THREE, ActiveXObject, TextureManager */
'use strict';

// Inspiration: https://github.com/lkolbly/threejs-x3dloader/blob/master/X3DLoader.js

THREE.X3DLoader = function(scene, manager) {
  this.manager = (typeof manager !== 'undefined') ? manager : THREE.DefaultLoadingManager;
  this.scene = scene;
};

THREE.X3DLoader.prototype = {
  constructor: THREE.X3DLoader,

  load: function(url, onLoad, onProgress, onError) {
    console.log('X3D: Loading ' + url);
    var scope = this;
    var loader = new THREE.FileLoader(scope.manager);
    loader.load(url, function(text) {
      if (typeof onLoad !== 'undefined')
        onLoad(scope.parse(text));
    });
  },

  parse: function(text) {
    var object = new THREE.Object3D();

    console.log('X3D: Parsing');

    var xml = null;
    if (window.DOMParser) {
      var parser = new DOMParser();
      xml = parser.parseFromString(text, 'text/xml');
    } else { // Internet Explorer
      xml = new ActiveXObject('Microsoft.XMLDOM');
      xml.async = false;
      xml.loadXML(text);
    }

    var scene = xml.getElementsByTagName('Scene')[0];
    this.parseNode(object, scene);
    object.userData.x3dType = 'Group';

    return object;
  },

  parseNode: function(parentObject, node) {
    console.log('Parse Node');

    var object = null;
    if (node.tagName === 'Transform')
      object = this.parseTransform(node);
    else if (node.tagName === 'Shape')
      object = this.parseShape(node);
    else if (node.tagName === 'DirectionalLight')
      object = this.parseDirectionalLight(node, this.scene);
    else if (node.tagName === 'Group') {
      console.log('Parse Group', object);
      object = new THREE.Object3D();
      object.userData.x3dType = 'Group';
      this.parseChildren(node, object);
    } else
      this.parseChildren(node, parentObject);

    if (object)
      parentObject.add(object);
  },

  parseChildren: function(node, currentObject) {
    // Parse children.
    for (let i = 0; i < node.childNodes.length; i++) {
      var child = node.childNodes[i];
      if (typeof child.tagName !== 'undefined')
        this.parseNode(currentObject, child);
    }
  },

  parseTransform: function(transform) {
    console.log('Parse Transform');

    var object = new THREE.Object3D();
    object.userData.x3dType = 'Transform';

    var position = convertStringToVec3(getNodeAttribute(transform, 'translation', '0 0 0'));
    object.position.copy(position);
    var scale = convertStringToVec3(getNodeAttribute(transform, 'scale', '1 1 1'));
    object.scale.copy(scale);
    var quaternion = convertStringToQuaternion(getNodeAttribute(transform, 'rotation', '0 1 0 0'));
    object.quaternion.copy(quaternion);

    this.parseChildren(transform, object);
    return object;
  },

  parseShape: function(shape) {
    console.log('Parse Shape');

    var geometry = new THREE.Geometry();
    var material = new THREE.MeshBasicMaterial({color: 0xffffff});

    for (let i = 0; i < shape.childNodes.length; i++) {
      var child = shape.childNodes[i];
      if (typeof child.tagName === 'undefined')
        continue;
      if (child.tagName === 'Appearance')
        material = this.parseAppearance(child);
      else if (child.tagName === 'PBRAppearance')
        material = this.parsePBRAppearance(child);
      else if (child.tagName === 'IndexedFaceSet')
        geometry = this.parseIndexedFaceSet(child);
      else if (child.tagName === 'Sphere')
        geometry = this.parseSphere(child);
      else
        console.log('Unknown node: ' + child.tagName);
    }

    var mesh = new THREE.Mesh(geometry, material);
    mesh.userData.x3dType = 'Shape';
    return mesh;
  },

  parseAppearance: function(appearance) {
    console.log('Parse Appearance');

    var mat = new THREE.MeshBasicMaterial({color: 0xffffff});
    mat.userData.x3dType = 'Appearance';

    // Get the Material tag
    var material = appearance.getElementsByTagName('Material')[0];
    if (typeof material === 'undefined')
      return mat;

    // Pull out the standard colors
    var diffuse = convertStringTorgb(getNodeAttribute(material, 'diffuseColor', '0.8 0.8 0.8'));
    var specular = convertStringTorgb(getNodeAttribute(material, 'specularColor', '0 0 0'));
    var emissive = convertStringTorgb(getNodeAttribute(material, 'emissiveColor', '0 0 0'));
    var shininess = parseFloat(getNodeAttribute(material, 'shininess', '0.2'));

    // Check to see if there is a texture
    var imageTexture = appearance.getElementsByTagName('ImageTexture');
    var colorMap;
    if (imageTexture.length > 0)
      colorMap = this.parseImageTexture(imageTexture[0]);

    var materialSpecifications = {color: diffuse, specular: specular, emissive: emissive, shininess: shininess};
    if (colorMap)
      materialSpecifications.map = colorMap;

    mat = new THREE.MeshPhongMaterial(materialSpecifications);
    mat.userData.x3dType = 'Appearance';

    return mat;
  },

  parsePBRAppearance: function(pbrAppearance) {
    console.log('Parse PBRAppearance');

    var baseColor = convertStringTorgb(getNodeAttribute(pbrAppearance, 'baseColor', '1 1 1'));
    var roughness = parseFloat(getNodeAttribute(pbrAppearance, 'roughness', '0'));
    var metalness = parseFloat(getNodeAttribute(pbrAppearance, 'metalness', '1'));
    var emissiveColor = convertStringTorgb(getNodeAttribute(pbrAppearance, 'emissiveColor', '0 0 0'));

    var materialSpecifications = {
      color: baseColor,
      roughness: roughness,
      metalness: metalness,
      emissive: emissiveColor
    };

    var imageTextures = pbrAppearance.getElementsByTagName('ImageTexture');
    for (let t = 0; t < imageTextures.length; t++) {
      var imageTexture = imageTextures[t];
      var type = getNodeAttribute(imageTexture, 'type', '');
      if (type === 'baseColor')
        materialSpecifications.map = this.parseImageTexture(imageTexture);
      // else if (type === 'occlusion')  // Not working as expected.
      //   materialSpecifications.aoMap = this.parseImageTexture(imageTexture);
      else if (type === 'roughness') {
        materialSpecifications.roughnessMap = this.parseImageTexture(imageTexture);
        materialSpecifications.roughness = 1.0;
      } else if (type === 'metalness')
        materialSpecifications.metalnessMap = this.parseImageTexture(imageTexture);
      else if (type === 'normal')
        materialSpecifications.normalMap = this.parseImageTexture(imageTexture);
      else if (type === 'emissive')
        materialSpecifications.emissiveMap = this.parseImageTexture(imageTexture);
    }

    var loader = new THREE.CubeTextureLoader();
    loader.setPath('/robot-designer/assets/common/textures/cubic/');
    materialSpecifications.envMap = loader.load([
      'noon_sunny_empty_right.jpg', 'noon_sunny_empty_left.jpg',
      'noon_sunny_empty_top.jpg', 'noon_sunny_empty_bottom.jpg',
      'noon_sunny_empty_front.jpg', 'noon_sunny_empty_back.jpg'
    ]);

    var mat = new THREE.MeshStandardMaterial(materialSpecifications);
    mat.userData.x3dType = 'PBRAppearance';

    return mat;
  },

  parseImageTexture: function(imageTexture) {
    console.log('Parse ImageTexture');
    // Possible improvement: load the texture in an asynchronous way.

    var filename = getNodeAttribute(imageTexture, 'url', '');
    filename = filename.split(/['"\s]/).filter(n => n);

    // look for already loaded textures
    var textureManager = new TextureManager();
    var loadedTexture = textureManager.getTexture(filename);
    if (loadedTexture)
      return loadedTexture;

    var loader = new THREE.TextureLoader();
    var texture = loader.load(
      filename[0],
      (texture) => { // onLoad callback
        if (typeof this.ontextureload !== 'undefined')
          this.ontextureload();
      },
      undefined, // onProgress callback
      (err) => { // onError callback
        console.error('An error happened when loading' + filename + ': ' + err);
      }
    );
    if (loadedTexture)
      return loadedTexture;
    return texture;
  },

  parseIndexedFaceSet: function(ifs) {
    console.log('Parse IndexedFaceSet');

    var coordinate = ifs.getElementsByTagName('Coordinate')[0];
    var textureCoordinate = ifs.getElementsByTagName('TextureCoordinate')[0];

    var geometry = new THREE.Geometry();
    geometry.userData = { 'x3dType': 'IndexedFaceSet' };

    var indices = getNodeAttribute(ifs, 'coordIndex', '').split(/\s/);
    var verticesStr = getNodeAttribute(coordinate, 'point', '');
    var hasTexCoord = 'texCoordIndex' in ifs.attributes;
    var texcoordIndexStr = hasTexCoord ? getNodeAttribute(ifs, 'texCoordIndex', '') : '';
    var texcoordsStr = hasTexCoord ? getNodeAttribute(textureCoordinate, 'point', '') : '';
    var creaseAngle = 0.8 * parseFloat(getNodeAttribute(ifs, 'creaseAngle', '0')); // 0.8 factor empirically found.

    var verts = verticesStr.split(/\s/);
    for (let i = 0; i < verts.length; i += 3) {
      var v = new THREE.Vector3();
      v.x = parseFloat(verts[i + 0]);
      v.y = parseFloat(verts[i + 1]);
      v.z = parseFloat(verts[i + 2]);
      geometry.vertices.push(v);
    }

    if (hasTexCoord) {
      var texcoords = texcoordsStr.split(/\s/);
      var uvs = [];
      for (let i = 0; i < texcoords.length; i += 2) {
        v = new THREE.Vector2();
        v.x = parseFloat(texcoords[i + 0]);
        v.y = parseFloat(texcoords[i + 1]);
        uvs.push(v);
      }
    }

    // Now pull out the face indices
    if (hasTexCoord)
      var texIndices = texcoordIndexStr.split(/\s/);
    for (let i = 0; i < indices.length; i++) {
      var faceIndices = [];
      var uvIndices = [];
      while (parseFloat(indices[i]) >= 0) {
        faceIndices.push(parseFloat(indices[i]));
        if (hasTexCoord)
          uvIndices.push(parseFloat(texIndices[i]));
        i++;
      }

      while (faceIndices.length > 3) {
        // Take the last three, make a triangle, and remove the
        // middle one (works for convex & continuously wrapped)
        if (hasTexCoord) {
          // Add to the UV layer
          geometry.faceVertexUvs[0].push([
            uvs[parseFloat(uvIndices[uvIndices.length - 3])].clone(),
            uvs[parseFloat(uvIndices[uvIndices.length - 2])].clone(),
            uvs[parseFloat(uvIndices[uvIndices.length - 1])].clone()
          ]);
          // Remove the second-to-last vertex
          var tmp = uvIndices[uvIndices.length - 1];
          uvIndices.pop();
          uvIndices[uvIndices.length - 1] = tmp;
        }
        // Make a face
        geometry.faces.push(new THREE.Face3(
          faceIndices[faceIndices.length - 3],
          faceIndices[faceIndices.length - 2],
          faceIndices[faceIndices.length - 1]
        ));
        // Remove the second-to-last vertex
        tmp = faceIndices[faceIndices.length - 1];
        faceIndices.pop();
        faceIndices[faceIndices.length - 1] = tmp;
      }

      // Make a face with the final three
      if (faceIndices.length === 3) {
        if (hasTexCoord) {
          geometry.faceVertexUvs[0].push([
            uvs[parseFloat(uvIndices[uvIndices.length - 3])].clone(),
            uvs[parseFloat(uvIndices[uvIndices.length - 2])].clone(),
            uvs[parseFloat(uvIndices[uvIndices.length - 1])].clone()
          ]);
        }

        geometry.faces.push(new THREE.Face3(
          faceIndices[0], faceIndices[1], faceIndices[2]
        ));
      }
    }

    geometry.computeBoundingSphere();

    if (creaseAngle > 0)
      geometry.computeAngleVertexNormals(creaseAngle);
    else
      geometry.computeFaceNormals();

    return geometry;
  },

  parseSphere: function(sphere) {
    console.log('Parse Sphere');

    var radius = getNodeAttribute(sphere, 'radius', '1');
    var subdivision = getNodeAttribute(sphere, 'subdivision', '8,8').split(',');
    var sphereGeometry = new THREE.SphereGeometry(radius, subdivision[0], subdivision[1]);
    sphereGeometry.userData = { 'x3dType': 'Sphere' };
    return sphereGeometry;
  },

  parseDirectionalLight: function(light, scene) {
    // TODO shadows
    console.log('Parse DirectionalLight');

    var intensity = getNodeAttribute(light, 'intensity', '1');
    var direction = getNodeAttribute(light, 'direction', '0,0,-1').split(',');
    var lightObject = new THREE.DirectionalLight(0xffffff, intensity);
    // TODO fix position
    //lightObject.position.copy(direction);
    lightObject.target.position.copy(direction);
    scene.add(lightObject.target);
    return lightObject;
  },

  parseBackground: function(background) {
    console.log('Parse Background');
  }
};

function getNodeAttribute(node, attributeName, defaultValue) {
  if (node === undefined)
    console.log('error');
  if (attributeName in node.attributes)
    return node.attributes.getNamedItem(attributeName).value;
  return defaultValue;
}

function convertStringToVec3(s) {
  s = s.split(/\s/);
  var v = new THREE.Vector3(parseFloat(s[0]), parseFloat(s[1]), parseFloat(s[2]));
  return v;
}

function convertStringToQuaternion(s) {
  var pos = s.split(/\s/);
  var q = new THREE.Quaternion();
  q.setFromAxisAngle(
    new THREE.Vector3(parseFloat(pos[0]), parseFloat(pos[1]), parseFloat(pos[2])),
    parseFloat(pos[3])
  );
  return q;
}

function convertStringTorgb(s) {
  var v = convertStringToVec3(s);
  return new THREE.Color(v.x, v.y, v.z);
}

// Source: https://gist.github.com/Ni55aN/90c017fafbefd3e31ef8d98ab6566cfa
// Demo: https://codepen.io/Ni55aN/pen/zROmoe?editors=0010
THREE.Geometry.prototype.computeAngleVertexNormals = function(angle) {
  function weightedNormal(normals, vector) {
    var normal = new THREE.Vector3();
    for (let i = 0, l = normals.length; i < l; i++) {
      if (normals[i].angleTo(vector) < angle)
        normal.add(normals[ i ]);
    }
    return normal.normalize();
  }

  this.computeFaceNormals();

  var vertexNormals = [];
  for (let i = 0, l = this.vertices.length; i < l; i++)
    vertexNormals[ i ] = [];
  for (let i = 0, fl = this.faces.length; i < fl; i++) {
    let face = this.faces[i];
    vertexNormals[face.a].push(face.normal);
    vertexNormals[face.b].push(face.normal);
    vertexNormals[face.c].push(face.normal);
  }

  for (let i = 0, fl = this.faces.length; i < fl; i++) {
    let face = this.faces[i];
    face.vertexNormals[0] = weightedNormal(vertexNormals[face.a], face.normal);
    face.vertexNormals[1] = weightedNormal(vertexNormals[face.b], face.normal);
    face.vertexNormals[2] = weightedNormal(vertexNormals[face.c], face.normal);
  }

  if (this.faces.length > 0)
    this.normalsNeedUpdate = true;
};
