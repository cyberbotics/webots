/* global THREE, ActiveXObject, MeshManager, TextureManager */
'use strict';

// Inspiration: https://github.com/lkolbly/threejs-x3dloader/blob/master/X3DLoader.js

// TODO missing nodes/attributes:
// - shadows
// - lights
// - texture transform
// - viewpoint
// - geometry primitives:
//   - ElevationGrid
//   - IndexedLineSet
//   - PointSet
//   - Orientation/Texture mapping mismatch: Cone, Sphere, Cylinder, Capsule
// - USE/DEF dictionary

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
    object.name = 'n0';

    return object;
  },

  parseNode: function(parentObject, node) {
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
    } else if (node.tagName === 'Viewpoint')
      object = this.parseViewpoint(node);
    else if (node.tagName === 'Background')
      object = this.parseBackground(node);
    else
      this.parseChildren(node, parentObject);

    if (!object)
      return;

    this.setName(node, object);
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
      else if (child.tagName === 'Box')
        geometry = this.parseBox(child);
      else if (child.tagName === 'Cone')
        geometry = this.parseCone(child);
      else if (child.tagName === 'Cylinder')
        geometry = this.parseCylinder(child);
      else if (child.tagName === 'IndexedFaceSet')
        geometry = this.parseIndexedFaceSet(child);
      else if (child.tagName === 'Plane')
        geometry = this.parsePlane(child);
      else if (child.tagName === 'Sphere')
        geometry = this.parseSphere(child);
      else
        console.log('X3dLoader: Unknown node: ' + child.tagName);
    }

    var mesh = new THREE.Mesh(geometry, material);
    mesh.userData.x3dType = 'Shape';
    return mesh;
  },

  parseAppearance: function(appearance) {
    var mat = new THREE.MeshBasicMaterial({color: 0xffffff});
    mat.userData.x3dType = 'Appearance';
    this.setName(appearance, mat); // TODO set name for image transform, material etc.

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
    var textureTransform = appearance.getElementsByTagName('TextureTransform');
    var colorMap;
    if (imageTexture.length > 0)
      colorMap = this.parseImageTexture(imageTexture[0], textureTransform);

    var materialSpecifications = {color: diffuse, specular: specular, emissive: emissive, shininess: shininess};
    if (colorMap)
      materialSpecifications.map = colorMap;

    mat = new THREE.MeshPhongMaterial(materialSpecifications);
    mat.userData.x3dType = 'Appearance';

    return mat;
  },

  parsePBRAppearance: function(pbrAppearance) {
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

    var textureTransform = pbrAppearance.getElementsByTagName('TextureTransform');
    var imageTextures = pbrAppearance.getElementsByTagName('ImageTexture');
    for (let t = 0; t < imageTextures.length; t++) {
      var imageTexture = imageTextures[t];
      var type = getNodeAttribute(imageTexture, 'type', '');
      if (type === 'baseColor')
        materialSpecifications.map = this.parseImageTexture(imageTexture, textureTransform);
      // else if (type === 'occlusion')  // Not working as expected.
      //   materialSpecifications.aoMap = this.parseImageTexture(imageTexture, textureTransform);
      else if (type === 'roughness') {
        materialSpecifications.roughnessMap = this.parseImageTexture(imageTexture, textureTransform);
        materialSpecifications.roughness = 1.0;
      } else if (type === 'metalness')
        materialSpecifications.metalnessMap = this.parseImageTexture(imageTexture, textureTransform);
      else if (type === 'normal')
        materialSpecifications.normalMap = this.parseImageTexture(imageTexture, textureTransform);
      else if (type === 'emissive')
        materialSpecifications.emissiveMap = this.parseImageTexture(imageTexture, textureTransform);
    }

    // TODO -> asynchronous texture load
    /* var loader = new THREE.CubeTextureLoader();
    loader.setPath('/robot-designer/assets/common/textures/cubic/');
    materialSpecifications.envMap = loader.load([
      'noon_sunny_empty_right.jpg', 'noon_sunny_empty_left.jpg',
      'noon_sunny_empty_top.jpg', 'noon_sunny_empty_bottom.jpg',
      'noon_sunny_empty_front.jpg', 'noon_sunny_empty_back.jpg'
    ]); */

    var mat = new THREE.MeshStandardMaterial(materialSpecifications);
    mat.userData.x3dType = 'PBRAppearance';

    return mat;
  },

  parseImageTexture: function(imageTexture, textureTransform) {
    var texture = new THREE.Texture();

    var filename = getNodeAttribute(imageTexture, 'url', '');
    filename = filename.split(/['"\s]/).filter(n => n);

    // look for already loaded texture.
    var textureManager = new TextureManager();
    // load the texture in an asynchronous way.
    var image = textureManager.loadOrRetrieveTexture(filename[0], texture);
    if (image) // else it could be updated later
      texture.image = image;

    var wrapS = getNodeAttribute(imageTexture, 'repeatS', 'true');
    var wrapT = getNodeAttribute(imageTexture, 'repeatT', 'true');
    texture.wrapS = wrapS === 'true' ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;
    texture.wrapT = wrapT === 'true' ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;

    if (textureTransform && textureTransform[0]) {
      texture.matrixAutoUpdate = false;

      var center = convertStringToVec2(getNodeAttribute(textureTransform[0], 'center', '0 0'));
      // texture.center.set(center.x, -center.y - 1.0);
      var rotation = parseFloat(getNodeAttribute(textureTransform[0], 'rotation', '0'));
      // texture.rotation = rotation;
      var scale = convertStringToVec2(getNodeAttribute(textureTransform[0], 'scale', '1 1'));
      // texture.repeat.set(scale.x, scale.y); // TODO differences with X3D -> not scaled around center
      var translation = convertStringToVec2(getNodeAttribute(textureTransform[0], 'translation', '0 0'));
      // texture.offset.set(-translation.x, -translation.y);
      texture.onUpdate = () => {
        // X3D UV transform matrix differs from THREE.js default one
        /*var tM = new THREE.Matrix4();
        tM.makeTranslation(translation.x, translation.y, 0.0);
        var cM = new THREE.Matrix4();
        cM.makeTranslation(center.x, -center.y - 1.0);
        var minusCM = new THREE.Matrix4();
        minusCM.makeTranslation(-center.x, center.y + 1.0, 0.0);
        var sM = new THREE.Matrix4();
        sM.makeScale(scale.x, scale.y, 1.0);
        var rM = new THREE.Matrix4();
        rM.makeRotationZ(rotation);
        var transform = new THREE.Matrix4();
        transform.multiply(minusCM).multiply(sM).multiply(rM).multiply(cM).multiply(tM);
        texture.matrix.getNormalMatrix(transform);*/

        var c = Math.cos(rotation);
        var s = Math.sin(rotation);
        var sx = scale.x;
        var sy = scale.y;
        var cx = center.x;
        var cy = center.y;
        var tx = -translation.x;
        var ty = 1.0 - translation.y;
        texture.matrix.set(
          sx * c, sx * s, -sx * (c * tx + s * ty - c * cx + s * (cy + 1)) - cx,
          -sy * s, sy * c, -sy * (-s * tx + c * ty + s * cx + c * (cy - 1)) + cy + 1,
          0, 0, 1
        );
      };
      texture.needsUpdate = true;
    }

    return texture;
  },

  parseIndexedFaceSet: function(ifs) {
    var meshManager = new MeshManager();
    var useName = getNodeAttribute(ifs, 'USE', null);
    if (useName)
      return meshManager.getGeometry(useName);

    var coordinate = ifs.getElementsByTagName('Coordinate')[0];
    var textureCoordinate = ifs.getElementsByTagName('TextureCoordinate')[0];

    var geometry = new THREE.Geometry();
    geometry.userData = { 'x3dType': 'IndexedFaceSet' };
    this.setName(ifs, geometry);

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

    var defName = getNodeAttribute(ifs, 'DEF', null);
    if (defName) {
      meshManager = new MeshManager();
      meshManager.addGeometry(defName, geometry);
    }

    return geometry;
  },

  parseBox: function(box) {
    var size = convertStringToVec3(getNodeAttribute(box, 'size', '2 2 2'));
    var boxGeometry = new THREE.BoxBufferGeometry(size.x, size.y, size.z);
    boxGeometry.userData = { 'x3dType': 'Box' };
    this.setName(box, boxGeometry);
    return boxGeometry;
  },

  parseCone: function(cone) {
    var radius = getNodeAttribute(cone, 'bottomRadius', '0');
    var height = getNodeAttribute(cone, 'height', '0');
    var subdivision = getNodeAttribute(cone, 'subdivision', '32');
    var openEnded = getNodeAttribute(cone, 'bottom', 'true') !== 'true';
    // TODO var openSided = getNodeAttribute(cone, 'side', 'true') === 'true' ? false : true;
    var coneGeometry = new THREE.ConeBufferGeometry(radius, height, subdivision, 1, openEnded);
    coneGeometry.userData = { 'x3dType': 'Cone' };
    this.setName(cone, coneGeometry);
    return coneGeometry;
  },

  parseCylinder: function(cylinder) {
    var radius = getNodeAttribute(cylinder, 'radius', '0');
    var height = getNodeAttribute(cylinder, 'height', '0');
    var subdivision = getNodeAttribute(cylinder, 'subdivision', '32');
    var openEnded = getNodeAttribute(cylinder, 'bottom', 'true') !== 'true';
    // TODO var openSided = getNodeAttribute(cylinder, 'side', 'true') === 'true' ? false : true;
    // TODO var openTop = getNodeAttribute(cylinder, 'top', 'true') === 'true' ? false : true;
    var cylinderGeometry = new THREE.CylinderBufferGeometry(radius, radius, height, subdivision, 1, openEnded);
    cylinderGeometry.userData = { 'x3dType': 'Cylinder' };
    this.setName(cylinder, cylinderGeometry);
    return cylinderGeometry;
  },

  parsePlane: function(plane) {
    var size = getNodeAttribute(plane, 'size', '2,2').split(',');
    var planeGeometry = new THREE.PlaneBufferGeometry(size[0], size[1]);
    planeGeometry.userData = { 'x3dType': 'Plane' };
    this.setName(plane, planeGeometry);
    return planeGeometry;
  },

  parseSphere: function(sphere) {
    var radius = getNodeAttribute(sphere, 'radius', '1');
    var subdivision = getNodeAttribute(sphere, 'subdivision', '8,8').split(',');
    var sphereGeometry = new THREE.SphereBufferGeometry(radius, subdivision[0], subdivision[1]);
    sphereGeometry.userData = { 'x3dType': 'Sphere' };
    this.setName(sphere, sphereGeometry);
    return sphereGeometry;
  },

  parseDirectionalLight: function(light, scene) {
    // TODO shadows
    var ambientIntensity = getNodeAttribute(light, 'ambientIntensity', '0');
    var color = convertStringTorgb(getNodeAttribute(light, 'color', '1 1 1'));
    var direction = convertStringToVec3(getNodeAttribute(light, 'direction', '0 0 -1'));
    var intensity = getNodeAttribute(light, 'intensity', '1');
    // TODO var on = getNodeAttribute(light, 'on', 'true') === 'true';

    if (ambientIntensity > 0) {
      var ambientLightObject = new THREE.AmbientLight(color, ambientIntensity);
      this.scene.add(ambientLightObject);
    }

    var lightObject = new THREE.DirectionalLight(color, intensity);
    lightObject.userData = { 'x3dType': 'DirectionalLight' };
    this.setName(light, lightObject);
    lightObject.position.set(-direction.x, -direction.y, -direction.z);
    return lightObject;
  },

  parseBackground: function(background) {
    var color = getNodeAttribute(background, 'skyColor', '0 0 0');
    this.scene.background = convertStringTorgb(color);

    var cubeTextureEnabled = false;
    var attributeNames = ['rightUrl', 'leftUrl', 'topUrl', 'bottomUrl', 'frontUrl', 'backUrl'];
    var urls = [];
    for (var i = 0; i < 6; i++) {
      var url = getNodeAttribute(background, attributeNames[i], null);
      if (url) {
        cubeTextureEnabled = true;
        url = url.split(/['"\s]/).filter(n => n)[0];
      }
      urls.push(url);
    }

    if (cubeTextureEnabled) {
      var cubeTexture = new THREE.CubeTexture();
      var textureManager = new TextureManager();
      var missing = 0;
      for (i = 0; i < 6; i++) {
        if (urls[i] === null)
          continue;
        // look for already loaded texture or load the texture in an asynchronous way.
        missing++;
        var image = textureManager.loadOrRetrieveTexture(urls[i], cubeTexture, i);
        if (image) {
          cubeTexture.images[i] = image;
          missing--;
        }
      }
      this.scene.background = cubeTexture;
      if (missing === 0)
        cubeTexture.needsUpdate = true;
    }

    return null;
  },

  parseViewpoint: function(viewpoint) {
    var fov = getNodeAttribute(viewpoint, 'fieldOfView', '0.785');
    var near = getNodeAttribute(viewpoint, 'zNear', '0.1');
    var far = getNodeAttribute(viewpoint, 'zFar', '2000');
    // set default aspect ratio 1 that will be updated on window resize.
    var camera = this.scene.camera;
    if (camera) {
      // TODO set fov, near, far from Viewpoint node
      // camera.fov = fov;
      // camera.near = near;
      // camera.far = far;
    } else {
      console.log('Parse Viewpoint:: error camera');
      camera = new THREE.PerspectiveCamera(fov, 1, near, far);
    }

    if ('position' in viewpoint.attributes) {
      var position = getNodeAttribute(viewpoint, 'position', '0 0 10');
      camera.position.copy(convertStringToVec3(position));
    }
    if ('orientation' in viewpoint.attributes) {
      var quaternion = convertStringToQuaternion(getNodeAttribute(viewpoint, 'orientation', '0 1 0 0'));
      camera.quaternion.copy(quaternion);
    }
    camera.updateProjectionMatrix();
    return null;
  },

  setName: function(node, object) {
    var id = getNodeAttribute(node, 'id', null);
    if (id)
      object.name = String(id);
  }
};

function getNodeAttribute(node, attributeName, defaultValue) {
  if (!node || !node.attributes)
    console.log('error');
  if (attributeName in node.attributes)
    return node.attributes.getNamedItem(attributeName).value;
  return defaultValue;
}

function convertStringToVec2(s) {
  s = s.split(/\s/);
  var v = new THREE.Vector2(parseFloat(s[0]), parseFloat(s[1]));
  return v;
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
