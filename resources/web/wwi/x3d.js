/* global THREE, ActiveXObject, TextureManager */
'use strict';

// Inspiration: https://github.com/lkolbly/threejs-x3dloader/blob/master/X3DLoader.js

// TODO missing nodes/attributes:
// - shadows
// - lights
// - texture transform
// - Cubemap
// - geometry primitives:
//   - ElevationGrid
//   - IndexedLineSet
//   - PointSet
//   - Texture mapping mismatch: Sphere
// some node attributes are also missing (see TODOs)

THREE.X3DLoader = function(sceneManager, loadManager) {
  this.manager = (typeof loadManager !== 'undefined') ? loadManager : THREE.DefaultLoadingManager;
  this.scene = sceneManager.scene;
  this.camera = sceneManager.camera;
  this.worldInfo = sceneManager.worldInfo;
  this.root = sceneManager.root;
  this.defDictionary = [];
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
    this.root = object;

    return object;
  },

  getDefNode: function(node) {
    var useName = getNodeAttribute(node, 'USE', '');
    if (useName !== '') {
      // look for DEF nodes
      var entry = null;
      for (var i = this.defDictionary.length - 1; i >= 0; i--) {
        if (this.defDictionary[i].name === useName && this.defDictionary[i].tagName === node.tagName) {
          entry = this.defDictionary[i];
          break;
        }
      }
      if (entry) {
        entry.use.push(node);
        return entry.def;
      }
      console.error('X3dLoader: no matching DEF "' + useName + '" node of type "' + node.tagName + '".');
    }
    return null;
  },

  setDefNode: function(node, object) {
    var defName = getNodeAttribute(node, 'DEF', '');
    if (defName !== '')
      this.defDictionary.push({name: defName, tagName: node.tagName, def: object, use: []});
  },

  parseNode: function(parentObject, node) {
    var object = this.getDefNode(node);
    if (object) {
      parentObject.add(object.clone());
      return;
    }

    if (node.tagName === 'Transform')
      object = this.parseTransform(node);
    else if (node.tagName === 'Shape')
      object = this.parseShape(node);
    else if (node.tagName === 'DirectionalLight')
      object = this.parseDirectionalLight(node, parentObject);
    else if (node.tagName === 'PointLight')
      object = this.parsePointLight(node, parentObject);
    else if (node.tagName === 'SpotLight')
      object = this.parseSpotLight(node, parentObject);
    else if (node.tagName === 'Group') {
      console.log('Parse Group', object);
      object = new THREE.Object3D();
      object.userData.x3dType = 'Group';
      this.parseChildren(node, object);
    } else if (node.tagName === 'Viewpoint')
      object = this.parseViewpoint(node);
    else if (node.tagName === 'Background')
      object = this.parseBackground(node);
    else if (node.tagName === 'WorldInfo')
      this.worldInfo = this.parseWorldInfo(node);
    else
      this.parseChildren(node, parentObject);

    if (!object)
      return;

    this.setDefNode(node, object);
    this.setCustomId(node, object);
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
    object.userData.solid = getNodeAttribute(transform, 'solid', 'false') === 'true';
    object.userData.window = getNodeAttribute(transform, 'window', '');
    object.userData.name = getNodeAttribute(transform, 'name', '');

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
    var geometry = null;
    var material = null;
    var angle = 0; // mesh rotation to match Webots appearance

    for (let i = 0; i < shape.childNodes.length; i++) {
      var child = shape.childNodes[i];
      if (typeof child.tagName === 'undefined')
        continue;

      // check if USE node and return the DEF node
      var defObject = this.getDefNode(child);
      if (defObject) {
        if (defObject instanceof THREE.Geometry || defObject instanceof THREE.BufferGeometry)
          geometry = defObject;
        else if (defObject instanceof THREE.Material)
          material = defObject;
        // else error
        continue;
      }

      if (!material) {
        if (child.tagName === 'Appearance')
          material = this.parseAppearance(child);
        else if (child.tagName === 'PBRAppearance')
          material = this.parsePBRAppearance(child);
        if (material) {
          this.setDefNode(child, material);
          this.setCustomId(child, material);
          continue;
        }
      }

      if (!geometry) {
        if (child.tagName === 'Box')
          geometry = this.parseBox(child);
        else if (child.tagName === 'Cone') {
          geometry = this.parseCone(child);
          angle = Math.PI / 2;
        } else if (child.tagName === 'Cylinder') {
          geometry = this.parseCylinder(child);
          angle = Math.PI / 2;
        } else if (child.tagName === 'IndexedFaceSet')
          geometry = this.parseIndexedFaceSet(child);
        else if (child.tagName === 'Sphere')
          geometry = this.parseSphere(child);
        if (geometry) {
          this.setDefNode(child, geometry);
          this.setCustomId(child, geometry);
          continue;
        }
      }

      console.log('X3dLoader: Unknown node: ' + child.tagName);
    }

    // apply default geometry and/or material
    if (!geometry)
      geometry = new THREE.Geometry();
    if (!material)
      material = new THREE.MeshBasicMaterial({color: 0xffffff});

    var mesh = new THREE.Mesh(geometry, material);
    if (angle !== 0)
      mesh.rotateOnAxis(new THREE.Vector3(0, 1, 0), angle);
    mesh.userData.x3dType = 'Shape';
    return mesh;
  },

  parseAppearance: function(appearance) {
    var mat = new THREE.MeshBasicMaterial({color: 0xffffff});
    mat.userData.x3dType = 'Appearance';
    // TODO set object name for image transform, material etc.

    // Get the Material tag
    var material = appearance.getElementsByTagName('Material')[0];
    if (typeof material === 'undefined')
      return mat;

    var materialSpecifications = {};
    var defMaterial = this.getDefNode(material);
    if (defMaterial) {
      materialSpecifications = {
        color: defMaterial.color,
        specular: defMaterial.specular,
        emissive: defMaterial.emissive,
        shininess: defMaterial.shininess
      };
    } else {
      // Pull out the standard colors
      var diffuse = convertStringTorgb(getNodeAttribute(material, 'diffuseColor', '0.8 0.8 0.8'));
      var specular = convertStringTorgb(getNodeAttribute(material, 'specularColor', '0 0 0'));
      var emissive = convertStringTorgb(getNodeAttribute(material, 'emissiveColor', '0 0 0'));
      var shininess = parseFloat(getNodeAttribute(material, 'shininess', '0.2'));
      materialSpecifications = {color: diffuse, specular: specular, emissive: emissive, shininess: shininess};
      this.setDefNode(material, mat);
    }

    // Check to see if there is a texture
    var imageTexture = appearance.getElementsByTagName('ImageTexture');
    var colorMap;
    if (imageTexture.length > 0) {
      colorMap = this.parseImageTexture(imageTexture[0], appearance.getElementsByTagName('TextureTransform'));
      if (colorMap)
        materialSpecifications.map = colorMap;
    }

    mat = new THREE.MeshPhongMaterial(materialSpecifications);
    mat.userData.x3dType = 'Appearance';
    if (material)
      this.setCustomId(material, mat);

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
    // TODO issues with DEF and USE textures with different image transform!
    var texture = this.getDefNode(imageTexture);
    if (texture)
      return texture;

    texture = new THREE.Texture();

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

    var transformObject = null;
    if (textureTransform && textureTransform[0]) {
      transformObject = this.getDefNode(textureTransform[0]);
      if (!transformObject) {
        transformObject = {
          center: convertStringToVec2(getNodeAttribute(textureTransform[0], 'center', '0 0')),
          rotation: parseFloat(getNodeAttribute(textureTransform[0], 'rotation', '0')),
          scale: convertStringToVec2(getNodeAttribute(textureTransform[0], 'scale', '1 1')),
          translation: convertStringToVec2(getNodeAttribute(textureTransform[0], 'translation', '0 0'))
        };
        this.setDefNode(textureTransform[0], transformObject);
      }

      // texture.matrixAutoUpdate = false;
      texture.center.set(transformObject.center.x, -transformObject.center.y - 1.0);
      texture.rotation = transformObject.rotation;
      texture.repeat.set(transformObject.scale.x, transformObject.scale.y); // TODO differences with X3D -> not scaled around center
      texture.offset.set(-transformObject.translation.x, -transformObject.translation.y);
      /* texture.onUpdate = () => {
        // X3D UV transform matrix differs from THREE.js default one
        /* var tM = new THREE.Matrix4();
        tM.makeTranslation(transformObject.translation.x, transformObject.translation.y, 0.0);
        var cM = new THREE.Matrix4();
        cM.makeTranslation(transformObject.center.x, -transformObject.center.y - 1.0);
        var minusCM = new THREE.Matrix4();
        minusCM.makeTranslation(-transformObject.center.x, transformObject.center.y + 1.0, 0.0);
        var sM = new THREE.Matrix4();
        sM.makeScale(transformObject.scale.x, transformObject.scale.y, 1.0);
        var rM = new THREE.Matrix4();
        rM.makeRotationZ(transformObject.rotation);
        var transform = new THREE.Matrix4();
        transform.multiply(minusCM).multiply(sM).multiply(rM).multiply(cM).multiply(tM);
        texture.matrix.getNormalMatrix(transform);

        var c = Math.cos(transformObject.rotation);
        var s = Math.sin(transformObject.rotation);
        var sx = transformObject.scale.x;
        var sy = transformObject.scale.y;
        var cx = transformObject.center.x;
        var cy = transformObject.center.y;
        var tx = -transformObject.translation.x;
        var ty = 1.0 - transformObject.translation.y;
        texture.matrix.set(
          sx * c, sx * s, -sx * (c * tx + s * ty - c * cx + s * (cy + 1)) - cx,
          -sy * s, sy * c, -sy * (-s * tx + c * ty + s * cx + c * (cy - 1)) + cy + 1,
          0, 0, 1
        );
      }; */
      texture.needsUpdate = true;

      this.setCustomId(textureTransform[0], texture);
    }

    this.setCustomId(imageTexture, texture);
    this.setDefNode(imageTexture, texture);
    return texture;
  },

  parseIndexedFaceSet: function(ifs) {
    var coordinate = ifs.getElementsByTagName('Coordinate')[0];
    var textureCoordinate = ifs.getElementsByTagName('TextureCoordinate')[0];

    var geometry = new THREE.Geometry();
    geometry.userData = { 'x3dType': 'IndexedFaceSet' };
    if (!coordinate)
      return geometry;

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

    this.setCustomId(coordinate, geometry);
    if (hasTexCoord)
      this.setCustomId(textureCoordinate, geometry);

    return geometry;
  },

  parseBox: function(box) {
    var size = convertStringToVec3(getNodeAttribute(box, 'size', '2 2 2'));
    var boxGeometry = new THREE.BoxBufferGeometry(size.x, size.y, size.z);
    boxGeometry.userData = { 'x3dType': 'Box' };
    return boxGeometry;
  },

  parseCone: function(cone) {
    var radius = getNodeAttribute(cone, 'bottomRadius', '0');
    var height = getNodeAttribute(cone, 'height', '0');
    var subdivision = getNodeAttribute(cone, 'subdivision', '32');
    var openEnded = getNodeAttribute(cone, 'bottom', 'true') !== 'true';
    // TODO var openSided = getNodeAttribute(cone, 'side', 'true') === 'true' ? false : true;
    // set thetaStart = Math.PI / 2 to match X3D texture mapping
    var coneGeometry = new THREE.ConeBufferGeometry(radius, height, subdivision, 1, openEnded, Math.PI / 2);
    coneGeometry.userData = { 'x3dType': 'Cone' };
    return coneGeometry;
  },

  parseCylinder: function(cylinder) {
    var radius = getNodeAttribute(cylinder, 'radius', '0');
    var height = getNodeAttribute(cylinder, 'height', '0');
    var subdivision = getNodeAttribute(cylinder, 'subdivision', '32');
    var openEnded = getNodeAttribute(cylinder, 'bottom', 'true') !== 'true';
    // TODO var openSided = getNodeAttribute(cylinder, 'side', 'true') === 'true' ? false : true;
    // TODO var openTop = getNodeAttribute(cylinder, 'top', 'true') === 'true' ? false : true;
    // set thetaStart = Math.PI / 2 to match X3D texture mapping
    var cylinderGeometry = new THREE.CylinderBufferGeometry(radius, radius, height, subdivision, 1, openEnded, Math.PI / 2);
    cylinderGeometry.userData = { 'x3dType': 'Cylinder' };
    return cylinderGeometry;
  },

  parseSphere: function(sphere) {
    var radius = getNodeAttribute(sphere, 'radius', '1');
    var subdivision = getNodeAttribute(sphere, 'subdivision', '8,8').split(',');
    var sphereGeometry = new THREE.SphereBufferGeometry(radius, subdivision[0], subdivision[1], -Math.PI / 2); // thetaStart: -Math.PI/2
    sphereGeometry.userData = { 'x3dType': 'Sphere' };
    return sphereGeometry;
  },

  parseDirectionalLight: function(light, parentObject) {
    // TODO shadows
    var ambientIntensity = getNodeAttribute(light, 'ambientIntensity', '0');
    var color = convertStringTorgb(getNodeAttribute(light, 'color', '1 1 1'));
    var direction = convertStringToVec3(getNodeAttribute(light, 'direction', '0 0 -1'));
    var intensity = getNodeAttribute(light, 'intensity', '1');
    var castShadows = convertStringToVec3(getNodeAttribute(light, 'castShadows', 'true'));
    // TODO var on = getNodeAttribute(light, 'on', 'true') === 'true';

    if (ambientIntensity > 0) {
      var ambientLightObject = new THREE.AmbientLight(color, ambientIntensity);
      parentObject.add(ambientLightObject);
    }

    var lightObject = new THREE.DirectionalLight(color, intensity);
    lightObject.castShadows = castShadows;
    lightObject.userData = { 'x3dType': 'DirectionalLight' };
    lightObject.position.set(-direction.x, -direction.y, -direction.z);
    return lightObject;
  },

  parsePointLight: function(light, parentObject) {
    // TODO shadows
    var ambientIntensity = getNodeAttribute(light, 'ambientIntensity', '0');
    var color = convertStringTorgb(getNodeAttribute(light, 'color', '1 1 1'));
    var intensity = getNodeAttribute(light, 'intensity', '1');
    var location = convertStringToVec3(getNodeAttribute(light, 'location', '0 0 0'));
    var castShadows = convertStringToVec3(getNodeAttribute(light, 'castShadows', 'true'));
    // TODO var on = getNodeAttribute(light, 'on', 'true') === 'true';
    // var attenuation = getNodeAttribute(light, 'attenuation', '1 0 0');
    // var radius = convertStringToVec3(getNodeAttribute(light, 'radius', '100'));

    if (ambientIntensity > 0) {
      var ambientLightObject = new THREE.AmbientLight(color, ambientIntensity);
      parentObject.add(ambientLightObject);
    }

    var lightObject = new THREE.PointLight(color, intensity);
    lightObject.castShadows = castShadows;
    lightObject.userData = { 'x3dType': 'PointLight' };
    lightObject.position.set(location.x, location.y, location.z);
    return lightObject;
  },

  parseSpotLight: function(light, parentObject) {
    // TODO shadows
    var ambientIntensity = getNodeAttribute(light, 'ambientIntensity', '0');
    // var attenuation = getNodeAttribute(light, 'attenuation', '1 0 0');
    var beamWidth = getNodeAttribute(light, 'beamWidth', '5707963');
    var color = convertStringTorgb(getNodeAttribute(light, 'color', '1 1 1'));
    var cutOffAngle = getNodeAttribute(light, 'cutOffAngle', '5707963');
    // TODO var direction = convertStringToVec3(getNodeAttribute(light, 'direction', '0 0 -1'));
    var intensity = getNodeAttribute(light, 'intensity', '1');
    var location = convertStringToVec3(getNodeAttribute(light, 'location', '0 0 0'));
    // TODO var on = getNodeAttribute(light, 'on', 'true') === 'true';
    // var radius = convertStringToVec3(getNodeAttribute(light, 'radius', '100'));
    var castShadows = convertStringToVec3(getNodeAttribute(light, 'castShadows', 'true')) === 'true';
    if (ambientIntensity > 0) {
      var ambientLightObject = new THREE.AmbientLight(color, ambientIntensity);
      parentObject.add(ambientLightObject);
    }

    var lightObject = new THREE.SpotLight(color, intensity);
    lightObject.position.set(location.x, location.y, location.z);
    lightObject.angle = beamWidth;
    lightObject.penumbra = cutOffAngle;
    lightObject.castShadows = castShadows;
    lightObject.userData = { 'x3dType': 'SpotLight' };
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
    var camera = this.camera;
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

    // set Webots specific attributes.
    camera.userData.followedId = getNodeAttribute(viewpoint, 'followedId', null);
    camera.userData.followSmoothness = getNodeAttribute(viewpoint, 'followSmoothness', null);
    return null;
  },

  parseWorldInfo: function(worldInfo) {
    return {
      title: getNodeAttribute(worldInfo, 'title', ''),
      window: getNodeAttribute(worldInfo, 'window', '')
    };
  },

  setCustomId: function(node, object) {
    // Some THREE.js nodes, like the material and IndexedFaceSet, merges multiple X3D nodes.
    // In order to be able to retrieve the node to be updated, we need to assign to the object all the ids of the merged X3D nodes.
    if (!node || !object)
      return;
    var id = getNodeAttribute(node, 'id', null);
    if (id) {
      if (object.name !== '')
        object.name = object.name + ';' + String(id);
      else
        object.name = String(id);
    }
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
