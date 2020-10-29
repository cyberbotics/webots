function render() {
  _wr_scene_render(_wr_scene_get_instance(), null, true);
}

function renderLoop() {
   render();
   //window.setTimeout(renderLoop, 1000 / 60);
}

function create_wren_scene() {
  // Generate a dummy texture.
  let texture = _wr_texture_2d_new();
  let ptr = allocate(intArrayFromString("dummy.jpg"), 'i8', ALLOC_NORMAL);
  _wr_texture_2d_set_file_path(texture, ptr);
  _wr_texture_set_size(texture, 256, 256);
  let content = _wrjs_dummy_texture();
  _wr_texture_2d_set_data(texture, content);
  _wr_texture_setup(texture);


  let sphereProgram = _wr_shader_program_new();
  _wr_shader_program_use_uniform(sphereProgram, 0);
  _wr_shader_program_use_uniform(sphereProgram, 1);
  _wr_shader_program_use_uniform(sphereProgram, 2);
  _wr_shader_program_use_uniform(sphereProgram, 16);
  _wr_shader_program_use_uniform(sphereProgram, 17);
  _wr_shader_program_use_uniform_buffer(sphereProgram, 0);
  _wr_shader_program_use_uniform_buffer(sphereProgram, 4);

  Module.ccall('wr_shader_program_set_vertex_shader_path', null, ['number', 'string'], [sphereProgram, "../../resources/wren/shaders/default.vert"]);
  Module.ccall('wr_shader_program_set_fragment_shader_path', null, ['number', 'string'], [sphereProgram, "../../resources/wren/shaders/default.frag"]);

  _wr_shader_program_setup(sphereProgram);



  let sphereRenderable = _wr_renderable_new();
  let sphereMesh = _wr_static_mesh_unit_sphere_new(2, true, false);
  let sphereMaterial = _wr_phong_material_new();
  _wr_material_set_default_program(sphereMaterial, sphereProgram);
  _wr_material_set_texture(sphereMaterial, texture, 0);
  let sphereTransform = _wr_transform_new();

  _wr_renderable_set_mesh(sphereRenderable, sphereMesh);
  _wr_renderable_set_material(sphereRenderable, sphereMaterial, null);
  _wr_transform_attach_child(sphereTransform, sphereRenderable);

  root = _wr_scene_get_root(_wr_scene_get_instance());
  _wr_transform_attach_child(root, sphereTransform);
}

function main() {
  create_wren_scene();

  renderLoop();

  console.log("Continue");

}

class MyParser {
  constructor() {
    this.Nodes = [];
  }

  parse(text){
    console.log('X3D: Parsing');

    let xml = null;
    if (window.DOMParser) {
      let parser = new DOMParser();
      xml = parser.parseFromString(text, 'text/xml');
    } else { // Internet Explorer
      xml = new ActiveXObject('Microsoft.XMLDOM');
      xml.async = false;
      xml.loadXML(text);
    }

    let scene = xml.getElementsByTagName('Scene')[0];
    console.log(scene);
    if (scene === undefined) {
      console.error("Scene not found");
    } else {
      console.log("Une scene");
      this.parseNode(scene);
    }

    //_wr_scene_render(_wr_scene_get_instance(), null, true);
    main();
    console.log("FINI");
  }

  parseNode(node) {
    if(node.tagName === 'Scene') {
      let id = getNodeAttribute(node, 'id');
      new WbScene(id);
      this.parseChildren(node);
    } else if (node.tagName === 'WorldInfo') {
      this.parseWorldInfo(node);
    } else if (node.tagName === 'Viewpoint') {
      this.parseViewpoint(node);
    } else if (node.tagName === 'Shape') {
      this.parseShape(node);
    } else {
      console.log(node.tagName);
      console.error("The parser doesn't support this type of node");
    }
  }

  parseChildren(node, currentObject) {
    for (let i = 0; i < node.childNodes.length; i++) {
      let child = node.childNodes[i];
      if (typeof child.tagName !== 'undefined'){
        this.parseNode(child);
      }

    }
  }

  parseWorldInfo(node){
    console.log("Un WorldInfo");
  }

  parseViewpoint(node){
    /*Information à disposition
    Id
    Orientation: vec4
    Position: vec3
    Exposure : float ou int
    bloomThreshold: float ou int
    zNear = float
    followsmoothness
    */
    let id = getNodeAttribute(node, 'id');
    let orientation = convertStringToQuaternion(getNodeAttribute(node, 'orientation'));
    let position = convertStringToVec3(getNodeAttribute(node, 'position'));
    let exposure = parseFloat(getNodeAttribute(node, 'exposure'));
    let bloomThreshold = parseFloat(getNodeAttribute(node, 'bloomThreshold'));
    let far = parseFloat(getNodeAttribute(node, 'far'));
    let zNear = parseFloat(getNodeAttribute(node, 'zNear'));
    let followsmoothness = parseFloat(getNodeAttribute(node, 'followsmoothness'));

    let viewpoint = new WbViewpoint(id, orientation, position, exposure, bloomThreshold, zNear, far, followsmoothness);
  }

  parseShape(node){
    console.log("Une Shape");
    let id = getNodeAttribute(node, 'id');
    let castShadows = getNodeAttribute(node, 'castShadows').toLowerCase() === 'true';

    let shape = new WbShape(id, castShadows);

    let geometry = this.parseGeometry(node.childNodes[0]);
    shape.geometry = geometry;
    shape.createWrenObjects();
  }

  parseGeometry(node){
    let geometry;
    if(node.tagName === 'Box') {
      geometry = this.parseBox(node);
    } else {
      geometry = undefined
    }
    return geometry;
  }

  parseBox(node) {
    console.log("Une box");
    let id = getNodeAttribute(node, 'id');
    let size = convertStringToVec3(getNodeAttribute(node, 'size'));
    if(size === undefined)
      size = glm.vec3(0.1,0.1,0.1);

    return new WbBox(id, size);
  }
}

function getNodeAttribute(node, attributeName) {
  console.assert(node && node.attributes);
  if (attributeName in node.attributes)
    return node.attributes.getNamedItem(attributeName).value;
  return undefined;
}

function convertStringToVec3(s) {
  s = s.split(/\s/);
  let v = new glm.vec3(parseFloat(s[0]), parseFloat(s[1]), parseFloat(s[2]));
  return v;
}

function convertStringToQuaternion(s) {
  let pos = s.split(/\s/);
  let q = glm.angleAxis(parseFloat(pos[3]), new glm.vec3(parseFloat(pos[0]), parseFloat(pos[1]), parseFloat(pos[2])));
  return q;
}
