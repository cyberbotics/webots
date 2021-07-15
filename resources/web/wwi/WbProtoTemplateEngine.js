export default class WbProtoTemplateEngine {
  constructor() {

  };

  minimalTemplate() {
    const a = `
  function render(text) {
    return text;
  };
  let ___vrml = '';
  let ___tmp;

  const context = { %context% };

  const fields = { %fields% };

  %body%`;

    return a;
  };

  evaluateTemplate(templateUrl, fields, body) {
    const xmlhttp = new XMLHttpRequest();
    xmlhttp.open('GET', templateUrl, true);
    xmlhttp.overrideMimeType('plain/text');
    xmlhttp.onreadystatechange = async() => {
      if (xmlhttp.readyState === 4 && (xmlhttp.status === 200 || xmlhttp.status === 0)) // Some browsers return HTTP Status 0 when using non-http protocol (for file://)
        await this.fillTemplate(xmlhttp.responseText, fields, body);
    };
    xmlhttp.send();
  };

  encodeFields(parameters) {
    //  field SFVec3f size 2 1 1
    //  field SFColor color 0 1 1
    return 'size: {value: {x: 2, y: 1, z: 1}, defaultValue: {x: 2, y: 1, z: 1}}, color: {value: {r: 0, g: 1, b: 1}, defaultValue: {r: 0, g: 1, b: 1}}';
  };

  encodeBody(parameters) {
    /*
    %<
      let a = fields.size.value;
    >%
    Shape {
      castShadow TRUE
      geometry Box {
        size %<= a.x >% 1 1
      }
      appearance PBRAppearance {
        baseColor IS color
      }
    }
    */

    const a = 'let a = fields.size.value; ___vrml += render(` Shape { castShadow TRUE geometry Box { size `); ___tmp =  a.x ; ___vrml += eval("___tmp"); ___vrml += render(` 1 1 } appearance PBRAppearance { baseColor IS color } } `);';
    return a;
  };

  fillTemplate(text, fields, body) {
    console.log('Empty Template: \n' + text);
    let script = text.replace('%import%', '');
    script = script.replace('%context%', '');
    script = script.replace('%fields%', fields);
    script = script.replace('%body%', body);

    script = script.replace('export function', 'function');
    console.log('Filled Template: \n' + script);

    const result = eval(script);
    console.log(result);
  };
}
