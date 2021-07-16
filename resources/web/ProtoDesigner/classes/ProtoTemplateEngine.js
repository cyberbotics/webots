export default class ProtoTemplateEngine {
  constructor(fields, body) {
    this.template = `
    function render(text) {
        return text;
      };

      let ___vrml = '';
      let ___tmp;

      const context = { %context% };

      const fields = { %fields% };

      %body%
    `;

    // fill template
    this.template = this.template.replace('%import%', '');
    this.template = this.template.replace('%context%', '');
    this.template = this.template.replace('%fields%', fields);
    this.template = this.template.replace('%body%', body);

    console.log('Filled Template: \n' + this.template);
  };

  generateVrml() {
    /*
    console.log('Empty Template: \n' + text);
    let script = text.replace('%import%', '');
    script = script.replace('%context%', '');
    script = script.replace('%fields%', fields);
    script = script.replace('%body%', body);

    script = script.replace('export function', 'function');
    console.log('Filled Template: \n' + script);
    */
    return eval(this.template);
  };
}
