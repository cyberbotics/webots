import * as wbrotation from './templating/modules/webots/wbrotation.js';
import * as wbutility from './templating/modules/webots/wbutility.js';
import * as wbgeometry from './templating/modules/webots/wbgeometry.js';
import * as wbrandom from './templating/modules/webots/wbrandom.js';
import * as wbvector2 from './templating/modules/webots/wbvector2.js';
import * as wbvector3 from './templating/modules/webots/wbvector3.js';

export default class TemplateEngine {
  constructor(version) {
    this.id = Date.now();
    this.minimalTemplate = `
    function render(text) {
      return text;
    };

    let ___vrml = '';
    let ___tmp;

    const context = %context%;

    const fields = { %fields% };

    %body%
    `;

    // fake context
    this.context = `
    {
      world: '',
      proto: '',
      project_path: '',
      webots_version: {major: ${version.major}, revision: ${version.revision}},
      webots_home: '',
      temporary_files_path: '',
      os: 'linux',
      id: ${this.id},
      coordinate_system: 'ENU'
    }
    `;

    this.gOpeningToken = '%<';
    this.gClosingToken = '>%';
  };

  encodeProtoBodyForEval(body) {
    let indexClosingToken = 0;
    let lastIndexClosingToken = -1;
    const expressionToken = this.gOpeningToken + '=';

    let jsBody = '';
    while (true) {
      let indexOpeningToken = body.indexOf(this.gOpeningToken, indexClosingToken);
      if (indexOpeningToken === -1) { // no more matches
        if (indexClosingToken < body.length) {
          // what comes after the last closing token is plain vrml
          // note: ___vrml is a local variable to the generateVrml javascript function
          jsBody += '___vrml += render(`' + body.substr(indexClosingToken, body.length - indexClosingToken) + '`);';
          break;
        }
      }

      indexClosingToken = body.indexOf(this.gClosingToken, indexOpeningToken);
      if (indexClosingToken === -1)
        throw new Error('Expected JavaScript closing token \'>%\' is missing.');

      indexClosingToken = indexClosingToken + this.gClosingToken.length; // point after the template token

      if (indexOpeningToken > 0 && lastIndexClosingToken === -1)
        // what comes before the first opening token should be treated as plain vrml
        jsBody += '___vrml += render(`' + body.substr(0, indexOpeningToken) + '`);';

      if (lastIndexClosingToken !== -1 && indexOpeningToken - lastIndexClosingToken > 0)
        // what is between the previous closing token and the current opening token should be treated as plain vrml
        jsBody += '___vrml += render(`' + body.substr(lastIndexClosingToken, indexOpeningToken - lastIndexClosingToken) + '`);';

      // anything in-between the tokens is either an expression or plain JavaScript
      let statement = body.substr(indexOpeningToken, indexClosingToken - indexOpeningToken);
      // if it starts with '%<=' it's an expression
      if (statement.startsWith(expressionToken)) {
        statement = statement.replace(expressionToken, '').replace(this.gClosingToken, '');
        // note: ___tmp is a local variable to the generateVrml javascript function
        jsBody += '___tmp = ' + statement + '; ___vrml += eval(\'___tmp\');';
      } else // raw javascript snippet, remove the tokens
        jsBody += statement.replace(this.gOpeningToken, '').replace(this.gClosingToken, '');

      lastIndexClosingToken = indexClosingToken;
    }

    // remove imports from the body, if any is present (all necessary templating modules are imported in this file)
    const matches = jsBody.match(/(import(?:.*?from.*?'.*?')[;\s])/g);
    if (matches !== null) {
      for (const match of matches)
        jsBody = jsBody.replace(match, '');
    }

    return jsBody;
  };

  generateVrml(fields, body) {
    let template = this.minimalTemplate;
    const jsBody = this.encodeProtoBodyForEval(body);

    // fill template
    template = template.replace('%context%', this.context);
    template = template.replace('%fields%', fields);
    template = template.replace('%body%', jsBody);

    return eval(template);
  };
}
