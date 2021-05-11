const template = document.createElement('template');

template.innerHTML = `
<link type="text/css" href="https://cyberbotics.com/wwi/Wrenjs/css/animation.css" rel="stylesheet"/>

<script src="https://www.cyberbotics.com/jquery/1.11.3/jquery.min.js" ></script>
<script src="https://www.cyberbotics.com/jquery-ui/1.11.4/jquery-ui.min.js"></script>
<script src="https://www.cyberbotics.com/jquery-dialogextend/2.0.4/jquery.dialogextend.min.js"></script>
<script src="https://www.cyberbotics.com/ace/1.2.0/ace.js"></script>
<script src='https://git.io/glm-js.min.js'></script>
<script src="https://cyberbotics.com/wwi/Wrenjs/enum.js"></script>
<script>
  var Module = [];
  Module['locateFile'] = function(path, prefix) {

  // if it's a data file, use a custom dir
  if (path.endsWith(".data"))
    return "https://cyberbotics.com/wwi/Wrenjs/" + path;

  // otherwise, use the default, the prefix (JS file's dir) + the path
  return prefix + path;
  }
</script>
<div id="view3d" style="height:80%"></div>
`;

export default class AnimationSlider extends HTMLElement {
  constructor() {
    super();
    this._shadowRoot = this.attachShadow({ mode: 'open' });
    this._shadowRoot.appendChild(template.content.cloneNode(true));

    let script = document.createElement('script');
    script.textContent = `var Module = [];
        Module['locateFile'] = function(path, prefix) {

        // if it's a data file, use a custom dir
        if (path.endsWith(".data"))
          return "https://cyberbotics.com/wwi/Wrenjs/" + path;

        // otherwise, use the default, the prefix (JS file's dir) + the path
        return prefix + path;
      }`;
    this._shadowRoot.appendChild(script);

    script = document.createElement('script');
    script.src = 'https://git.io/glm-js.min.js';
    this._shadowRoot.appendChild(script);

    script = document.createElement('script');
    script.src = 'https://cyberbotics.com/wwi/Wrenjs/enum.js';
    this._shadowRoot.appendChild(script);

    script = document.createElement('script');
    script.src = 'https://cyberbotics.com/wwi/Wrenjs/wrenjs.js';
    this._shadowRoot.appendChild(script);

    script = document.createElement('script');
    script.type = 'module';
    script.src = '../webots/resources/web/wwi/init_animation.js';
    this._shadowRoot.appendChild(script);
  }
}

window.customElements.define('webots-animation', AnimationSlider);
