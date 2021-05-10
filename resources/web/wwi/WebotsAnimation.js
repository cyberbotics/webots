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
<script src="https://cyberbotics.com/wwi/Wrenjs/wrenjs.js"></script>
<script type="module" src="https://cyberbotics.com/wwi/Wrenjs/init_animation.js"></script>

<div id="view3d" style="height:80%"></div>
`;

export default class AnimationSlider extends HTMLElement {
}
