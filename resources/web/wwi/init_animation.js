import {webots} from './webots.js';

function init() {
  const name = location.pathname.substring(location.pathname.lastIndexOf('/') + 1).replace('.html', '');
  let view = new webots.View(document.getElementById('playerDiv'));
  view.open(name + '.x3d');
  view.setAnimation(name + '.json', 'play', true);
}

if (!!window.chrome)
  init();
else {
  Module['onRuntimeInitialized'] = _ => {
    console.log('wasm loaded ');
    init();
  };
}
