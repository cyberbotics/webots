import {webots} from './Webots.js';

function init() {
  const name = location.pathname.substring(location.pathname.lastIndexOf('/') + 1).replace('.html', '');
  let view = new webots.View(document.getElementById('view3d'));
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

let leftPane = document.createElement('div');
leftPane.className = 'left';
leftPane.id = 'leftPane'

let rightPane = document.createElement('div');
rightPane.className = 'right';
rightPane.id = 'rightPane';

let timeSlider = document.createElement('input');
timeSlider.className = 'time-slider';
timeSlider.type = 'range';
timeSlider.min = 0;
timeSlider.max = 6000;
timeSlider.step = 1;
timeSlider.value = 0;
timeSlider.id = 'timeSlider';

let playButton = document.createElement('button');
playButton.className = 'player-btn icon-play';

let fullscreenButton = document.createElement('button');
fullscreenButton.className = 'player-btn icon-fullscreen';

let currentTime = document.createElement('span');
currentTime.innerHTML = '00:00.<small>00</small>';
currentTime.className = 'current-time';
currentTime.disabled = false;

let timeDivider = document.createElement('span');
timeDivider.innerHTML = '\\';
timeDivider.className = 'time-divider';

let totalTime = document.createElement('span');
totalTime.innerHTML = '00:00.<small>00</small>';
totalTime.className = 'total-time';

document.getElementById('playBar').appendChild(timeSlider);
document.getElementById('playBar').appendChild(leftPane);
document.getElementById('playBar').appendChild(rightPane);
document.getElementById('leftPane').appendChild(playButton);
document.getElementById('leftPane').appendChild(currentTime);
document.getElementById('leftPane').appendChild(timeDivider);
document.getElementById('leftPane').appendChild(totalTime);
document.getElementById('rightPane').appendChild(fullscreenButton);
function updateSliderBackground() {
  // Hack for webkit-browsers which don't support input range progress indication
  var percent = (ui.value / 100) * 100;
  document.getElementById('timeSlider').style.background = '-webkit-linear-gradient(left, #e00 0%, #e00 ' + percent + '%, rgba(204,204,204, 0.7) ' + percent + '%)';
};
updateSliderBackground();
