function requestFullscreen(view) {
  const elem = view.view3D;
  if (typeof elem.requestFullscreen !== 'undefined')
    elem.requestFullscreen();
  else if (typeof elem.msRequestFullscreen !== 'undefined')
    elem.msRequestFullscreen();
  else if (typeof elem.mozRequestFullScreen !== 'undefined')
    elem.mozRequestFullScreen();
  else if (typeof elem.webkitRequestFullscreen !== 'undefined')
    elem.webkitRequestFullscreen();
}

function exitFullscreen(scene) {
  if (typeof document.exitFullscreen !== 'undefined')
    document.exitFullscreen();
  else if (typeof document.msExitFullscreen !== 'undefined')
    document.msExitFullscreen();
  else if (typeof document.mozCancelFullScreen !== 'undefined')
    document.mozCancelFullScreen();
  else if (typeof document.webkitExitFullscreen !== 'undefined')
    document.webkitExitFullscreen();

  // Fix bug where toolbar can be hidden by the canvas on exit fullscreen
  if (typeof scene !== 'undefined') {
    setTimeout(() => {
      scene.resize();
      scene.resize();
      scene.resize();
    }, 100);
  }
}

function onFullscreenChange(fullscreenButton, exitFullscreenButton) {
  const element = document.fullScreenElement || document.mozFullScreenElement || document.webkitFullScreenElement || document.msFullScreenElement || document.webkitCurrentFullScreenElement;
  if (element != null) {
    fullscreenButton.style.display = 'none';
    exitFullscreenButton.style.display = 'inline';
  } else {
    fullscreenButton.style.display = 'inline';
    exitFullscreenButton.style.display = 'none';
  }
}

export {requestFullscreen, exitFullscreen, onFullscreenChange};
